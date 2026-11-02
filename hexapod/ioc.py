#!/usr/bin/env python3
"""
caproto IOC for the Aerotech HEX300-230HL hexapod. It talks to the Automation1
controller directly through the Python API (hexapod/a1.py).

PVs
---
X, Y, Z        motor records for the translations (controller axes X, Y, Z)
RX, RY, RZ     motor records for the rotations (controller axes A, B, C)
Enable         Disable/Enable the axes; Enable also turns on Global mode if all struts are homed (no homing)
Enabled_RBV    1 when X..C are enabled
Mode_RBV       GetHexapodMode(): -10 not initialized, 0 Disable, 1 Local, 2 Global, 3 Strut.
               The IOC always uses Global mode (X, Y, Z, A, B, C). It turns Global on when all
               struts are homed (after Enable, homing, or before a move) and never homes by itself.
CoR:MeasX/Y/Z  center of rotation (mm) measured from the top plate center
               with the hexapod at its zero pose
CoR:Frame      Base: send the point in bottom base coordinates (z + 230.25)
               Platform: send the measured offset unchanged
CoR:Apply      send CoR:Meas* to SetRotationPoint()
CoR:Reset      put the rotation point back at the top plate center
CoR:X/Y/Z_RBV  GetRotationPoint() readback
StopAll        abort X..C
ClearErrors    acknowledge all axis faults and task errors (AckAll is the same)
HomeAll        home the struts one at a time (in Strut mode), then Global mode
<motor>.HOMF   (or HOMR) home that motor's strut drive (in Strut mode); Global mode once all are homed
ExecuteCommand AeroScript command line (256 characters)
Message        result of the last command

Soft limits
-----------
<motor>.HLM/.LLM (and .DHLM/.DLLM) are loaded from motor_limits.json at startup and saved back
whenever they are changed. They are the platform-pose travel limits, in the motor's units. Clients such
as pyepics' Motor refuse every move while HLM == LLM, so an axis missing from the file cannot be moved
from them. The controller's own software limits belong to the strut drives and are not used here.
"""
import asyncio
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from textwrap import dedent

import automation1 as a1
from caproto import ChannelType
from caproto.server import PVGroup, SubGroup, pvproperty, run, template_arg_parser

from .a1 import Hexapod, IP

DEFAULT_PREFIX = '12idhxpPY:'
# Dial soft limits per PV name, {"X": [low, high], ...}
LIMITS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motor_limits.json')
# Always Global mode: the motors are the platform pose (X, Y, Z in mm; A, B, C in degrees).
UNITS = {'X': 'mm', 'Y': 'mm', 'Z': 'mm', 'A': 'deg', 'B': 'deg', 'C': 'deg'}
LABELS = {'X': 'X', 'Y': 'Y', 'Z': 'Z', 'A': 'RX', 'B': 'RY', 'C': 'RZ'}
# Controller axes 0-5 are named X..C and own the strut drives; 6-11 are Xv..Cv.
# X..C are Cartesian only while the hexapod transform is active. When the hexapod is not
# initialized, commanding X moves strut 1 directly, so moves are refused (see check_init).
HEX_AXES = ['X', 'Y', 'Z', 'A', 'B', 'C']
NOT_INITIALIZED = -10            # GetHexapodMode() when the hexapod library is not initialized
GLOBAL_MODE = 2                  # GetHexapodMode() = HexapodMode.Global (Disable 0, Local 1, Strut 3)
MOVE_TASK = 1
POLL_PERIOD = 0.1                # s, axis status
SLOW_POLL_PERIOD = 1.0           # s, hexapod mode and rotation point
DONE_TOLERANCE = 1e-4            # commanded position this close to the target counts as arrived
SETTLE_TIME = 1.0                # s, after which "motion done" alone ends a move short of its target

# motor record MSTA bits
MSTA_DIRECTION = 1 << 0
MSTA_DONE = 1 << 1
MSTA_PLUS_LS = 1 << 2
MSTA_POSITION = 1 << 5
MSTA_PROBLEM = 1 << 9
MSTA_MOVING = 1 << 10
MSTA_GAIN_SUPPORT = 1 << 11
MSTA_MINUS_LS = 1 << 13
MSTA_HOMED = 1 << 14

PLUS_LIMIT = int(a1.AxisFault.CwEndOfTravelLimitFault) | int(a1.AxisFault.CwSoftwareLimitFault)
MINUS_LIMIT = int(a1.AxisFault.CcwEndOfTravelLimitFault) | int(a1.AxisFault.CcwSoftwareLimitFault)


@dataclass
class AxisState:
    fbk: float      # program position feedback
    cmd: float      # program position command
    drive: int      # DriveStatus bits
    status: int     # AxisStatus bits
    fault: int      # AxisFault bits
    t: float        # time.monotonic() of the read

    @property
    def enabled(self):
        return bool(self.drive & int(a1.DriveStatus.Enabled))

    @property
    def moving(self):
        return bool(self.drive & int(a1.DriveStatus.MoveActive))

    @property
    def done(self):
        return bool(self.status & int(a1.AxisStatus.MotionDone))

    @property
    def homed(self):
        return bool(self.status & int(a1.AxisStatus.Homed))

    @property
    def plus_limit(self):
        return bool(self.fault & PLUS_LIMIT)

    @property
    def minus_limit(self):
        return bool(self.fault & MINUS_LIMIT)


class HexapodHW:
    """Blocking Automation1 calls. Run them through HexapodIOC.call()."""

    def __init__(self, host):
        self.hp = Hexapod(host)
        self.cmd = self.hp.controller.runtime.commands
        self.cfg = a1.StatusItemConfiguration()
        for ax in HEX_AXES:
            for item in (a1.AxisStatusItem.ProgramPositionFeedback, a1.AxisStatusItem.ProgramPositionCommand,
                         a1.AxisStatusItem.DriveStatus, a1.AxisStatusItem.AxisStatus, a1.AxisStatusItem.AxisFault):
                self.cfg.axis.add(item, ax)

    def read_status(self):
        res = self.hp.controller.runtime.status.get_status_items(self.cfg)
        now = time.monotonic()

        def get(item, ax):
            return res.axis.get(item, ax).value

        return {ax: AxisState(fbk=get(a1.AxisStatusItem.ProgramPositionFeedback, ax),
                              cmd=get(a1.AxisStatusItem.ProgramPositionCommand, ax),
                              drive=int(get(a1.AxisStatusItem.DriveStatus, ax)),
                              status=int(get(a1.AxisStatusItem.AxisStatus, ax)),
                              fault=int(get(a1.AxisStatusItem.AxisFault, ax)),
                              t=now)
                for ax in HEX_AXES}

    def soft_limits(self):
        """{axis: (low, high)} for axes whose controller software limits are active, else None."""
        limits = {}
        for ax in HEX_AXES:
            p = self.hp.controller.runtime.parameters.axes[ax].protection
            active = int(p.softwarelimitsetup.value) != 0
            limits[ax] = (p.softwarelimitlow.value, p.softwarelimithigh.value) if active else None
        return limits

    def move(self, axis, position, speed, abort_first=False):
        if abort_first:
            self.cmd.motion.abort(axis)
            self.cmd.motion.waitformotiondone(axis, MOVE_TASK)
        self.cmd.motion.moveabsolute(axis, [position], [speed], MOVE_TASK)

    def abort(self, axes):
        self.cmd.motion.abort(axes)

    def set_axis_enabled(self, axis, enable):
        if enable:
            self.cmd.motion.enable(axis, MOVE_TASK)
        else:
            self.cmd.motion.disable(axis)


async def _put(prop, value):
    """Write an internally computed value only when it changed. Skips putters."""
    enum_strings = getattr(prop, 'enum_strings', None)
    if enum_strings and isinstance(value, int):
        value = enum_strings[value]
    if prop.value != value:
        await prop.write(value, verify_value=False)


class HexMotor(PVGroup):
    """Motor record for one hexapod axis. Dial position is the controller program position."""

    motor = pvproperty(value=0.0, name='', record='motor', precision=4)

    def __init__(self, *args, axis, **kwargs):
        super().__init__(*args, **kwargs)
        self.axis = axis
        self.state = None
        self._move_id = 0
        self._moving = False
        self._stop_requested = False
        self._direction = 1
        self._tweak_task = None

    @property
    def ioc(self):
        return self.parent

    @property
    def fields(self):
        return self.motor.field_inst

    async def initialize(self, state, speed):
        f = self.fields
        await f.description.write(f'Hexapod {LABELS[self.axis]}')
        await f.engineering_units.write(UNITS[self.axis])
        await f.velocity.write(speed)
        await f.motor_step_size.write(1e-5)
        await f.done_moving_to_value.write(1)
        limits = self.ioc.dial_limits.get(LABELS[self.axis])
        if limits is not None:
            await self.set_dial_limits(*limits)
        else:
            await self.ioc.report(f'{LABELS[self.axis]}: no soft limits in {self.ioc.limits_file}; '
                                  'set HLM/LLM before moving it from a client')
        await self.update(state)
        await f.dial_desired_value.write(state.fbk)
        await self.motor.write(f.user_readback_value.value, verify_value=False)

    async def set_dial_limits(self, low, high):
        """Write DLLM/DHLM and the matching LLM/HLM (user = dial + OFF; DIR is not supported)."""
        f = self.fields
        off = f.user_offset.value
        await _put(f.dial_low_limit, low)
        await _put(f.dial_high_limit, high)
        await _put(f.user_low_limit, low + off)
        await _put(f.user_high_limit, high + off)

    async def limit_changed(self, low, high):
        """A client changed one limit field: sync the others and save them."""
        await self.set_dial_limits(low, high)
        await self.ioc.save_limits(LABELS[self.axis], low, high)

    async def update(self, state):
        self.state = state
        f = self.fields
        moving = state.moving or self._moving
        await _put(f.dial_readback_value, state.fbk)
        await _put(f.user_readback_value, state.fbk + f.user_offset.value)
        await _put(f.motor_is_moving, int(moving))
        if not self._moving:
            await _put(f.done_moving_to_value, int(state.done))
        await _put(f.enable_control, int(state.enabled))
        await _put(f.user_high_limit_switch, int(state.plus_limit))
        await _put(f.user_low_limit_switch, int(state.minus_limit))
        msta = MSTA_GAIN_SUPPORT
        msta |= MSTA_DIRECTION if self._direction > 0 else 0
        msta |= MSTA_DONE if f.done_moving_to_value.value else 0
        msta |= MSTA_PLUS_LS if state.plus_limit else 0
        msta |= MSTA_MINUS_LS if state.minus_limit else 0
        msta |= MSTA_POSITION if state.enabled else 0
        msta |= MSTA_PROBLEM if state.fault else 0
        msta |= MSTA_MOVING if moving else 0
        msta |= MSTA_HOMED if state.homed else 0
        await _put(f.motor_status, msta)

    async def move_to(self, dial):
        """Move to a dial position and wait until done. Returns the final user VAL."""
        ioc = self.ioc
        if ioc.check_init and ioc.hexapod_mode == NOT_INITIALIZED:
            msg = f'{self.axis}: hexapod is not initialized (GetHexapodMode() = -10); not moving'
            await ioc.report(msg)
            raise RuntimeError(msg)
        limits = ioc.soft_limits.get(self.axis)
        if limits is not None and limits[1] - limits[0] < DONE_TOLERANCE:
            msg = (f'{self.axis}: controller software limits are [{limits[0]:g}, {limits[1]:g}]; '
                   'restore them before moving')
            await ioc.report(msg)
            raise RuntimeError(msg)
        await ioc.ensure_global_mode()
        f = self.fields
        self._move_id += 1
        move_id = self._move_id
        abort_first = self._moving or (self.state is not None and not self.state.done)
        self._moving = True
        self._stop_requested = False
        if self.state is not None:
            self._direction = 1 if dial >= self.state.fbk else -1
        await f.dial_desired_value.write(dial, verify_value=False)
        await f.done_moving_to_value.write(0)
        await f.motor_is_moving.write(1)
        try:
            await ioc.call(ioc.hw.move, self.axis, dial, f.velocity.value, abort_first)
        except Exception as ex:
            self._moving = False
            await f.motor_is_moving.write(0)
            await f.done_moving_to_value.write(1)
            await ioc.report(f'{self.axis} move failed: {ex}')
            raise
        t0 = time.monotonic()
        while True:
            await asyncio.sleep(POLL_PERIOD)
            if move_id != self._move_id:
                return self.motor.value  # a newer move took over and will finish the put
            st = self.state
            if st is None or st.t <= t0:
                continue
            arrived = abs(st.cmd - dial) < DONE_TOLERANCE
            if st.fault or (st.done and (arrived or self._stop_requested or time.monotonic() - t0 > SETTLE_TIME)):
                break
        self._moving = False
        await f.motor_is_moving.write(0)
        await f.done_moving_to_value.write(1)
        if not arrived:
            # stopped or faulted: VAL follows RBV, as the motor record does
            if st.fault:
                names = [m.name for m in a1.AxisFault if int(m) and st.fault & int(m)]
                await ioc.report(f'{self.axis} stopped at {st.fbk:g}: {", ".join(names) or hex(st.fault)}')
            else:
                await ioc.report(f'{self.axis} stopped at {st.fbk:g}, short of the target {dial:g}')
            return f.user_readback_value.value
        return self.motor.value

    async def stop(self):
        self._stop_requested = True
        await self.ioc.call(self.ioc.hw.abort, self.axis)

    async def home(self):
        f = self.fields
        self._move_id += 1  # ends any move_to() wait in progress
        self._moving = True
        await f.done_moving_to_value.write(0)
        await f.motor_is_moving.write(1)
        try:
            await self.ioc.run_command(f'Home {self.axis}', self.ioc.hw.hp.home_axis, self.axis)
        finally:
            self._moving = False
            await f.motor_is_moving.write(0)
            await f.done_moving_to_value.write(1)
        await self.ioc.after_homing()

    async def tweak(self, sign):
        target = self.motor.value + sign * self.fields.tweak_step_size.value
        self._tweak_task = asyncio.ensure_future(self.motor.write(target))

    @motor.putter
    async def motor(self, instance, value):
        f = self.fields
        if f.set_use_switch.value == 'Set':
            # redefine the current position instead of moving
            await f.user_offset.write(value - f.dial_readback_value.value, verify_value=False)
            return value
        hlm, llm = f.user_high_limit.value, f.user_low_limit.value
        if hlm != llm and not llm <= value <= hlm:
            await f.limit_violation.write(1)
            await self.ioc.report(f'{self.axis}: {value} is outside the limits [{llm}, {hlm}]')
            return instance.value
        await _put(f.limit_violation, 0)
        if f.stop_pause_move_go.value in ('Stop', 'Pause'):
            return value
        await instance.write(value, verify_value=False)  # publish the new target right away
        return await self.move_to(value - f.user_offset.value)

    @motor.fields.stop.putter
    async def motor(fields, instance, value):
        if value:
            await fields.parent.group.stop()
        return 0

    @motor.fields.stop_pause_move_go.putter
    async def motor(fields, instance, value):
        if value in ('Stop', 'Pause'):
            await fields.parent.group.stop()
        return value

    @motor.fields.enable_control.putter
    async def motor(fields, instance, value):
        m = fields.parent.group
        await m.ioc.call(m.ioc.hw.set_axis_enabled, m.axis, value == 'Enable')
        return value

    @motor.fields.home_forward.putter
    async def motor(fields, instance, value):
        if value:
            await fields.parent.group.home()
        return 0

    @motor.fields.home_reverse.putter
    async def motor(fields, instance, value):
        if value:
            await fields.parent.group.home()
        return 0

    @motor.fields.tweak_motor_forward.putter
    async def motor(fields, instance, value):
        if value:
            await fields.parent.group.tweak(1)
        return 0

    @motor.fields.tweak_motor_reverse.putter
    async def motor(fields, instance, value):
        if value:
            await fields.parent.group.tweak(-1)
        return 0

    @motor.fields.user_offset.putter
    async def motor(fields, instance, value):
        # keep the dial target, shift VAL and RBV by the new offset
        await fields.parent.write(fields.dial_desired_value.value + value, verify_value=False)
        # the dial limits stay put; the user limits move with the offset
        await _put(fields.user_low_limit, fields.dial_low_limit.value + value)
        await _put(fields.user_high_limit, fields.dial_high_limit.value + value)
        return value

    @motor.fields.user_high_limit.putter
    async def motor(fields, instance, value):
        off = fields.user_offset.value
        await fields.parent.group.limit_changed(fields.dial_low_limit.value, value - off)
        return value

    @motor.fields.user_low_limit.putter
    async def motor(fields, instance, value):
        off = fields.user_offset.value
        await fields.parent.group.limit_changed(value - off, fields.dial_high_limit.value)
        return value

    @motor.fields.dial_high_limit.putter
    async def motor(fields, instance, value):
        await fields.parent.group.limit_changed(fields.dial_low_limit.value, value)
        return value

    @motor.fields.dial_low_limit.putter
    async def motor(fields, instance, value):
        await fields.parent.group.limit_changed(value, fields.dial_high_limit.value)
        return value


class HexapodIOC(PVGroup):
    __doc__ = __doc__

    # PV names; controller axes A, B, C are the rotations about X, Y, Z
    x = SubGroup(HexMotor, axis='X', prefix='X')
    y = SubGroup(HexMotor, axis='Y', prefix='Y')
    z = SubGroup(HexMotor, axis='Z', prefix='Z')
    rx = SubGroup(HexMotor, axis='A', prefix='RX')
    ry = SubGroup(HexMotor, axis='B', prefix='RY')
    rz = SubGroup(HexMotor, axis='C', prefix='RZ')

    enable = pvproperty(name='Enable', value='Disable', enum_strings=['Disable', 'Enable'],
                        dtype=ChannelType.ENUM, doc='Enable/disable the axes (no homing)')
    enabled_rbv = pvproperty(name='Enabled_RBV', value=0, read_only=True)
    mode_rbv = pvproperty(name='Mode_RBV', value=0, read_only=True,
                          doc='GetHexapodMode(): -10 not initialized, 0 Disable, 1 Local, 2 Global, 3 Strut')

    cor_meas_x = pvproperty(name='CoR:MeasX', value=0.0, precision=4, units='mm')
    cor_meas_y = pvproperty(name='CoR:MeasY', value=0.0, precision=4, units='mm')
    cor_meas_z = pvproperty(name='CoR:MeasZ', value=0.0, precision=4, units='mm')
    cor_frame = pvproperty(name='CoR:Frame', value='Base', enum_strings=['Base', 'Platform'],
                           dtype=ChannelType.ENUM)
    cor_apply = pvproperty(name='CoR:Apply', value=0)
    cor_reset = pvproperty(name='CoR:Reset', value=0)
    cor_x_rbv = pvproperty(name='CoR:X_RBV', value=0.0, precision=4, units='mm', read_only=True)
    cor_y_rbv = pvproperty(name='CoR:Y_RBV', value=0.0, precision=4, units='mm', read_only=True)
    cor_z_rbv = pvproperty(name='CoR:Z_RBV', value=0.0, precision=4, units='mm', read_only=True)

    stop_all = pvproperty(name='StopAll', value=0)
    ack_all = pvproperty(name='AckAll', value=0)
    clear_errors = pvproperty(name='ClearErrors', value=0)
    home_all = pvproperty(name='HomeAll', value=0)
    execute_command = pvproperty(name='ExecuteCommand', value='', dtype=ChannelType.CHAR, max_length=256,
                                 string_encoding='latin-1', report_as_string=True)
    message = pvproperty(name='Message', value='', dtype=ChannelType.CHAR, max_length=256,
                         string_encoding='latin-1', report_as_string=True, read_only=True)
    heartbeat = pvproperty(name='Heartbeat', value=0, read_only=True)

    def __init__(self, *args, host=IP, check_init=True, limits_file=LIMITS_FILE, **kwargs):
        self.hw = HexapodHW(host)
        self.check_init = check_init
        self.hexapod_mode = None
        self.soft_limits = {}
        self.limits_file = limits_file
        self.dial_limits = self.load_limits(limits_file)
        self.executor = ThreadPoolExecutor(max_workers=4)
        super().__init__(*args, **kwargs)
        self.motors = [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    async def call(self, fn, *args):
        return await asyncio.get_running_loop().run_in_executor(self.executor, fn, *args)

    @staticmethod
    def load_limits(path):
        """{PV name: (low, high)} dial limits from the JSON file; {} if it is missing or unreadable."""
        try:
            with open(path) as f:
                raw = json.load(f)
            return {name: (float(lo), float(hi)) for name, (lo, hi) in raw.items()}
        except FileNotFoundError:
            return {}
        except Exception as ex:
            print(f'WARNING: ignoring soft limits file {path}: {ex}')
            return {}

    async def save_limits(self, name, low, high):
        """Record one axis' dial limits and rewrite the file atomically. A failed save is reported, not raised."""
        self.dial_limits[name] = (low, high)
        data = {n: list(lim) for n, lim in self.dial_limits.items()}
        tmp = self.limits_file + '.tmp'
        try:
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
                f.write('\n')
            os.replace(tmp, self.limits_file)
        except Exception as ex:
            await self.report(f'Could not save soft limits to {self.limits_file}: {ex}')

    async def report(self, text):
        self.log.info(text)
        await self.message.write(text[:255], verify_value=False)

    async def slow_poll(self):
        try:
            self.hexapod_mode = await self.call(self.hw.hp.get_hexapod_mode)
            await _put(self.mode_rbv, self.hexapod_mode)
        except Exception as ex:
            self.log.warning('GetHexapodMode failed: %s', ex)
        try:
            x, y, z = await self.call(self.hw.hp.get_rotation_point)
            await _put(self.cor_x_rbv, x)
            await _put(self.cor_y_rbv, y)
            await _put(self.cor_z_rbv, z)
        except Exception as ex:
            self.log.warning('GetRotationPoint failed: %s', ex)
        try:
            self.soft_limits = await self.call(self.hw.soft_limits)
        except Exception as ex:
            self.log.warning('Reading software limits failed: %s', ex)
        await self.heartbeat.write(self.heartbeat.value + 1, verify_value=False)

    @heartbeat.startup
    async def heartbeat(self, instance, async_lib):
        await self.slow_poll()
        axes = await self.call(self.hw.read_status)
        for m in self.motors:
            speed = await self.call(self.hw.hp.get_speed, m.axis)
            await m.initialize(axes[m.axis], speed)
        last_slow = time.monotonic()
        while True:
            t0 = time.monotonic()
            try:
                axes = await self.call(self.hw.read_status)
            except Exception as ex:
                await self.report(f'Status read failed: {ex}')
                await asyncio.sleep(1.0)
                continue
            for m in self.motors:
                await m.update(axes[m.axis])
            await _put(self.enabled_rbv, int(all(axes[ax].enabled for ax in HEX_AXES)))
            if t0 - last_slow >= SLOW_POLL_PERIOD:
                last_slow = t0
                await self.slow_poll()
            await asyncio.sleep(max(0.0, POLL_PERIOD - (time.monotonic() - t0)))

    async def run_command(self, label, fn, *args):
        try:
            result = await self.call(fn, *args)
        except Exception as ex:
            await self.report(f'{label} failed: {ex}')
            raise
        await self.report(f'{label}: OK')
        return result

    @enable.putter
    async def enable(self, instance, value):
        if value == 'Enable':
            await self.run_command('Enable', self.hw.hp.enable_all_axes)
            if self.all_homed():
                await self.ensure_global_mode()
            else:
                await self.report('Enabled. Struts are not homed: press Home all; Global mode turns on after homing')
        else:
            await self.run_command('Disable', self.hw.hp.disable_all_axes)
        return value

    def all_homed(self):
        return all(m.state is not None and m.state.homed for m in self.motors)

    async def refresh_mode(self):
        self.hexapod_mode = await self.call(self.hw.hp.get_hexapod_mode)
        await _put(self.mode_rbv, self.hexapod_mode)

    async def ensure_global_mode(self):
        """Turn on Global mode (X, Y, Z, A, B, C kinematics) if it is not active.
        Refuses when a strut is not homed, because EnableGlobalMode() would then home all struts."""
        if self.hexapod_mode == GLOBAL_MODE:
            return
        if not self.all_homed():
            msg = 'Struts are not homed; press Home all before moving'
            await self.report(msg)
            raise RuntimeError(msg)
        await self.run_command('EnableGlobalMode()', self.hw.hp.set_hexapod_mode, 'Global')
        await self.refresh_mode()
        await self.sync_targets()  # positions are now in X, Y, Z, A, B, C

    async def sync_targets(self):
        """Read fresh status and set each VAL to its RBV (after homing or a mode change)."""
        axes = await self.call(self.hw.read_status)
        for m in self.motors:
            await m.update(axes[m.axis])
            await m.fields.dial_desired_value.write(axes[m.axis].fbk, verify_value=False)
            await m.motor.write(m.fields.user_readback_value.value, verify_value=False)

    async def after_homing(self):
        await self.sync_targets()
        await self.refresh_mode()
        if self.all_homed():
            await self.ensure_global_mode()

    async def apply_rotation_point(self, x, y, z):
        frame = 'base' if self.cor_frame.value == 'Base' else 'platform'
        point = await self.run_command('SetRotationPoint', self.hw.hp.set_center_of_rotation_from_measurement,
                                       x, y, z, frame)
        rbv = await self.call(self.hw.hp.get_rotation_point)
        await _put(self.cor_x_rbv, rbv[0])
        await _put(self.cor_y_rbv, rbv[1])
        await _put(self.cor_z_rbv, rbv[2])
        await self.report(f'Rotation point {tuple(round(p, 4) for p in point)} ({frame} frame), '
                          f'readback {tuple(round(p, 4) for p in rbv)}')

    @cor_apply.putter
    async def cor_apply(self, instance, value):
        if value:
            await self.apply_rotation_point(self.cor_meas_x.value, self.cor_meas_y.value, self.cor_meas_z.value)
        return 0

    @cor_reset.putter
    async def cor_reset(self, instance, value):
        if value:
            await self.apply_rotation_point(0.0, 0.0, 0.0)
        return 0

    @stop_all.putter
    async def stop_all(self, instance, value):
        if value:
            for m in self.motors:
                m._stop_requested = True
            await self.run_command('StopAll', self.hw.abort, HEX_AXES)
        return 0

    async def clear_all_errors(self):
        faults = await self.run_command('Clear errors', self.hw.hp.clear_errors)
        remaining = {ax: f'0x{f:x}' for ax, f in faults.items() if f}
        if remaining:
            await self.report(f'Clear errors: still faulted {remaining} (axis on a limit switch?)')

    @ack_all.putter
    async def ack_all(self, instance, value):
        if value:
            await self.clear_all_errors()
        return 0

    @clear_errors.putter
    async def clear_errors(self, instance, value):
        if value:
            await self.clear_all_errors()
        return 0

    @home_all.putter
    async def home_all(self, instance, value):
        if value:
            try:
                await self.run_command('HomeAll', self.hw.hp.home_all)
            finally:
                await self.after_homing()
        return 0

    @execute_command.putter
    async def execute_command(self, instance, value):
        if value:
            await self.run_command(value, self.hw.cmd.execute, value)
        return value


def main():
    parser, split_args = template_arg_parser(default_prefix=DEFAULT_PREFIX, desc=dedent(__doc__),
                                             supported_async_libs=['asyncio'])
    parser.add_argument('--host', default=IP, help='Automation1 controller host name')
    parser.add_argument('--no-init-check', action='store_true',
                        help='allow moves while GetHexapodMode() reports the hexapod is not initialized')
    parser.add_argument('--limits-file', default=LIMITS_FILE,
                        help='JSON file with the motors\' dial soft limits, {"X": [low, high], ...}')
    args = parser.parse_args()
    ioc_options, run_options = split_args(args)
    ioc = HexapodIOC(host=args.host, check_init=not args.no_init_check, limits_file=args.limits_file,
                     **ioc_options)
    run(ioc.pvdb, **run_options)


if __name__ == '__main__':
    main()
