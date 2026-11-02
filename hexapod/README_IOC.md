# Hexapod caproto IOC

`hexapod/ioc.py` is a caproto IOC for the Aerotech HEX300-230HL hexapod. It talks to the Automation1 controller (`at-hex-12id.xray.aps.anl.gov`) directly through the Python API in `hexapod/a1.py`. It does not go through the `12idhxpAT` EPICS IOC.

The six hexapod axes are served as motor records, so they can be used in step scans. Separate PVs enable and disable the hexapod and set the center of rotation.

## Running

```
python -m hexapod.ioc --list-pvs
```

| Option | Default | Meaning |
|---|---|---|
| `--prefix` | `12idhxpPY:` | PV prefix. The default differs from `12idhxpAT:` so the two IOCs cannot collide. |
| `--host` | `at-hex-12id.xray.aps.anl.gov` | Automation1 controller. |
| `--interfaces` | all | Network interfaces to serve on. Use `127.0.0.1` for local testing. |
| `--no-init-check` | off | Allow moves while the hexapod reports it is not initialized. See [Safety](#safety). |

The automation1 Python package must match the controller firmware (currently 2.13.1). After a firmware upgrade, install the matching package from the MDK:

```
C:\Users\s12idc\Anaconda3\python.exe -m pip install --upgrade automation1==2.13.1 --find-links "C:\Program Files\Aerotech\Automation1-MDK\APIs\Python" --no-index
```

With Anaconda on this PC, numpy fails to load unless the Anaconda directories are on `PATH`:

```powershell
$env:PATH = "C:\Users\s12idc\Anaconda3;C:\Users\s12idc\Anaconda3\Library\bin;C:\Users\s12idc\Anaconda3\Library\mingw-w64\bin;C:\Users\s12idc\Anaconda3\Scripts;" + $env:PATH
C:\Users\s12idc\Anaconda3\python.exe -m hexapod.ioc
```

Run only one of this IOC and `12idhxpAT` at a time for motion. Both would command the same controller.

## PVs

### Motors

| PV | Controller axis | Units |
|---|---|---|
| `X` | X | mm |
| `Y` | Y | mm |
| `Z` | Z | mm |
| `RX` | A (rotation about X) | deg |
| `RY` | B (rotation about Y) | deg |
| `RZ` | C (rotation about Z) | deg |

The IOC always runs the hexapod in Global mode, so the motors move the platform pose. Global mode needs all six struts homed. The IOC never homes on its own: if a strut is not homed, moves are refused with "press Home all".

The names differ from the `12idhxpAT` IOC, which uses `m1`…`m6` (m1 = Z, m2 = X, m3 = Y, m4 = A, m5 = B, m6 = C).

Supported motor record fields:

| Fields | Behavior |
|---|---|
| `VAL`, `RBV`, `DVAL`, `DRBV` | Target and readback. A put-callback on `VAL` completes when the move is done. |
| `DMOV`, `MOVN` | Done-moving and moving flags. |
| `STOP`, `SPMG` | Abort the move. After a stop or fault, `VAL` is set to `RBV`. |
| `VELO` | Move speed. Initialized from the controller's default axis speed. |
| `HLM`, `LLM`, `LVIO` | User limits, enforced by the IOC. Equal limits (e.g. both 0) mean no limits. |
| `OFF`, `SET` | User offset. With `SET` = Set, a put to `VAL` redefines the current position instead of moving. |
| `TWV`, `TWF`, `TWR` | Tweak. |
| `CNEN` | Enable or disable that axis. |
| `MSTA`, `HLS`, `LLS` | Status bits and limit switches from the controller axis status and faults. |

The dial position is the controller's program position (feedback for `RBV`, command for move completion). Status is polled at 10 Hz.

### Hexapod

| PV | Function |
|---|---|
| `Enable` | `Enable`: enable axes 0–11, then `EnableGlobalMode()` if all struts are homed. It never homes. `Disable`: disable axes 0–11. |
| `Enabled_RBV` | 1 when X, Y, Z, A, B, C are enabled. |
| `Mode_RBV` | `GetHexapodMode()`, the library's `HexapodMode` enum: 0 Disable, 1 Local, 2 Global, 3 Strut; −10 means not initialized. The IOC always uses **Global** mode. It turns Global on after Enable, after homing, or before a move, but only when all struts are homed. The controller starts in Disable after a reset. There is no mode selection. |
| `StopAll` | Abort X, Y, Z, A, B, C. |
| `ClearErrors`, `AckAll` | Acknowledge all axis faults and task errors. `Message` lists any axis still faulted, for example one sitting on a limit switch. |
| `HomeAll` | Home axes 0–5 one at a time in Strut mode, then switch to Global mode. |
| `<motor>.HOMF` / `.HOMR` | Home that motor's strut drive in Strut mode. Once all six are homed, switch to Global mode. In the GUI this is the Home button on each row. |
| `ExecuteCommand` | Execute one AeroScript line (up to 256 characters). |
| `Message` | Result or error of the last command. |
| `Heartbeat` | Increments once per second while the IOC is polling. |

### Center of rotation

| PV | Function |
|---|---|
| `CoR:MeasX`, `CoR:MeasY`, `CoR:MeasZ` | Center of rotation in mm, measured from the center of the top plate with the hexapod at its zero pose. |
| `CoR:Frame` | `Base` (default): send the point in bottom base coordinates. `Platform`: send the measured offset unchanged. |
| `CoR:Apply` | Write 1 to send the measured point to `SetRotationPoint()`. |
| `CoR:Reset` | Write 1 to put the rotation point back at the top plate center. |
| `CoR:X_RBV`, `CoR:Y_RBV`, `CoR:Z_RBV` | `GetRotationPoint()` readback, polled once per second. |

The top plate moves, so a point measured from it is converted to the fixed bottom base frame:

```
base = (MeasX, MeasY, MeasZ + 230.25)
```

230.25 mm is the height of the top plate mounting surface above the bottom base mounting surface at the zero pose. It comes from the HEX300-230HL geometry in `HexCfg.ini`: base pivot height 17 + pivot-to-pivot offset 196.25 + platform pivot height 17.

Example: rotate about a point 50 mm above the top plate center.

```
caput 12idhxpPY:CoR:MeasZ 50
caput 12idhxpPY:CoR:Apply 1
caget -S 12idhxpPY:Message
```

## GUI

`hexapod/gui.py` is a PyQt5 window for this IOC, laid out like the APS motor MEDM screens. It uses pyepics, so the IOC's PVs must be reachable over Channel Access.

```
python -m hexapod.gui [--prefix 12idhxpPY:]
```

- **Hexapod:** Enable and Disable buttons, the enabled state, a Mode menu with the `Mode_RBV` readback (red "Not initialized" at −10), Ack faults, and STOP ALL.
- **Motors:** one row per axis with name, low and high limit lights, readback, drive entry (press Return to move), tweak `<` / step / `>`, a Done/Moving indicator, STOP, and More. More opens the user/dial view: limits, readback, drive, offset, speed, tweak step, Use/Set calibration, torque enable and MSTA.
- **Center of Rotation:** X, Y, Z edit boxes for the position measured from the top plate center with the hexapod at home.
  - **Set** writes the boxes to `CoR:MeasX/Y/Z` and then `CoR:Apply`.
  - **Reset** sets the boxes and `CoR:Meas*` to 0, 0, 0, then writes `CoR:Reset`.
  - The Controller row shows `CoR:X/Y/Z_RBV`. The Frame menu sets `CoR:Frame`.
- **Message line:** shows the IOC's `Message`.

Colors follow MEDM: light blue for controls, blue text on grey for readbacks, and white for a disconnected PV.

## Python API

The same functions are available from `a1.py` without the IOC:

```python
from hexapod import Hexapod
hp = Hexapod()
hp.get_hexapod_mode()                                     # -10 = not initialized
hp.enable_all_axes()                                      # enable axes, then EnableGlobalMode()
hp.set_center_of_rotation_from_measurement(0, 0, 50)      # base frame by default
hp.get_rotation_point()
```

| Method | AeroScript |
|---|---|
| `set_hexapod_mode(mode)` | `DisableHexapod()`, `EnableGlobalMode()`, `EnableLocalMode()`, `EnableStrutMode()` |
| `enable_kinematics()`, `enable_work()` | `EnableGlobalMode()` |
| `disable_kinematics()` | `DisableHexapod()` |
| `get_hexapod_mode()` | `GetHexapodMode()` |
| `set_rotation_point(x, y, z)` | `SetRotationPoint(x, y, z)` |
| `get_rotation_point()` | `GetRotationPoint($x, $y, $z)` |
| `set_center_of_rotation_from_measurement(x, y, z, frame='base')` | Converts as above, then `SetRotationPoint` |

Getter values are returned through task variables `$itask[200]` and `$rtask[200..202]` on task 4. While the IOC is running, getters called from another `Hexapod()` session can collide with the IOC's once-per-second poll ("another immediate command is already executing"). Retry the call, or use the IOC's PVs instead.

## Controller notes

- **Hexapod library:** the controller auto-imports `HexCLibrary.a1lib` (C transform, state machine `HexapodCStateMachine` on task 2). Functions from the older `HexapodLibrary` do not compile: `EnableWork`, `SetBaseToWork`, `SetToolPoint`, `ActivateTool`, `EnableTool`, `ClearAllTools`, `HexapodStatus`. The vendor example `/Hexapod/User Programs/ExampleUserProgram.ascript` uses those older names. The a1.py methods `set_work`, `set_tool`, `activate_tool`, `enable_tool` and `set_current_as_zero` still call them and will fail.
- **Axis names:** axes 0–5 are named X, Y, Z, A, B, C and own the strut drives. Axes 6–11 are Xv, Yv, Zv, Av, Bv, Cv. There are no axes named ST1–ST6. The PSO/fly code in `a1.py`, `ephex.py` and `fly_test.py` now uses `X` (formerly `ST1`) as the PSO axis.
- **Configuration file:** the state machine reads only `/Hexapod/Config Files/HexCfg.ini`. It needs `[SystemSettings]`, `[HexapodSettings.1]` (model, task, `ax_X`…`ax_C`, correction file) and the model section including `ScrewLead`. The previous file lacked the first two, which left the hexapod uninitialized. It was replaced on 2026-10-30 with `controller_config/HexCfg.ini.proposed`; backups of the old files are in `controller_config/`.
- **License and firmware:** after the INI fix, the state machine failed with "HX1 option not present" until the controller was upgraded to firmware 2.13.1. On 2.13.1, task 2 runs (`ProgramRunning`), `GetHexapodMode()` returns 0 and `GetHexapodState()` answers.
- **Task state values:** 3 = `ProgramReady` (loaded, not running), 4 = `ProgramRunning`, 8 = `Error`.
- **Parameters reset by the 2.13.1 upgrade:** homing (HomeType 1 → 4, HomeOffset ≈ −12 → 0, HomeSetup, HomeSpeed), software limits (±13.1 → 0, i.e. off), encoder sine/cosine correction, PositionErrorThreshold, VelocityCommandThreshold, ramp and average-current settings. With HomeType 4 ("at current position"), Home does not move; it only zeroes the position where the strut is. `controller_config/restore_params.py` restores these from a saved MCD and leaves servo tuning alone. Run it without `--apply` first to see the changes.
- **Checking a call without running it:** wrap it in a branch that never executes. It is compiled, so an unknown name raises an error, but nothing runs:

  ```python
  hp.controller.runtime.commands.execute('if (1 == 0)\nEnableGlobalMode()\nend')
  ```

## Safety

When the hexapod is not initialized (`Mode_RBV` = −10), the hexapod transform is not active. A move on X, Y, Z, A, B or C then moves a single strut directly, not the platform.

By default the IOC refuses moves in that state. The put fails with an error and `Message` shows the reason. Initialize the hexapod (currently done from the vendor software) before moving. Use `--no-init-check` only if you understand that motor moves are strut moves.

## Status

Tested on 2026-10-30 with the IOC bound to localhost:

- All motor fields and hexapod PVs read correctly. `Mode_RBV` read −10.
- A put to the Z motor (then named `m1`) was refused because the hexapod is not initialized. Nothing moved.
- `STOP` on the Z motor ran the abort hook and reset to 0.

Not tested yet: motor moves, `Enable`, `Mode`, `CoR:Apply`/`CoR:Reset` and `HomeAll`. The hexapod reported "not initialized" during the first tests. It initializes since the firmware 2.13.1 upgrade (`Mode_RBV` = 0), so these can now be tried.

Open questions:

- **Rotation-point frame:** it is not known whether `SetRotationPoint` expects base or platform coordinates. After initializing, set a test point, compare `CoR:*_RBV`, and do a small rotation to confirm the pivot before relying on `CoR:Apply`. Switch `CoR:Frame` to `Platform` if base coordinates turn out to be wrong.
- **Mode values:** the meaning of `GetHexapodMode()` values other than −10 is not known.
