#!/usr/bin/env python3
"""
PyQt GUI for the hexapod caproto IOC (hexapod/ioc.py), laid out like the APS
motor MEDM screens.

Run:
    python -m hexapod.gui [--prefix 12idhxpPY:]
"""
import argparse
import os
import sys

# pyepics uses PYTHONIOENCODING as a codec name, which fails for values like "utf-8:surrogateescape"
os.environ.setdefault('PYEPICS_ENCODING', 'utf-8')
import epics
from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QComboBox, QDialog, QFrame, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QVBoxLayout, QWidget)

DEFAULT_PREFIX = '12idhxpPY:'
MOTORS = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
PREC = 4

# MEDM color map
BG = '#c8c8c8'
ENTRY_BG = '#73dfff'
READBACK_FG = '#0a00b8'
MONITOR_BG = '#dadada'
GREEN = '#00d800'
YELLOW = '#fbf34a'
RED = '#fd0000'
WHITE = '#ffffff'


class Channel(QObject):
    """pyepics PV whose monitor and connection callbacks arrive as Qt signals on the GUI thread."""
    value = pyqtSignal(object, str)
    connection = pyqtSignal(bool)

    def __init__(self, name):
        super().__init__()
        self.name = name
        self.pv = epics.PV(name, callback=self._on_value, connection_callback=self._on_connection,
                           auto_monitor=True)

    def _on_value(self, value=None, char_value=None, **kw):
        self.value.emit(value, char_value if char_value is not None else '')

    def _on_connection(self, conn=False, **kw):
        self.connection.emit(bool(conn))

    def subscribe(self, on_value, on_connection=None):
        """Connect callbacks and replay the current state, since the PV may already be connected."""
        self.value.connect(on_value)
        if on_connection is not None:
            self.connection.connect(on_connection)
            on_connection(self.pv.connected)
        if self.pv.connected and self.pv.value is not None:
            on_value(self.pv.value, self.pv.char_value or '')

    def put(self, value, wait=False):
        if self.pv.connected:
            self.pv.put(value, wait=wait, timeout=5.0)


class Channels:
    def __init__(self):
        self._channels = {}

    def __call__(self, name):
        if name not in self._channels:
            self._channels[name] = Channel(name)
        return self._channels[name]


def _style(widget, bg, fg='black', bold=False, border=True):
    widget.setStyleSheet(f'background-color: {bg}; color: {fg};'
                         f'{" font-weight: bold;" if bold else ""}'
                         f'{" border: 1px solid #808080;" if border else ""}')


def _fmt(value, char_value, prec):
    if isinstance(value, float):
        return f'{value:.{prec}f}'
    return char_value


def readback(ch, prec=PREC, width=None, big=False):
    """MEDM text monitor: blue text on grey, white when disconnected."""
    label = QLabel('')
    label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    if width:
        label.setMinimumWidth(width)
    if big:
        f = label.font()
        f.setPointSize(f.pointSize() + 3)
        f.setBold(True)
        label.setFont(f)
    _style(label, MONITOR_BG, READBACK_FG)

    def connection(connected):
        _style(label, MONITOR_BG if connected else WHITE, READBACK_FG)
        if not connected:
            label.setText('')

    ch.subscribe(lambda v, s: label.setText(_fmt(v, s, prec)), connection)
    return label


def entry(ch, prec=PREC, width=90):
    """MEDM text entry: puts on Return, follows the PV while not being edited."""
    edit = QLineEdit()
    edit.setAlignment(Qt.AlignRight)
    edit.setMinimumWidth(width)
    _style(edit, ENTRY_BG)

    def update(v, s):
        if not (edit.hasFocus() and edit.isModified()):  # don't overwrite what the user is typing
            edit.setText(_fmt(v, s, prec))

    def put():
        text = edit.text().strip()
        try:
            ch.put(float(text))
        except ValueError:
            ch.put(text)
        edit.clearFocus()

    ch.subscribe(update, lambda c: _style(edit, ENTRY_BG if c else WHITE))
    edit.returnPressed.connect(put)
    return edit


def indicator(ch, states, width=70):
    """Label whose text and color follow the PV value. states: {value: (text, color)}."""
    label = QLabel('')
    label.setAlignment(Qt.AlignCenter)
    label.setMinimumWidth(width)
    _style(label, BG)

    def update(v, s):
        text, color = states.get(v, (s, BG))
        label.setText(text)
        _style(label, color, bold=True)

    def connection(connected):
        if not connected:
            label.setText('')
            _style(label, WHITE)

    ch.subscribe(update, connection)
    return label


def put_button(text, ch, value, bg=ENTRY_BG, fg='black', width=None):
    button = QPushButton(text)
    _style(button, bg, fg, bold=bg == RED)
    if width:
        button.setFixedWidth(width)
    button.clicked.connect(lambda: ch.put(value))
    return button


def confirm_button(text, ch, value, question, width=None):
    """Button that asks for confirmation before the put, for commands that move hardware."""
    button = QPushButton(text)
    _style(button, ENTRY_BG)
    if width:
        button.setFixedWidth(width)

    def clicked():
        if QMessageBox.question(button, text, question) == QMessageBox.Yes:
            ch.put(value)

    button.clicked.connect(clicked)
    return button


def enum_menu(ch):
    """MEDM menu for an enum PV."""
    combo = QComboBox()
    combo.setMinimumWidth(80)
    _style(combo, ENTRY_BG)
    state = {'filling': False}

    def update(v, s):
        state['filling'] = True
        if combo.count() == 0 and ch.pv.enum_strs:
            combo.addItems(list(ch.pv.enum_strs))
        if isinstance(v, int) and 0 <= v < combo.count():
            combo.setCurrentIndex(v)
        state['filling'] = False

    combo.currentIndexChanged.connect(lambda i: None if state['filling'] else ch.put(i))
    ch.subscribe(update)
    return combo


class MotorDetail(QDialog):
    """User/dial view of one motor record, like motorx_more / motorx_setup."""

    def __init__(self, channels, pv, parent=None):
        super().__init__(parent)
        self.setWindowTitle(pv)
        f = lambda field: channels(f'{pv}.{field}')
        grid = QGridLayout(self)
        title = readback(f('DESC'), width=200)
        title.setAlignment(Qt.AlignCenter)
        grid.addWidget(title, 0, 0, 1, 3)
        grid.addWidget(QLabel('User'), 1, 1, Qt.AlignCenter)
        grid.addWidget(QLabel('Dial'), 1, 2, Qt.AlignCenter)
        rows = [
            ('Hi limit', entry(f('HLM')), None),
            ('Readback', readback(f('RBV'), big=True), readback(f('DRBV'), big=True)),
            ('Drive', entry(f('VAL')), readback(f('DVAL'))),
            ('Lo limit', entry(f('LLM')), None),
        ]
        for r, (name, user, dial) in enumerate(rows, start=2):
            grid.addWidget(QLabel(name), r, 0)
            grid.addWidget(user, r, 1)
            if dial is not None:
                grid.addWidget(dial, r, 2)
        r = len(rows) + 2
        settings = [('Offset (OFF)', entry(f('OFF'))), ('Speed (VELO)', entry(f('VELO'))),
                    ('Tweak (TWV)', entry(f('TWV'))), ('Units', readback(f('EGU'))),
                    ('Calibration', enum_menu(f('SET'))), ('Torque (CNEN)', enum_menu(f('CNEN'))),
                    ('Status (MSTA)', readback(f('MSTA'), prec=0))]
        for i, (name, widget) in enumerate(settings):
            grid.addWidget(QLabel(name), r + i, 0)
            grid.addWidget(widget, r + i, 1)
        note = QLabel('User = Dial + OFF. Calibration "Set": a new Drive value\n'
                      'redefines the user position without moving.')
        note.setStyleSheet('color: #404040;')
        grid.addWidget(note, r + len(settings), 0, 1, 3)
        grid.addWidget(put_button('STOP', f('STOP'), 1, bg=RED, fg=WHITE), r + len(settings) + 1, 0, 1, 3)


class HexapodWindow(QWidget):
    def __init__(self, prefix):
        super().__init__()
        self.prefix = prefix
        self.channels = Channels()
        self.details = {}
        self.setWindowTitle(f'Hexapod ({prefix})')
        self.setStyleSheet(f'QWidget {{ background-color: {BG}; }}'
                           f'QGroupBox {{ font-weight: bold; border: 1px solid #808080; margin-top: 8px; }}'
                           f'QGroupBox::title {{ subcontrol-origin: margin; left: 6px; }}')
        layout = QVBoxLayout(self)
        title = QLabel(f'Hexapod   {prefix}')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(f'background-color: {READBACK_FG}; color: white; font-size: 14pt;'
                            'font-weight: bold; padding: 4px;')
        layout.addWidget(title)
        layout.addWidget(self.hexapod_box())
        layout.addWidget(self.motor_box())
        layout.addWidget(self.cor_box())
        message = readback(self.ch('Message'))
        message.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        layout.addWidget(message)

    def ch(self, suffix):
        return self.channels(self.prefix + suffix)

    def hexapod_box(self):
        box = QGroupBox('Hexapod')
        row = QHBoxLayout(box)
        row.addWidget(put_button('Enable', self.ch('Enable'), 1, width=80))
        row.addWidget(put_button('Disable', self.ch('Enable'), 0, width=80))
        row.addWidget(indicator(self.ch('Enabled_RBV'), {0: ('Disabled', YELLOW), 1: ('Enabled', GREEN)}))
        row.addSpacing(12)
        # Always Strut mode (set by the IOC). GetHexapodMode(): Disable 0, Local 1, Global 2, Strut 3
        row.addWidget(QLabel('Mode'))
        row.addWidget(indicator(self.ch('Mode_RBV'), {-10: ('Not initialized', RED), 0: ('Disabled', YELLOW),
                                                       1: ('Local', GREEN), 2: ('Global', GREEN),
                                                       3: ('Strut', GREEN)}, width=100))
        row.addSpacing(12)
        row.addWidget(put_button('Clear errors', self.ch('ClearErrors'), 1, width=90))
        row.addWidget(confirm_button('Home all', self.ch('HomeAll'), 1,
                                     'Home all six struts, one at a time?', width=90))
        row.addStretch()
        stop = put_button('STOP ALL', self.ch('StopAll'), 1, bg=RED, fg=WHITE, width=110)
        stop.setMinimumHeight(32)
        row.addWidget(stop)
        return box

    def motor_box(self):
        box = QGroupBox('Motors')
        grid = QGridLayout(box)
        headers = ['', 'Name', 'Lo', 'Readback', 'Hi', 'Drive', '', 'Tweak', '', 'Status', '', '', '']
        for col, text in enumerate(headers):
            if text:
                grid.addWidget(QLabel(text), 0, col, Qt.AlignCenter)
        limit = {0: ('', BG), 1: ('LIM', RED)}
        for r, name in enumerate(MOTORS, start=1):
            pv = self.prefix + name
            f = lambda field, pv=pv: self.channels(f'{pv}.{field}' if field else pv)
            label = QLabel(name)
            label.setStyleSheet('font-weight: bold;')
            grid.addWidget(label, r, 0)
            grid.addWidget(readback(f('DESC'), width=110), r, 1)
            grid.addWidget(indicator(f('LLS'), limit, width=34), r, 2)
            rbv = readback(f('RBV'), width=120, big=True)
            grid.addWidget(rbv, r, 3)
            grid.addWidget(indicator(f('HLS'), limit, width=34), r, 4)
            grid.addWidget(entry(f(''), width=100), r, 5)
            grid.addWidget(put_button('<', f('TWR'), 1, width=28), r, 6)
            grid.addWidget(entry(f('TWV'), width=60), r, 7)
            grid.addWidget(put_button('>', f('TWF'), 1, width=28), r, 8)
            grid.addWidget(indicator(f('DMOV'), {1: ('Done', GREEN), 0: ('Moving', YELLOW)}), r, 9)
            grid.addWidget(put_button('STOP', f('STOP'), 1, bg=RED, fg=WHITE, width=60), r, 10)
            grid.addWidget(confirm_button('Home', f('HOMF'), 1,
                                          f'Home the {name} strut drive?', width=50), r, 11)
            more = QPushButton('More')
            _style(more, BG)
            more.clicked.connect(lambda _, pv=pv: self.show_detail(pv))
            grid.addWidget(more, r, 12)
        return box

    def show_detail(self, pv):
        if pv not in self.details:
            self.details[pv] = MotorDetail(self.channels, pv, self)
        self.details[pv].show()
        self.details[pv].raise_()

    def cor_box(self):
        box = QGroupBox('Center of Rotation (mm from the top plate center, hexapod at home)')
        grid = QGridLayout(box)
        self.cor_edits = {}
        for col, axis in enumerate('XYZ'):
            grid.addWidget(QLabel(axis), 0, 1 + col, Qt.AlignCenter)
            edit = QLineEdit('0.0')
            edit.setAlignment(Qt.AlignRight)
            _style(edit, ENTRY_BG)
            meas = self.ch(f'CoR:Meas{axis}')
            # show the last value sent until the user edits the box
            meas.subscribe(lambda v, s, edit=edit: None if edit.hasFocus() or edit.isModified()
                           else edit.setText(f'{v:.{PREC}f}'))
            self.cor_edits[axis] = edit
            grid.addWidget(edit, 1, 1 + col)
            grid.addWidget(readback(self.ch(f'CoR:{axis}_RBV')), 2, 1 + col)
        grid.addWidget(QLabel('Position'), 1, 0)
        grid.addWidget(QLabel('Controller'), 2, 0)
        set_button = QPushButton('Set')
        reset_button = QPushButton('Reset')
        for b in (set_button, reset_button):
            _style(b, ENTRY_BG, bold=True)
            b.setMinimumWidth(80)
        set_button.clicked.connect(self.cor_set)
        reset_button.clicked.connect(self.cor_reset)
        grid.addWidget(set_button, 1, 4)
        grid.addWidget(reset_button, 2, 4)
        grid.addWidget(QLabel('Frame'), 0, 5)
        grid.addWidget(enum_menu(self.ch('CoR:Frame')), 1, 5)
        return box

    def cor_set(self):
        try:
            values = {axis: float(edit.text()) for axis, edit in self.cor_edits.items()}
        except ValueError:
            return
        for axis, value in values.items():
            self.ch(f'CoR:Meas{axis}').put(value, wait=True)
            self.cor_edits[axis].setModified(False)
        self.ch('CoR:Apply').put(1)

    def cor_reset(self):
        for axis, edit in self.cor_edits.items():
            edit.setText(f'{0:.{PREC}f}')
            edit.setModified(False)
            self.ch(f'CoR:Meas{axis}').put(0.0, wait=True)
        self.ch('CoR:Reset').put(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--prefix', default=DEFAULT_PREFIX, help='IOC PV prefix')
    args = parser.parse_args()
    app = QApplication(sys.argv)
    app.setFont(QFont('Arial', 9))
    window = HexapodWindow(args.prefix)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
