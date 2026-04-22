"""
HoldMyCoffee Dashboard
Real-time BLE monitoring and control dashboard for the stabilization platform.
Qt6 + PyQtGraph + OpenGL 3D visualization.
"""

import sys
import asyncio
import collections
import logging
from functools import partial

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QGroupBox,
    QFrame, QSplitter, QStatusBar, QSizePolicy, QListWidget,
    QDialog, QDialogButtonBox, QProgressBar,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread, pyqtSlot, QPropertyAnimation, QEasingCurve, QRect
from PyQt6.QtGui import QFont, QColor, QPalette, QIcon, QPainter, QPen, QBrush, QLinearGradient, QRadialGradient

from serial_connection import SerialWorker, PortSelectDialog
from ble_connection import TelemetryData, CONTROLLER_NAMES
from platform_3d import create_platform_view

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

TARGET_PITCH    = -0.2
TARGET_ROLL     = -175.44
TARGET_DISTANCE = 125.0

HISTORY_LEN = 500

DARK_BG       = "#0d0f14"
PANEL_BG      = "#12151c"
CARD_BG       = "#181c26"
CARD_BORDER   = "#252a38"
ACCENT_BLUE   = "#3d8ef5"
ACCENT_AMBER  = "#f5a623"
ACCENT_GREEN  = "#27c98a"
ACCENT_RED    = "#f55555"
ACCENT_PURPLE = "#a855f7"
ACCENT_ORANGE = "#fb923c"
TEXT_PRIMARY  = "#e8eaf0"
TEXT_MUTED    = "#5a6070"
TEXT_DIM      = "#383e50"
PLOT_BG_COLOR = "#0d0f14"

COLORS = {
    "pitch":   ACCENT_BLUE,
    "roll":    ACCENT_ORANGE,
    "height":  ACCENT_GREEN,
    "u_pitch": ACCENT_PURPLE,
    "u_roll":  ACCENT_AMBER,
}

DARK_STYLE = f"""
QMainWindow, QWidget {{
    background-color: {DARK_BG};
    color: {TEXT_PRIMARY};
    font-family: 'JetBrains Mono', 'Consolas', 'Courier New', monospace;
}}
QGroupBox {{
    border: 1px solid {CARD_BORDER};
    border-radius: 10px;
    margin-top: 14px;
    padding-top: 20px;
    font-weight: 600;
    font-size: 10px;
    letter-spacing: 1.5px;
    color: {TEXT_MUTED};
    text-transform: uppercase;
    background-color: {PANEL_BG};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 8px;
    background-color: {PANEL_BG};
}}
QPushButton {{
    background-color: #1a1e2a;
    border: 1px solid #2a2e3e;
    border-radius: 6px;
    padding: 8px 16px;
    font-size: 11px;
    font-family: 'JetBrains Mono', 'Consolas', monospace;
    letter-spacing: 0.5px;
    color: {TEXT_PRIMARY};
    min-height: 30px;
}}
QPushButton:hover {{
    background-color: #202535;
    border-color: #3a4050;
    color: white;
}}
QPushButton:pressed {{
    background-color: #252b3a;
}}
QPushButton:checked {{
    background-color: #1a2d54;
    border-color: {ACCENT_BLUE};
    color: {ACCENT_BLUE};
}}
QPushButton:disabled {{
    background-color: #111318;
    color: {TEXT_DIM};
    border-color: {TEXT_DIM};
}}
QPushButton#connectBtn {{
    background-color: #0d2a1a;
    border: 1px solid #1a5a32;
    color: {ACCENT_GREEN};
    font-weight: 700;
    letter-spacing: 1px;
}}
QPushButton#connectBtn:hover {{
    background-color: #122e1e;
    border-color: {ACCENT_GREEN};
}}
QPushButton#disconnectBtn {{
    background-color: #2a0d0d;
    border: 1px solid #5a1a1a;
    color: {ACCENT_RED};
}}
QPushButton#disconnectBtn:hover {{
    background-color: #2e1212;
    border-color: {ACCENT_RED};
}}
QStatusBar {{
    background-color: #080a0e;
    color: {TEXT_MUTED};
    font-size: 10px;
    letter-spacing: 0.5px;
    border-top: 1px solid {CARD_BORDER};
}}
QFrame#separator {{
    background-color: {CARD_BORDER};
    max-height: 1px;
}}
QScrollBar:vertical {{
    background: {DARK_BG};
    width: 6px;
    border-radius: 3px;
}}
QScrollBar::handle:vertical {{
    background: #2a2e3e;
    border-radius: 3px;
    min-height: 30px;
}}
"""


class DeviationBar(QWidget):
    def __init__(self, color: str, parent=None):
        super().__init__(parent)
        self.setFixedHeight(4)
        self._color = QColor(color)
        self._fraction = 0.0

    def set_fraction(self, v: float):
        self._fraction = max(0.0, min(1.0, v))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = self.rect()
        p.fillRect(r, QColor(30, 34, 46))
        bar_w = int(r.width() * self._fraction)
        if bar_w > 0:
            grad = QLinearGradient(0, 0, r.width(), 0)
            grad.setColorAt(0.0, self._color.darker(180))
            grad.setColorAt(1.0, self._color)
            p.fillRect(0, 0, bar_w, r.height(), QBrush(grad))
        p.end()


class MetricCard(QFrame):
    def __init__(self, label: str, unit: str = "°", color: str = ACCENT_BLUE,
                 target: float = None, target_range: float = None, parent=None):
        super().__init__(parent)
        self._color = color
        self._target = target
        self._target_range = target_range or 10.0
        self._unit = unit

        self.setMinimumHeight(110)
        self.setStyleSheet(f"""
            MetricCard {{
                background-color: {CARD_BG};
                border: 1px solid {CARD_BORDER};
                border-radius: 10px;
            }}
        """)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 12, 14, 10)
        outer.setSpacing(0)

        top_row = QHBoxLayout()
        top_row.setSpacing(0)

        self._label = QLabel(label)
        self._label.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 2px; font-weight: 600;"
        )
        top_row.addWidget(self._label)
        top_row.addStretch()

        self._dot = QLabel("●")
        self._dot.setStyleSheet(f"color: {TEXT_DIM}; font-size: 7px;")
        top_row.addWidget(self._dot)
        outer.addLayout(top_row)

        outer.addSpacing(6)

        value_row = QHBoxLayout()
        value_row.setSpacing(4)
        value_row.setAlignment(Qt.AlignmentFlag.AlignBaseline)

        self._value = QLabel("—")
        self._value.setStyleSheet(
            f"color: {color}; font-size: 28px; font-weight: 700; letter-spacing: -0.5px;"
        )
        value_row.addWidget(self._value)

        self._unit_lbl = QLabel(unit)
        self._unit_lbl.setStyleSheet(
            f"color: {color}; font-size: 12px; font-weight: 400; padding-bottom: 4px;"
        )
        value_row.addWidget(self._unit_lbl)
        value_row.addStretch()
        outer.addLayout(value_row)

        outer.addSpacing(6)

        self._bar = DeviationBar(color, self)
        outer.addWidget(self._bar)

        outer.addSpacing(5)

        target_row = QHBoxLayout()

        self._target_lbl = QLabel("")
        self._target_lbl.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 0.5px;")
        target_row.addWidget(self._target_lbl)
        target_row.addStretch()

        self._delta_lbl = QLabel("")
        self._delta_lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 9px;")
        target_row.addWidget(self._delta_lbl)

        outer.addLayout(target_row)

        if target is not None:
            self._target_lbl.setText(f"TARGET  {target:{'+.2f' if target != int(target) else '+.0f'}}{unit}")

    def set_value(self, val: float, fmt: str = ".1f"):
        self._value.setText(f"{val:{fmt}}")
        self._dot.setStyleSheet(f"color: {self._color}; font-size: 7px;")

        if self._target is not None:
            delta = val - self._target
            frac = 1.0 - min(abs(delta) / self._target_range, 1.0)
            self._bar.set_fraction(frac)
            sign = "+" if delta >= 0 else ""
            self._delta_lbl.setStyleSheet(
                f"color: {'#f55555' if abs(delta) > self._target_range * 0.5 else TEXT_MUTED}; font-size: 9px;"
            )
            self._delta_lbl.setText(f"Δ {sign}{delta:{fmt}}{self._unit}")
        else:
            self._bar.set_fraction(1.0)

    def set_target(self, val: float | None, fmt: str = ".2f"):
        self._target = val
        if val is not None:
            self._target_lbl.setText(f"TARGET  {val:+{fmt}}{self._unit}")
        else:
            self._target_lbl.setText("")


class PulseIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(10, 10)
        self._connected = False
        self._opacity = 1.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._phase = 0.0

    def set_connected(self, v: bool):
        self._connected = v
        if v:
            self._timer.start(50)
        else:
            self._timer.stop()
            self._opacity = 1.0
            self.update()

    def _tick(self):
        import math
        self._phase += 0.15
        self._opacity = 0.4 + 0.6 * (0.5 + 0.5 * math.sin(self._phase))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor(ACCENT_GREEN if self._connected else ACCENT_RED)
        color.setAlphaF(self._opacity)
        p.setBrush(QBrush(color))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(1, 1, 8, 8)
        p.end()


class ScanDialog(QDialog):
    device_selected = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("BLE Device Scanner")
        self.setMinimumSize(420, 320)
        self.setStyleSheet(DARK_STYLE)
        self.devices = []

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(16, 16, 16, 16)

        self.status_label = QLabel("Click 'Scan' to find BLE devices...")
        self.status_label.setStyleSheet(f"color: {TEXT_MUTED}; font-size: 11px;")
        layout.addWidget(self.status_label)

        self.device_list = QListWidget()
        self.device_list.setStyleSheet(f"""
            QListWidget {{
                background-color: {CARD_BG};
                border: 1px solid {CARD_BORDER};
                border-radius: 8px;
                color: {TEXT_PRIMARY};
                font-size: 11px;
                padding: 4px;
            }}
            QListWidget::item:selected {{
                background-color: #1a2d54;
                color: {ACCENT_BLUE};
                border-radius: 4px;
            }}
            QListWidget::item:hover {{
                background-color: #1a1e2a;
                border-radius: 4px;
            }}
        """)
        layout.addWidget(self.device_list)

        btn_layout = QHBoxLayout()
        self.scan_btn = QPushButton("⟳  SCAN")
        self.scan_btn.clicked.connect(self._on_scan)
        btn_layout.addWidget(self.scan_btn)

        self.buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        self.buttons.accepted.connect(self._on_accept)
        self.buttons.rejected.connect(self.reject)
        btn_layout.addWidget(self.buttons)
        layout.addLayout(btn_layout)

        self._worker = None

    def set_worker(self, worker):
        self._worker = worker
        worker.scan_finished.connect(self._on_scan_result)

    def _on_scan(self):
        self.scan_btn.setEnabled(False)
        self.status_label.setText("Scanning for BLE devices...")
        self.device_list.clear()
        if self._worker:
            QTimer.singleShot(0, self._worker.scan)

    def _on_scan_result(self, devices):
        self.devices = devices
        self.device_list.clear()
        for d in devices:
            name = d.name or "Unknown"
            address = getattr(d, "address", "?")
            rssi_raw = getattr(d, "rssi", None)
            rssi_text = f"{rssi_raw} dBm" if rssi_raw is not None else "n/a"
            label = f"{name}  [{address}]  RSSI: {rssi_text}"
            if "HM" in name.upper() or "AT-09" in name.upper() or "HMSOFT" in name.upper():
                label = "★  " + label
            self.device_list.addItem(label)
        self.scan_btn.setEnabled(True)
        self.status_label.setText(
            f"Found {len(devices)} device(s)."
        )

    def _on_accept(self):
        idx = self.device_list.currentRow()
        if 0 <= idx < len(self.devices):
            self.device_selected.emit(self.devices[idx])
            self.accept()


class BLEWorker(QObject):
    telemetry_received  = pyqtSignal(object)
    connection_changed  = pyqtSignal(bool)
    scan_finished       = pyqtSignal(list)
    error               = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        from ble_connection import BLEConnection
        self.ble = BLEConnection()
        self._loop = None

    @pyqtSlot()
    def init_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self.ble.set_callbacks(
            on_telemetry=lambda td: self.telemetry_received.emit(td),
            on_connection_changed=lambda c: self.connection_changed.emit(c),
        )

    @pyqtSlot()
    def scan(self):
        try:
            devices = self._loop.run_until_complete(self.ble.scan(timeout=5.0))
            self.scan_finished.emit(devices)
        except Exception as e:
            self.error.emit(f"Scan failed: {e}")
            self.scan_finished.emit([])

    @pyqtSlot(object)
    def connect_device(self, device):
        try:
            self._loop.run_until_complete(self.ble.connect(device))
        except Exception as e:
            self.error.emit(f"Connect failed: {e}")

    @pyqtSlot()
    def disconnect_device(self):
        try:
            self._loop.run_until_complete(self.ble.disconnect())
        except Exception as e:
            self.error.emit(f"Disconnect failed: {e}")

    @pyqtSlot(int)
    def send_mode(self, mode: int):
        try:
            self._loop.run_until_complete(self.ble.send_mode(mode))
        except Exception as e:
            self.error.emit(f"Send failed: {e}")


class Dashboard(QMainWindow):
    _sig_connect    = pyqtSignal(object)
    _sig_disconnect = pyqtSignal()
    _sig_send_mode  = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("HoldMyCoffee  ·  Stabilization Dashboard")
        self.resize(1520, 900)

        self.t_buf       = collections.deque(maxlen=HISTORY_LEN)
        self.pitch_buf   = collections.deque(maxlen=HISTORY_LEN)
        self.roll_buf    = collections.deque(maxlen=HISTORY_LEN)
        self.u_pitch_buf = collections.deque(maxlen=HISTORY_LEN)
        self.u_roll_buf  = collections.deque(maxlen=HISTORY_LEN)
        self.height_buf  = collections.deque(maxlen=HISTORY_LEN)
        self.t0          = None
        self.current_mode = 1

        self._setup_worker()
        self._build_ui()
        self.setStyleSheet(DARK_STYLE)

        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(33)

    def _setup_worker(self):
        self.ble_thread = QThread()
        self.ble_worker = SerialWorker()
        self.ble_worker.moveToThread(self.ble_thread)
        self.ble_thread.started.connect(self.ble_worker.init_loop)

        self.ble_worker.telemetry_received.connect(self._on_telemetry)
        self.ble_worker.connection_changed.connect(self._on_connection_changed)
        self.ble_worker.error.connect(self._on_ble_error)

        self._sig_connect.connect(self.ble_worker.connect_device)
        self._sig_disconnect.connect(self.ble_worker.disconnect_device)
        self._sig_send_mode.connect(self.ble_worker.send_mode)

        self.ble_thread.start()

    def _make_plot(self, title: str, y_label: str, y_unit: str) -> pg.PlotWidget:
        pw = pg.PlotWidget()
        pw.setBackground(PLOT_BG_COLOR)
        pw.setLabel("left",   y_label, y_unit,
                    **{"color": TEXT_MUTED, "font-size": "10px"})
        pw.setLabel("bottom", "Time", "s",
                    **{"color": TEXT_MUTED, "font-size": "10px"})
        pw.showGrid(x=True, y=True, alpha=0.08)
        pw.getPlotItem().setTitle(
            f'<span style="color:{TEXT_MUTED};font-size:10px;letter-spacing:2px;">{title}</span>'
        )
        pw.getAxis("left").setPen(pg.mkPen(CARD_BORDER))
        pw.getAxis("bottom").setPen(pg.mkPen(CARD_BORDER))
        pw.getAxis("left").setTextPen(pg.mkPen(TEXT_MUTED))
        pw.getAxis("bottom").setTextPen(pg.mkPen(TEXT_MUTED))
        pw.setStyleSheet(f"border: 1px solid {CARD_BORDER}; border-radius: 10px;")
        return pw

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        left = QVBoxLayout()
        left.setSpacing(10)

        conn_group = QGroupBox("CONNECTION")
        conn_lay = QVBoxLayout(conn_group)
        conn_lay.setSpacing(8)
        conn_lay.setContentsMargins(12, 8, 12, 12)

        status_row = QHBoxLayout()
        self._pulse = PulseIndicator()
        status_row.addWidget(self._pulse)
        self.conn_status = QLabel("DISCONNECTED")
        self.conn_status.setStyleSheet(
            f"color: {ACCENT_RED}; font-size: 10px; letter-spacing: 1.5px; font-weight: 600;"
        )
        status_row.addWidget(self.conn_status)
        status_row.addStretch()
        conn_lay.addLayout(status_row)

        self.connect_btn = QPushButton("CONNECT")
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.clicked.connect(self._on_connect_click)
        conn_lay.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("DISCONNECT")
        self.disconnect_btn.setObjectName("disconnectBtn")
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self._on_disconnect_click)
        conn_lay.addWidget(self.disconnect_btn)

        left.addWidget(conn_group)

        ctrl_group = QGroupBox("CONTROLLER")
        ctrl_lay = QVBoxLayout(ctrl_group)
        ctrl_lay.setSpacing(6)
        ctrl_lay.setContentsMargins(12, 8, 12, 12)
        self.mode_buttons = {}
        for mode_id, name in CONTROLLER_NAMES.items():
            btn = QPushButton(name.upper())
            btn.setCheckable(True)
            btn.clicked.connect(partial(self._on_mode_select, mode_id))
            ctrl_lay.addWidget(btn)
            self.mode_buttons[mode_id] = btn
        self.mode_buttons[1].setChecked(True)
        left.addWidget(ctrl_group)

        view3d_group = QGroupBox("3D PLATFORM")
        view3d_lay = QVBoxLayout(view3d_group)
        view3d_lay.setContentsMargins(8, 8, 8, 8)
        self.platform_view = create_platform_view()
        view3d_lay.addWidget(self.platform_view)
        left.addWidget(view3d_group, stretch=1)

        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(240)
        root.addWidget(left_w)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet(f"color: {CARD_BORDER};")
        root.addWidget(sep)

        right = QVBoxLayout()
        right.setSpacing(10)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(8)

        self.card_pitch = MetricCard(
            "PITCH", "°", COLORS["pitch"],
            target=TARGET_PITCH, target_range=5.0
        )
        self.card_roll = MetricCard(
            "ROLL", "°", COLORS["roll"],
            target=TARGET_ROLL, target_range=5.0
        )
        self.card_height = MetricCard(
            "DISTANCE", " mm", COLORS["height"],
            target=TARGET_DISTANCE, target_range=20.0
        )
        self.card_ctrl_pitch = MetricCard(
            "U PITCH", "°", COLORS["u_pitch"]
        )
        self.card_ctrl_roll = MetricCard(
            "U ROLL", "°", COLORS["u_roll"]
        )
        self.card_mode = MetricCard("CONTROLLER", "", "#e8eaf0")

        for card in [self.card_pitch, self.card_roll, self.card_height,
                     self.card_ctrl_pitch, self.card_ctrl_roll, self.card_mode]:
            cards_row.addWidget(card)
        right.addLayout(cards_row)

        pg.setConfigOptions(antialias=True, background=PLOT_BG_COLOR, foreground=TEXT_MUTED)

        self.angle_plot = self._make_plot("PITCH & ROLL ANGLES", "Angle", "°")
        self.angle_plot.addLegend(
            offset=(10, 10),
            pen=pg.mkPen(CARD_BORDER),
            brush=pg.mkBrush("#12151c"),
            labelTextColor=TEXT_MUTED,
        )
        self.pitch_curve = self.angle_plot.plot(
            [], [], pen=pg.mkPen(COLORS["pitch"], width=2), name="Pitch"
        )
        self.roll_curve = self.angle_plot.plot(
            [], [], pen=pg.mkPen(COLORS["roll"], width=2), name="Roll"
        )
        self.pitch_target_line = pg.InfiniteLine(
            pos=TARGET_PITCH, angle=0,
            pen=pg.mkPen(COLORS["pitch"], width=1, style=Qt.PenStyle.DashLine)
        )
        self.angle_plot.addItem(self.pitch_target_line)
        self.roll_target_line = pg.InfiniteLine(
            pos=TARGET_ROLL, angle=0,
            pen=pg.mkPen(COLORS["roll"], width=1, style=Qt.PenStyle.DashLine)
        )
        self.angle_plot.addItem(self.roll_target_line)
        right.addWidget(self.angle_plot, stretch=2)

        plots_bottom = QHBoxLayout()
        plots_bottom.setSpacing(10)

        self.control_plot = self._make_plot("CONTROL SIGNALS", "u", "°")
        self.control_plot.addLegend(
            offset=(10, 10),
            pen=pg.mkPen(CARD_BORDER),
            brush=pg.mkBrush("#12151c"),
            labelTextColor=TEXT_MUTED,
        )
        self.u_pitch_curve = self.control_plot.plot(
            [], [], pen=pg.mkPen(COLORS["u_pitch"], width=2), name="u_pitch"
        )
        self.u_roll_curve = self.control_plot.plot(
            [], [], pen=pg.mkPen(COLORS["u_roll"], width=2), name="u_roll"
        )
        plots_bottom.addWidget(self.control_plot, stretch=1)

        self.height_plot = self._make_plot("DISTANCE", "Distance", "mm")
        self.height_target_line = pg.InfiniteLine(
            pos=TARGET_DISTANCE, angle=0,
            pen=pg.mkPen(COLORS["height"], width=1, style=Qt.PenStyle.DashLine)
        )
        self.height_plot.addItem(self.height_target_line)
        self.height_curve = self.height_plot.plot(
            [], [], pen=pg.mkPen(COLORS["height"], width=2), name="Distance"
        )
        plots_bottom.addWidget(self.height_plot, stretch=1)

        right.addLayout(plots_bottom, stretch=1)
        root.addLayout(right, stretch=1)

        sb = self.statusBar()
        sb.showMessage("READY  ·  Connect a device to begin")

    def _on_connect_click(self):
        dialog = PortSelectDialog(self)
        dialog.port_selected.connect(lambda info: self._sig_connect.emit(info))
        dialog.exec()

    def _on_disconnect_click(self):
        self._sig_disconnect.emit()

    def _on_mode_select(self, mode_id: int):
        self.current_mode = mode_id
        for mid, btn in self.mode_buttons.items():
            btn.setChecked(mid == mode_id)
        self._sig_send_mode.emit(mode_id)
        self.statusBar().showMessage(f"MODE  →  {CONTROLLER_NAMES[mode_id].upper()}")

    @pyqtSlot(object)
    def _on_telemetry(self, td: TelemetryData):
        if self.t0 is None:
            self.t0 = td.time_ms

        t_sec = (td.time_ms - self.t0) / 1000.0
        self.t_buf.append(t_sec)
        self.pitch_buf.append(td.pitch_deg)
        self.roll_buf.append(td.roll_deg)
        self.u_pitch_buf.append(td.u_pitch)
        self.u_roll_buf.append(td.u_roll)
        self.height_buf.append(td.distance_mm)

        self.card_pitch.set_value(td.pitch_deg)
        self.card_roll.set_value(td.roll_deg)
        self.card_height.set_value(td.distance_mm, ".0f")
        self.card_ctrl_pitch.set_value(td.u_pitch, ".2f")
        self.card_ctrl_roll.set_value(td.u_roll, ".2f")
        self.card_mode._value.setText(CONTROLLER_NAMES.get(td.mode, "?"))

        try:
            self.card_pitch.set_target(td.target_pitch)
            self.card_roll.set_target(td.target_roll)
        except Exception:
            pass

        if td.mode != self.current_mode:
            self.current_mode = td.mode
            for mid, btn in self.mode_buttons.items():
                btn.setChecked(mid == td.mode)

        self.platform_view.set_orientation(td.pitch_deg, td.roll_deg + 175.44, td.motor_height_pos)
        self.platform_view.set_targets(td.target_pitch, td.target_roll + 175.44)

        try:
            self.pitch_target_line.setPos(td.target_pitch)
            self.roll_target_line.setPos(td.target_roll)
        except Exception:
            pass
        self.height_target_line.setPos(TARGET_DISTANCE)

    @pyqtSlot(bool)
    def _on_connection_changed(self, connected: bool):
        self._pulse.set_connected(connected)
        if connected:
            self.conn_status.setText("CONNECTED")
            self.conn_status.setStyleSheet(
                f"color: {ACCENT_GREEN}; font-size: 10px; letter-spacing: 1.5px; font-weight: 600;"
            )
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.statusBar().showMessage("CONNECTED  ·  Receiving telemetry")
        else:
            self.conn_status.setText("DISCONNECTED")
            self.conn_status.setStyleSheet(
                f"color: {ACCENT_RED}; font-size: 10px; letter-spacing: 1.5px; font-weight: 600;"
            )
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.statusBar().showMessage("DISCONNECTED")

    @pyqtSlot(str)
    def _on_ble_error(self, msg: str):
        self.statusBar().showMessage(f"ERR  ·  {msg}")
        logger.error(msg)

    def _refresh_plots(self):
        if len(self.t_buf) < 2:
            return
        t = np.array(self.t_buf)
        self.pitch_curve.setData(t, np.array(self.pitch_buf))
        self.roll_curve.setData(t, np.array(self.roll_buf))
        self.u_pitch_curve.setData(t, np.array(self.u_pitch_buf))
        self.u_roll_curve.setData(t, np.array(self.u_roll_buf))
        self.height_curve.setData(t, np.array(self.height_buf))

    def closeEvent(self, event):
        self._sig_disconnect.emit()
        self.ble_thread.quit()
        self.ble_thread.wait(2000)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window,          QColor(13, 15, 20))
    palette.setColor(QPalette.ColorRole.WindowText,      QColor(232, 234, 240))
    palette.setColor(QPalette.ColorRole.Base,            QColor(18, 21, 28))
    palette.setColor(QPalette.ColorRole.AlternateBase,   QColor(24, 28, 38))
    palette.setColor(QPalette.ColorRole.Text,            QColor(232, 234, 240))
    palette.setColor(QPalette.ColorRole.Button,          QColor(26, 30, 42))
    palette.setColor(QPalette.ColorRole.ButtonText,      QColor(232, 234, 240))
    palette.setColor(QPalette.ColorRole.Highlight,       QColor(61, 142, 245))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    window = Dashboard()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()