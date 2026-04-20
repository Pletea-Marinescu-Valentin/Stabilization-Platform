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
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread, pyqtSlot
from PyQt6.QtGui import QFont, QColor, QPalette, QIcon

from ble_connection import BLEConnection, TelemetryData, CONTROLLER_NAMES
from platform_3d import create_platform_view

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# --------------- Style ---------------

DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #1a1a24;
    color: #e0e0e0;
    font-family: 'Segoe UI', 'Inter', sans-serif;
}
QGroupBox {
    border: 1px solid #3a3a4a;
    border-radius: 8px;
    margin-top: 12px;
    padding-top: 18px;
    font-weight: bold;
    font-size: 12px;
    color: #b0b0c0;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
}
QPushButton {
    background-color: #2e3a4d;
    border: 1px solid #4a5568;
    border-radius: 6px;
    padding: 8px 18px;
    font-size: 12px;
    color: #e0e0e0;
    min-height: 28px;
}
QPushButton:hover {
    background-color: #3a4a60;
    border-color: #5a6a80;
}
QPushButton:pressed {
    background-color: #4a5a70;
}
QPushButton:checked, QPushButton[active="true"] {
    background-color: #2a6bcf;
    border-color: #4a8aef;
    color: white;
}
QPushButton:disabled {
    background-color: #1e1e28;
    color: #555;
}
QPushButton#connectBtn {
    background-color: #1a7a3a;
    border-color: #2a9a4a;
    font-weight: bold;
}
QPushButton#connectBtn:hover {
    background-color: #2a9a4a;
}
QPushButton#disconnectBtn {
    background-color: #7a2a2a;
    border-color: #9a3a3a;
}
QComboBox {
    background-color: #2e3a4d;
    border: 1px solid #4a5568;
    border-radius: 6px;
    padding: 6px 12px;
    color: #e0e0e0;
    min-height: 28px;
}
QComboBox::drop-down {
    border: none;
    width: 24px;
}
QLabel#metricValue {
    font-size: 26px;
    font-weight: bold;
    color: #4a9eff;
}
QLabel#metricLabel {
    font-size: 10px;
    color: #888;
    text-transform: uppercase;
}
QLabel#statusLabel {
    font-size: 11px;
    padding: 4px 8px;
}
QStatusBar {
    background-color: #14141c;
    color: #888;
    font-size: 11px;
}
QFrame#separator {
    background-color: #3a3a4a;
    max-height: 1px;
}
"""

PLOT_BG = "#1e1e28"
PLOT_GRID_COLOR = (60, 60, 75, 80)
COLORS = {
    "pitch": "#4a9eff",
    "roll": "#ff6a4a",
    "height": "#4adf8a",
    "u_pitch": "#c084fc",
    "u_roll": "#f59e42",
    "target": "#555555",
}

HISTORY_LEN = 500  # ~10 seconds at 50 Hz


# --------------- BLE Worker (runs in QThread) ---------------

class BLEWorker(QObject):
    """Runs asyncio BLE operations in a separate thread."""
    telemetry_received = pyqtSignal(object)
    connection_changed = pyqtSignal(bool)
    scan_finished = pyqtSignal(list)
    error = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ble = BLEConnection()
        self._loop: asyncio.AbstractEventLoop = None

    @pyqtSlot()
    def init_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self.ble.set_callbacks(
            on_telemetry=self._on_telemetry,
            on_connection_changed=lambda c: self.connection_changed.emit(c),
        )

    def _on_telemetry(self, td: TelemetryData):
        self.telemetry_received.emit(td)

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


# --------------- Metric Card Widget ---------------

class MetricCard(QFrame):
    """Single metric display card."""

    def __init__(self, label: str, unit: str = "°", color: str = "#4a9eff", parent=None):
        super().__init__(parent)
        self.setFixedHeight(90)
        self.setStyleSheet(f"""
            MetricCard {{
                background-color: #22222e;
                border: 1px solid #2e2e3e;
                border-radius: 10px;
                padding: 8px;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(2)

        self.label = QLabel(label)
        self.label.setObjectName("metricLabel")
        self.label.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self.value = QLabel("--")
        self.value.setObjectName("metricValue")
        self.value.setStyleSheet(f"color: {color};")
        self.value.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self.unit_str = unit
        layout.addWidget(self.label)
        layout.addWidget(self.value)

    def set_value(self, val: float, fmt: str = ".1f"):
        self.value.setText(f"{val:{fmt}}{self.unit_str}")


# --------------- Scan Dialog ---------------

class ScanDialog(QDialog):
    """Dialog for scanning and selecting a BLE device."""

    device_selected = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("BLE Device Scanner")
        self.setMinimumSize(380, 300)
        self.setStyleSheet(DARK_STYLE)
        self.devices = []

        layout = QVBoxLayout(self)
        self.status_label = QLabel("Click 'Scan' to find BLE devices...")
        layout.addWidget(self.status_label)

        self.device_list = QListWidget()
        layout.addWidget(self.device_list)

        btn_layout = QHBoxLayout()
        self.scan_btn = QPushButton("Scan")
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

    def set_worker(self, worker: BLEWorker):
        self._worker = worker
        worker.scan_finished.connect(self._on_scan_result)

    def _on_scan(self):
        self.scan_btn.setEnabled(False)
        self.status_label.setText("Scanning...")
        self.device_list.clear()
        if self._worker:
            QTimer.singleShot(0, self._worker.scan)

    def _on_scan_result(self, devices):
        self.devices = devices
        self.device_list.clear()
        for d in devices:
            name = d.name or "Unknown"
            self.device_list.addItem(f"{name}  [{d.address}]")
        self.scan_btn.setEnabled(True)
        self.status_label.setText(f"Found {len(devices)} device(s)")

    def _on_accept(self):
        idx = self.device_list.currentRow()
        if 0 <= idx < len(self.devices):
            self.device_selected.emit(self.devices[idx])
            self.accept()


# --------------- Main Dashboard Window ---------------

class Dashboard(QMainWindow):
    # Signals for thread-safe BLE calls
    _sig_connect = pyqtSignal(object)
    _sig_disconnect = pyqtSignal()
    _sig_send_mode = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("HoldMyCoffee — Stabilization Dashboard")
        self.resize(1440, 850)

        # Data buffers
        self.t_buf = collections.deque(maxlen=HISTORY_LEN)
        self.pitch_buf = collections.deque(maxlen=HISTORY_LEN)
        self.roll_buf = collections.deque(maxlen=HISTORY_LEN)
        self.u_pitch_buf = collections.deque(maxlen=HISTORY_LEN)
        self.u_roll_buf = collections.deque(maxlen=HISTORY_LEN)
        self.height_buf = collections.deque(maxlen=HISTORY_LEN)
        self.t0 = None
        self.current_mode = 1

        self._setup_ble_thread()
        self._build_ui()
        self.setStyleSheet(DARK_STYLE)

        # Refresh timer for plots (30 fps)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._refresh_plots)
        self.plot_timer.start(33)

    # ---- BLE Thread Setup ----

    def _setup_ble_thread(self):
        self.ble_thread = QThread()
        self.ble_worker = BLEWorker()
        self.ble_worker.moveToThread(self.ble_thread)
        self.ble_thread.started.connect(self.ble_worker.init_loop)

        self.ble_worker.telemetry_received.connect(self._on_telemetry)
        self.ble_worker.connection_changed.connect(self._on_connection_changed)
        self.ble_worker.error.connect(self._on_ble_error)

        self._sig_connect.connect(self.ble_worker.connect_device)
        self._sig_disconnect.connect(self.ble_worker.disconnect_device)
        self._sig_send_mode.connect(self.ble_worker.send_mode)

        self.ble_thread.start()

    # ---- UI Building ----

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # ===== LEFT PANEL (controls + 3D) =====
        left_panel = QVBoxLayout()
        left_panel.setSpacing(10)

        # Connection group
        conn_group = QGroupBox("Connection")
        conn_lay = QVBoxLayout(conn_group)

        self.connect_btn = QPushButton("Connect BLE")
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.clicked.connect(self._on_connect_click)
        conn_lay.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setObjectName("disconnectBtn")
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self._on_disconnect_click)
        conn_lay.addWidget(self.disconnect_btn)

        self.conn_status = QLabel("● Disconnected")
        self.conn_status.setObjectName("statusLabel")
        self.conn_status.setStyleSheet("color: #ff5555;")
        conn_lay.addWidget(self.conn_status)

        left_panel.addWidget(conn_group)

        # Controller selection
        ctrl_group = QGroupBox("Controller")
        ctrl_lay = QVBoxLayout(ctrl_group)
        self.mode_buttons = {}
        for mode_id, name in CONTROLLER_NAMES.items():
            btn = QPushButton(name)
            btn.setCheckable(True)
            btn.clicked.connect(partial(self._on_mode_select, mode_id))
            ctrl_lay.addWidget(btn)
            self.mode_buttons[mode_id] = btn
        self.mode_buttons[1].setChecked(True)
        left_panel.addWidget(ctrl_group)

        # 3D View
        view3d_group = QGroupBox("3D Platform View")
        view3d_lay = QVBoxLayout(view3d_group)
        self.platform_view = create_platform_view()
        view3d_lay.addWidget(self.platform_view)
        left_panel.addWidget(view3d_group, stretch=1)

        left_widget = QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(300)
        main_layout.addWidget(left_widget)

        # ===== RIGHT PANEL (metrics + plots) =====
        right_layout = QVBoxLayout()
        right_layout.setSpacing(8)

        # Metric cards row
        metrics_layout = QHBoxLayout()
        metrics_layout.setSpacing(8)

        self.card_pitch = MetricCard("PITCH", "°", COLORS["pitch"])
        self.card_roll = MetricCard("ROLL", "°", COLORS["roll"])
        self.card_height = MetricCard("HEIGHT", " mm", COLORS["height"])
        self.card_ctrl_pitch = MetricCard("U PITCH", "°", COLORS["u_pitch"])
        self.card_ctrl_roll = MetricCard("U ROLL", "°", COLORS["u_roll"])
        self.card_mode = MetricCard("CONTROLLER", "", "#ffffff")

        for card in [self.card_pitch, self.card_roll, self.card_height,
                     self.card_ctrl_pitch, self.card_ctrl_roll, self.card_mode]:
            metrics_layout.addWidget(card)

        right_layout.addLayout(metrics_layout)

        # Plots
        pg.setConfigOptions(antialias=True, background=PLOT_BG, foreground="#cccccc")

        # Pitch/Roll angle plot
        self.angle_plot = pg.PlotWidget(title="Pitch & Roll Angles")
        self.angle_plot.setLabel("left", "Angle", "°")
        self.angle_plot.setLabel("bottom", "Time", "s")
        self.angle_plot.showGrid(x=True, y=True, alpha=0.15)
        self.angle_plot.addLegend(offset=(10, 10))
        self.pitch_curve = self.angle_plot.plot([], [], pen=pg.mkPen(COLORS["pitch"], width=2), name="Pitch")
        self.roll_curve = self.angle_plot.plot([], [], pen=pg.mkPen(COLORS["roll"], width=2), name="Roll")
        # Target lines
        self.pitch_target_line = pg.InfiniteLine(pos=-0.2, angle=0,
            pen=pg.mkPen(COLORS["pitch"], width=1, style=Qt.PenStyle.DashLine))
        self.angle_plot.addItem(self.pitch_target_line)
        right_layout.addWidget(self.angle_plot, stretch=1)

        # Control signal plot
        self.control_plot = pg.PlotWidget(title="Control Signals")
        self.control_plot.setLabel("left", "u", "°")
        self.control_plot.setLabel("bottom", "Time", "s")
        self.control_plot.showGrid(x=True, y=True, alpha=0.15)
        self.control_plot.addLegend(offset=(10, 10))
        self.u_pitch_curve = self.control_plot.plot([], [], pen=pg.mkPen(COLORS["u_pitch"], width=2), name="u_pitch")
        self.u_roll_curve = self.control_plot.plot([], [], pen=pg.mkPen(COLORS["u_roll"], width=2), name="u_roll")
        right_layout.addWidget(self.control_plot, stretch=1)

        # Height / distance plot
        self.height_plot = pg.PlotWidget(title="Distance (Height)")
        self.height_plot.setLabel("left", "Distance", "mm")
        self.height_plot.setLabel("bottom", "Time", "s")
        self.height_plot.showGrid(x=True, y=True, alpha=0.15)
        self.height_target_line = pg.InfiniteLine(pos=125.0, angle=0,
            pen=pg.mkPen(COLORS["height"], width=1, style=Qt.PenStyle.DashLine))
        self.height_plot.addItem(self.height_target_line)
        self.height_curve = self.height_plot.plot([], [], pen=pg.mkPen(COLORS["height"], width=2), name="Distance")
        right_layout.addWidget(self.height_plot, stretch=1)

        main_layout.addLayout(right_layout, stretch=1)

        # Status bar
        self.statusBar().showMessage("Ready — connect a BLE device to start")

    # ---- Callbacks ----

    def _on_connect_click(self):
        dialog = ScanDialog(self)
        dialog.set_worker(self.ble_worker)
        dialog.device_selected.connect(lambda dev: self._sig_connect.emit(dev))
        dialog.exec()

    def _on_disconnect_click(self):
        self._sig_disconnect.emit()

    def _on_mode_select(self, mode_id: int):
        self.current_mode = mode_id
        for mid, btn in self.mode_buttons.items():
            btn.setChecked(mid == mode_id)
        self._sig_send_mode.emit(mode_id)
        self.statusBar().showMessage(f"Sent mode: {CONTROLLER_NAMES[mode_id]}")

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

        # Update metric cards
        self.card_pitch.set_value(td.pitch_deg)
        self.card_roll.set_value(td.roll_deg)
        self.card_height.set_value(td.distance_mm, ".0f")
        self.card_ctrl_pitch.set_value(td.u_pitch, ".2f")
        self.card_ctrl_roll.set_value(td.u_roll, ".2f")
        self.card_mode.value.setText(CONTROLLER_NAMES.get(td.mode, "?"))

        # Update controller button highlight from telemetry
        if td.mode != self.current_mode:
            self.current_mode = td.mode
            for mid, btn in self.mode_buttons.items():
                btn.setChecked(mid == td.mode)

        # Update 3D view
        self.platform_view.set_orientation(td.pitch_deg, td.roll_deg + 175.44, td.motor_height_pos)
        self.platform_view.set_targets(-0.2, 0.0)

    @pyqtSlot(bool)
    def _on_connection_changed(self, connected: bool):
        if connected:
            self.conn_status.setText("● Connected")
            self.conn_status.setStyleSheet("color: #44dd66;")
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.statusBar().showMessage("BLE connected — receiving telemetry")
        else:
            self.conn_status.setText("● Disconnected")
            self.conn_status.setStyleSheet("color: #ff5555;")
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self.statusBar().showMessage("BLE disconnected")

    @pyqtSlot(str)
    def _on_ble_error(self, msg: str):
        self.statusBar().showMessage(f"Error: {msg}")
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


# --------------- Entry Point ---------------

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(26, 26, 36))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(224, 224, 224))
    palette.setColor(QPalette.ColorRole.Base, QColor(30, 30, 40))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(34, 34, 46))
    palette.setColor(QPalette.ColorRole.Text, QColor(224, 224, 224))
    palette.setColor(QPalette.ColorRole.Button, QColor(46, 58, 77))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(224, 224, 224))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(42, 107, 207))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    window = Dashboard()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
