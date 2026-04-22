"""
HoldMyCoffee Dashboard
Real-time BLE monitoring and control dashboard for the stabilization platform.
Qt6 + PyQtGraph + OpenGL 3D visualization.
"""

import sys
import math
import asyncio
import collections
import logging
from functools import partial

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QGroupBox, QFrame,
    QSizePolicy, QListWidget, QDialog, QDialogButtonBox,
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread, pyqtSlot
from PyQt6.QtGui import QColor, QPalette, QPainter, QBrush, QLinearGradient

from serial_connection import SerialWorker, PortSelectDialog
from ble_connection import TelemetryData, CONTROLLER_NAMES
from platform_3d import create_platform_view

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

TARGET_PITCH    = -0.2
TARGET_ROLL     = -175.44
TARGET_DISTANCE = 125.0
TOLERANCE_DEG   = 2.0

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

CONTROLLER_INFO = {
    1: {
        "name":  "PID",
        "full":  "Proportional–Integral–Derivative",
        "desc":  (
            "Classic error-based controller tuned via SIMC rules. "
            "Most robust roll disturbance rejection under increasing payload. "
            "Reliable and consistent across all fill levels."
        ),
        "color": ACCENT_BLUE,
        "badge": "ROBUST",
    },
    2: {
        "name":  "RST",
        "full":  "Polynomial RST (Pole-Placement)",
        "desc":  (
            "Two-degree-of-freedom discrete-time controller designed via "
            "Nelder-Mead optimization on the cascaded plant model. "
            "Best pitch steady-state at half-fill; degrades at higher mass."
        ),
        "color": ACCENT_ORANGE,
        "badge": "SENSITIVE",
    },
    3: {
        "name":  "LQG",
        "full":  "Linear Quadratic Gaussian",
        "desc":  (
            "Kalman filter state estimator + LQR optimal feedback. "
            "Best overall CPA: lowest steady-state RMSE and IAE on roll "
            "across all payloads, best pitch disturbance rejection in all cases."
        ),
        "color": ACCENT_GREEN,
        "badge": "BEST CPA",
    },
    4: {
        "name":  "MRAC",
        "full":  "Model Reference Adaptive Control",
        "desc":  (
            "Online adaptive gain law driven by tracking error vs. a 1st-order "
            "reference model (τ=0.5 s). Lowest control effort in all disturbance "
            "tests; poorest tracking accuracy at 40 Hz sample rate."
        ),
        "color": ACCENT_PURPLE,
        "badge": "LOW EFFORT",
    },
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
        self._color    = QColor(color)
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
        self._color        = color
        self._target       = target
        self._target_range = target_range or 10.0
        self._unit         = unit

        self.setMinimumHeight(108)
        self.setStyleSheet(f"""
            MetricCard {{
                background-color: {CARD_BG};
                border: 1px solid {CARD_BORDER};
                border-radius: 10px;
            }}
        """)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 10, 14, 10)
        outer.setSpacing(0)

        top_row = QHBoxLayout()
        self._label_w = QLabel(label)
        self._label_w.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 2px; font-weight: 600;"
        )
        top_row.addWidget(self._label_w)
        top_row.addStretch()
        self._dot = QLabel("●")
        self._dot.setStyleSheet(f"color: {TEXT_DIM}; font-size: 7px;")
        top_row.addWidget(self._dot)
        outer.addLayout(top_row)

        outer.addSpacing(4)

        value_row = QHBoxLayout()
        value_row.setSpacing(3)
        value_row.setAlignment(Qt.AlignmentFlag.AlignBaseline)
        self._value = QLabel("—")
        self._value.setStyleSheet(
            f"color: {color}; font-size: 26px; font-weight: 700; letter-spacing: -0.5px;"
        )
        value_row.addWidget(self._value)
        self._unit_lbl = QLabel(unit)
        self._unit_lbl.setStyleSheet(
            f"color: {color}; font-size: 11px; font-weight: 400; padding-bottom: 3px;"
        )
        value_row.addWidget(self._unit_lbl)
        value_row.addStretch()
        outer.addLayout(value_row)

        outer.addSpacing(5)
        self._bar = DeviationBar(color, self)
        outer.addWidget(self._bar)
        outer.addSpacing(5)

        bottom_row = QHBoxLayout()
        self._target_lbl = QLabel("")
        self._target_lbl.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 0.5px;"
        )
        bottom_row.addWidget(self._target_lbl)
        bottom_row.addStretch()
        self._delta_lbl = QLabel("")
        self._delta_lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 9px;")
        bottom_row.addWidget(self._delta_lbl)
        outer.addLayout(bottom_row)

        if target is not None:
            self._target_lbl.setText(f"TARGET  {target:+.2f}{unit}")

    def set_value(self, val: float, fmt: str = ".1f"):
        self._value.setText(f"{val:{fmt}}")
        self._dot.setStyleSheet(f"color: {self._color}; font-size: 7px;")
        if self._target is not None:
            delta = val - self._target
            frac  = 1.0 - min(abs(delta) / self._target_range, 1.0)
            self._bar.set_fraction(frac)
            sign = "+" if delta >= 0 else ""
            bad  = abs(delta) > self._target_range * 0.5
            self._delta_lbl.setStyleSheet(
                f"color: {'#f55555' if bad else TEXT_MUTED}; font-size: 9px;"
            )
            self._delta_lbl.setText(f"Δ {sign}{delta:{fmt}}{self._unit}")
        else:
            self._bar.set_fraction(1.0)


class PulseIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(10, 10)
        self._connected = False
        self._opacity   = 1.0
        self._phase     = 0.0
        self._timer     = QTimer(self)
        self._timer.timeout.connect(self._tick)

    def set_connected(self, v: bool):
        self._connected = v
        if v:
            self._timer.start(50)
        else:
            self._timer.stop()
            self._opacity = 1.0
            self.update()

    def _tick(self):
        self._phase  += 0.15
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


class LiveStatsWidget(QFrame):
    """Rolling performance metrics updated at ~5 Hz."""

    WINDOW = 500

    def __init__(self, axis: str, color: str, parent=None):
        super().__init__(parent)
        self._color  = color
        self._errors = collections.deque(maxlen=self.WINDOW)
        self._u_vals = collections.deque(maxlen=self.WINDOW)
        self._ts     = 0.025

        self.setStyleSheet(f"""
            LiveStatsWidget {{
                background-color: {CARD_BG};
                border: 1px solid {CARD_BORDER};
                border-radius: 10px;
            }}
        """)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(12, 8, 12, 8)
        outer.setSpacing(4)

        header = QLabel(f"{axis.upper()}  LIVE STATS")
        header.setStyleSheet(
            f"color: {color}; font-size: 9px; letter-spacing: 2px; font-weight: 700;"
        )
        outer.addWidget(header)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"color: {CARD_BORDER};")
        outer.addWidget(sep)

        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(2)

        metrics = [
            ("RMSE",     "°",  "rmse",    0, 0),
            ("IAE",      "°s", "iae",     1, 0),
            ("MAX DEV",  "°",  "maxdev",  2, 0),
            ("ERR NOW",  "°",  "errnow",  0, 3),
            ("MEAN ERR", "°",  "meanerr", 1, 3),
            ("CTRL EFF", "°",  "ctrleff", 2, 3),
        ]
        self._rows = {}
        for lbl, unit, key, row, col in metrics:
            l = QLabel(lbl)
            l.setStyleSheet(
                f"color: {TEXT_MUTED}; font-size: 8px; letter-spacing: 1px;"
            )
            v = QLabel("—")
            v.setStyleSheet(
                f"color: {TEXT_PRIMARY}; font-size: 12px; font-weight: 600;"
            )
            u = QLabel(unit)
            u.setStyleSheet(f"color: {TEXT_DIM}; font-size: 8px;")
            grid.addWidget(l, row * 2,     col)
            grid.addWidget(v, row * 2 + 1, col)
            grid.addWidget(u, row * 2 + 1, col + 1)
            self._rows[key] = v

        outer.addLayout(grid)

    def push(self, error: float, u_val: float):
        self._errors.append(error)
        self._u_vals.append(u_val)

    def refresh(self):
        if len(self._errors) < 2:
            return
        e = np.array(self._errors)
        u = np.array(self._u_vals)

        rmse    = float(np.sqrt(np.mean(e ** 2)))
        iae     = float(np.sum(np.abs(e)) * self._ts)
        maxdev  = float(np.max(np.abs(e)))
        errnow  = float(e[-1])
        meanerr = float(np.mean(e))
        ctrl    = float(np.sum(np.abs(np.diff(u))))

        def _color_for(val, limit):
            if limit is None:
                return TEXT_PRIMARY
            if abs(val) > limit:
                return ACCENT_RED
            if abs(val) < limit * 0.3:
                return ACCENT_GREEN
            return TEXT_PRIMARY

        for key, val, lim in [
            ("rmse",    rmse,    2.0),
            ("iae",     iae,     None),
            ("maxdev",  maxdev,  5.0),
            ("errnow",  errnow,  2.0),
            ("meanerr", meanerr, 1.0),
            ("ctrleff", ctrl,    None),
        ]:
            c   = _color_for(val, lim)
            txt = f"{val:+.3f}" if abs(val) < 10 else f"{val:+.1f}"
            self._rows[key].setText(txt)
            self._rows[key].setStyleSheet(
                f"color: {c}; font-size: 12px; font-weight: 600;"
            )


class ControllerInfoPanel(QFrame):
    """Describes the currently active controller; used for jury / demo context."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            ControllerInfoPanel {{
                background-color: {CARD_BG};
                border: 1px solid {CARD_BORDER};
                border-radius: 10px;
            }}
        """)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 10, 14, 10)
        outer.setSpacing(4)

        top = QHBoxLayout()
        self._name_lbl = QLabel("—")
        self._name_lbl.setStyleSheet(
            f"color: {TEXT_PRIMARY}; font-size: 16px; font-weight: 700; letter-spacing: 1px;"
        )
        top.addWidget(self._name_lbl)
        top.addStretch()
        self._badge = QLabel("")
        self._badge.setStyleSheet(
            f"font-size: 8px; letter-spacing: 1.5px; font-weight: 700; "
            f"padding: 2px 6px; border-radius: 4px;"
        )
        top.addWidget(self._badge)
        outer.addLayout(top)

        self._full_lbl = QLabel("")
        self._full_lbl.setStyleSheet(
            f"color: {TEXT_MUTED}; font-size: 9px; letter-spacing: 0.5px;"
        )
        outer.addWidget(self._full_lbl)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet(f"color: {CARD_BORDER};")
        outer.addWidget(sep)

        self._desc_lbl = QLabel("")
        self._desc_lbl.setWordWrap(True)
        self._desc_lbl.setStyleSheet(
            f"color: #8090a8; font-size: 10px; line-height: 1.5;"
        )
        outer.addWidget(self._desc_lbl)

    def set_controller(self, mode_id: int):
        info = CONTROLLER_INFO.get(mode_id)
        if not info:
            return
        c = info["color"]
        self._name_lbl.setText(info["name"])
        self._name_lbl.setStyleSheet(
            f"color: {c}; font-size: 16px; font-weight: 700; letter-spacing: 1px;"
        )
        self._full_lbl.setText(info["full"])
        self._desc_lbl.setText(info["desc"])
        self._badge.setText(info["badge"])
        self._badge.setStyleSheet(
            f"font-size: 8px; letter-spacing: 1.5px; font-weight: 700; "
            f"padding: 2px 6px; border-radius: 4px; "
            f"background: {c}22; color: {c}; border: 1px solid {c}55;"
        )


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
        self.status_label.setText("Scanning...")
        self.device_list.clear()
        if self._worker:
            QTimer.singleShot(0, self._worker.scan)

    def _on_scan_result(self, devices):
        self.devices = devices
        self.device_list.clear()
        for d in devices:
            name    = d.name or "Unknown"
            address = getattr(d, "address", "?")
            rssi    = getattr(d, "rssi", None)
            rssi_t  = f"{rssi} dBm" if rssi is not None else "n/a"
            label   = f"{name}  [{address}]  RSSI: {rssi_t}"
            if any(k in name.upper() for k in ("HM", "AT-09", "HMSOFT")):
                label = "★  " + label
            self.device_list.addItem(label)
        self.scan_btn.setEnabled(True)
        self.status_label.setText(f"Found {len(devices)} device(s).")

    def _on_accept(self):
        idx = self.device_list.currentRow()
        if 0 <= idx < len(self.devices):
            self.device_selected.emit(self.devices[idx])
            self.accept()


class Dashboard(QMainWindow):
    _sig_connect    = pyqtSignal(object)
    _sig_disconnect = pyqtSignal()
    _sig_send_mode  = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("HoldMyCoffee  ·  Stabilization Dashboard")
        self.resize(1680, 980)

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

        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self._refresh_stats)
        self.stats_timer.start(200)

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
            f'<span style="color:{TEXT_MUTED};font-size:10px;'
            f'letter-spacing:2px;">{title}</span>'
        )
        pw.getAxis("left").setPen(pg.mkPen(CARD_BORDER))
        pw.getAxis("bottom").setPen(pg.mkPen(CARD_BORDER))
        pw.getAxis("left").setTextPen(pg.mkPen(TEXT_MUTED))
        pw.getAxis("bottom").setTextPen(pg.mkPen(TEXT_MUTED))
        pw.setStyleSheet(
            f"border: 1px solid {CARD_BORDER}; border-radius: 10px;"
        )
        return pw

    def _add_target_band(self, plot: pg.PlotWidget, target: float, color: str):
        """Target dashed line + two dotted white ±TOLERANCE_DEG boundary lines."""
        target_line = pg.InfiniteLine(
            pos=target, angle=0,
            pen=pg.mkPen(color, width=1, style=Qt.PenStyle.DashLine),
        )
        lo_line = pg.InfiniteLine(
            pos=target - TOLERANCE_DEG, angle=0,
            pen=pg.mkPen("#ffffff", width=1, style=Qt.PenStyle.DotLine),
        )
        hi_line = pg.InfiniteLine(
            pos=target + TOLERANCE_DEG, angle=0,
            pen=pg.mkPen("#ffffff", width=1, style=Qt.PenStyle.DotLine),
        )
        plot.addItem(target_line)
        plot.addItem(lo_line)
        plot.addItem(hi_line)
        return target_line

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # ===== LEFT PANEL =====
        left = QVBoxLayout()
        left.setSpacing(8)

        conn_group = QGroupBox("CONNECTION")
        conn_lay = QVBoxLayout(conn_group)
        conn_lay.setSpacing(6)
        conn_lay.setContentsMargins(12, 6, 12, 10)

        status_row = QHBoxLayout()
        self._pulse = PulseIndicator()
        status_row.addWidget(self._pulse)
        self.conn_status = QLabel("DISCONNECTED")
        self.conn_status.setStyleSheet(
            f"color: {ACCENT_RED}; font-size: 10px; "
            f"letter-spacing: 1.5px; font-weight: 600;"
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

        ctrl_group = QGroupBox("CONTROLLER SELECT")
        ctrl_lay = QVBoxLayout(ctrl_group)
        ctrl_lay.setSpacing(4)
        ctrl_lay.setContentsMargins(12, 6, 12, 10)
        self.mode_buttons = {}
        for mode_id, name in CONTROLLER_NAMES.items():
            btn = QPushButton(name.upper())
            btn.setCheckable(True)
            btn.clicked.connect(partial(self._on_mode_select, mode_id))
            ctrl_lay.addWidget(btn)
            self.mode_buttons[mode_id] = btn
        self.mode_buttons[1].setChecked(True)
        left.addWidget(ctrl_group)

        self.ctrl_info_panel = ControllerInfoPanel()
        self.ctrl_info_panel.set_controller(1)
        left.addWidget(self.ctrl_info_panel)

        view3d_group = QGroupBox("3D PLATFORM VIEW")
        view3d_lay = QVBoxLayout(view3d_group)
        view3d_lay.setContentsMargins(6, 6, 6, 6)
        self.platform_view = create_platform_view()
        try:
            self.platform_view.setCameraParams(distance=80)
        except Exception:
            pass
        view3d_lay.addWidget(self.platform_view)
        left.addWidget(view3d_group, stretch=1)

        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setFixedWidth(260)
        root.addWidget(left_w)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet(f"color: {CARD_BORDER};")
        root.addWidget(sep)

        # ===== RIGHT PANEL =====
        right = QVBoxLayout()
        right.setSpacing(8)

        # --- Metric cards ---
        cards_row = QHBoxLayout()
        cards_row.setSpacing(6)

        self.card_pitch = MetricCard(
            "PITCH", "°", COLORS["pitch"],
            target=TARGET_PITCH, target_range=TOLERANCE_DEG * 2,
        )
        self.card_roll = MetricCard(
            "ROLL", "°", COLORS["roll"],
            target=TARGET_ROLL, target_range=TOLERANCE_DEG * 2,
        )
        self.card_height = MetricCard(
            "DISTANCE", " mm", COLORS["height"],
            target=TARGET_DISTANCE, target_range=10.0,
        )
        self.card_ctrl_pitch = MetricCard("U PITCH", "°", COLORS["u_pitch"])
        self.card_ctrl_roll  = MetricCard("U ROLL",  "°", COLORS["u_roll"])

        for card in [self.card_pitch, self.card_roll, self.card_height,
                     self.card_ctrl_pitch, self.card_ctrl_roll]:
            cards_row.addWidget(card)
        right.addLayout(cards_row)

        pg.setConfigOptions(
            antialias=True, background=PLOT_BG_COLOR, foreground=TEXT_MUTED
        )

        # --- Top plots: Pitch | Roll (2x2 layout, top row) ---
        plots_top = QHBoxLayout()
        plots_top.setSpacing(8)

        self.pitch_plot = self._make_plot("PITCH ANGLE", "Angle", "°")
        self.pitch_plot.addLegend(
            offset=(10, 10), pen=pg.mkPen(CARD_BORDER),
            brush=pg.mkBrush(PANEL_BG), labelTextColor=TEXT_MUTED,
        )
        self.pitch_curve = self.pitch_plot.plot(
            [], [], pen=pg.mkPen(COLORS["pitch"], width=2), name="Pitch",
        )
        self._pitch_target_line = self._add_target_band(
            self.pitch_plot, TARGET_PITCH, COLORS["pitch"]
        )
        plots_top.addWidget(self.pitch_plot, stretch=1)

        self.roll_plot = self._make_plot("ROLL ANGLE", "Angle", "°")
        self.roll_plot.addLegend(
            offset=(10, 10), pen=pg.mkPen(CARD_BORDER),
            brush=pg.mkBrush(PANEL_BG), labelTextColor=TEXT_MUTED,
        )
        self.roll_curve = self.roll_plot.plot(
            [], [], pen=pg.mkPen(COLORS["roll"], width=2), name="Roll",
        )
        self._roll_target_line = self._add_target_band(
            self.roll_plot, TARGET_ROLL, COLORS["roll"]
        )
        plots_top.addWidget(self.roll_plot, stretch=1)

        right.addLayout(plots_top, stretch=2)

        # --- Bottom plots: Control signals | Distance ---
        plots_bot = QHBoxLayout()
        plots_bot.setSpacing(8)

        self.control_plot = self._make_plot("CONTROL SIGNALS", "u", "°")
        self.control_plot.addLegend(
            offset=(10, 10), pen=pg.mkPen(CARD_BORDER),
            brush=pg.mkBrush(PANEL_BG), labelTextColor=TEXT_MUTED,
        )
        self.u_pitch_curve = self.control_plot.plot(
            [], [], pen=pg.mkPen(COLORS["u_pitch"], width=2), name="u_pitch",
        )
        self.u_roll_curve = self.control_plot.plot(
            [], [], pen=pg.mkPen(COLORS["u_roll"], width=2), name="u_roll",
        )
        plots_bot.addWidget(self.control_plot, stretch=1)

        self.height_plot = self._make_plot("DISTANCE", "Distance", "mm")
        self._add_target_band(self.height_plot, TARGET_DISTANCE, COLORS["height"])
        self.height_curve = self.height_plot.plot(
            [], [], pen=pg.mkPen(COLORS["height"], width=2), name="Distance",
        )
        plots_bot.addWidget(self.height_plot, stretch=1)

        right.addLayout(plots_bot, stretch=1)

        # --- Live stats row ---
        stats_row = QHBoxLayout()
        stats_row.setSpacing(8)
        self.stats_pitch = LiveStatsWidget("Pitch", COLORS["pitch"])
        self.stats_roll  = LiveStatsWidget("Roll",  COLORS["roll"])
        stats_row.addWidget(self.stats_pitch, stretch=1)
        stats_row.addWidget(self.stats_roll,  stretch=1)
        right.addLayout(stats_row)

        root.addLayout(right, stretch=1)
        self.statusBar().showMessage("READY  ·  Connect a device to begin")

    # ---- Callbacks ----

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
        self.ctrl_info_panel.set_controller(mode_id)
        self.statusBar().showMessage(
            f"MODE  →  {CONTROLLER_NAMES[mode_id].upper()}"
        )

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

        self.stats_pitch.push(td.pitch_deg - TARGET_PITCH, td.u_pitch)
        self.stats_roll.push(td.roll_deg   - TARGET_ROLL,  td.u_roll)

        self.card_pitch.set_value(td.pitch_deg)
        self.card_roll.set_value(td.roll_deg)
        self.card_height.set_value(td.distance_mm, ".0f")
        self.card_ctrl_pitch.set_value(td.u_pitch, ".2f")
        self.card_ctrl_roll.set_value(td.u_roll,   ".2f")

        if td.mode != self.current_mode:
            self.current_mode = td.mode
            for mid, btn in self.mode_buttons.items():
                btn.setChecked(mid == td.mode)
            self.ctrl_info_panel.set_controller(td.mode)

        self.platform_view.set_orientation(
            td.pitch_deg, td.roll_deg + 175.44, td.motor_height_pos
        )
        self.platform_view.set_targets(td.target_pitch, td.target_roll + 175.44)

    @pyqtSlot(bool)
    def _on_connection_changed(self, connected: bool):
        self._pulse.set_connected(connected)
        if connected:
            self.conn_status.setText("CONNECTED")
            self.conn_status.setStyleSheet(
                f"color: {ACCENT_GREEN}; font-size: 10px; "
                f"letter-spacing: 1.5px; font-weight: 600;"
            )
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.statusBar().showMessage("CONNECTED  ·  Receiving telemetry")
        else:
            self.conn_status.setText("DISCONNECTED")
            self.conn_status.setStyleSheet(
                f"color: {ACCENT_RED}; font-size: 10px; "
                f"letter-spacing: 1.5px; font-weight: 600;"
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

    def _refresh_stats(self):
        self.stats_pitch.refresh()
        self.stats_roll.refresh()

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