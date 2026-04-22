"""
HoldMyCoffee Dashboard - USB Serial Connection Module
Replaces BLE with direct Teensy USB serial (pyserial).
Same Qt signal interface as the BLE worker so the dashboard needs minimal changes.
"""

import logging
import threading

import serial
import serial.tools.list_ports
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QListWidget, QComboBox, QDialogButtonBox,
)

from ble_connection import TelemetryData, CONTROLLER_NAMES  # reuse data classes

logger = logging.getLogger(__name__)

DEFAULT_BAUD = 115200


# --------------- Serial Connection (low-level) ---------------

class SerialConnection:
    """Reads Teensy telemetry over USB serial in a background thread."""

    def __init__(self):
        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._buffer = ""
        self._on_telemetry = None
        self._on_connection_changed = None

    def set_callbacks(self, on_telemetry, on_connection_changed):
        self._on_telemetry = on_telemetry
        self._on_connection_changed = on_connection_changed

    @staticmethod
    def list_ports():
        return list(serial.tools.list_ports.comports())

    def connect(self, port: str, baud: int = DEFAULT_BAUD):
        self._serial = serial.Serial(port, baud, timeout=0.05)
        self._running = True
        self._buffer = ""
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        if self._on_connection_changed:
            self._on_connection_changed(True)
        logger.info(f"Serial connected: {port} @ {baud}")

    def disconnect(self):
        self._running = False
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        if self._on_connection_changed:
            self._on_connection_changed(False)
        logger.info("Serial disconnected")

    def send_mode(self, mode: int):
        if self._serial and self._serial.is_open:
            self._serial.write(str(mode).encode("utf-8"))
            logger.info(f"Sent mode: {mode}")

    def _read_loop(self):
        while self._running:
            try:
                waiting = self._serial.in_waiting
                if waiting:
                    chunk = self._serial.read(waiting).decode("utf-8", errors="replace")
                    self._buffer += chunk
                    while "\n" in self._buffer:
                        line, self._buffer = self._buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        print("RX LINE:", line)
                        td = TelemetryData.from_line(line)
                        if td:
                            if self._on_telemetry:
                                self._on_telemetry(td)
                        else:
                            print("UNPARSED:", line)
            except serial.SerialException as e:
                logger.error(f"Serial error: {e}")
                self._running = False
                if self._on_connection_changed:
                    self._on_connection_changed(False)
                break


# --------------- Qt Worker (runs in QThread) ---------------

class SerialWorker(QObject):
    """Same signal interface as BLEWorker — drop-in replacement in Dashboard."""
    telemetry_received = pyqtSignal(object)
    connection_changed = pyqtSignal(bool)
    scan_finished = pyqtSignal(list)   # list of serial.tools.list_ports.ListPortInfo
    error = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.conn = SerialConnection()

    @pyqtSlot()
    def init_loop(self):
        """Called when the QThread starts."""
        self.conn.set_callbacks(
            on_telemetry=lambda td: self.telemetry_received.emit(td),
            on_connection_changed=lambda c: self.connection_changed.emit(c),
        )

    @pyqtSlot()
    def scan(self):
        ports = SerialConnection.list_ports()
        self.scan_finished.emit(ports)

    @pyqtSlot(object)
    def connect_device(self, port_info):
        """port_info is a (port_name, baud) tuple."""
        try:
            port, baud = port_info
            self.conn.connect(port, baud)
        except Exception as e:
            self.error.emit(f"Connect failed: {e}")

    @pyqtSlot()
    def disconnect_device(self):
        try:
            self.conn.disconnect()
        except Exception as e:
            self.error.emit(f"Disconnect failed: {e}")

    @pyqtSlot(int)
    def send_mode(self, mode: int):
        try:
            self.conn.send_mode(mode)
        except Exception as e:
            self.error.emit(f"Send failed: {e}")


# --------------- Port Selection Dialog ---------------

class PortSelectDialog(QDialog):
    """Dialog to pick a COM port and baud rate."""

    port_selected = pyqtSignal(object)   # emits (port_str, baud_int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Select Serial Port")
        self.setMinimumSize(380, 260)
        self._ports = []

        layout = QVBoxLayout(self)
        self.status_label = QLabel("Click 'Refresh' to list available ports.")
        layout.addWidget(self.status_label)

        self.port_list = QListWidget()
        layout.addWidget(self.port_list)

        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel("Baud rate:"))
        self.baud_combo = QComboBox()
        for b in [115200, 9600, 57600, 38400]:
            self.baud_combo.addItem(str(b))
        self.baud_combo.setCurrentText("115200")
        baud_row.addWidget(self.baud_combo)
        baud_row.addStretch()
        layout.addLayout(baud_row)

        btn_row = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh)
        btn_row.addWidget(self.refresh_btn)
        self.buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        self.buttons.accepted.connect(self._on_accept)
        self.buttons.rejected.connect(self.reject)
        btn_row.addWidget(self.buttons)
        layout.addLayout(btn_row)

        self._refresh()

    def _refresh(self):
        self._ports = SerialConnection.list_ports()
        self.port_list.clear()
        for p in self._ports:
            label = f"{p.device}  —  {p.description}"
            if "teensy" in p.description.lower() or "usb serial" in p.description.lower():
                label = "[TEENSY] " + label
            self.port_list.addItem(label)
        self.status_label.setText(f"Found {len(self._ports)} port(s). Select Teensy USB Serial.")

    def _on_accept(self):
        idx = self.port_list.currentRow()
        if 0 <= idx < len(self._ports):
            port = self._ports[idx].device
            baud = int(self.baud_combo.currentText())
            self.port_selected.emit((port, baud))
            self.accept()
