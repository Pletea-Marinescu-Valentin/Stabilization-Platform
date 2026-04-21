"""
HoldMyCoffee Dashboard - BLE Communication Module
Connects to HM-10 BLE module on Teensy 4.1 and parses telemetry.
"""

import asyncio
import logging
from dataclasses import dataclass
from typing import Optional, Callable

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

logger = logging.getLogger(__name__)

# HM-10 BLE UART Service/Characteristic UUIDs
HM10_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
HM10_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

CONTROLLER_NAMES = {1: "PID", 2: "RST", 3: "LQG", 4: "Adaptive"}


@dataclass
class TelemetryData:
    time_ms: int = 0
    mode: int = 1
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    u_pitch: float = 0.0
    u_roll: float = 0.0
    target_pitch: float = 0.0
    target_roll: float = 0.0
    motor_pitch_pos: float = 0.0
    motor_roll_pos: float = 0.0
    motor_height_pos: float = 0.0
    distance_mm: float = 0.0

    @staticmethod
    def from_line(line: str) -> Optional["TelemetryData"]:
        """Parse: T,<time>,<mode>,<pitch>,<roll>,<u_p>,<u_r>,<tgt_p>,<tgt_r>,<mp>,<mr>,<mh>,<dist>"""
        line = line.strip()
        if not line.startswith("T,"):
            return None
        parts = line[2:].split(",")
        if len(parts) < 12:
            return None
        try:
            return TelemetryData(
                time_ms=int(parts[0]),
                mode=int(parts[1]),
                pitch_deg=float(parts[2]),
                roll_deg=float(parts[3]),
                u_pitch=float(parts[4]),
                u_roll=float(parts[5]),
                target_pitch=float(parts[6]),
                target_roll=float(parts[7]),
                motor_pitch_pos=float(parts[8]),
                motor_roll_pos=float(parts[9]),
                motor_height_pos=float(parts[10]),
                distance_mm=float(parts[11]),
            )
        except (ValueError, IndexError):
            return None


class BLEConnection:
    """Manages BLE connection to HM-10 module."""

    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.device: Optional[BLEDevice] = None
        self.connected = False
        self._buffer = ""
        self._on_telemetry: Optional[Callable[[TelemetryData], None]] = None
        self._on_connection_changed: Optional[Callable[[bool], None]] = None

    def set_callbacks(
        self,
        on_telemetry: Callable[[TelemetryData], None],
        on_connection_changed: Callable[[bool], None],
    ):
        self._on_telemetry = on_telemetry
        self._on_connection_changed = on_connection_changed

    async def scan(self, timeout: float = 5.0) -> list[BLEDevice]:
        """Scan for BLE devices."""
        devices = await BleakScanner.discover(timeout=timeout)
        # Put HM-10-like devices first so selection is easier.
        def score_device(d: BLEDevice) -> int:
            name = (d.name or "").upper()
            score = 0
            if "HMSOFT" in name:
                score += 100
            if "HM" in name or "AT-09" in name or "MLT" in name:
                score += 50
            return score

        return sorted(devices, key=score_device, reverse=True)

    async def connect(self, device: BLEDevice):
        """Connect to a BLE device using the BLEDevice object directly (avoids Windows address issues)."""
        self.device = device
        # Pass the BLEDevice object directly — avoids find_device_by_address failures on Windows
        self.client = BleakClient(device, disconnected_callback=self._on_disconnect)
        await self.client.connect()
        self.connected = True
        if self._on_connection_changed:
            self._on_connection_changed(True)
        # Subscribe to notifications
        await self.client.start_notify(HM10_CHAR_UUID, self._notification_handler)
        logger.info(f"Connected to {device.name} ({device.address})")

    async def disconnect(self):
        if self.client and self.connected:
            await self.client.disconnect()
        self.connected = False
        if self._on_connection_changed:
            self._on_connection_changed(False)

    async def send_mode(self, mode: int):
        """Send controller mode command (1-4)."""
        if not self.connected or not self.client:
            return
        data = str(mode).encode("utf-8")
        await self.client.write_gatt_char(HM10_CHAR_UUID, data)
        logger.info(f"Sent mode: {mode}")

    def _on_disconnect(self, client: BleakClient):
        self.connected = False
        logger.info("BLE disconnected")
        if self._on_connection_changed:
            self._on_connection_changed(False)

    def _notification_handler(self, sender, data: bytearray):
        """Handle incoming BLE data, accumulate and parse lines."""
        self._buffer += data.decode("utf-8", errors="replace")
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            td = TelemetryData.from_line(line)
            if td and self._on_telemetry:
                self._on_telemetry(td)
