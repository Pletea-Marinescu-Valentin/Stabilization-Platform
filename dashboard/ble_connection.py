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
        """Parse: T,<time_ms>,<mode>,<pitch>,<roll>,<u_p>,<u_r>,<tgt_p>,<tgt_r>,<mp>,<mr>,<mh>,<dist>
        Teensy format: T,%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.1f
        """
        # Strip all whitespace including \r
        line = line.strip()
        # Must start with "T,"
        if not line.startswith("T,"):
            return None
        parts = line[2:].split(",")
        # Need exactly 12 fields after "T,"
        if len(parts) < 12:
            return None
        try:
            return TelemetryData(
                time_ms=int(float(parts[0])),   # float() first handles any trailing whitespace
                mode=int(float(parts[1])),
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
        except (ValueError, IndexError) as e:
            logger.debug(f"TelemetryData parse error on {line!r}: {e}")
            return None


class BLEConnection:
    """Manages BLE connection to HM-10 / BT05 module."""

    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.device: Optional[BLEDevice] = None
        self.connected = False
        self._buffer = ""
        self._uart_char = None   # resolved GATT characteristic object
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
        """Connect to a BLE device using the BLEDevice object directly."""
        self.device = device
        self._uart_char = None
        self.client = BleakClient(device, disconnected_callback=self._on_disconnect)
        await self.client.connect()

        # Resolve the UART characteristic object explicitly (avoids UUID-string lookup failures)
        for service in self.client.services:
            for char in service.characteristics:
                if char.uuid.lower() == HM10_CHAR_UUID.lower():
                    self._uart_char = char
                    break

        if self._uart_char is None:
            await self.client.disconnect()
            raise RuntimeError(
                f"UART characteristic {HM10_CHAR_UUID} not found on {device.name}. "
                "Check that BT05 advertising FFE0 service."
            )

        self.connected = True
        if self._on_connection_changed:
            self._on_connection_changed(True)

        # Subscribe to notifications using the resolved characteristic object
        await self.client.start_notify(self._uart_char, self._notification_handler)
        logger.info(f"Connected to {device.name} ({device.address}), char handle={self._uart_char.handle}")

    async def disconnect(self):
        if self.client and self.connected:
            await self.client.disconnect()
        self.connected = False
        self._uart_char = None
        if self._on_connection_changed:
            self._on_connection_changed(False)

    async def send_mode(self, mode: int):
        """Send controller mode command (1-4) using write-without-response."""
        if not self.connected or not self.client or self._uart_char is None:
            return
        data = str(mode).encode("utf-8")
        # Use write-without-response — BT05 supports it and avoids GATT attribute errors
        await self.client.write_gatt_char(self._uart_char, data, response=False)
        logger.info(f"Sent mode: {mode}")

    def _on_disconnect(self, client: BleakClient):
        self.connected = False
        self._uart_char = None
        logger.info("BLE disconnected")
        if self._on_connection_changed:
            self._on_connection_changed(False)

    def _notification_handler(self, sender, data: bytearray):
        """Handle incoming BLE data with robust fragmentation handling.

        BLE packets can be split arbitrarily — we accumulate in a buffer
        and only process complete lines (terminated by \\n or \\r\\n).
        The last incomplete fragment stays in the buffer for the next packet.
        """
        chunk = data.decode("utf-8", errors="replace")
        logger.info(f"BLE RX [{len(data)}B]: {data.hex(' ')}  text={chunk!r}")
        self._buffer += chunk

        # Split on \n — handles both \n and \r\n (strip() removes \r later)
        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
            # Strip \r\n and spaces — robust against \r\n and \n line endings
            line = line.strip()
            if not line:
                continue

            print("RX LINE:", line)

            td = TelemetryData.from_line(line)
            if td:
                logger.info(f"Parsed OK — mode={td.mode} pitch={td.pitch_deg:.1f} roll={td.roll_deg:.1f} dist={td.distance_mm:.0f}mm")
                if self._on_telemetry:
                    self._on_telemetry(td)
            else:
                print("UNPARSED LINE:", line)
