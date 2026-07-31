import asyncio
import logging
from dataclasses import dataclass
from typing import Optional, Callable

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice

logger = logging.getLogger(__name__)

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
        line = line.strip()
        if not line.startswith("T,"):
            return None
        parts = line[2:].split(",")
        if len(parts) < 12:
            return None
        try:
            return TelemetryData(
                time_ms=int(float(parts[0])),
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

    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.device: Optional[BLEDevice] = None
        self.connected = False
        self._buffer = ""
        self._uart_char = None
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
        devices = await BleakScanner.discover(timeout=timeout)
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
        self.device = device
        self._uart_char = None
        self.client = BleakClient(device, disconnected_callback=self._on_disconnect)
        await self.client.connect()

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
        if not self.connected or not self.client or self._uart_char is None:
            return
        data = str(mode).encode("utf-8")
        await self.client.write_gatt_char(self._uart_char, data, response=False)
        logger.info(f"Sent mode: {mode}")

    def _on_disconnect(self, client: BleakClient):
        self.connected = False
        self._uart_char = None
        logger.info("BLE disconnected")
        if self._on_connection_changed:
            self._on_connection_changed(False)

    def _notification_handler(self, sender, data: bytearray):
        chunk = data.decode("utf-8", errors="replace")
        logger.info(f"BLE RX [{len(data)}B]: {data.hex(' ')}  text={chunk!r}")
        self._buffer += chunk

        while "\n" in self._buffer:
            line, self._buffer = self._buffer.split("\n", 1)
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
