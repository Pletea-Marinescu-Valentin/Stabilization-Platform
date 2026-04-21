"""
BLE Scanner utility — lists all nearby BLE devices and their services.
Run this to find the address of your HM-10 / BT4.0 module before using the dashboard.

Usage:
    python dashboard/ble_scan.py
"""
import asyncio
from bleak import BleakScanner, BleakClient


async def main():
    print("\nScanning for BLE devices (5 seconds)...\n")
    devices = await BleakScanner.discover(timeout=5.0)

    if not devices:
        print("No BLE devices found.")
        return

    print(f"{'#':<4} {'Name':<22} {'Address':<20}")
    print("-" * 50)
    for i, d in enumerate(devices):
        print(f"{i:<4} {(d.name or 'Unknown'):<22} {d.address:<20}")

    print()
    choice = input("Enter number to inspect services (or Enter to skip): ").strip()
    if not choice.isdigit():
        return

    idx = int(choice)
    if idx < 0 or idx >= len(devices):
        print("Invalid choice.")
        return

    dev = devices[idx]
    print(f"\nConnecting to {dev.name} [{dev.address}]...")

    try:
        async with BleakClient(dev, timeout=10.0) as client:
            print(f"Connected: {client.is_connected}\n")
            print("Services & Characteristics:")
            print("-" * 60)
            for service in client.services:
                print(f"  SERVICE  {service.uuid}  — {service.description}")
                for char in service.characteristics:
                    props = ", ".join(char.properties)
                    print(f"    CHAR   {char.uuid}  [{props}]")
                    print(f"           {char.description}")
    except Exception as e:
        print(f"Failed to connect: {e}")


if __name__ == "__main__":
    asyncio.run(main())
