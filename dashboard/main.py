import asyncio
from bleak import BleakClient

address = "CC:4D:75:4E:A8:9D"
CHAR_UUID = "00000001-0000-1000-8000-00805f9b34fb"

async def main():
    async with BleakClient(address) as client:
        print("Conectat!")

        def handle(sender, data):
            print("PRIMIT:", data)

        await client.start_notify(CHAR_UUID, handle)

        while True:
            msg = input("Trimite: ")
            await client.write_gatt_char(CHAR_UUID, msg.encode())

asyncio.run(main())