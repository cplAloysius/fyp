import asyncio
import sys
from itertools import count, takewhile
from typing import Iterator

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

from numpy import interp
import smbus
import time
from Arm_Lib import Arm_Device

Arm = Arm_Device()
time.sleep(.1)

Arm.Arm_serial_servo_write6(90, 90, 0, 90, 90, 90, 500)
offset = -99999

time.sleep(1)

UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
UART_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
UART_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


# Aligns robotic arm heading with heading of sensor
def get_offset(reading, offset):
    offset_val = (reading + offset) % 360
    if offset_val > 180:
        offset_val -= 360
    
    return offset_val

# TIP: you can get this function and more from the ``more-itertools`` package.
def sliced(data: bytes, n: int) -> Iterator[bytes]:
    """
    Slices *data* into chunks of size *n*. The last slice may be smaller than
    *n*.
    """
    return takewhile(len, (data[i : i + n] for i in count(0, n)))

async def uart_terminal():
    """This is a simple "terminal" program that uses the Nordic Semiconductor
    (nRF) UART service. It reads from stdin and sends each line of data to the
    remote device. Any data received from the device is printed to stdout.
    """

    def match_nus_uuid(device: BLEDevice, adv: AdvertisementData):
        # This assumes that the device includes the UART service UUID in the
        # advertising data. This test may need to be adjusted depending on the
        # actual advertising data supplied by the device.
        if UART_SERVICE_UUID.lower() in adv.service_uuids:
            return True

        return False

    device = await BleakScanner.find_device_by_filter(match_nus_uuid)

    if device is None:
        print("no matching device found, you may need to edit match_nus_uuid().")
        sys.exit(1)

    def handle_disconnect(_: BleakClient):
        print("Device was disconnected, goodbye.")
        # cancelling all tasks effectively ends the program
        for task in asyncio.all_tasks():
            task.cancel()

    def handle_rx(_: BleakGATTCharacteristic, data: bytearray):
        print("received:", data)
        data = data.decode('utf-8')
        data = data.split(",")
        
        roll = float(data[0])
        pitch = float(data[1])
        heading = float(data[2])
        
        global offset
        if (offset == -99999):
            offset = 90 - heading
        
        offset_heading = get_offset(heading, offset)
        heading = offset_heading
        
        if (heading < 0):
            if (heading > -90):
                heading = 0
            else:
                heading = 180
        
        #if (roll+90 > 0):
            #Arm.Arm_serial_servo_write(5, roll+90, 500)
            
        #pitch above 90 degrees, bend servo 3 only    
        if (pitch <= 0):
            Arm.Arm_serial_servo_write6(heading, 90, abs(pitch), 90, 90, 90, 500)
            #Arm.Arm_serial_servo_write(3, abs(pitch), 500)
            #Arm.Arm_serial_servo_write(4, 90, 500)
            #Arm.Arm_serial_servo_write(2, 90, 500)
            
        #pitch below 90 degrees, bend servo 2 and 4, servo 3 stays at 90 degrees
        
        if (pitch > 0):
            ang2 = pitch + 90
            ang4 = 75 - pitch
            if (ang2 > 120):
                ang2 = 120
            if (ang4 < 0):
                ang4 = 0
                ang2 = interp(pitch, [75, 90], [120, 90])
            
            Arm.Arm_serial_servo_write6(heading, ang2, 0, ang4, 90, 90, 500)
        '''
            if (pitch >= 70):
                ang = 90-pitch*1.3
                if (ang < 0): 
                    ang = 0
                Arm.Arm_serial_servo_write(4, ang, 500)
                Arm.Arm_serial_servo_write(2, interp(pitch, [70,90],[118,90]), 500)
        
            if (pitch < 70):
                Arm.Arm_serial_servo_write(4, 90-pitch*1.3, 500)
                Arm.Arm_serial_servo_write(2, 90+pitch/2.5, 500)
        '''
        #if (heading < 180):
         #   Arm.Arm_serial_servo_write(1, heading, 500)
            
        #cmd = data.decode('utf-8')
        #if (cmd == '+'):
        #   print("open")
        #    Arm.Arm_serial_servo_write(arm_id, 50, 500)
        #    time.sleep(1)
        #elif (cmd == '-'):
        #    print("close")
        #    Arm.Arm_serial_servo_write(arm_id, 120, 500)
        #    time.sleep(1)

    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        await client.start_notify(UART_TX_CHAR_UUID, handle_rx)

        print("Connected, start typing and press ENTER...")

        loop = asyncio.get_running_loop()
        nus = client.services.get_service(UART_SERVICE_UUID)
        rx_char = nus.get_characteristic(UART_RX_CHAR_UUID)

        while True:
            # This waits until you type a line and press ENTER.
            # A real terminal program might put stdin in raw mode so that things
            # like CTRL+C get passed to the remote device.
            data = await loop.run_in_executor(None, sys.stdin.buffer.readline)

            # data will be empty on EOF (e.g. CTRL+D on *nix)
            if not data:
                break

            # some devices, like devices running MicroPython, expect Windows
            # line endings (uncomment line below if needed)
            # data = data.replace(b"\n", b"\r\n")

            # Writing without response requires that the data can fit in a
            # single BLE packet. We can use the max_write_without_response_size
            # property to split the data into chunks that will fit.

            for s in sliced(data, rx_char.max_write_without_response_size):
                await client.write_gatt_char(rx_char, s, response=False)

            print("sent:", data)


if __name__ == "__main__":
    try:
        asyncio.run(uart_terminal())
    except asyncio.CancelledError:
        # task is cancelled on disconnect, so we ignore this error
        pass
