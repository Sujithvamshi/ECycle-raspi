import RPi.GPIO as GPIO
import smbus

import time

address = 0x48

bus = smbus.SMBus(1)
A0 = 0x43

while True:
    try:
        bus.write_byte(address,A0)

        value = bus.read_byte(address)

        print(value)

        time.sleep(0.1)
    except:
        print("Need to interface")
