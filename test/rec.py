import time
import RPi.GPIO as GPIO
import serial
import discovery
import math
import os

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(18, GPIO.HIGH)

ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)



while 1:
    try:
        x = ser.read()
        x = data.decode()
        print('{}'.format(x))
    except:
        pass
