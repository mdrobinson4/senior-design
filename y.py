import threading
import time
import RPi.GPIO as GPIO
import serial
import discovery

# NOTE: SEND = 1, LISTEN = 0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(18, GPIO.HIGH)

# send / send time
op_time = 0.3
# time when we will listen for ack
ack_time = op_time / 10.0

ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

while True:
    if ser.in_waiting > 0:
        try:
            str = ser.read()
            print("{}".format(str.decode()))
        except:
            pass
    ser.reset_input_buffer()
    #else:
     #   print("not in range")