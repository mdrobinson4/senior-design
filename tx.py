import time
import serial
import RPi.GPIO as GPIO


ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT, initial=GPIO.LOW)

GPIO.output(2, GPIO.HIGH)

i = 0
while 1:
    ser.write(b'%d' % (i))
    if (ser.in_waiting > 0):
        x = ser.read(1)
        #if (int(x) == i):
        print("SENT -> Success", int(i), int(x))
    time.sleep(0.5)
    i += 1
    print("Sent: ", i)