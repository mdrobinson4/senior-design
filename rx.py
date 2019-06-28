import time
import serial
import RPi.GPIO as GPIO    # Import Raspberry Pi GPIO library

ser = serial.Serial(
	port='/dev/serial0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
)
counter=0

GPIO.setwarnings(False)    # Ignore warning for now
GPIO.setmode(GPIO.BOARD)   # Use physical pin numbering
GPIO.setup(8, GPIO.OUT, initial=GPIO.LOW)   # Set pin 8 to be an output pin and set initial

while 1:
    if (ser.in_waiting > 0):
        i = ser.read(1)
        GPIO.output(8, GPIO.HIGH) # Turn on
        try:
            i = int(i)
            print ("Received : %d" % i)
            i += 1
            print ("Senging : %d" % i)
            ser.write(b'%d' % (i))
        except:
            print("ERROR")
    else:
        GPIO.output(8, GPIO.LOW) # Turn off
        
