import time
import serial
import RPi.GPIO as GPIO    # Import Raspberry Pi GPIO library

# dc=0.520×100=2.5% -> 90

# dc=1.520×100=7.5% -> 0 

# dc=2.520×100=12.5% -> 180



ser = serial.Serial(
    port='/dev/serial0',
        baudrate = 115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
)
counter=0

GPIO.setwarnings(False)    # Ignore warning for now
GPIO.setmode(GPIO.BCM)   # Use physical pin numbering
GPIO.setup(2, GPIO.OUT, initial=GPIO.LOW)   # Set pin 8 to be an output pin and set initial
GPIO.output(2, GPIO.HIGH) # Turn on

# Set up servos
GPIO.setup(3, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

xaxis = GPIO.PWM(3, 50)
yaxis = GPIO.PWM(4, 50)

xaxis.start(7.5)

while 1:
    if (ser.in_waiting > 0):
        i = ser.read(1)
        ser.write(b'%d' % (1))
        
        try:
            
            i = int(i)
            xaxis.ChangeDutyCycle(2.5)
            print ("Received : %d" % i)
        except:
            print("ERROR")
            
        
