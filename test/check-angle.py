import RPi.GPIO as GPIO
import re

GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)

servoz = GPIO.PWM(2, 50)
servoy = GPIO.PWM(3, 50)
servoz.ChangeDutyCycle(0)
servoy.ChangeDutyCycle(0)
servoz.start(0)
servoy.start(0)
try:
    while(1):
        inp = input("angle1 angle2\n")
        print(inp)
        angles = re.match(r'(\d+(\.\d+)?)\s+(\d+(\.\d+)?)', inp)
        print(angles.groups())
        servoz.ChangeDutyCycle(((float(angles.group(1)))))
        servoy.ChangeDutyCycle(((float(angles.group(3)))))
except KeyboardInterrupt:
    servoz.ChangeDutyCycle((2.5))
    servoy.ChangeDutyCycle((2.5))
servoz.stop()
servoy.stop()
GPIO.cleanup()
