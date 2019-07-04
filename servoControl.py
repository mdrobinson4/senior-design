import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

servoZPin = 3
servoYPin = 2

GPIO.setmode(GPIO.BCM)

GPIO.setup(servoYPin, GPIO.OUT)
GPIO.setup(servoZPin, GPIO.OUT)

servoZ = GPIO.PWM(servoZPin, 50)
servoY = GPIO.PWM(servoYPin, 50)

servoZ.start(7.5)
servoZ.ChangeDutyCycle(7.5)

servoY.start(7.5)
servoY.ChangeDutyCycle(7.5)