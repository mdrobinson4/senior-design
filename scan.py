import RPi.GPIO as GPIO
import threading
import time
import serial
import numpy as np
import math
import sys
from numpy import*
from numpy.linalg import norm
from socket import*

GPIO.setwarnings(False)

servoZPin = 3
servoYPin = 2

GPIO.setmode(GPIO.BCM)

GPIO.setup(servoYPin, GPIO.OUT)
GPIO.setup(servoZPin, GPIO.OUT)

servoZ = GPIO.PWM(servoZPin, 50)
servoY = GPIO.PWM(servoYPin, 50)

# half angle from the axis of propagation for transmissions. 
#The angle of field-of-view is 2 * beta

beta = 
