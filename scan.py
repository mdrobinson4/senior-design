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

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)
  
def scan():
  i = 0
  while i < pointCount:
    servoY.ChangeDutyCycle(translate(theta[i], 0, 180, 0, 12.5))
    servoZ.ChangeDutyCycle(translate(phi[i], 0, 180, 0, 12.5))
    time.sleep(step[i])
    i += 1

GPIO.setwarnings(False)

servoZPin = 3
servoYPin = 2

GPIO.setmode(GPIO.BCM)

GPIO.setup(servoYPin, GPIO.OUT)
GPIO.setup(servoZPin, GPIO.OUT)

servoZ = GPIO.PWM(servoZPin, 50)
servoY = GPIO.PWM(servoYPin, 50)

# half angle from the axis of propagation for transmissions. 
# The angle of field-of-view is 2 * beta
beta = 24

# resolution
pointCount = 1000

# steps -> will be used to determine how long the servo will
# be in a certain position. This is needed because the difference
# between two consecutive angles will not be equal. Which is not 
# preferred since we want the angular speed to be constant, for now
steps = np.zeros(pointCount)

# the z-axis angle
theta = np.zeros(pointCount)
# the x-axis angle
phi = np.zeros(pointCount)

# x, y, and z axis points
x = np.zeros(pointCount)
y = np.zeros(pointCount)
z = np.zeros(pointCount)

points = np.linspace(-np.pi, np.pi, pointCount)

# convergence width
convWidth = math.radians(beta) ** (1 / 2)

# number of rotations
n = np.pi / convWidth

for i in range(1, pointCount):
  # calculate the new x, y, and z values
  x[i] = math.cos(s[i] / 2) * math.sin(s[i] * n)
  y[i] = math.sin(s[i] / 2) * math.cos(s[i] *n)
  z[i] = math.sin(s[i] / 2)
  
  # calculate the radius of the sphere
  r = (x[i]**2 + y[i]**2 + z[i]**2)**(1/2)
  
  # calculate theta the z-axis angle
  theta[i] = math.degrees(math.arcos(z[i] / r))
  # calculate phi the x-axis angle
  phi[i] = math.degrees(math.arsin(y[i] / x[i]))
  
  prevVals = np.array([x[i - 1], y[i - 1], z[i - 1]])
  currVals = np.array([x[i], y[i], z[i]])
  
  step[i - 1] = math.arcos(np.dot(prevVals, currVals)) / (np.linalg.norm(prevVals) * np.linalg.norm(currVals))
  step[i - 1] = math.degrees(step[i - 1] ) / beta
scan()
  
  
  
  
  




