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
  theta[i] = math.arcos(z[i] / r)
  # calculate phi the x-axis angle
  phi[i] = math.arsin(y[i] / x[i])
  
  
  




