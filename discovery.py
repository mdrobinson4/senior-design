import RPi.GPIO as GPIO
import time
import numpy as np
import math
from numpy import*
from numpy.linalg import norm
import serial

class discovery:
    def _init__(self):
        GPIO.setup(servoYPin, GPIO.OUT)
        GPIO.setup(servoZPin, GPIO.OUT)
        self.algned = False
        self.servoZPin = 3
        self.servoYPin = 2
        # half angle from the axis of propagation for transmissions.
        # The angle of field-of-view is 2 * beta
        self.beta = 24
        self.omega = 180.0
        # resolution
        self.pointCount = 1000
        # steps -> will be used to determine how long the servo will
        # be in a certain position. This is needed because the difference
        # between two consecutive angles will not be equal. Which is not
        # preferred since we want the angular speed to be constant, for now
        self.steps = np.zeros(pointCount)
        # the z-axis angle
        self.theta = np.zeros(pointCount)
        # the x-axis angle
        self.phi = np.zeros(pointCount)
        # x, y, and z axis points
        self.x = np.zeros(pointCount)
        self.y = np.zeros(pointCount)
        self.z = np.zeros(pointCount)
        self.step = np.zeros(pointCount)
        self.s = np.linspace(-np.pi, np.pi, pointCount)
        # convergence width
        self.convWidth = math.radians(beta) * (2**(1/2))
        # number of rotations
        self.n = np.pi / convWidth

        self.servoZ = GPIO.PWM(servoZPin, 50)
        self.servoY = GPIO.PWM(servoYPin, 50)
        GPIO.output(resetPin, GPIO.HIGH)
        self.servoZ.start(7.5)
        self.servoY.start(7.5)
        self.createPath()

        def translate(self, value, leftMin, leftMax, rightMin, rightMax):
            # Figure out how 'wide' each range is
            leftSpan = leftMax - leftMin
            rightSpan = rightMax - rightMin
            # Convert the left range into a 0-1 range (float)
            valueScaled = float(value - leftMin) / float(leftSpan)
            # Convert the 0-1 range into a value in the right range.
            return rightMin + (valueScaled * rightSpan)

        def scan(self):
            i = 0
            while not self.aligned:
                self.servoY.ChangeDutyCycle(translate(self.theta[i], 0, 180, 0, 12.5))
                self.servoZ.ChangeDutyCycle(translate(self.phi[i], 0, 180, 0, 12.5))
                time.sleep(self.step[i])
                i += 1

        def createPath(self):
            for i in range(1, pointCount):
                # calculate the new x, y, and z values
                self.x[i] = 3*math.cos(self.s[i] / 2) * math.sin(self.s[i] * self.n)
                self.y[i] = 3*math.cos(self.s[i] / 2) * math.cos(self.s[i] * self.n)
                self.z[i] = 3*math.sin(self.s[i] / 2)
                # calculate the radius of the sphere
                r = (self.x[i]**2 + self.y[i]**2 + sef.z[i]**2)**(1/2)
                # calculate theta the z-axis angle
                self.theta[i] = math.degrees(math.acos(self.z[i] / r))
                # calculate phi the x-axis angle
                self.phi[i] = math.degrees(math.atan(self.y[i] / self.x[i]))
                # needed since we can only rotate 180 degrees
                # still not fully confident about this part
                if self.x[i]<0 and self.y[i]<0:
                    self.phi[i] = 180 - self.phi[i]
                elif self.x[i]>=0 and self.y[i]<0:
                    self.phi[i] = - self.phi[i]
                elif self.x[i]<0 and self.y[i]>=0:
                    self.phi[i] =  180 + self.phi[i]

                prevVals = np.array([self.x[i - 1], self.y[i - 1], self.z[i - 1]])
                currVals = np.array([self.x[i], self.y[i], self.z[i]])

                self.step[i-1] = arccos(np.dot(prevVals, currVals) / (np.linalg.norm(prevVals) * np.linalg.norm(currVals)))
                self.step[i-1] = math.degrees(self.step[i-1] ) / self.omega

            def setAligned(self):
                self.aligned = True

            def checkAligned(self):
                return self.aligned
