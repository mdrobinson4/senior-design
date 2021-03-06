import RPi.GPIO as GPIO
import time
import numpy as np
import math
from numpy import*
from numpy.linalg import norm
import serial
import os

id = os.getenv('ID')

# MODE: 1 -> Transmit || 0 -> Receive

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)


def simplify(num, denom):
    commDenom = math.gcd(num, denom)
    return (num / commDenom, denom / commDenom)

class Discovery:
    def __init__(self):
        # initialize servos
        self.servoZ = GPIO.PWM(2, 50)
        self.servoY= GPIO.PWM(3, 50)

        self.aligned = False
        self.discoveryFailed = False
        self.mode = '0'

        # half angle from the axis of propagation for transmissions.
        # The angle of field-of-view is 2 * beta
        self.fullAngleDiv = 56
        # width of coverage (y)
        self.covWidth = (self.fullAngleDiv / 2) * math.sqrt(2)
        # number of rotations necessary to scan 3d area
        self.n = 180 / self.covWidth
        # transmission angular velocity [ degrees / second ]
        self.wT = int(200)
        # reception angular velocity [ degrees / second ]
        self.wR = int(170)
        # Receiver (p) rounds and transmission (q) rounds
        (self.p, self.q) = simplify(self.wR, self.wT)

        # time we spend in each mode [ each slot ]
        self.pseudo_slot = (2*1.28*self.n*180*self.q) / (self.wT)
        # average handshake time (100 tests)
        self.beacon_time = 0.00842599630355835
        self.send_time = 0.00006556272506713867
        print('handshake time: {}, pseudo slot time: {}'.format(self.beacon_time, self.pseudo_slot))
        print('{} > {}'.format(self.fullAngleDiv*(self.p+self.q), 1.28*self.n*180))

        # resolution
        self.pointCount = 1000
        # steps -> will be used to determine how long the servo will
        # be in a certain position. This is needed because the difference
        # between two consecutive angles will not be equal. Which is not
        # preferred since we want the angular speed to be constant, for now
        self.steps = np.zeros(self.pointCount)
        self.theta = np.zeros(self.pointCount)
        self.phi = np.zeros(self.pointCount)
        self.tranStep = np.zeros(self.pointCount)
        self.recStep = np.zeros(self.pointCount)

        # x, y, and z axis points
        self.x = np.zeros(self.pointCount)
        self.y = np.zeros(self.pointCount)
        self.z = np.zeros(self.pointCount)
        self.status = np.zeros(self.pointCount)
        self.front = 1
        self.step = np.zeros(self.pointCount)
        self.s = np.linspace(-np.pi, np.pi, self.pointCount)

    # sets the path that the servo will take
    def createPath():
        lin = np.linspace(-math.pi, math.pi, num=self.pointCount)
        p = 0
        for i in range(1, self.pointCount):
            p = math.cos(lin[i]/2)
            x[i] = p*math.sin(self.n*lin[i])
            y[i] = p*math.cos(self.n*lin[i])
            z[i] = math.sin(lin[i]/2)
        for i in range(1, self.pointCount):
            r = (self.x[i]**2 + self.y[i]**2 + self.z[i]**2)
            # calculate theta the z-axis angle [ in degrees ]
            self.theta[i] = math.degrees(math.acos(self.z[i] / r))
            self.phi[i] = math.degrees(math.atan(self.y[i]/ self.x[i])) + 90
            # create an array of the previous coordinates
            prev = np.array([self.x[i - 1] or 0, self.y[i - 1] or 0, self.z[i - 1] or 0])
            # create an array of the current coordinates
            curr = np.array([self.x[i], self.y[i], self.z[i]])
            # calculate the change in the angle
            if np.linalg.norm(prev) == 0 or np.linalg.norm(curr) == 0:
                angleChange = 0
            else:
                angleChange = math.degrees(((math.acos(np.dot(prev, curr) / (np.linalg.norm(prev) * np.linalg.norm(curr))))))
            # calculate the amount of time for the servo to rest
            # so we maintain constant transmission and receiving mode speeds
            tranStep[i] = angleChange / self.wT
            recStep[i] = angleChange / self.wR
        # reverse the theta/phi angles and transmission/reception sleep times
        if id == '1':
            self.phi = np.append(self.phi[::-1], self.phi)
            self.theta = np.append(self.theta[::-1], self.theta)
            self.tranStep = np.append(self.tranStep[::-1], self.tranStep)
            self.recStep = np.append(self.recStep[::-1], self.recStep)
        else:
            self.phi = np.append(self.phi, self.phi[::-1])
            self.theta = np.append(self.theta, self.theta[::-1])
            self.tranStep = np.append(self.tranStep, self.tranStep[::-1])
            self.recStep = np.append(self.recStep, self.recStep[::-1])

    def unit_vector(self, vector):
        """ Returns the unit vector of the vector.  """
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        """ Returns the angle in radians between vectors v1 and v2"""
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

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
        j = 0
        while not self.aligned and not self.discoveryFailed:
            rStep = 0
            tStep = 0
            j = i % (self.pointCount*2)
            theta = self.translate((self.theta[j]/18)+2.5, 2.5, 12.5, 0, 15)
            phi = self.translate((self.phi[j]/18)+2.5, 2.5, 12.5, 0, 15)
            if abs(self.phi[j] - self.phi[j-1]) >= 170:
                self.front = 0
            else:
                self.front = 1
            if i == 0:  # if we're just starting
                self.servoY.start(theta)
                self.servoZ.start(phi)
            else:
                try:
                    self.servoY.ChangeDutyCycle(theta)
                    self.servoZ.ChangeDutyCycle(phi)
                except ValueError:
                    print(self.theta[j],theta, self.phi[j],phi)
                    raise
            if self.front == 0:
                k = j - 1
                while (abs(self.phi[k] - self.phi[k-1]) < 170) and k > 0:
                        tStep += self.tranStep[k]
                        rStep += self.recStep[k]
                        k -= 1
                time.sleep(0.5)
            else:
                if self.mode == '1':
                    time.sleep(self.tranStep[j])
                else:
                    time.sleep(self.recStep[j])
            i += 1
            self.front = 1
        if not self.aligned == True:
            GPIO.cleanup()

    def checkFront(self):
        return self.front

    def setAligned(self):
        self.aligned = True

    def changeMode(self, mode):
        self.mode = mode

    def getPseudoSlotTime(self):
        return self.pseudo_slot

    def getBeaconTime(self):
        return self.beacon_time
