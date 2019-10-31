import RPi.GPIO as GPIO
import time
import numpy as np
import math
from numpy import*
from numpy.linalg import norm
import serial

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
        self.wT = 300
        # reception angular velocity [ degrees / second ]
        self.wR = 290
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
        self.frontFlag = 1
        self.step = np.zeros(self.pointCount)
        self.s = np.linspace(-np.pi, np.pi, self.pointCount)

    # sets the path that the servo will take
    def createPath(self):
        lin = np.linspace(-math.pi, math.pi, num=self.pointCount)
        p = 0
        for i in range(1, self.pointCount):
            p = math.cos(lin[i]/2)
            self.x[i] = p*math.sin(self.n*lin[i])
            self.y[i] = p*math.cos(self.n*lin[i])
            self.z[i] = math.sin(lin[i]/2)

            # calculate the radius of the sphere
            r = (self.x[i]**2 + self.y[i]**2 + self.z[i]**2)**(1/2)
            # calculate theta the z-axis angle [ in degrees ]
            self.theta[i] = math.degrees(math.acos(self.z[i] / r))
            # calculate phi the x-axis angle [ in degrees ]
            self.phi[i] = math.degrees(math.atan(self.y[i] / self.x[i]))

            # needed since we can only rotate 180 degrees
            if self.x[i] < 0 and self.y[i] < 0:
                self.phi[i] = 180 - self.phi[i]
            elif self.x[i] > 0 and self.y[i] < 0:
                self.phi[i] *= -1
            elif self.x[i] < 0 and self.y[i] > 0:
                self.phi[i] += 180.0
            else:
                self.status[i] = 1  # facing the front, not the back

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
            self.tranStep[i-1] = angleChange / self.wT
            self.recStep[i-1] = angleChange / self.wR
        # reverse the theta/phi angles and transmission/reception sleep times
        self.phi = np.append(self.phi, self.phi[::-1])
        self.theta = np.append(self.theta, self.theta[::-1])
        self.tranStep = np.append(self.tranStep, self.tranStep[::-1])
        self.recStep = np.append(self.recStep, self.recStep[::-1])
        self.status = np.append(self.status, self.status[::-1])

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
            j = i % (self.pointCount*2)
            self.frontFlag = self.getStatus(j) # set the front flag so the handshake code can access it
            theta = self.translate((self.theta[j]/18)+2.5, 2.5, 12.5, 2.2, 11.7) # translate the theta value (z axis)
            phi = self.translate((self.phi[j]/18)+2.5, 2.5, 12.5, 2.2, 11.7) # translate the phi value (x,y axis)
            if i == 0:  # if we're just starting
                self.servoY.start(phi)
                self.servoZ.start(theta)
            else:
                self.servoY.ChangeDutyCycle(theta)
                self.servoZ.ChangeDutyCycle(phi)
            # necessary since
            if self.mode == '1':
                time.sleep(self.tranStep[j])
            elif self.mode == '0':
                time.sleep(self.recStep[j])
            i += 1
        if not self.aligned == True:
            GPIO.cleanup()

    def getStatus(self, pos):
        # return 0 if we are currently in the back
        if self.status[pos] == 0:
            return 0
        # if we are currently in the front, see how long we will remain in the front
        elif self.status[pos] == 1:
            i = pos
            timeTilBack = 0
            # how long will we remain in the front
            while self.status[i] == 1:
                timeTilBack += self.recStep[i]
                i += 1
            # we will remain in the front long enough to complete handshake
            if timeTilBack > self.beacon_time + self.send_time:
                return 1
            else:
                return 0

    def checkFront(self):
        return self.frontFlag

    def setAligned(self):
        self.aligned = True

    def changeMode(self, mode):
        self.mode = mode

    def getPseudoSlotTime(self):
        return self.pseudo_slot

    def getBeaconTime(self):
        return self.beacon_time
