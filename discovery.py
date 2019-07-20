import RPi.GPIO as GPIO
import time
import numpy as np
import math
from numpy import*
from numpy.linalg import norm
import serial

# MODE: 1 -> Transmit || 0 -> Receive

GPIO.setup(self.3, GPIO.OUT) # 
GPIO.setup(self.2, GPIO.OUT) # 

def simplifyFraciont(num, denom):
    commDenom = gcd(numer, denom)
    return (num / commDenom, denom / commDenom)

class Discovery:
    def __init__(self):
        # initialize servos
        self.servoZ = GPIO.PWM(3, 50)
        self.servoY = GPIO.PWM(2, 50)
        self.servoZ.start(2.5)
        self.servoY.start(2.5)
        
        self.aligned = False
        self.mode = '1'
        self.handshakeTime = 0


        # half angle from the axis of propagation for transmissions.
        # The angle of field-of-view is 2 * beta
        self.beta = 12
        # width of coverage y
        self.convWidth = math.radians(self.beta) * (2**(1/2))
        # number of rotations [ in radians ]
        self.n = math.pi / self.convWidth
        
        # reception angular velocity [ in degrees ]
        self.wR = 40 
        # transmission angular velocity [ in degrees ]
        self.wT = 30
        # Receiver (p) rounds and transmission (q) rounds
        (self.p, self.q) = simplify(self.wR, self.wT)
        
        # time we spend in each mode
        self.op_time = (2*1.28*self.n*math.pi*self.q) / math.radians(self.wT)
        # amount of time that beacon lasts
        self.beacon_time = ((self.p*math.radians(24)) + (self.q*math.radians(24)) - (1.28*n*math.pi)) / (8*self.q*math.radians(self.wR))
        
        # Check Theorem 1
        print('p*angle(T) + q*angle(R) > 1.28*n*pi')
        print('{} > {}'.format((self.p*math.radians(24))+(self.q*math.radians(24)), 1.28*self.n*math.pi))
       
        
        # resolution
        self.pointCount = 1000
        # steps -> will be used to determine how long the servo will
        # be in a certain position. This is needed because the difference
        # between two consecutive angles will not be equal. Which is not
        # preferred since we want the angular speed to be constant, for now
        self.steps = np.zeros(self.pointCount)
        # the z-axis angle
        self.theta = np.zeros(self.pointCount)
        # the x-axis angle
        self.phi = np.zeros(self.pointCount)

        # x, y, and z axis points
        self.x = np.zeros(self.pointCount)
        self.y = np.zeros(self.pointCount)
        self.z = np.zeros(self.pointCount)
        self.step = np.zeros(self.pointCount)
        self.s = np.linspace(-np.pi, np.pi, self.pointCount)
    
    # sets the path that the servo will take
    def createPath(self):
        for i in range(1, self.pointCount):
            # calculate the new x, y, and z values
            self.x[i] = math.cos(self.s[i] / 2) * math.sin(self.s[i] * self.n)
            self.y[i] = math.cos(self.s[i] / 2) * math.cos(self.s[i] * self.n)
            self.z[i] = math.sin(self.s[i] / 2)
            
            # calculate the radius of the sphere
            r = (self.x[i]**2 + self.y[i]**2 + self.z[i]**2)**(1/2)
            # calculate theta the z-axis angle [ in degrees ]
            self.theta[i] = math.degrees(math.acos(self.z[i] / r))
            # calculate phi the x-axis angle [ in degrees ]
            self.phi[i] = math.degrees(math.atan(self.y[i] / self.x[i]))
            
            # needed since we can only rotate 180 degrees
            # still not fully confident about this part
            if self.x[i]<0 and self.y[i]<0:
                self.phi[i] = 180 - self.phi[i]
            elif self.x[i] >= 0 and self.y[i] < 0:
                self.phi[i] *= -1
            elif self.x[i] < 0 and self.y[i] >= 0:
                self.phi[i] += 180.0
                
            # create an array of the previous coordinates
            prev = np.array([self.x[i - 1] or 0, self.y[i - 1] or 0, self.z[i - 1] or 0])
            # create an array of the current coordinates
            curr = np.array([self.x[i], self.y[i], self.z[i]])
            # calculate the change in the angle
            angleChange = math.degrees(((math.acos(np.dot(prev, curr) / (np.linalg.norm(prev) * np.linalg.norm(curr)))))
            # calculate the amount of time for the servo to rest
            # so we maintain constant transmission and receiving mode speeds
            self.tranStep[i-1] = angleChange[i-1] / self.wT
            self.recStep[i-1] = angleChange[i-1] / self.wR

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
        while not self.aligned:
            j = i % self.pointCount
            # print("theta: {}, phi: {}".format(self.theta[j], self.phi[j]))
            self.servoY.ChangeDutyCycle(self.translate(self.theta[j], 0, 180, 0, 12.5))
            self.servoZ.ChangeDutyCycle(self.translate(self.phi[j], 0, 180, 0, 12.5))
            # necessary since
            if self.mode == '1':
                time.sleep(self.tranStep[i])
            elif self.mode == '0':
                time.sleep(self.recStep[i])
            i += 1
        GPIO.cleanup()

    def setAligned(self):
        self.aligned = True
    
   def changeMode(self, mode):
        self.mode = mode
        
                                   
