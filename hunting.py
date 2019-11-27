import os
import pickle
import math
from dotenv import load_dotenv
import numpy as np
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)

servoTheta = GPIO.PWM(3, 50)
servoPhi = GPIO.PWM(2, 50)
exitThread = False
currMode = '-1'
backFlag = False

load_dotenv()

def main():
    id = os.getenv('id')
    seq = generateSeq(id)
    path = pickle.load(open("path.p", "rb"))
    servoPath(path, seq)

def servoPath(path, seq):
    global backFlag
    (phi, theta, tranWait, recWait) = getPath(path)
    i = 0
    (phiRad, thetaRad) = convertValues(phi[i], theta[i])
    servoPhi.start(phiRad)
    servoTheta.start(thetaRad)
    while not exitThread:
        j = i % len(phi)
        if abs(phi[j] - phi[j-1]) >= 170:
            backFlag = True
            tranSleep = 0.5
            recSleep = 0.5
        else:
            backFlag = False
            tranSleep = tranWait[j]
            recSleep = recWait[j]
        (phiRad, thetaRad) = convertValues(phi[j], theta[j])
        print(phiRad, thetaRad)
        servoPhi.ChangeDutyCycle(phiRad)
        servoTheta.ChangeDutyCycle(thetaRad)
        if currMode == '1':
            time.sleep(tranSleep)
        elif currMode == '0':
            time.sleep(recSleep)
        i += 1
        backFlag = False

def convertValues(phi, theta):
    phi = translate((phi / 18) + 2.5, 2.5, 12.5, 2.2, 11.7)
    theta = translate((theta / 18) + 2.5, 2.5, 12.5, 2.2, 11.7)
    return (phi, theta)


def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def getPath(path):
    phi = path['phi']
    theta = path['theta']
    tranWait = path['tranWait']
    recWait = path['recWait']
    # duplicate path values so it is traversed in reverse
    if id == '1':
        phi = phi + phi[::-1]
        theta = theta + theta[::-1]
        tranWait = tranWait + tranWait[::-1]
        recWait = recWait + recWait[::-1]
    elif id == '0':
        phi = phi[::-1] + phi
        theta = theta[::-1] + theta
        tranWait = tranWait[::-1] + tranWait
        recWait = recWait[::-1] + recWait
    return (phi, theta, tranWait, recWait)

def generateSeq(id):
    seq = ''
    for i in range(0, math.floor(len(id) / 2) + 1):
        seq += '0'
    for i in range(0, math.ceil(len(id) / 2)):
        seq += '1'
    return seq

main()
