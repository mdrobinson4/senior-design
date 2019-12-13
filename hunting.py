import os
import pickle
import math
from dotenv import load_dotenv
import numpy as np
import time
import RPi.GPIO as GPIO
import serial
import threading

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(2, GPIO.OUT)
GPIO.setup(3, GPIO.OUT)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(18, GPIO.HIGH)

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

load_dotenv()
id = os.getenv('id')

servoTheta = GPIO.PWM(3, 50)
servoPhi = GPIO.PWM(2, 50)
exitThread = False
currMode = str(id)
backFlag = True


def main():
    try:
        start = time.time()
        seq = generateSeq(id)
        print('mode sequence: {}'.format(seq))
        path = pickle.load(open("path.p", "rb"))
        servoThread = threading.Thread(target=servoPath, args=(path, seq))
        handshakeThread = threading.Thread(target=handshake, args=(path, seq))
        handshakeThread.start()
        servoThread.start()
        servoThread.join()
        handshakeThread.join()
        end = time.time()
        print(start,"\n",end,"\n",end-start)
    except KeyboardInterrupt:
        ser.close()
        raise

def handshake(path, seq):
    try:
        global currMode
        print(ser.in_waiting)
        ser.read(ser.in_waiting)
        ser.reset_input_buffer() # reset input buffer
        ser.reset_output_buffer() # reset input buffer
        ackWaitTime = 2*path['ackWait']
        slotTime = path['slotTime']
        #for mode in seq
        i = 0
        while i < len(seq) and not exitThread:
            mode = seq[0]
            print('current mode: {}'.format(mode))
            slotEndTime = slotTime + time.time()
            currMode = mode
            if currMode == '1':
                while time.time() < slotEndTime and not exitThread:
                    checkBackFlag(slotEndTime)  # see if we are traversing the back
                    if slotEndTime - time.time() >= ackWaitTime:
                        sendSyn()
                        listenForAck(ackWaitTime)
                    elif slotEndTime - time.time() < ackWaitTime and (slotEndTime - time.time()) > 0:
                        time.sleep(slotEndTime - time.time())
            elif currMode == '0':
                listenForSyn(slotEndTime, ackWaitTime)
            i += 1
    except KeyboardInterrupt:
        ser.close()

def sendSyn():
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(('hello').encode())

def listenForAck(ackWaitTime):
    global exitThread
    endTime = ackWaitTime + time.time()
    timeout = endTime - time.time()
    while endTime > time.time() and not exitThread:
        ser.timeout = timeout#endTime - time.time()
        x = ser.read(3)
        try:
            data = x.decode()
            if data == 'ack':
                exitThread = True
                print('Aligned!')
                return
        except:
            #print(x)
            ser.reset_output_buffer()


def listenForSyn(slotEndTime, ackWaitTime):
    global exitThread
    while time.time() < slotEndTime:
        checkBackFlag(slotEndTime)
        if time.time() >= slotEndTime:
            return
        if time.time() + ackWaitTime >= slotEndTime:
            ser.timeout = slotEndTime - time.time()
        else:
            ser.timeout = ackWaitTime
        x = ser.read(5)
        try:
            data = x.decode()
            if data == 'hello':
                for i in range(100):
                    ser.write(('ack').encode())
                    #if listenForSynAck(ackWaitTime) == True:
                        #break
                    #time.sleep(0.001)
                exitThread = True
                print('Aligned!')
                return
        except:
            pass


def checkBackFlag(slotEndTime):
    #return
    while backFlag == True and time.time() < slotEndTime:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

def servoPath(path, seq):
    global backFlag
    (phi, theta, tranWait, recWait) = getPath(path)
    i = 1
    (phiRad, thetaRad) = convertValues(phi[0], theta[0])
    servoPhi.start(phiRad)
    servoTheta.start(thetaRad)
    while not exitThread:
        j = i % len(phi)
        (phiRad, thetaRad) = convertValues(phi[j], theta[j])
        backFlag = False
        #print('[phi: {}, theta: {}]\n\n\n\n'.format(phiRad, thetaRad))
        #print((phi[i]/18) + 2.5)
        servoPhi.ChangeDutyCycle(phiRad)
        servoTheta.ChangeDutyCycle(thetaRad)
        if currMode == '1':
            time.sleep(tranWait[j])
        else:
            time.sleep(tranWait[j])
        i += 1
        
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
    seq = str(id)
    for i in range(0, math.floor(len(id) / 2) + 1):
        seq += '0'
    for i in range(0, math.ceil(len(id) / 2)):
        seq += '1'
    return seq
    
if __name__ == "__main__":
    main()
