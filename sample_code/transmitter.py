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

p2 = GPIO.PWM(servoZPin, 50)
p = GPIO.PWM(servoYPin, 50)

sf = 1

theta = 24
dmax = 1
sqrtof2 = 2**(1/2.0)
n = 180.0/(sqrtof2*theta)
print(n)
nop = 1000
omega = 180
dr = .005

xa = np.zeros(nop)
ya = np.zeros(nop)
za = np.zeros(nop)

r = 0.0
thetaa = np.zeros(nop)
phia = np.zeros(nop)

sa = np.linspace(-n*np.pi, n*np.pi, nop)

global rec
rec = np.zeros(nop)
ang = np.zeros(nop)
step = np.zeros(nop)
global i

for i in range(1,nop):
    xa[i] = dmax*math.cos((sa[i]/(2*n)))*math.sin(sa[i])
    ya[i] = dmax*math.cos((sa[i]/(2*n)))*math.cos(sa[i])
    za[i] = dmax*math.sin((sa[i]/(2*n)))

    r = (xa[i]**2 + ya[i]**2 + za[i]**2)**(0.5)

    thetaa[i] = math.acos(za[i]/r)*(180/np.pi)
    if xa[i]<0 and ya[i]<0:
        phia[i] = 180 - math.atan(ya[i]/xa[i])*(180/np.pi)
        rec[i] = 1
    elif xa[i]>=0 and ya[i]<0:
        phia[i] = - math.atan(ya[i]/xa[i])*(180/np.pi)
        rec[i] = 1
    elif xa[i]>=0 and ya[i]>=0:
        phia[i] = math.atan(ya[i]/xa[i])*(180/np.pi)
        rec[i] = 0
    elif xa[i]<0 and ya[i]>=0:
        phia[i] =  180 + math.atan(ya[i]/xa[i])*(180/np.pi)
        rec[i] = 0


    x = xa[i-1]
    y = ya[i-1]
    z = za[i-1]
    a = array([x, y, z])
    b = array([0, 0, 0])
    x2 = xa[i]
    y2 = ya[i]
    z2 = za[i]
    c = array([x2, y2, z2])
    abNorm = (b-a)/norm(b-a)
    bcNorm = (b-c)/norm(b-c)
    
    
    res = abNorm[0]*bcNorm[0] + abNorm[1]*bcNorm[1] + abNorm[2]*bcNorm[2]
    ang[i] = arccos(res)*180.0/pi
    step[i-1] = ang[i]/omega
    
    step1 = np.zeros(nop)
    
    step1[i-1] = ((arccos(np.dot(a, c) / (np.linalg.norm(a) * np.linalg.norm(c))) ) * (180 / np.pi)) / omega
    
counter = 0
counter2 = 0
flag = 0
flag2 = 0
angle = 0.0
global Master
global Hello
Master = 1
Hello = 1

if sf==1:
    p.start(2.5)
    p2.start(2.5)

def scanxyz(threadName, counter, counter2, flag, flag2):
    global i
    i=0
    global Master
    while True:
            angle_thetaa = float(thetaa[i%1000])/18+2.5
            p.ChangeDutyCycle(angle_thetaa)
            angle_phia = float(phia[i%1000])/18+2.5
            p2.ChangeDutyCycle(angle_phia)
            #print(angle_thetaa, angle_phia)
            time.sleep(step[i%1000])
            i = i + 1
            if i == -10:
                Hello = 0
                Master = 0
                print ("Reached end of scan path. No receiver found. Writing \"100\" to file...")


scanxyz("", counter, counter2, flag, flag2)