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

sf = 1

theta = 24
dmax = 3
sqrtof2 = 2**(1/2.0)
n = 180.0/(sqrtof2*theta)
nop = 1000
omega = 180
dr = .005

receiverName = '192.168.8.20'
if len(sys.argv) > 1:
    x = 12000 + int(sys.argv[1])
    receiverPort = x
    print "receiverPort = " + str(x)
else:
    receiverPort = 12000
    print "receiverPort = " + str(12000)
xa = np.zeros(nop)
ya = np.zeros(nop)
za = np.zeros(nop)

r = 0.0
thetaa = np.zeros(nop)
phia = np.zeros(nop)

sa = np.linspace(-n*np.pi, n*np.pi, nop)

xa[0] = dmax*math.cos((sa[0]/(2*n)))*math.sin(sa[0])
ya[0] = dmax*math.cos((sa[0]/(2*n)))*math.cos(sa[0])
za[0] = dmax*math.sin((sa[0]/(2*n)))
phia[0] = 180 - math.atan(ya[0]/xa[0])*(180/np.pi)
#thetaa[0] = 180

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



GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
    
counter = 0
counter2 = 0
flag = 0
flag2 = 0
angle = 0.0
global Master
global Hello
Master = 1
Hello = 1

port1 = serial.Serial("/dev/ttyUSB1",baudrate=115200,timeout = 3.0)
port2 = serial.Serial("/dev/ttyUSB0",baudrate=115200,timeout = 3.0)

p2 = GPIO.PWM(18, 50)
p = GPIO.PWM(23, 50)

if sf==1:
    p.start(2.5)
    p2.start(2.5)

def readlineCR(port):
    rv = ""
    while True:
        ch = port.read()
        #print ch
        rv += ch
        if ch=='\r' or ch=='':
            return rv

class myThreadtransmit(threading.Thread):

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        print "Starting " + self.name
        transmit(self.name)

class myThreadreceive(threading.Thread):

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        print "Starting " + self.name
        receive(self.name)

class myThreadscanxyz(threading.Thread):

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        print "Starting " + self.name
        scanxyz(self.name, counter, counter2, flag, flag2)

def transmit(threadName):
    #print "ThreadT: Sending \"Hello\"..."
    while Hello:
        global i
        port1.write("Hello\r\n")
        time.sleep(dr)
    #print "ThreadT: Writing \"ACK\"..."
    port1.write("ack\r\n")
    for i in range(1,10):
        port1.write("Ack\r\n")
        time.sleep(dr)
    print "ThreadT: Finished thread: " + threadName

def receive(threadName):
    port2.flushInput()
    global Master
    print r'ThreadR: Receiving (waiting for "B-ACK")...'
    while Master:
        rcv = readlineCR(port2)
        if rcv == '':
            pass        
        elif "B-ACK" in rcv and rec[i]==1:
                print "ThreadR: Received " + str(rcv) 
                global Hello
                global Master
                Hello = 0
                Master = 0
                t2 = time.time()
                dt = t2 - t1
                print "Time spent: " + str(dt) + "; writing to file..."
                fh = open("SSSanity_Omega"+(str(omega))+"_Transmitter10.csv","a")
                fh.write(str(dt))
                fh.write("\n")
                fh.close()
		sys.exit()		
        else:
            pass
    thread1.stop()
    thread2.stop()
    if sf==1:
        thread3.stop()
    GPIO.cleanup()

def scanxyz(threadName, counter, counter2, flag, flag2):

    global i
    i=0
    global Master
    while Master:
            angle_thetaa = float(thetaa[i])/18+2.5
            p.ChangeDutyCycle(angle_thetaa)
            angle_phia = float(phia[i])/18+2.5
            p2.ChangeDutyCycle(angle_phia)
            time.sleep(step[i])
            i = i + 1
            if i == nop:
##                global i
##                i=0
                global Hello
                global Master
                Hello = 0
                Master = 0
		print "Reached end of scan path. No receiver found. Writing \"100\" to file..."
                fh = open("SSSanity_Omega"+(str(omega))+"_Transmitter10.csv","a")
                fh.write("100")
                fh.write("\n")
                fh.close()
     
    
try:
    senderSocket = socket(AF_INET, SOCK_DGRAM)

    senderSocket.sendto('Hello', (receiverName, receiverPort))
    #message, receiverAddress = senderSocket.recvfrom(2048)
    #print message
    #if message == 'B-ACK':
    #    print 'B-ACK received and sending ACK'
        #senderSocket.sendto('ACK', (receiverName, receiverPort))
    #print 'Starting 3D scanning'
    print senderSocket.gettimeout()
    senderSocket.close()

    
    t1 = time.time()
    thread1 = myThreadtransmit(1, "Thread-transmit")
    thread2 = myThreadreceive(2, "Thread-receive")
    thread3 = myThreadscanxyz(3, "Thread-scanxyz")


    thread1.start()
    thread2.start()
    if sf==1:
        thread3.start()

    
except KeyboardInterrupt:
    thread1.stop()
    thread2.stop()
    if sf==1:
        thread3.stop()
    GPIO.cleanup()
