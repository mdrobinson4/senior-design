import threading
import time
import RPi.GPIO as GPIO
import serial
import discovery

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(18, GPIO.HIGH)

op_time = 0.5
ack_time = op_time / 3

ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

def listenForSyn(end_time):
    global aligned
    time_passed = time.time()
    while not aligned and time.time() < end_time:
        data = []
        print('listening for syn')
        ser.timeout = end_time - time_passed
        x = ser.read(1)
        data = [y.decode() for y in x]
        if len(data) == 1:
            if data[0] == syn:
                str = ("{},{}".format(syn, data[0] + 1)).encode()
                ser.write(str)
                aligned = True
                disc.setAligned()
                print('Aligned!')

# end_time <= ack_time
def listenForAck(end_time):
    global aligned
    time_passed = time.time()
    while not aligned and time.time() < end_time:
        data = []
        print('Listening for ack')
        ser.timeout = end_time - time_passed
        x = ser.read(2)
        data = [y.decode() for y in x]
        if len(data) == 2:
            if data[0] == syn and data[1] == syn + 1:
                aligned = True
                disc.setAligned()
                print('Aligned!')
        

def sendSyn():
    str = ("{},{}".format(syn, 0)).encode()
    ser.write(str)
    #print("just sent: {}".format(str.decode()))  

def handshake():
    global synRec
    global ackRec
    global aligned

    while not aligned:
        end_time = time.time() + op_time
        while time.time() < end_time:
            sendSyn()
            # if we will go over the designated operation time after listening to ack
            # decrease the length of time that we listen for ack
            if time.time() + ack_time > end_time:
                end_ack = time.time() + op_time - end_time
            else:
                end_ack = ack_time
            listenForAck(end_ack)
        
        listenForSyn(end_time)
    
    
    

if __name__ == "__main__":
    # the designated syn
    syn = 1
    synRec = ackRec = 0 
    aligned = False
    # servo path class
    
    disc = discovery.Discovery()
    disc.createPath()
    
    servoPathThread = threading.Thread(target=disc.scan, daemon=True)
    handshakeThread = threading.Thread(target=handshake, daemon=True)
    servoPathThread.start()
    handshakeThread.start()
