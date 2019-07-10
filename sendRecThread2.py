import threading
import time
import RPi.GPIO as GPIO
import serial
import discovery

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(18, GPIO.HIGH)

ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

def listenForSyn():
    global synRec
    global ackRec
    global aligned
    data = []
    try:
        while ser.in_waiting > 0:
            x = (ser.read()).decode()
            if x != ',' and x != '\r' and x != '\n':
                data.append(int(x))
        if len(data) > 0:
            print("received syn: {}".format(data))
            synRec = data[0]
            ackRec = data[1]
            if synRec == syn and ackRec == 0:
                str = ("{},{}\r\n".format(syn, synRec + 1)).encode()
                ser.write(str)
                print("just sent: {}".format(str.decode()))
                aligned = True
                print("Aligned!")
                disc.setAligned()
            else:
                synRec = 0
                ackRec = 0
    except:
        pass

def listenForAck():
    global ackRec
    global synRec
    global aligned
    data = []
    try:
        while ser.in_waiting > 0:
            x = (ser.read()).decode()
            if x != ',' and x != '\r' and x != '\n' and x != ' ':
                data.append(int(x))
        if len(data) > 0:
            print("just received ack and syn: {}".format(data))
            synRec = data[0]
            ackRec = data[1]
            if synRec == syn and ackRec == syn + 1:
                aligned = True
                print("Aligned!")
                disc.setAligned()
            else:
                ackRec = 0
                synRec = 0
    except:
        pass

def sendSyn():
    str = ("{},{}\r\n".format(syn, 0)).encode()
    ser.write(str)
    print("just sent: {}".format(str.decode()))  

def handshake():
    global synRec
    global ackRec
    global aligned

    while not aligned:
        sendTime = time.time() + opTime
        while time.time() < sendTime and ackRec == 0:
            sendSyn()

            listenTime = time.time() + ackWaitTime
            while time.time() < listenTime and time.time() < sendTime and ackRec == 0:
                listenForAck()
        synRec = 0
        ackRec = 0

        listenTime = time.time() + opTime
        while time.time() < listenTime and synRec == 0:
            listenForSyn()
    synRec = 0
    ackRec = 0
    
    

if __name__ == "__main__":
    syn = 1
    # time to listen for ack after sending syn
    ackWaitTime = 0.2
    # total time to send/receive
    opTime = 0.5
    synRec = ackRec = 0 
    aligned = False
    # servo path class
    
    disc = discovery.Discovery()
    disc.createPath()
    
    servoPathThread = threading.Thread(target=disc.scan, daemon=True)
    handshakeThread = threading.Thread(target=handshake, daemon=True)
    servoPathThread.start()
    handshakeThread.start()
