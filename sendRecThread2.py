import threading
import time
import RPi.GPIO as GPIO
import serial
import struct

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

def readSerial():
    data = ""
    while True:
        x = ser.read()
        data += x
        if x == '\r' or x == '':
            return data

def listenForSyn():
    print("listending for syn")
    global synRec
    global ackRec
    count = 0
    data = []

    #print("listening for syn... {}".format(count))
    count += 1
    try:
        while ser.in_waiting > 0:
            x = (ser.read()).decode()
            if x != ',' and x != '\r' and x != '\n':
                data.append(float(x))
        if len(data) > 0:
            #ser.reset_input_buffer()
            print("received syn: {}".format(data))
            synRec = data[0]
            ackRec = data[1]
            if synRec != 0 and ackRec == 0:
                str = ("{},{}\r\n".format(syn, synRec + 1)).encode()
                ser.write(str)
                aligned = True
            else:
                synRec = 0
                ackRec = 0
        #ser.reset_input_buffer()

    except:
        pass

def listenForAck():
    print("listening for ack")
    global ackRec
    global synRec
    count = 0
    data = []

    #print("Listening for ack.. {}".format(count))
    count += 1
    #print("ser: {}".format(ser.in_waiting))
    try:
        while ser.in_waiting > 0:
            x = (ser.read()).decode()
            if x != ',' and x != '\r' and X != '\n':
                data.append(float(x))
        if len(data) > 0:
            #ser.reset_input_buffer()
            print("received ack and syn: {}".format(data))
            synRec = data[0]
            ackRec = data[1]
            if synRec != 0 and ackRec == syn + 1:
                aligned = True
                print("Aligned!")
            else:
                ackRec = 0
                synRec = 0
        #ser.reset_input_buffer()
    except:
        pass

def sendSyn():

    str = ("{},{}\r\n".format(syn, 0)).encode()
    ser.write(str)
                

def main():
    global synRec
    global ackRec
    global aligned
    global stopThread

    while not aligned:
        synTime = time.time() + synSendTime
        while time.time() < synTime and ackRec == 0:
            # sending syn
            str = ("{},{}\r\n".format(syn, 0)).encode()
            ser.write(str)
            listenTime = time.time() + ackWaitTime
            while time.time() < listenTime and ackRec == 0:
                # listen for ack back
                listenForAck()
        synRec = 0
        ackRec = 0

        #ser.reset_input_buffer()
        tEnd = time.time() + synWaitTime
        while time.time() < tEnd and synRec == 0:
            # listen for syn
            listenForSyn()
    synRec = 0
    ackRec = 0




if __name__ == "__main__":
    resetPin = 18
    syn = 1

    ackWaitTime = 0.5

    synWaitTime = 2
    synSendTime = 2

    stopThread = False
    synRec = ackRec = aligned = 0
    GPIO.setup(resetPin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(resetPin, GPIO.HIGH)

    # create send and receive thread classes and pass corresponding
    # functions to each class
    main()
