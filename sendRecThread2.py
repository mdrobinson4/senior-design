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

def listenForSyn():
    global synRec
    global ackRec

    while True:
        if (ser.in_waiting > 0):
            try:
                data = struct.unpack('>ll', ser.read())
                synRec = data[0]
                ackRec = data[1]
                if synRec != 0 and ackRec == 0:
                    ser.write(struct.pack('>ll', syn, synRec + 1))
                    listenForSyn.join()
                    aligned = True

            except:
                print('error')

def listenForAck():
    global ackRec
    global synRec

    while True:
        if (ser.in_waiting > 0):
            try:
                data = struct.unpack('>ll', ser.read())
                synRec = data[0]
                ackRec = data[1]
                if synRec != 0 and ackRec == syn + 1:
                    aligned = True
                    listenForAckThread.join()
            except:
                print('error')

def main():
    global synRec
    global ackRec
    global aligned

    listenForAckThread = threading.thread(target=listenForAck, daemon=True)
    listenForSynThread = threading.thread(target=listenForSyn, daemon=True)

    while not aligned:
        ser.write(struct.pack('>ll', syn, 0))

        # listen for ack back
        listenForAckThread.start()
        time.sleep(ackWaitTime)
        listenForAckThread.join()

        # listen for syn
        listenForSynThread.start()
        time.sleep(synWaitTime)
        listenForSynThread.join()





if __name__ == "__main__":
    resetPin = 18
    syn = 1
    ackWaitTime = 2
    synRec = ackRec = aligned = 0
    GPIO.setup(resetPin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(resetPin, GPIO.HIGH)

    # create send and receive thread classes and pass corresponding
    # functions to each class
    main()
