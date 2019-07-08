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


class thread(threading.Thread):
    def __init__(self, name, target):
        self._stopevent = threading.Event()
        self._sleepperiod = 0.0
        self.target = target
        threading.Thread.__init__(self, name=name)

    def run(self):
        while not self._stopevent.isSet():
            try:
                self.target()
                #self._stopevent.wait(self._sleepperiod)
            except KeyboardInterrupt:
                sendThread.join()
                receiveThread.join()

        print("%s ends" % (self.getName()))

    def join(self, timeout=None):
        # Stop the thread
        self._stopevent.set()
        threading.Thread.join(self)


def send():
    global aligned
      
    # have not received syn
    if synRec == 0:
        # send syn
        ser.write(struct.pack('>ll', syn, 0))
        # received a syn but not ack
    elif synRec != 0 and ackRec == 0:
        # send syn and ackRec + 1
        ser.write(struct.pack('>ll', syn, synRec + 1))
    # received syn and ack
    elif synRec != 0 and ackRec == syn + 1:
        # send synRec + 1
        print("JOIN")
        ser.write(struct.pack('>ll', synRec + 1, 0))
        aligned = True
        sendThread.join()
        receiveThread.join()
    # received ack
    elif synRec == syn + 1:
        print("JOIN")
        aligned = True
        sendThread.join()
        receiveThread.join()

def receive():
    global synRec
    global ackRec
    
    if (ser.in_waiting > 0):
        try:
            data = ser.read()
            data = struct.unpack('>ll',data)
            print(data)
        except:
            data = 0
            return
    try:
        synRec = data[0]
    except:
        synRec = 0
    try:
        ackRec = data[1]
    except:
        ackRec = 0
    
if __name__ == "__main__":
    resetPin = 18
    syn = 1
    ack = synRec = ackRec = aligned = 0
    GPIO.setup(resetPin, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(resetPin, GPIO.HIGH)
        
    # create send and receive thread classes and pass corresponding
    # functions to each class
        
    sendThread = thread("send", send)
    receiveThread = thread("receive", receive)
        
    sendThread.start()
    receiveThread.start()
    



