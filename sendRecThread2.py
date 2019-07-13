import threading
import time
import RPi.GPIO as GPIO
import serial
import discovery

# NOTE: SEND = 1, LISTEN = 0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
GPIO.output(18, GPIO.HIGH)

# send / send time
op_time = 2
# time when we will listen for ack
ack_time = op_time / 10.0

ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)

def getBits(str):
    return ''.join(format(ord(x), 'b') for x in str)

def incBits(str):
    return bin(int(getBits(str),2) + int('0001',2))[2:]
    

def getSerial():
    # Extract serial from cpuinfo file
    cpuserial = "0000000000000000"
    try:
        f = open('/proc/cpuinfo','r')
        for line in f:
            if line[0:6]=='Serial':
                cpuserial = line[10:26]
        f.close()
    except:
        cpuserial = "ERROR000000000"
        return cpuserial
    return  cpuserial.strip("0")

def listenForSyn(end_time):
    #ser.reset_input_buffer()
    #ser.reset_output_buffer()
    
    ser.timeout = 0
    #ser.read()
    
    global aligned
    time_passed = time.time()
    while not aligned and time.time() < end_time:
        data = []
        ser.timeout = end_time - time_passed
        x = ser.read(len(syn))
        try:
            # decode data
            data = x.decode()
            print('Received: _{}_ in listenForSyn at {}'.format(data, time.time()))
        except:
            continue
        if len(data) == len(syn):
                # send your id and the other pi's id + 1
                try:
                    #y = bin(int(data,2) + int('0001',2)
                    str = ("{},{}".format(syn, incBits(data))).encode()
                except:
                    continue
                ser.write(str)
                print("Sent: _{}_ in listenForSyn at {}".format(str, time.time()))
                aligned = True
                disc.setAligned()
                print('Aligned!')

# end_time <= ack_time
def listenForAck(end_time):
    #ser.reset_input_buffer()
    #ser.reset_output_buffer()
    
    ser.timeout = 0
    ser.read()
    
    global aligned
    time_passed = time.time()
    while not aligned and time.time() < end_time:
        data = []
        ser.timeout = end_time - time_passed
        x = ser.read((len(syn)*2)+1)
        try:
            # decode data
            data = x.decode()
            # convert string to array
            data = data.split(',')
            print('ack: __{}__ at: {}'.format(data, time.time()))
        except:
            continue
        if len(data) == 2:
            print('Received: _{}_ in listenForAck at {}'.format(data, time.time()))
            #data = [y.decode() for y in x]
            print(data[1], bin(int(id,2)+int('001',2))[2:])
            if data[1] == syn:
                aligned = True
                disc.setAligned()
                print('Aligned!')
        

def sendSyn():
    #ser.reset_input_buffer()
    #ser.reset_output_buffer()
    
    str = ("{}".format(syn)).encode()
    ser.write(str)
    #print("sent: {} in sendSyn".format(str.decode()))  

def handshake():
    global synRec
    global ackRec
    global aligned
    global id
    
    i = 0
    while not aligned:
        # send syn
        if id[i % len(id)] == '1':
            end_time = time.time() + op_time
            while time.time() < end_time and not aligned:
                sendSyn()
                # if we will go over the designated operation time after listening to ack
                # decrease the length of time that we listen for ack
                if time.time() + ack_time > end_time:
                    end_ack = time.time() + op_time - end_time
                else:
                    end_ack = ack_time + time.time()
                listenForAck(end_ack)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        # listen for syn
        elif id[i % len(id)] == '0':
            end_time = time.time() + op_time
            listenForSyn(end_time)
        i += 1
    
    
    

if __name__ == "__main__":
    # the designated syn
    syn = getSerial()
    id = getBits(syn)
    ack = bin(int(id,2) + int('0001',2))[2:]
    print(ack, syn)
    
    Rec = ackRec = 0 
    aligned = False
    # servo path class
    
    disc = discovery.Discovery()
    disc.createPath()
    
    servoPathThread = threading.Thread(target=disc.scan, daemon=True)
    handshakeThread = threading.Thread(target=handshake, daemon=True)
    servoPathThread.start()
    handshakeThread.start()
