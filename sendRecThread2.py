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
op_time = 0.0
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

# convert text to bits
def text_to_bits(text, encoding='utf-8', errors='surrogatepass'):
    bits = bin(int.from_bytes(text.encode(encoding, errors), 'big'))[2:]
    return bits.zfill(8 * ((len(bits) + 7) // 8))

# convert bits to text
def text_from_bits(bits, encoding='utf-8', errors='surrogatepass'):
    n = int(bits, 2)
    return n.to_bytes((n.bit_length() + 7) // 8, 'big').decode(encoding, errors) or '\0'

def getBits(str):
    return ''.join(format(ord(x), 'b') for x in str)

# increment string's bits
def incBits(str):
    return text_from_bits(bin(int(text_to_bits(str),2) + int('0001',2))[2:])

# returns the raspberry pi's serial number
# strips the leading zeros
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
    return  cpuserial[8:]

# listens for a syn
def listenForSyn(end_time):
    global aligned
    time_passed = time.time()
    while not aligned and time.time() < end_time:
        data = []
        # how long the serial input will sit waiting for len(
        ser.timeout = end_time - time_passed
        # clear input buffer
        ser.reset_input_buffer()
        x = ser.read(len(syn))
        try:
            # decode data
            data = x.decode()
        except:
            continue
        # make sure we got all of the data
        if len(data) == len(syn):
            # send your id and the other pi's id + 1
            try:
                str = ("{},{}".format(syn, incBits(data))).encode()
            except:
                continue
            ser.write(str)
            aligned = True
            disc.setAligned()
            print('Aligned!')

# listen for an ack response
def listenForAck(end_time):
    global aligned
    time_passed = time.time()
    # continue looping until we run out of time or we are aligned
    while not aligned and time.time() < end_time:
        data = []
        ser.timeout = end_time - time_passed
        # clear input buffer
        ser.reset_input_buffer()
        # read in the pi2's id and pi1 + 1 id
        x = ser.read((len(syn)*2)+1)
        try:
            # decode data
            data = x.decode()
            # convert string to array
            data = data.split(',')
        except:
            continue
        if len(data) == 2:
            # ensure that we got the correct response
            if data[1] == incBits(syn):
                aligned = True
                disc.setAligned()
                print('Aligned!')
        
# send syn
def sendSyn():
    # clear output buffer
    ser.reset_output_buffer()
    str = ("{}".format(syn)).encode()
    ser.write(str)

# 2-way handshake
def handshake():
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
    # get the pi's serial number as text
    syn = getSerial()
    # convert serial number text to bits
    id = text_to_bits(syn)
    # add floor(len(syn)/2) + 1 "0" bits and ceiling(len(syn)/2) "1" bits
    id = "{}000001111".format(id)
    syn = text_from_bits(id)
    
    # breaks threads if true
    aligned = False
    # servo path class
    disc = discovery.Discovery()
    disc.createPath()
    global op_time = dis.handShakeTime / 2
    # servo path and handshake threads
    servoPathThread = threading.Thread(target=disc.scan, daemon=True)
    handshakeThread = threading.Thread(target=handshake, daemon=True)
    servoPathThread.start()
    handshakeThread.start()
