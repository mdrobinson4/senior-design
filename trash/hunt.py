import threading
import time
import RPi.GPIO as GPIO
import serial
import discovery
import math
import os
import argparse

parser = argparse.ArgumentParser(description='Scan?')
parser.add_argument('scanFlag', metavar='N', type=int)
args = parser.parse_args()

from dotenv import load_dotenv
project_folder = os.path.expanduser('~/senior-design')  # adjust as appropriate
load_dotenv(os.path.join(project_folder, '.env'))

# NOTE: SEND = 1, LISTEN = 0

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
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

ser.reset_input_buffer() # reset input buffer
ser.reset_output_buffer() # reset input buffer

aligned = False

# listen for synchronous signal
def listenForSyn(op_time, beacon_time, id, disc):
    global aligned
    # time when we will stop listening for syn
    end_time = op_time + time.time()
    beacon_end = beacon_time + time.time()

    # keep listening for syn until we run out of time
    while time.time() < end_time and not aligned:
        flag = disc.checkFront()
        # check to see if we are facingthe front
        while flag == 0 and time.time() < end_time:
            # reset buffers
            flag = disc.checkFront()
        # clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        # exit if we don't have time left
        if time.time() >= end_time:
            return
        # set the read timeout to the handshake time
        if time.time() + beacon_time >= end_time:
            ser.timeout = end_time - time.time()
        else:
            ser.timeout = beacon_time
        if ser.in_waiting > 0:
            x = ser.read(5) # wait for synchronous signal ('hello')
            try:
                # decode data
                data = x.decode()
                if data == 'hello':
                    ser.write(('ack').encode())
                    aligned = True
                    disc.setAligned()
                    print('Aligned!')
                    return
            except:
                pass

# listen for an ack response
def listenForAck(beacon_time, id, disc):
    global aligned
    # at (end_time) we will exit
    end_time = time.time() + beacon_time # when we will leave stop listening for acknowledgement
    if time.time() >= end_time:
        return
    ser.timeout = end_time - time.time() # receive timeout
    if ser.in_waiting > 0:
        x = ser.read(3) # read in ('ack') from other node
        # decode data and see if it is what we were expecting: ('ack')
        try:
            data = x.decode()
            if data == 'ack':
                aligned = True
                disc.setAligned()
                print('Aligned!')
                return
        except:
            pass

# send synchronous signal
def sendSyn(beacon_time, id):
    ser.write(('hello').encode())   # send synchronous signal to other node
    # reset the buffers
    ser.reset_input_buffer()
    ser.reset_output_buffer()


# 2-way handshake
def handshake(disc, id):
    i = 0
    j = 0
    beacon_time = disc.getBeaconTime() # handshake time
    op_time = disc.getPseudoSlotTime() # slot time

    while j < len(id) and not aligned: # loop through pseudo slot sequence until we're aligned
        j = i % len(id)
        slot_end_time = op_time + time.time() # when the slot will end
        if id[j] == '1': # mode 1 -> transmission mode
            disc.changeMode(id[j]) # change mode to transmission -> change angular velocity to 300 deg / sec
            # repeatedly send syn and then listen for ack for op_time seconds
            while time.time() < slot_end_time and not aligned:
                flag = disc.checkFront() # check to see if we are facing front
                # constantly check to see if we facing the front
                while flag == 0 and time.time() < slot_end_time:
                    flag = disc.checkFront()
                # only go through process if we have enough time to get a response
                if slot_end_time - time.time() >= beacon_time:
                    sendSyn(beacon_time, id) # send synchronous signal
                    listenForAck(beacon_time, id, disc) # wait for response
                elif slot_end_time - time.time() > 0:   # sleep for the remaining time
                    time.sleep(slot_end_time - time.time())
        # listen for syn
        elif id[j] == '0':
            # change mode to reception -> affects the angular velocity
            disc.changeMode(id[j])
            # listen for an initial syn
            listenForSyn(op_time, beacon_time, id, disc)
        i += 1
    # the 3d area was fully scanned, but we still failed to find the other node
    if not aligned:
        print('Could not find other node')
        disc.discoveryFailed = True
    return

def genSequence(id):
    seq = ''
    idLen = len(id)
    for i in range(0, math.floor(idLen / 2) + 1):
        seq += '0'
    for i in range(0, math.ceil(idLen / 2)):
        seq += '1'
    return seq

if __name__ == "__main__":
    id = os.getenv('ID')
    seq = generateSeq(id)
