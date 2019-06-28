import time
import serial


ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

i = 0
while 1:
    if i > 9:
        i = 0
    ser.write(b'%d' % (i))

    time.sleep(0.05)
    i += 1
    if(ser.in_waiting > 0):
        x = ser.read(1)
        print ("Sent : %d || Received : %d" % int(i), int(x))
