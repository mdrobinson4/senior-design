import time
import RPi.GPIO as GPIO
import serial

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
    timeout=None,
    write_timeout=None
)

start_start_time = 0
send_time = 0

#while (ser.in_waiting < 1):
res = ser.read()
data = res.decode()

start_time = time.time()
time.sleep(.1)
ser.write(b'2')

send_time = time.time() - start_time
print('It took [{}] seconds to send 2'.format(send_time))

