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
)

start_start_time = 0
send_time = 0

while (ser.in_waiting < 0):
  res = ser.read(1)

res = res.decode()
start_time = time.time()
str = '2'.encode()

ser.write(str)
send_time = time.time() - start_time
print('It took [{}] seconds to send [{}]'.format(send_time, str))
