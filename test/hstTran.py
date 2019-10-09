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

send_time = 0
total_time = 0
start_time = time.time()

send_time = time.time()
str = ('1').encode()
ser.write(str)
while (ser.in_waiting < 0):
  res = ser.read(1)
res = res.decode()

total_time = time.time() - start_Time

print('It took [{}] seconds to send [{}]'.format(send_time, str))
print('It took [{}} seconds to receive [{}]'.format(total_time, res))
