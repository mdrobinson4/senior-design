import time
import RPi.GPIO as GPIO
import serial


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
    timeout=None
)

send_time = 0
total_time = 0

start_time = time.time()
str = ('1').encode()
ser.write(str)
send_time = time.time() - start_time

res = ser.read()
data = res.decode()

total_time = time.time() - start_time

print('It took [{}] seconds to send [{}]'.format(send_time, str))
print('It took [{}] seconds to receive [{}]'.format(total_time, data))
