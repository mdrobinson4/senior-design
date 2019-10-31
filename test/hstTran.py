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

send_time = []
total_time = []

for i in range(100):
    start_time = time.time()
    ser.write(('hello').encode())
    send_time.append(time.time() - start_time)
    res = ser.read(3)
    data = res.decode()
    total_time.append(time.time() - start_time)

print('got: {}'.format(data))
print('send time: {}'.format(sum(send_time)/len(send_time)))
print('total handshake time: {}'.format(sum(total_time)/len(total_time)))

