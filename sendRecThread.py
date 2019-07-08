import threading
import time


class thread(threading.Thread):
    def __init__(self, name, target):
        self._stopevent = threading.Event()
        self._sleepperiod = 0.0
        self.target = target
        threading.Thread.__init__(self, name=name)

    def run(self):
        while not self._stopevent.isSet():
            print("loop %d" % (count))
            self.target()
            # self._stopevent.wait(self._sleepperiod)

        print("%s ends" % (self.getName()))

    def join(self, timeout=None):
        # Stop the thread
        self._stopevent.set()
        threading.Thread.join(self, timeout)

if __name__ == "__main__":
    send = thread("send")
    receive = thread("receive")
    
    send.start()
    receive.start()


def send():
  global synRec
  global ackRec
  
  if synRec == 0:
    # send syn
    ser.write(struct.pack('l', syn))
  elif synRec != 0 and ackRec == 0:
    # send syn and ackRec + 1
    ser.write(struct.pack('ll', syn, ackRec + 1))
  elif synRec != 0 and ackRec == syn + 1:
    # send synRec
    ser.write(struct.pack('l', synRec))
  elif synRec == syn + 1:
    aligned = True
    send.join()
    receive.join()

def receive():
  global synRec
  global ackRec
  global aligned

  data = ser.read()
  data = struct.unpack(data)
  try:
    synRec = data[0]
  except:
    synRec = 0
  try:
    ackRec = data[1]
  except:
    ackRec = 0
