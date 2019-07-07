import threading


class TestThread(threading.Thread):
    def __init__(self, name="TestThread"):
        print("HERE")
        """ constructor, setting initial variables """
        self._stopevent = threading.Event()
        self._sleepperiod = 1.0

        threading.Thread.__init__(self, name=name)

    def run(self):
        print("%s starts" % (self.getName()))

        count = 0
        while not self._stopevent.isSet():
            count += 1
            print("loop %d" % (count))
            self._stopevent.wait(self._sleepperiod)

        print("%s ends" % (self.getName()))

    def join(self, timeout=None):
        """ Stop the thread. """
        self._stopevent.set()
        threading.Thread.join(self, timeout)

if __name__ == "__main__":
    testthread = TestThread()
    testthread.start()

    testthread1 = TestThread()
    testthread1.start()

    import time

    time.sleep(10.0)

    testthread.join()


def send():
  global synRec
  global ackRec
  global aligned = False

  while not aligned:
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

def receive():
  global synRec
  global ackRec
  global aligned

  while not aligned:
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
