#!/usr/bin/python

import sys
import struct
import SFClient
from threading import *

AM_TYPE=12

class DataThread(Thread):
  def __init__(self):
    Thread.__init__(self)

  def run(self):
    sfc = SFClient.SFClient(sys.argv[1], int(sys.argv[2]))
    while True:
      data = sfc.read().data
      #check length and AM_TYPE
      if (len(data) > 6) and (data[6]==AM_TYPE):
        data = data[7:]
        matrix = [struct.unpack(">b",chr(c))[0] for c in data]
        print matrix


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print "usage: Listener.py <host> <port>"
        sys.exit(1)

    dataThread = DataThread()
    dataThread.start()

