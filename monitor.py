#!/usr/bin/python
import time, serial
import signal, os, signal
from xbee import *

if __name__ == '__main__':

  ser = serial.Serial(
    '/dev/ttyUSB0',
    baudrate=9600,
    interCharTimeout=None
  )

  myXBee = XBee900HP(ser)

  def printMessage(frame):
    if ord(frame[0]) == 0x90:
      print "Message received"
      print frame[12:]

  myXBee.onFrameReceived(printMessage)

  def handler(signum, frame):
    global myXBee
    myXBee.stop()
    listen = False
    print "Ending program execution"

  signal.signal(signal.SIGTERM, handler)
  signal.signal(signal.SIGINT, handler)

  myXBee.start()
