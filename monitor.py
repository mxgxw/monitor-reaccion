#!/usr/bin/python
import time, serial
import signal, os, sys

millis = lambda: int(round(time.time()*1000))

class ModemStatus():
  WAIT_MODEM = 1
  COMMAND_MODE = 2
  API_MODE2 = 3

class ReadStatus():
  RSP_WAIT = 1
  RSP_LMSB = 2
  RSP_LLSB = 3
  RSP_RDDATA = 4

class XBee900HP:

  buff = ''

  HardSerial = None

  WAIT_TIMEOUT = 10000

  seq = 0
  rcvSize = 0
  rcvBuffer = None

  xBeeStatus = ModemStatus.WAIT_MODEM
  buffer = ""
  buffPos = 0

  d = 0
  
  responseFound = None

  lMSB = 0
  lLSB = 0

  packetSize = 0
  checkSum = 0
  
  lastData = 0
  lastSerialData = 0

  readStatus  = ReadStatus.RSP_WAIT

  waitForInit = True

  confirmATAP2 = False
  confirmATCN = False

  continueListening = True

  escapeNext = False

  def __init__(self, serialPort):
    self.HardSerial = serialPort

  def start(self):
    self.HardSerial.open()
    self.init()
    print "Modem in API Mode 2. Listening for messages."
    while self.continueListening:
      self.listen()
    self.HardSerial.close()    
 
  def stop(self):
    continueListening = False
    
  def init(self):
    while self.waitForInit:
      if self.xBeeStatus == ModemStatus.WAIT_MODEM:
        print 'Entering command mode.'
        self.HardSerial.write("+++");
        if self.waitFor("OK"):
          print 'Now in command mode, changing to API MODE 2'
          self.xBeeStatus = ModemStatus.COMMAND_MODE
      elif self.xBeeStatus == ModemStatus.COMMAND_MODE:
        self.HardSerial.write("ATAP2\r\n")
        confirmATAP2 = self.waitFor("OK")
        self.HardSerial.write("ATCN\r\n")
        confirmATCN = self.waitFor("OK")
        if confirmATAP2 and confirmATCN:
          self.waitForInit = False
          self.xBeeStatus = ModemStatus.API_MODE2
          print "Modem detected, switched to API Mode 2."
        else:
          self.xBeeStatus = ModemStatus.WAIT_MODEM
          print "Modem not detected, keep waiting for modem."

  def sendTo64(self, addr_high, addr_low, data):
    if self.xBeeStatus != ModemStatus.API_MODE2:
      return

    mSize = len(data)+14

    Sum = 0
    chkSum = 0

    tmpData = 0

    # Header
    self.HardSerial.write(0x7E)
    tmpData = (mSize & 0xFF00)>>8
    self.escapeAndWrite(tmpData)
    tmpData = mSize & 0xFF
    self.escapeAndWrite(tmpData)

    # Frame
    tmpData = 0x10
    self.escapeAndWrite(tmpData)
    Sum += tmpData

    # Current sequence number
    if self.seq == 0xFF:
      self.seq = 0x01;
    else:
      self.seq +=1
    self.escapeAndWrite(seq)
    Sum += seq;

    # High Address
    tmpData = (addr_high & 0xFF000000)>>24
    self.escapeAndWrite(tmpData)
    Sum += tmpData
    tmpData = (addr_high & 0xFF0000)>>16
    self.escapeAndWrite(tmpData)
    Sum += tmpData
    tmpData = (addr_high & 0xFF00)>>8
    self.escapeAndWrite(tmpData)
    Sum += tmpData
    tmpData = addr_high & 0xFF
    self.escapeAndWrite(tmpData)
    Sum += tmpData

    # Low Address
    tmpData = (addr_low & 0xFF000000)>>24
    self.escapeAndWrite(tmpData)
    Sum += tmpData
    tmpData = (addr_low & 0xFF0000)>>16
    self.escapeAndWrite(tmpData)
    Sum += tmpData
    tmpData = (addr_low & 0xFF00)>>8
    self.escapeAndWrite(tmpData)
    Sum += tmpData
    tmpData = addr_low & 0xFF
    self.escapeAndWrite(tmpData)
    Sum += tmpData

    # Frame options
    tmpData = 0xFF
    self.escapeAndWrite(tmpData)
    Sum += tmpData
    tmpData = 0xFE
    self.escapeAndWrite(tmpData)
    Sum += tmpData

    tmpData = 0x00
    self.escapeAndWrite(tmpData)
    Sum += tmpData

    tmpData = 0x30
    self.escapeAndWrite(tmpData)
    Sum += tmpData

    # Data
    for c in data:
      tmpData = ord(c)
      self.escapeAndWrite(tmpData)
      Sum += tmpData

    chkSum = 0xFF-(Sum&0xFF)
    self.escapeAndWrite(chkSum)

  def listen(self):
    if self.HardSerial.inWaiting()>0 and self.xBeeStatus == ModemStatus.API_MODE2:
      self.d = ord(self.HardSerial.read(1))
      print "Reading data"

      if self.d == 0x7D:
        self.escapeNext = True
        return

      if self.escapeNext:
        self.d = self.d ^ 0x20
        self.escapeNext = False

      if self.readStatus == ReadStatus.RSP_WAIT:
        print "RSP_WAIT"
        print format(self.d, '02x')
        if self.d == 0x7E:
          self.readStatus = ReadStatus.RSP_LMSB
          self.checkSum = 0
          if self.rcvSize>0:
            self.rcvBuffer = ''
            self.rcvSize = 0
      elif self.readStatus == ReadStatus.RSP_LMSB:
        print "RSP_LMSB"
        self.lMSB = self.d
        self.readStatus = ReadStatus.RSP_LLSB
      elif self.readStatus == ReadStatus.RSP_LLSB:
        print "RSP_LLSB"
        self.lLSB = self.d

        self.packetSize = self.lMSB
        self.packetSize = self.lMSB<<8
        self.packetSize = self.packetSize | self.lLSB

        if self.packetSize>0:
          self.rcvBuffer = ''
        else:
          self.readStatus = ReadStatus.RSP_WAIT
          return

        self.readStatus = ReadStatus.RSP_RDDATA
      elif self.readStatus == ReadStatus.RSP_RDDATA:
        print "RSP_RDDATA"
        if self.rcvSize<self.packetSize:
          self.rcvBuffer += chr(self.d)
          self.rcvSize+=1
          self.checkSum += self.d
        else:
          self.checkSum += self.d
          self.checkSum = 0xFF & self.checkSum
          print "Checksum:"
          print self.checkSum
          if self.checkSum == 0xFF:
            if self.frameReceivedHandler != None:
              print "Frame lenght: "+str(len(self.rcvBuffer))
              self.frameReceivedHandler(self.rcvBuffer)
          print "End of Frame"
          self.readStatus = ReadStatus.RSP_WAIT

  def onFrameReceived(self, handler):
    self.frameReceivedHandler = handler

  buff = ''
  def waitFor(self, response):
    # Implement wait for in python
    curr_time = millis()
    while (curr_time-millis())<10000:
      if self.HardSerial.inWaiting()>0:
        self.buff += self.HardSerial.read(1)
        print self.buff
        if '\r' in self.buff:
          print 'Endline received'
          lines = self.buff.split('\r')
          last_received = lines[-2]
          self.buff = lines[-1]
          if response in last_received:
            return True
    return False

  def escapeAndWrite(self, data):
    if data == 0x7E or data == 0x7D or data == 0x11 or data == 0x13:
      self.HardSerial.write(0x7D)
      self.HardSerial.write(data^0x20)
    else:
      self.HardSerial.write(data)

ser = serial.Serial(
  '/dev/ttyUSB0',
  baudrate=9600,
  interCharTimeout=None
)

myXBee = XBee900HP(ser)

def printMessage(msg):
  if ord(msg[0]) == 0x90:
    print "Message received"
    print msg[12:]

myXBee.onFrameReceived(printMessage)

def handler(signum, frame):
  global myXBee
  myXBee.stop()
  listen = False
  print "Ending program execution"

signal.signal(signal.SIGTERM, handler)
signal.signal(signal.SIGINT, handler)

myXBee.start()
