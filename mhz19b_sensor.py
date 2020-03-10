import numpy as np
from copy import copy
import serial

cmd_requestCO2   = [0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]
cmd_baselineOff  = [0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86]
cmd_baselineOn   = [0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6]
cmd_range2000    = [0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F]
cmd_range5000    = [0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x13, 0x88, 0xCB]
cmd_range10000   = [0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x27, 0x10, 0x2F]
cmd_zeroPointCal = [0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78]
cmd_spanPointCal = [0xFF, 0x01, 0x88, None, None, 0x00, 0x00, 0x00, None]


class mhz19b:
    def __init__(self, serial_port="/dev/ttyAMA0"):
        self.ser = serial.Serial(serial_port, 9600, timeout=1)
        self.CO2 = 0
        self.T = 0
        self.last_response = []

    def getCheckSum(self, msg):
        chk = 0
        for i in range(1, 8):
            chk += msg[i]
        chk = ~chk+1;
        return int(np.uint8(chk))

    def sendCommand(self, cmd):
        if (cmd[8] == None):
            cmd[8] = self.getCheckSum(cmd)
        self.ser.write(cmd)

    def readSerial(self):
        res = self.ser.read(9)
        return list(res)

    def requestValues(self):
        self.ser.reset_input_buffer()
        self.sendCommand(cmd_requestCO2)

    def requestAndReadValues(self):
        self.requestValues()
        return self.readValues()

    def updateLastValues(self, CO2, T, res):
        self.CO2 = CO2
        self.T = T
        self.last_response = res

    def readValues(self):
        res = self.readSerial()
        if (len(res) < 9 or res[8]!=self.getCheckSum(res)):
            return 0, 0
        CO2 = 256*res[2] + res[3]
        T = res[4] - 40;
        self.updateLastValues(CO2, T, res)
        return CO2, T

    def setRange2000(self):
        self.sendCommand(cmd_range2000)
    
    def setRange5000(self):
        self.sendCommand(cmd_range5000)
    
    def setRange10000(self):
        self.sendCommand(cmd_range10000)

    def activateABC(self):
        self.sendCommand(cmd_baselineOn)

    def deactivateABC(self):
        self.sendCommand(cmd_baselineOff)
    
    def calibrateZeroPoint(self):
        self.sendCommand(cmd_zeroPointCal)
    
    def calibrateSpanPoint(self, span):
        HH, LL = divmod(span, 256)
        cmd = copy(cmd_spanPointCal)
        cmd[3] = HH
        cmd[4] = LL
        cmd[8] = self.getCheckSum(cmd);
        self.sendCommand(cmd)

    def closeSerial(self):
        self.ser.close()