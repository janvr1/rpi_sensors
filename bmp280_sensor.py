from smbus2 import SMBus
import numpy as np

ADDR_CTRL_MEAS = 0xF4
ADDR_CONFIG = 0xF5
ADDR_STATUS = 0xF3
ADDR_RESET = 0xE0
ADDR_ID = 0xD0

class bmp280:
    def __init__(self, addr = 0x76):
        self.bus = SMBus(1)
        self.address = addr
        self.osrs_t = {0: 0b000, 1: 0b001, 2: 0b010, 4: 0b011, 8: 0b100, 16: 0b101}
        self.osrs_p = {0: 0b000, 1: 0b001, 2: 0b010, 4: 0b011, 8: 0b100, 16: 0b101}
        self.IIR_filter = {0: 0b000, 2: 0b001, 4: 0b010, 8: 0b011, 16: 0b100}
        self.t_sb = {0.5: 0b000, 62.5: 0b001, 125: 0b010, 250: 0b011, 500: 0b100, 1000: 0b101, 2000: 0b110, 4000: 0b111}
        self.mode = {"normal": 0b11, "forced": 0b01, "sleep": 0b00}
        self.T = 0
        self.p = 0
        self.h = 300
        self.p0 = 1013.25
        self.defaultConfiguration()
        self.readCalibrationData()

    def readMeasureBlock(self):
        start_addr = 0xF7
        stop_addr = 0xFC
        read_len = stop_addr-start_addr+1
        return self.bus.read_i2c_block_data(self.address, start_addr, read_len)

    def readCalibrationData(self):
        start_addr = 0x88
        stop_addr = 0x9F
        read_len = stop_addr-start_addr+1
        cal_data = self.bus.read_i2c_block_data(self.address, start_addr, read_len)
        self.dig_T1 = np.uint16(cal_data[0] | cal_data[1] << 8)
        self.dig_T2 = np.int16(cal_data[2] | cal_data[3] << 8)
        self.dig_T3 = np.int16(cal_data[4] | cal_data[5] << 8)
        self.dig_P1 = np.uint16(cal_data[6] | cal_data[7] << 8)
        self.dig_P2 = np.int16(cal_data[8] | cal_data[9] << 8)
        self.dig_P3 = np.int16(cal_data[10] | cal_data[11] << 8)
        self.dig_P4 = np.int16(cal_data[12] | cal_data[13] << 8)
        self.dig_P5 = np.int16(cal_data[14] | cal_data[15] << 8)
        self.dig_P6 = np.int16(cal_data[16] | cal_data[17] << 8)
        self.dig_P7 = np.int16(cal_data[18] | cal_data[19] << 8)
        self.dig_P8 = np.int16(cal_data[20] | cal_data[21] << 8)
        self.dig_P9 = np.int16(cal_data[22] | cal_data[23] << 8)
        self.dig_Tx = [self.dig_T1, self.dig_T2, self.dig_T3]
        self.dig_Px = [self.dig_P1, self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9]


    def writeControlReg(self, oversample_t, oversample_p, measure_mode):
        val = 0
        val |= self.osrs_t[oversample_t]<<5
        val |= self.osrs_p[oversample_p]<<2
        val |= self.mode[measure_mode]
        self.writeByte(ADDR_CTRL_MEAS, val)
    
    def readControlReg(self):
        return self.readByte(ADDR_CTRL_MEAS)

    def writeConfigReg(self, time_standby, iir_filter_length):
        val = 0
        val |= self.t_sb[time_standby]<<5
        val |= self.IIR_filter[iir_filter_length]<<2
        self.writeByte(ADDR_CONFIG, val)

    def defaultConfiguration(self):
        self.writeControlReg(2, 2, "normal")
        self.writeConfigReg(2000, 2)

    def getMeasurement(self):
        raw_data = self.readMeasureBlock()
        up = np.int32(((raw_data[0]<<12) | raw_data[1]<<4) | raw_data[2]>>4)
        ut = np.int32(((raw_data[3]<<12) | raw_data[4]<<4) | raw_data[5]>>4)
        T, t_fine = self.tempCompensation(ut)
        p = self.pressureCompensation(up, t_fine)
        self.updateValues(T, p/100)
        return round(T, 2), round(p/100, 1)
    
    def seaLevelPressure(self):
        return self.p*(1-(0.0065*self.h/(self.T+0.0065*self.h+273.15)))**(-5.257)

    def altitude(self):
        return ((self.p0/self.p)**(1/5.257)-1)*(self.T+273.15)/0.0065
    
    def updateValues(self, T, p):
        self.T = T
        self.p = p

    def tempCompensation(self, ut):
        var1 = (ut/16384 - self.dig_T1/1024)*self.dig_T2
        var2 = ((ut/131072 - self.dig_T1/8192)*(ut/131072 - self.dig_T1/8192))*self.dig_T3
        t_fine = var1+var2
        T = (var1+var2)/5120
        return T, t_fine

    def pressureCompensation(self, up, t_fine):
        var1 = t_fine/2 - 64000
        var2 = var1*var1*self.dig_P6/32768
        var2 = var2 + var1*self.dig_P5*2
        var2 = var2/4 + self.dig_P4*65536
        var1 = (self.dig_P3*var1*var1/524288 + self.dig_P2*var1)/524288
        var1 = (1 + var1/32768)*self.dig_P1
        p = 1048576 - up
        if (var1 == 0): return 0
        p = (p - var2/4096)*6250/var1
        var1 = self.dig_P9*p*p/2147483648
        var2 = p*self.dig_P8/32768
        p = p + (var1 + var2 + self.dig_P7)/16
        return p

    def readConfigReg(self):
        return self.readByte(ADDR_CONFIG)

    def reset(self):
        self.writeByte(ADDR_RESET, 0xB6)

    def readID(self):
        return self.readByte(ADDR_ID)
    
    def readSatus(self):
        return self.readByte(ADDR_STATUS)

    def writeByte(self, reg_addr, val):
        self.bus.write_byte_data(self.address, reg_addr, val)
    
    def readByte(self, reg_addr):
        return self.bus.read_byte_data(self.address, reg_addr)
