"""
    Author: Jonas Jelonek <jonas.jelonek@protonmail.ch>
    Created: 30.07.2021
    Modified: 15.11.2021

    Copyright (c) 2021 Jonas Jelonek
"""
__author__ = "Jonas Jelonek"
__copyright__ = "Copyright 2021, Jonas Jelonek"
__credits__ = [ "Jonas Jelonek" ]
__maintainer__ = __author__
__email__ = "jonas.jelonek@protonmail.ch"

import spidev
import time
import RPi.GPIO as GPIO
from array import *

"""
    Used adc: TI ADS1220

    Default pins for communication (pin mode: board)
        Pin 19  => SPI_MOSI (alias: DIN on adc)
        Pin 21  => SPI_MISO (alias: DOUT on adc)
        Pin 23  => SPI_CLK
        Pin 24  => SPI_CS
        Pin 27  => DRDY     (can be altered)
"""

class ADS1220:

    PIN_DRDY = 27                   # DATA READY pin from adc

    CMD_RESET = 0x06;               # 0b0000_011x
    CMD_START = 0x08;               # 0b0000_100x
    CMD_POWERDOWN = 0x04;           # 0b0000_001x
    CMD_RDATA = 0x10;               # 0b0001_xxxx
    CMD_RREG = 0x20;                # 0b0010_rrnn (rr - register, nn - bytes to read minus 1)
    CMD_WREG = 0x40;                # 0b0100_rrnn (rr - register, nn - bytes to write minus 1)
                                    # send all data bytes after this command

    # Configuration options for register 0
    CFG0_PGA_BYPASS_OFF = 0x0               # 0bxxxx_xxx0       PGA Bypass off
    CFG0_PGA_BYPASS_ON = 0x1;               # 0bxxxx_xxx1       PGA Bypass on

    CFG0_GAIN_1 = (0x0 << 1);               # 0bxxxx_000x       GAIN 1
    CFG0_GAIN_2 = (0x1 << 1);               # 0bxxxx_001x       GAIN 2
    CFG0_GAIN_4 = (0x2 << 1);               # 0bxxxx_010x       GAIN 4
    CFG0_GAIN_8 = (0x3 << 1);               # 0bxxxx_011x       GAIN 8
    CFG0_GAIN_16 = (0x4 << 1);              # 0bxxxx_100x       GAIN 16
    CFG0_GAIN_32 = (0x5 << 1);              # 0bxxxx_101x       GAIN 32
    CFG0_GAIN_64 = (0x6 << 1);              # 0bxxxx_110x       GAIN 64
    CFG0_GAIN_128 = (0x7 << 1);             # 0bxxxx_111x       GAIN 128

    CFG0_MUX_P0N1 = (0x0 << 4);             # 0b0000_xxxx       AIN0 -> AIN_P, AIN1 -> AIN_N
    CFG0_MUX_P0N2 = (0x1 << 4);             # 0b0001_xxxx       AIN0 -> AIN_P, AIN2 -> AIN_N
    CFG0_MUX_P0N3 = (0x2 << 4);             # 0b0010_xxxx       AIN0 -> AIN_P, AIN3 -> AIN_N
    CFG0_MUX_P1N2 = (0x3 << 4);             # 0b0011_xxxx       AIN1 -> AIN_P, AIN2 -> AIN_N
    CFG0_MUX_P1N3 = (0x4 << 4);             # 0b0100_xxxx       AIN1 -> AIN_P, AIN3 -> AIN_N
    CFG0_MUX_P2N3 = (0x5 << 4);             # 0b0101_xxxx       AIN2 -> AIN_P, AIN3 -> AIN_N
    CFG0_MUX_P1N0 = (0x6 << 4);             # 0b0110_xxxx       AIN1 -> AIN_P, AIN0 -> AIN_N
    CFG0_MUX_P3N2 = (0x7 << 4);             # 0b0111_xxxx       AIN3 -> AIN_P, AIN2 -> AIN_N
    CFG0_MUX_P0NAVSS = (0x8 << 4);          # 0b1000_xxxx       AIN0 -> AIN_P, AVSS -> AIN_N
    CFG0_MUX_P1NAVSS = (0x9 << 4);          # 0b1001_xxxx       AIN1 -> AIN_P, AVSS -> AIN_N
    CFG0_MUX_P2NAVSS = (0xa << 4);          # 0b1010_xxxx       AIN2 -> AIN_P, AVSS -> AIN_N
    CFG0_MUX_P3NAVSS = (0xb << 4);          # 0b1011_xxxx       AIN3 -> AIN_P, AVSS -> AIN_N
    CFG0_MUX_VREFP_VREFN = (0xc << 4);      # 0b1100_xxxx       (V_REFPx - V_REFNx) / 4 (PGA bypassed)
    CFG0_MUX_AVDD_AVSS = (0xd << 4);        # 0b1101_xxxx       (AVDD - AVSS) / 4 (PGA bypassed)
    CFG0_MUX_PN_SHORT = (0xe << 4);         # 0b1110_xxxx       AIN_P and AIN_N shorted to (AVDD + AVSS) / 2

    # Configuration options for register 1
    CFG1_BCS_OFF = 0x0;                     # 0bxxxx_xxx0       Burn-out current sources off
    CFG1_BCS_ON = 0x1;                      # 0bxxxx_xxx1       Burn-out current sources on

    CFG1_TS_OFF = (0x0 << 1);               # 0bxxxx_xx0x       Temperature sensor mode off
    CFG1_TS_ON = (0x1 << 1);                # 0bxxxx_xx1x       Temperature sensor mode on

    CFG1_CM_SINGLE_SHOT = (0x0 << 2);       # 0bxxxx_x0xx       Single-shot conversion mode
    CFG1_CM_CONTINUOUS = (0x1 << 2);        # 0bxxxx_x1xx       Continuous conversion mode

    CFG1_MODE_NORMAL = (0x0 << 3);          # 0bxxx0_0xxx       Operating mode: Normal
    CFG1_MODE_DUTY_CYCLE = (0x1 << 3);      # 0bxxx0_1xxx       Operating mode: Duty-Cycle
    CFG1_MODE_TURBO = (0x2 << 3);           # 0bxxx1_0xxx       Operating mode: Turbo

    CFG1_DR_STAGE1 = (0x0 << 5);            # NORMAL: 20SPS, DC: 5SPS, TURBO: 40SPS
    CFG1_DR_STAGE2 = (0x1 << 5);            # NORMAL: 45SPS, DC: 11.25SPS, TURBO: 90SPS
    CFG1_DR_STAGE3 = (0x2 << 5);            # NORMAL: 90SPS, DC: 22.5SPS, TURBO: 180SPS
    CFG1_DR_STAGE4 = (0x3 << 5);            # NORMAL: 175SPS, DC: 44SPS, TURBO: 350SPS
    CFG1_DR_STAGE5 = (0x4 << 5);            # NORMAL: 330SPS, DC: 82.5SPS, TURBO: 660SPS
    CFG1_DR_STAGE6 = (0x5 << 5);            # NORMAL: 600SPS, DC: 150SPS, TURBO: 1200SPS
    CFG1_DR_STAGE7 = (0x6 << 5);            # NORMAL: 1000SPS, DC: 250SPS, TURBO: 2000SPS

    # Configuration options for register 2
    CFG2_IDAC_OFF = 0x0;                    # IDAC current source off
    CFG2_IDAC_10UA = 0x1;                   # IDAC current source 10μA
    CFG2_IDAC_50UA = 0x2;                   # IDAC current source 50μA
    CFG2_IDAC_100UA = 0x3;                  # IDAC current source 100μA
    CFG2_IDAC_250UA = 0x4;                  # IDAC current source 250μA
    CFG2_IDAC_500UA = 0x5;                  # IDAC current source 500μA
    CFG2_IDAC_1000UA = 0x6;                 # IDAC current source 1000μA
    CFG2_IDAC_1500UA = 0x7;                 # IDAC current source 1500μA

    CFG2_PSW_ALWAYS_OPEN = (0x0 << 3);      # Always open
    CFG2_PSW_AUTO_OPEN = (0x1 << 3);        # Automatically open when START, close when POWERDOWN

    CFG2_FIR_NONE = (0x0 << 4);             # Disable FIR filter
    CFG2_FIR_BOTH = (0x1 << 4);             # FIR filter both 50Hz and 60Hz
    CFG2_FIR_50HZ = (0x2 << 4);             # FIR filter only 50Hz
    CFG2_FIR_60HZ = (0x3 << 4);             # FIR filter only 60Hz

    CFG2_VREF_INTERNAL = (0x0 << 6);        # Use internal reference voltage source
    CFG2_VREF_EXT_REFP0_REFN0 = (0x1 << 6); # Use external reference selected using REFP0 and REFN0
    CFG2_VREF_EXT_AIN0_AIN3 = (0x2 << 6);   # Use external reference selected using AIN0 and AIN3
    CFG2_VREF_EXT_ANALOG = (0x3 << 6);      # Use external reference voltage source on pins AVDD and AVSS

    # Configuration options register 3
    CFG3_DRDY_DEDICATED = (0x0 << 1);       # Signal DATA READY only on dedicated DRDY line
    CFG3_DRDY_BOTH = (0x1 << 1);            # Signal DATA READY on dedicated DRDY line and shared MOUT/DRDY line

    CFG3_I2MUX_DISABLE = (0x0 << 2);        # IDAC2 disabled
    CFG3_I2MUX_AIN0 = (0x1 << 2);           # IDAC2 connected to AIN_0
    CFG3_I2MUX_AIN1 = (0x2 << 2);           # IDAC2 connected to AIN_1
    CFG3_I2MUX_AIN2 = (0x3 << 2);           # IDAC2 connected to AIN_2
    CFG3_I2MUX_AIN3 = (0x4 << 2);           # IDAC2 connected to AIN_3
    CFG3_I2MUX_REFP0 = (0x5 << 2);          # IDAC2 connected to REFP_0
    CFG3_I2MUX_REFN0 = (0x6 << 2);          # IDAC2 connected to REFN_0

    CFG3_I1MUX_DISABLE = (0x0 << 5);        # IDAC1 disabled
    CFG3_I1MUX_AIN0 = (0x1 << 5);           # IDAC1 connected to AIN_0
    CFG3_I1MUX_AIN1 = (0x2 << 5);           # IDAC1 connected to AIN_1
    CFG3_I1MUX_AIN2 = (0x3 << 5);           # IDAC1 connected to AIN_2
    CFG3_I1MUX_AIN3 = (0x4 << 5);           # IDAC1 connected to AIN_3
    CFG3_I1MUX_REFP0 = (0x5 << 5);          # IDAC1 connected to REFP_0
    CFG3_I1MUX_REFN0 = (0x6 << 5);          # IDAC1 connected to REFN_0

    def __init__(self):
        self.spi = 0;

        self.init_GPIO();
        self.init_SPI();
        self.config_ADC();

    def init_GPIO(self):
        GPIO.setmode(GPIO.BCM);                     # Use board pin numbers instead of other 
        GPIO.setwarnings(True);                     # Allow / Disallow warnings
        GPIO.setup(ADS1220.PIN_DRDY, GPIO.IN);

    def init_SPI(self):
        self.spi = spidev.SpiDev();
        self.spi.open(0, 0);
        self.spi.max_speed_hz = 500000;
        self.spi.mode = 1;                          # ADS1220 only supports mode 1

    def read_register(self, register):
        opcode = ADS1220.CMD_RREG + (register << 2) + 0x00;         # Command opcode + register number (left-shifted by 2) + number of bytes to read minus 1
        data = self.spi.xfer([ opcode, 0x00 ]);                     # Write opcode + zero-byte to adc

        return data[1];

    def write_register(self, register, value):
        opcode = ADS1220.CMD_WREG + (register << 2) + 0x00;         # Command opcode + register number (left-shifted by 2) + number of bytes to write minus 1
        temp = self.spi.xfer([ opcode, value ]);                    # Write opcode + value to write to adc
        
        verify = self.read_register(register);
        if verify != value:
            raise ConnectionError("Failed to write configuration register (verification failed)!");

        return;

    def set_temperature_mode(self, on):
        mask = ADS1220.CFG1_TS_ON if on else ADS1220.CFG1_TS_OFF;
        register1 = self.read_register(1);
        self.write_register(1, (register1 & 0xfd) | mask);          # Mask with 0xfd (to clear TS bit)

    def start_conversion(self):
        opcode = ADS1220.CMD_START;
        self.spi.xfer([ opcode ]);
        return;

    def reset_adc(self):
        opcode = ADS1220.CMD_RESET;
        self.spi.xfer([ opcode ]);
        return;

    # Reads data from adc by snychronizing to it's DRDY (DATA READY) output.
    # This method should be preferred when working in single-shot mode.
    # Reading data with RDATA command after starting conversion may lead to
    # corrupted data.
    def __read_data_drdy(self):
        while GPIO.input(ADS1220.PIN_DRDY) == True:
            time.sleep(0.001);

        data = False;
        if GPIO.input(ADS1220.PIN_DRDY) == False:
            data = self.spi.xfer([ 0x00, 0x00, 0x00 ])

        return data;

    # Reads data by issuing RDATA command and reading the result.
    # This method should be used when in continuous mode.
    def __read_data_rdata(self):
        opcode = ADS1220.CMD_RDATA;
        data = self.spi.xfer([ opcode, 0x00, 0x00, 0x00 ]);

        return data[1:4];

    def read_internal_temp(self):
        self.start_conversion();

        data = self.__read_data_rdata();
        # Upper 14 bits of the data are the temperature.
        temp_raw = ((data[0] << 8) + data[1]) >> 2;

        temp = temp_raw * 0.03125;
        return temp;

    def set_gain(self, gain: int):
        if gain == 2:
            mask = ADS1220.CFG0_GAIN_2;
        elif gain == 4:
            mask = ADS1220.CFG0_GAIN_4;
        elif gain == 8:
            mask = ADS1220.CFG0_GAIN_8;
        elif gain == 16:
            mask = ADS1220.CFG0_GAIN_16;
        elif gain == 32:
            mask = ADS1220.CFG0_GAIN_32;
        elif gain == 64:
            mask = ADS1220.CFG0_GAIN_64;
        elif gain == 128:
            mask = ADS1220.CFG0_GAIN_128;
        else:
            mask = ADS1220.CFG0_GAIN_1;

        register0 = self.read_register(0);
        self.write_register(0, (register0 & 0xf1) | mask);       # Mask with 0xf0 (to clear GAIN bits)
        self.current_gain = gain;

    def next_lower_gain(self):
        if self.current_gain == 2:
            return 1;
        elif self.current_gain == 4:
            return 2;
        elif self.current_gain == 8:
            return 4;
        elif self.current_gain == 16:
            return 8;
        elif self.current_gain == 32:
            return 16;
        elif self.current_gain == 64:
            return 32;
        elif self.current_gain == 128:
            return 64;
        else: 
            return self.current_gain

    def next_higher_gain(self):
        if self.current_gain == 1:
            return 2;
        elif self.current_gain == 2:
            return 4;
        elif self.current_gain == 4:
            return 8;
        elif self.current_gain == 8:
            return 16;
        elif self.current_gain == 16:
            return 32;
        elif self.current_gain == 32:
            return 64;
        elif self.current_gain == 64:
            return 128;
        else:
            return self.current_again

    """ Runs conversion and reads raw value from ADC """
    def __read_raw_adc(self):
        if self.conv_mode == 1:
            val = int.from_bytes(self.__read_data_rdata(), byteorder="big", signed=True);
        else:
            self.start_conversion();
            val = int.from_bytes(self.__read_data_drdy(), byteorder="big", signed=True);

        return val;

    """ Converts the digitized value into a voltage """
    def __voltage_from_raw(self, raw: int):
        lsb = (2 * self.VREF / self.current_gain) / 2**24;
        return raw * lsb;

    """ Read value from ADC and convert to voltage """
    def read_adc_voltage(self):
        raw = self.__read_raw_adc();
        volt = self.__voltage_from_raw(raw);
        return (volt, self.current_gain, raw);

    def config_ADC(self):
        self.current_gain = 1;      # Adjust this to the setting
        cfg0 = ADS1220.CFG0_PGA_BYPASS_OFF | ADS1220.CFG0_GAIN_1 | ADS1220.CFG0_MUX_P1N2;

        self.conv_mode = 1;                     # 0 = single shot mode, 1 = continuous mode
        cfg1 = ADS1220.CFG1_BCS_OFF | ADS1220.CFG1_TS_OFF | ADS1220.CFG1_CM_CONTINUOUS | ADS1220.CFG1_MODE_TURBO | ADS1220.CFG1_DR_STAGE7;

        self.VREF = 2.048;                      # Set to +2.048V for internal reference
        cfg2 = ADS1220.CFG2_IDAC_OFF | ADS1220.CFG2_PSW_ALWAYS_OPEN | ADS1220.CFG2_FIR_NONE | ADS1220.CFG2_VREF_INTERNAL;
        cfg3 = ADS1220.CFG3_DRDY_DEDICATED | ADS1220.CFG3_I2MUX_DISABLE | ADS1220.CFG3_I1MUX_DISABLE;

        self.write_register(0, cfg0);
        self.write_register(1, cfg1);
        self.write_register(2, cfg2);
        self.write_register(3, cfg3);
