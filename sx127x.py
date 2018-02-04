# -*- coding: UTF8 -*-
# Micropython driver for Semtech SX127x famaly chips
# Author: Alex Zorg <azorg(at)mail.ru>
# Licenced by GPLv3

from machine import Pin, SPI
from time import sleep_ms

import gc
gc.collect()

MICROPYTHON = True

# Common registers
REG_FIFO      = 0x00 # FIFO read/write access
REG_OP_MODE   = 0x01 # Operation mode & LoRaTM/FSK selection

REG_FRF_MSB   = 0x06 # RF Carrier Frequency, MSB
REG_FRF_MID   = 0x07 # RF Carrier Frequency, Mid
REG_FRF_LSB   = 0x08 # RF Carrier Frequency, LSB
REG_PA_CONFIG = 0x09 # PA selection and Output power control
REG_PA_RAMP   = 0x0A # Controll of PA ramp time, low phase noise PLL
REG_OCP       = 0x0B # Over Current Protection control
REG_LNA       = 0x0C # LNA settings

REG_DIO_MAPPING_1 = 0x40 # Mapping of pins DIO0 to DIO3
REG_DIO_MAPPING_2 = 0x41 # Mapping of pins DIO4, DIO5, CLK-OUT frequency
REG_VERSION       = 0x42 # Semtech ID relating the silicon revision

REG_TCXO        = 0x4B # TCXO or XTAL input settings
REG_PA_DAC      = 0x4D # Higher power settings of the PA
REG_FORMER_TEMP = 0x5B # Stored temperature during the former IQ Calibration

REG_AGC_REF      = 0x61 # Adjustment of the AGC threshold
REG_AGC_THRESH_1 = 0x62 # ...
REG_AGC_THRESH_2 = 0x63 # ...
REG_AGC_THRESH_3 = 0x64 # ...

REG_PLL = 0x70 # Contral of the PLL bandwidth


# FSK/OOK mode registers
REG_BITRATE_MSB     = 0x02 # Bit rate settings, MSB
REG_BITRATE_LSB     = 0x03 # Bit rate settings, LSB
REG_FDEV_MSB        = 0x04 # Frequency Deviation settings, MSB (FSK)
REG_FDEV_LSB        = 0x05 # Frequency Deviation settings, LSB (FSK)

REG_RX_CONFIG       = 0x0D # AFC, AGC, ctrl
REG_RSSI_CONFIG     = 0x0E # RSSI
REG_RSSI_COLLISION  = 0x0F # RSSI Collision detector
REG_RSSI_TRESH      = 0x10 # RSSI Treshhold control
REG_RSSI_VALUE      = 0x11 # RSSI value in dBm (0.5 dB steps)
REG_RX_BW           = 0x12 # Channel Filter BW control
REG_AFC_BW          = 0x13 # AFC channel filter BW
REG_OOK_PEAK        = 0x14 # OOK demodulator
REG_OOK_FIX         = 0x15 # Treshold of the OOK demod
REG_OOK_AVG         = 0x16 # Average of the OOK demod

REG_AFC_FEI         = 0x1A # AFC and FEI control
REG_AFC_MSB         = 0x1B # Frequency correction value of the AFC, MSB
REG_AFC_LSB         = 0x1C # Frequency correction value of the AFC, LSB
REG_FEI_MSB         = 0x1D # Value of the calculated frequency error, MSB
REG_FEI_LSB         = 0x1E # Value of the calculated frequency error, LSB
REG_PREAMBLE_DETECT = 0x1F # Settings of preamble Detector
REG_RX_TIMEOUT_1    = 0x20 # Timeout Rx request and RSSI
REG_RX_TIMEOUT_2    = 0x21 # Timeout RSSI and PayloadReady
REG_RX_TIMEOUT_2    = 0x22 # Timeout RSSI and SyncAddress
REG_RX_DELAY        = 0x23 # Delay between Rx cycles
REG_OSC             = 0x24 # RC Oscillators Settings, CLK-OUT frequency
REG_PREAMBLE_L_MSB  = 0x25 # Preampbe length, MSB 
REG_PREAMBLE_L_LSB  = 0x26 # Preampbe length, LSB
REG_SYNC_CONFIG     = 0x27 # Sync Word Recognition control
REG_SYNC_VALUE_1    = 0x28 # Sync Word byte 1
REG_SYNC_VALUE_2    = 0x29 # Sync Word byte 2
REG_SYNC_VALUE_3    = 0x2A # Sync Word byte 3
REG_SYNC_VALUE_4    = 0x2B # Sync Word byte 4
REG_SYNC_VALUE_5    = 0x2C # Sync Word byte 5
REG_SYNC_VALUE_6    = 0x2D # Sync Word byte 6
REG_SYNC_VALUE_7    = 0x2E # Sync Word byte 7
REG_SYNC_VALUE_8    = 0x2F # Sync Word byte 8
REG_PACKET_CONFIG_1 = 0x30 # Packet mode settings
REG_PACKET_CONFIG_2 = 0x31 # Packet mode settings
REG_PAYLOAD_LEN     = 0x32 # Payload lenght settings
REG_NODE_ADRS       = 0x33 # Node address
REG_BROADCAST_ADRS  = 0x34 # Broadcast address
REG_FIFO_THRESH     = 0x35 # FIFO Theshold, Tx start condition
REG_SEQ_CONFIG_1    = 0x36 # Top level Sequencer settings
REG_SEQ_CONFIG_2    = 0x37 # Top level Sequencer settings
REG_TIMER_RESOL     = 0x38 # Timer 1 and 2 resolution control
REG_TIMER_1_COEF    = 0x39 # Timer 1 settings
REG_TIMER_2_COEF    = 0x3A # Timer 2 settings
REG_IMAGE_CAL       = 0x3B # Image callibration engine control
REG_TEMP            = 0x3C # Tempreture Sensor value
REG_LOW_BAT         = 0x3D # Low Battary Indicator Settings
REG_IRQ_FLAGS_1     = 0x3E # Status register: PLL lock state, Timeout, RSSI
REG_IRQ_FLAGS_2     = 0x3F # Status register: FIFO handing, flags, Low Battery

REG_PLL_HOP      = 0x44 # Control the fast frequency hopping mode
REG_BITRATE_FRAC = 0x5D # Fraction part in the Bit Rate division ratio


# LoRaTM mode registers
REG_FIFO_ADDR_PTR        = 0x0D
REG_FIFO_TX_BASE_ADDR    = 0x0E
REG_FIFO_RX_BASE_ADDR    = 0x0F
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_FIFO_RX_BYTE_ADDR    = 0x25

REG_IRQ_FLAGS_MASK = 0x11
REG_IRQ_FLAGS      = 0x12
REG_RX_NB_BYTES    = 0x13
REG_PKT_RSSI_VALUE = 0x1A
REG_PKT_SNR_VALUE  = 0x1B
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PREAMBLE_MSB   = 0x20
REG_PREAMBLE_LSB   = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_RSSI_WIDEBAND  = 0x2C

REG_DETECTION_OPTIMIZE  = 0x31
REG_INVERT_IQ           = 0x33
REG_DETECTION_THRESHOLD = 0x37
REG_SYNC_WORD           = 0x39

# Modes, REG_OP_MODE register (look `RegOpMode` in datasheeet)
# bits 2-0
MODE_SLEEP         = 0b000 # (0) Sleep
MODE_STDBY         = 0b001 # (1) Standby (default)
MODE_FS_TX         = 0b010 # (2) Frequency Synthesis TX (FSTx)
MODE_TX            = 0b011 # (3) Transmit (Tx)
MODE_FS_RX         = 0b100 # (4) Frequency Synthesis RX (FSRx)
MODE_RX_CONTINUOUS = 0b101 # (5) Receive (continuous) (Rx)
MODE_RX_SINGLE     = 0b110 # (6) Receive single (RXsingle) [LoRa mode only]
MODE_CAD           = 0b111 # (7) Channel Activity Detection (CAD) [in LoRa mode only]
MODES_MASK         = 0b111 # (7) Modes bit mask 1

# bit 3 (0 -> access to HF registers from 0x61 address, 1 -> access to LF registers)
MODE_LOW_FREQ_MODE_ON = 0b1000 # (0x08) `LowFrequencyModeOn` 

# bits 6-5 `ModulationType` [FSK/OOK modes only]
MODE_FSK    = 0b00000000 # (0x00) 0b00 -> FSK
MODE_OOK    = 0b00100000 # (0x40) 0b01 -> OOK
MODES_MASK2 = 0b01100000 # (0x60) Modes bit mask 2 

# bit 6 (allows access to FSK registers in 0x0D:0x3F in LoRa mode)
MODE_ACCESS_SHARED_REG = 0b01000000 # 0x40 `AccessSharedReg` (LoRa mode only)

# bit 7 (0 -> FSK/OOK mode, 1 -> LoRa mode)
MODE_LONG_RANGE = 0b10000000 # (0x80) bit 7: `LongRangeMode`

# REG_PA_CONFIG bits (look `RegPaConfig` in datasheet)
PA_SELECT = 0x80 # bit 7: `PaSelect`

# REG_IRQ_FLAGS (`RegIrqFlags` in datasheet) bits (LoRa)
IRQ_TX_DONE           = 0x08 # `TxDone`
IRQ_PAYLOAD_CRC_ERROR = 0x20 # `PayloadCrcError`

# REG_IRQn_FLAGS (`RegIrqFlagsN` in datasheet) bits (FSK/OOK)
IRQ1_RX_READY    = 0x50 # bit 6: `RxReady` in `RegIrqFlags1`
IRQ1_TX_READY    = 0x20 # bit 5: `TxReady` in `RegIrqFlags1`
IRQ2_PACKET_SENT = 0x08 # bit 3: `PacketSent` in `RegIrqFlags2`

# REG_IRQ_FLAGS_MASK (`RegIrqFlagsMask` in datasheet) bits (LoRa)
IRQ_RX_DONE_MASK = 0x40 # bit 6: `RxDoneMask`

# Buffer size
MAX_PKT_LENGTH = 255

FIFO_TX_BASE_ADDR = 0x00 # 0x80 FIXME
FIFO_RX_BASE_ADDR = 0x00 

# Constants
FXOSC = 32e6 # 32 MHz
FSTEP = FXOSC / 2**19 # ~61.03515625 Hz

# RX BandWith table
RX_BW_TABLE = (
  # mant exp kHz 
  (0b10, 7,   2.6),
  (0b01, 7,   3.1),
  (0b00, 7,   3.9),
  (0b10, 6,   5.2),
  (0b01, 6,   6.3),
  (0b00, 6,   7.8),
  (0b10, 5,  10.4),
  (0b01, 5,  12.5),
  (0b00, 5,  15.6),
  (0b10, 4,  20.8),
  (0b01, 4,  25.0),
  (0b00, 4,  31.3),
  (0b10, 3,  31.7),
  (0b01, 3,  50.0),
  (0b00, 3,  62.5),
  (0b10, 2,  83.3),
  (0b01, 2, 100.0),
  (0b00, 2, 125.0),
  (0b10, 1, 166.7),
  (0b01, 1, 200.0),
  (0b00, 1, 250.0))


def get_rx_bw(bw=10.4):
    for m, e, v in RX_BW_TABLE:
        if bw <= v:
            return m, e 
    return RX_BW_TABLE[-1][:2]


class SX127x:
    def __init__(self,
                 mode = 0, # 0 - LoRa, 1 - FSK, 2 - OOK
                 pars = {'freq_kHz':         433000, # kHz
                         'freq_Hz':          0,      # Hz
                         'power':            10,     # dBm
                         'crc':              False,
                         # LoRa mode:
                         'bw':               125e3,  # kHz
                         'sf':               10,     # 6..12
                         'cr':               5,      # 5...8
                         'ldr' :             None,   # Low Data Rate Optimize
                         'sw':               0x12,
                         'preamble':         8,      # 6...65k
                         'implicit_header':  False,
                         # FSK/OOK mode:
                         'bitrate':          4800.,  # bit/s
                         'fdev':             5000.,  # frequency deviation [Hz]
                         'rx_bw':            10.4,   # 2,6...250 kHz
                         'afc_bw':           50.0,   # 2,6...250 kHz
                         'afc':              True,
                         'fixed':            False},
                 gpio = {'led':    2,   # blue led
                         'led_on': 0,   # led on level (0 or 1)
                         'reset':  0,   # reset pin
                         'dio0':   4,   # DIO0 line
                         'cs':     15,  # SPI CS
                         'sck':    14,  # SPI SCK
                         'mosi':   13,  # SPI MOSI
                         'miso':   12}, # SPI MISO
                 spi_hardware = True,
                 spi_baudrate = None,
                 onReceive    = None): # receive callback

        # init GPIO
        self.pin_led = Pin(gpio['led'], Pin.OUT)
        self.led_on  = gpio['led_on']
        self.led(False) # LED off
        if gpio['reset'] != None:
          self.pin_reset = Pin(gpio['reset'], Pin.OUT, Pin.PULL_UP)
          self.pin_reset.value(1)
        else:
          self.pin_reset = None
        self.pin_dio0 = Pin(gpio['dio0'], Pin.IN,  Pin.PULL_UP)
        self.pin_cs   = Pin(gpio['cs'],   Pin.OUT, Pin.PULL_UP)
        self.pin_cs.value(1)

        # init SPI
        if spi_hardware:
            if spi_baudrate == None: spi_baudrate = 5000000 # 5MHz
            self.spi = SPI(1, baudrate=spi_baudrate, polarity=0, phase=0)
        else:
            if spi_baudrate == None: spi_baudrate = 500000 # 500kHz
            self.spi = SPI(-1, baudrate=spi_baudrate, polarity=0, phase=0,
                           sck=Pin(gpio['sck']),
                           mosi=Pin(gpio['mosi']),
                           miso=Pin(gpio['miso']))
                           #bits=8, firstbit=SPI.MSB, # FIXME
                           #sck=Pin(gpio['sck'], Pin.OUT, Pin.PULL_DOWN),
                           #mosi=Pin(gpio['mosi'], Pin.OUT, Pin.PULL_UP),
                           #miso=Pin(gpio['miso'], Pin.IN, Pin.PULL_UP))
        self.spi.init()
        self.onReceive(onReceive)        
        #self._lock = False
        self.reset()
        self.mode = 0 # LoRa mode by default (FIXME)
        self.init(mode, pars)


    def __exit__(self): 
        self.pin_dio0.irq(trigger=0, handler=None)
        self.spi.close()
    

    def spiTransfer(self, address, value=0x00):
        response = bytearray(1)
        self.pin_cs.value(0)
        self.spi.write(bytes([address]))
        self.spi.write_readinto(bytes([value]), response)
        self.pin_cs.value(1)
        return response


    def readReg(self, address, byteorder='big', signed=False):
        """read 8-bit register by SPI"""
        response = self.spiTransfer(address & 0x7F) 
        return int.from_bytes(response, byteorder)        
        

    def writeReg(self, address, value):
        """write 8-bit register by SPI"""
        self.spiTransfer(address | 0x80, value)


    def led(self, on=True):
        """on/off LED on GPIO pin"""
        self.pin_led.value(not self.led_on ^ on)


    def blink(self, times=1, on_ms=100, off_ms=20):
        """short blink LED on GPIO pin"""
        for i in range(times):
            self.led(1)
            sleep_ms(on_ms)
            self.led(0)
            sleep_ms(off_ms) 
            

    def reset(self, low_ms=100, high_ms=100, times=1):
        """hard reset SX127x chip"""
        if self.pin_reset:
            for i in range(times):
                self.pin_reset.value(1)
                sleep_ms(high_ms)
                self.pin_reset.value(0)
                sleep_ms(low_ms)
                self.pin_reset.value(1)
                sleep_ms(high_ms)

        
    def version(self):
        """get SX127x crystal revision"""
        return self.readReg(REG_VERSION)


    def setMode(self, mode):
        """set mode"""
        self.writeReg(REG_OP_MODE, (self.readReg(REG_OP_MODE) & ~MODES_MASK) | mode)


    def getMode(self):
        """get mode"""
        return self.readReg(REG_OP_MODE) & MODES_MASK


    def lora(self, lora=True):
        """switch to LoRa mode"""
        mode  = self.readReg(REG_OP_MODE)  # read mode
        sleep = (mode & ~MODES_MASK) | MODE_SLEEP
        self.writeReg(REG_OP_MODE, sleep)  # go to sleep
        if lora:
            sleep |= MODE_LONG_RANGE 
            mode  |= MODE_LONG_RANGE
        else:
            sleep &= ~MODE_LONG_RANGE 
            mode  &= ~MODE_LONG_RANGE
        self.writeReg(REG_OP_MODE, sleep)  # write "long range" bit
        self.writeReg(REG_OP_MODE, mode)   # restore old mode
        

    def isLora(self):
        """check LoRa (or FSK/OOK) mode"""
        mode = self.readReg(REG_OP_MODE) # read mode
        return True if (mode & MODE_LONG_RANGE) else False


    def fsk(self, fsk=True):
        """switch to FSK mode"""
        self.lora(not fsk)
        if fsk:
            self.writeReg(REG_OP_MODE, (self.readReg(REG_OP_MODE) & ~MODES_MASK2) | MODE_FSK)


    def ook(self, ook=True):
        """switch to OOK mode"""
        self.lora(not ook)
        if ook:
            self.writeReg(REG_OP_MODE, (self.readReg(REG_OP_MODE) & ~MODES_MASK2) | MODE_OOK)


    def sleep(self):
        """switch to Sleep Mode:"""
        self.setMode(MODE_SLEEP)


    def standby(self):
        """switch ro Standby mode"""
        self.setMode(MODE_STDBY)


    def init(self, mode=None, pars=None):
        """init chip"""
        if mode is not None: self.mode = mode
        if pars: self.pars = pars
            
        # check version
        version = self.version()
        print("SX127x selicon revision = 0x%02X" % version)
        if version != 0x12:
            raise Exception('Invalid SX127x selicon revision')
        
        # switch mode
        if self.mode == 1:
            self.fsk(True)
        elif self.mode == 2:
            self.ook(True)
        else: # self.mode == 0
            self.lora(True)
        
        # config RF frequency
        self.setFrequency(self.pars['freq_kHz'], self.pars['freq_Hz'])

        # set LNA boost
        self.writeReg(REG_LNA, self.readReg(REG_LNA) | 0x03)
        
        if self.mode == 0:
            # set LoRaTM options
            self.setBW(self.pars['bw'])
            self.setPower(self.pars['power'])
            self._implicitHeaderMode = None
            self.implicitHeaderMode(self.pars['implicit_header'])      
            sf = self.pars['sf']
            self.setSF(sf)
            ldr = self.pars['ldr']
            if ldr == None:
                ldr = True if sf >= 10 else False
            self.setLDR(ldr)
            self.setCR(self.pars['cr'])
            self.setPreamble(self.pars['preamble'])
            self.setSW(self.pars['sw'])
            self.enableCRC(self.pars['crc'])
            
            # set base addresses
            self.writeReg(REG_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR)
            self.writeReg(REG_FIFO_RX_BASE_ADDR, FIFO_RX_BASE_ADDR)

            # set DIO0 mapping (RxDone)
            self.writeReg(REG_DIO_MAPPING_1, 0x00)
        else:
            # set FSK/OOK options
            self.packet_mode(True)
            self.bitrate(  self.pars["bitrate"])
            self.fdev(     self.pars["fdev"])
            self.rxBW(     self.pars["rx_bw"])
            self.afcBW(    self.pars["afc_bw"])
            self.fixedLen( self.pars["fixed"])
            self.enableAFC(self.pars["afc"])
            self.enableCRC(self.pars["crc"])
            
            self.writeReg(REG_RSSI_TRESH, 0xFF) # default
            self.writeReg(REG_PREAMBLE_LSB, 8) # 3 by default
            
            self.writeReg(REG_SYNC_VALUE_1, 0x69) # 0x01 by default
            self.writeReg(REG_SYNC_VALUE_2, 0x81) # 0x01 by default
            self.writeReg(REG_SYNC_VALUE_3, 0x7E) # 0x01 by default
            self.writeReg(REG_SYNC_VALUE_4, 0x96) # 0x01 by default

            # set DIO0 mapping:
            #    in RxContin - SyncAddres
            #    in TxContin - TxReady
            #    in RxPacket - PayloadReady
            #    in TxPacket - PacketSent
            self.writeReg(REG_DIO_MAPPING_1, 0x00)

        self.standby() 

        
    def setFrequency(self, freq_kHz, freq_Hz=0):
        """set RF frequency [kHz * 1000 + Hz]"""
        self.freq = int(freq_kHz) * 1000 + freq_Hz # kHz + Hz -> Hz
        freq_code = int(round(self.freq / FSTEP))
        self.writeReg(REG_FRF_MSB, (freq_code >> 16) & 0xFF)
        self.writeReg(REG_FRF_MID, (freq_code >>  8) & 0xFF)
        self.writeReg(REG_FRF_LSB,  freq_code        & 0xFF)
        mode = self.readReg(REG_OP_MODE)
        if self.freq < 600000000: # FIXME ~ 600 MHz ?
            mode |=  MODE_LOW_FREQ_MODE_ON # LF
        else:
            mode &= ~MODE_LOW_FREQ_MODE_ON # HF
        self.writeReg(REG_OP_MODE, mode)


    def setPower(self, level, boost=True, MaxPower=7):
        """set TX Power level and boost"""
        MaxPower = min(max(MaxPower, 0), 7)
        if boost:
            # Select PA BOOST pin: Pout is limited to ~17..20 dBm
            # Pout = 17 - (15 - OutputPower) dBm
            OutputPower = min(max(level - 2, 0), 15)
            self.writeReg(REG_PA_CONFIG, PA_SELECT | OutputPower)
        else:
            # Select RFO pin: Pout is limited to ~14..15 dBm
            # Pmax = 10.8 + 0.6 * MaxPower  [dBm]
            # Pout = Pmax - (15 - OutputPower)) = 0...15 dBm if MaxPower=7
            OutputPower = min(max(level, 0), 15)
            self.writeReg(REG_PA_CONFIG, (MaxPower << 4) | OutputPower)
            
    
    def setHighPower(self, hp=True):
        """set high power on PA_BOOST up to +20 dBm"""
        if hp: # +3dB
            self.writeReg(REG_PA_DAC, 0x87) # power on PA_BOOST pin up to +20 dBm
        else:
            self.writeReg(REG_PA_DAC, 0x84) # default mode

    
    def enableCRC(self, crc=True):
        """enable/disable CRC"""
        if self.mode == 0: # LoRa mode
            reg = self.readReg(REG_MODEM_CONFIG_2)
            reg = (reg | 0x04) if crc else (reg & 0xFB) # `RxPayloadCrcOn`
            self.writeReg(REG_MODEM_CONFIG_2, reg)
        else: # FSK/OOK mode
            reg = self.readReg(REG_PACKET_CONFIG_1)
            reg = (reg | 0x08) if crc else (reg & 0xF7) # `CrcOn`
            self.writeReg(REG_PACKET_CONFIG_1, reg)
 
    
    def rssi(self):
        """get RSSI [dB] (LoRa)"""
        if self.mode == 0: # LoRa mode
            return self.readReg(REG_PKT_RSSI_VALUE) - \
                   (164 if self.freq < 868000000 else 157)
        else: # FSK/OOK mode
            return self.readReg(REG_RSSI_VALUE) * 0.5


    def snr(self):
        """get SNR [dB]"""
        if self.mode == 0: # LoRa mode
            return (self.readReg(REG_PKT_SNR_VALUE)) * 0.25
        else: # FSK/OOK mode
            return self.readReg(REG_RSSI_VALUE) * 0.5 # FIXME
        

    def getIrqFlags(self):
        """get IRQ flags for debug"""
        if self.mode == 0: # LoRa mode
            irqFlags = self.readReg(REG_IRQ_FLAGS)
            self.writeReg(REG_IRQ_FLAGS, irqFlags)
            return irqFlags
        else: # FSK/OOK mode
            irqFlags1 = self.readReg(REG_IRQ_FLAGS_1)
            #self.writeReg(REG_IRQ_FLAGS_1, irqFlags1)
            irqFlags2 = self.readReg(REG_IRQ_FLAGS_2)
            #self.writeReg(REG_IRQ_FLAGS_2, irqFlags2)
            return (irqFlags2 << 8) | irqFlags1

    
    def enableRxIrq(self, enable=True):
        """enable/disable interrupt by RX done for debug (LoRa)"""
        if self.mode == 0: # LoRa mode
            reg = self.readReg(REG_IRQ_FLAGS_MASK)
            if enable: reg &= ~IRQ_RX_DONE_MASK
            else:      reg |=  IRQ_RX_DONE_MASK
            self.writeReg(REG_IRQ_FLAGS_MASK, reg)
            
       
    def invertIQ(self, invert=True):
        """invert IQ channels (LoRa)"""
        if self.mode == 0:
            reg = self.readReg(REG_INVERT_IQ)
            if invert:
                reg |=  0x40 # `InvertIq` = 1 
            else:
                reg &= ~0x40 # `InvertIq` = 0
            self.writeReg(REG_INVERT_IQ, reg)


    def setSF(self, sf=10):
        """set Spreading Factor 6...12 (LoRa)"""
        if self.mode == 0:
            sf = min(max(sf, 6), 12)
            self.writeReg(REG_DETECTION_OPTIMIZE,  0xC5 if sf == 6 else 0xC3)
            self.writeReg(REG_DETECTION_THRESHOLD, 0x0C if sf == 6 else 0x0A)
            self.writeReg(REG_MODEM_CONFIG_2,
                               (self.readReg(REG_MODEM_CONFIG_2) & 0x0F) | ((sf << 4) & 0xF0))

            # set AGC auto on (internal AGC loop)
            self.writeReg(REG_MODEM_CONFIG_3,
                               self.readReg(REG_MODEM_CONFIG_3) | 0x04)

    def setLDR(self, ldr):
        """set Low Data Rate Optimisation (LoRa)"""
        if self.mode == 0:
            self.writeReg(REG_MODEM_CONFIG_3,
                               (self.readReg(REG_MODEM_CONFIG_3) & ~0x08) | 0x08 if ldr else 0)

    def setBW(self, sbw):
        """set signal Band With 7.8-500 kHz (LoRa)"""
        if self.mode == 0:
            bins = (7.8e3, 10.4e3, 15.6e3, 20.8e3, 31.25e3, 41.7e3, 62.5e3, 125e3, 250e3, 500e3)
            bw = 9 # max 500kHz
            for i in range(len(bins)):
                if sbw <= bins[i]:
                    bw = i
                    break
            self.writeReg(REG_MODEM_CONFIG_1, \
                               (self.readReg(REG_MODEM_CONFIG_1) & 0x0F) | (bw << 4))


    def setCR(self, denominator):
        """set Coding Rate [5..8] (LoRa)"""
        if self.mode == 0:
            denominator = min(max(denominator, 5), 8)        
            cr = denominator - 4
            self.writeReg(REG_MODEM_CONFIG_1, (self.readReg(REG_MODEM_CONFIG_1) & 0xF1) | (cr << 1))
        

    def setPreamble(self, length):
        """set preamble length [6...65535] (LoRa)"""
        if self.mode == 0:
            self.writeReg(REG_PREAMBLE_MSB, (length >> 8) & 0xFF)
            self.writeReg(REG_PREAMBLE_LSB, (length     ) & 0xFF)
        
        
    def setSW(self, sw): # LoRa mode only
        """set Sync Word (LoRa)"""
        if self.mode == 0:
            self.writeReg(REG_SYNC_WORD, sw)
         
    
    def implicitHeaderMode(self, implicitHeaderMode=False):
        """set implicitHeaderMode (LoRa)"""
        if self.mode == 0:
            if self._implicitHeaderMode != implicitHeaderMode:  # set value only if different.
                self._implicitHeaderMode = implicitHeaderMode
                modem_config_1 = self.readReg(REG_MODEM_CONFIG_1)
                config = modem_config_1 | 0x01 if implicitHeaderMode else modem_config_1 & 0xfe
                self.writeReg(REG_MODEM_CONFIG_1, config)
       
        
    def bitrate(self, bitrate=4800., frac=None):
        """set bitrate [bit/s] (FSK/OOK)"""
        if self.mode:
            if bitrate:
                code = int(round(FXOSC / bitrate)) # bit/s -> code
                self.writeReg(REG_BITRATE_MSB,  (code >> 8) & 0xFF)
                self.writeReg(REG_BITRATE_LSB,   code       & 0xFF)
                if frac:
                    self.writeReg(REG_BITRATE_FRAC, frac)


    def fdev(self, fdev=5000.):
        """set frequency deviation (FSK)"""
        if self.mode:
            if fdev:
                code = int(round(fdev / FSTEP)) # Hz -> code
                code = min(max(code, 0), 0x3FFF)
                self.writeReg(REG_FDEV_MSB, (code >> 8) & 0xFF)
                self.writeReg(REG_FDEV_LSB,  code       & 0xFF)


    def rxBW(self, bw=10.4):
        """set RX BW [kHz] (FSK/OOK)"""
        if self.mode:
            if bw:
                m, e = get_rx_bw(bw)
                self.writeReg(REG_RX_BW, (m << 3) | e)


    def afcBW(self, bw=50.0):
        """set AFC BW [kHz] (FSK/OOK)"""
        if self.mode:
            if bw:
                m, e = get_rx_bw(bw)
                self.writeReg(REG_AFC_BW, (m << 3) | e)


    def enableAFC(self, afc=True):
        """enable/disable AFC (FSK/OOK)"""
        if self.mode:
            reg = self.readReg(REG_RX_CONFIG)
            if afc: reg |=  0x10 # bit 4: AfcAutoOn -> 1
            else:   reg &= ~0x10 # bit 4: AfcAutoOn -> 0
            self.writeReg(REG_RX_CONFIG, reg)


    def fixedLen(self, fixed=False):
        """set Fixed or Variable packet mode (FSK/OOK)"""
        if self.mode:
            reg = self.readReg(REG_PACKET_CONFIG_1)
            if fixed: reg &= ~0x80 # bit 7: PacketFormar -> 0 (fixed size)
            else:     reg |=  0x80 # bit 7: PacketFormat -> 1 (variable size)
            self.writeReg(REG_PACKET_CONFIG_1, reg)


    def payloadLen(self, length=0x40):
        """set Payload length (FSK/OOK)"""
        if self.mode:
            self.writeReg(REG_PAYLOAD_LEN, length)


    def packetMode(self, packet=True):
        """set Packet or Continuous mode (FSK/OOK)"""
        if self.mode:
            reg = self.readReg(REG_PACKET_CONFIG_2)
            if packet: reg |=  0x40 # bit 6 -> 1 - Packet mode
            else:      reg &= ~0x40 # bit 6 -> 0 - Continuous mode
            self.writeReg(REG_PACKET_CONFIG_2, reg)


    def rxCalibrate(self):
        """RSSI and IQ callibration (FSK/OOK)"""
        if self.mode:
            reg = self.readReg(REG_IMAGE_CAL)
            reg |= 0x40 # ImageCalStart bit
            self.writeReg(REG_IMAGE_CAL, reg)
            while (self.readReg(REG_IMAGE_CAL) & 0x20): # ImageCalRunning
                pass # FIXME: check timeout


    #def aquire_lock(self, lock=False):        
    #    if not MICROPYTHON: # MicroPython is single threaded, doesn't need lock.
    #        if lock:
    #            while self._lock: pass
    #            self._lock = True
    #        else:
    #            self._lock = False
    
            
    def send(self, string, implicitHeader=False):
        """send packet (LoRa/FSK/OOK)"""
        #self.aquire_lock(True)  # wait until RX_Done, lock and begin writing.
        self.setMode(MODE_STDBY)
        buf = string.encode()
        size = len(buf)
        
        if self.mode == 0: # LoRa mode
            self.implicitHeaderMode(implicitHeader)

            # reset FIFO address and paload length 
            self.writeReg(REG_FIFO_ADDR_PTR, FIFO_TX_BASE_ADDR)
            self.writeReg(REG_PAYLOAD_LENGTH, 0)

            # check size (FIXME!!!)
            currentLength = self.readReg(REG_PAYLOAD_LENGTH)
            size = min(size, (MAX_PKT_LENGTH - FIFO_TX_BASE_ADDR - currentLength))

            # write data
            for i in range(size):
                self.writeReg(REG_FIFO, buf[i])
        
            # update length        
            self.writeReg(REG_PAYLOAD_LENGTH, currentLength + size)

            # end packet
            self.setMode(MODE_TX) # put in TX mode

            # wait for TX done, standby automatically on TX_DONE
            while (self.readReg(REG_IRQ_FLAGS) & IRQ_TX_DONE) == 0:
                pass # FIXME: check timeout
            
            # clear IRQ's
            self.writeReg(REG_IRQ_FLAGS, IRQ_TX_DONE)
            
        else: # FSK/OOK mode
            if self.readReg(REG_PACKET_CONFIG_1) & 0x80: # `PacketFormat`
                self.writeReg(REG_FIFO, size) # variable length
            else:
                self.writeReg(REG_PAYLOAD_LEN, size) # fixed length
            
            # set `DataMode` to Packet and PayloadLength(10:8)
            self.writeReg(REG_PACKET_CONFIG_2, 0x40 | ((size >> 8) & 0x7))
            
            self.writeReg(REG_FIFO_THRESH, 24) #!!! FIXME !!!
            
            # start TX mode
            self.setMode(MODE_TX)
            
            # wait `TxRaedy` (bit 5 in `RegIrqFlags1`)
            while ((self.readReg(REG_IRQ_FLAGS_1) & IRQ1_TX_READY) == 0):
                pass # FIXME: check timeout

            # write data to FIFO
            for i in range(size):
                self.writeReg(REG_FIFO, buf[i])
            
            # check FIFO bellow thresold (bit 5 in `RegIrqFlags2`)
            #if ((self.readReg(REG_IRQ_FLAGS_2) & 0x20)): pass

            # wait `PacketSent` (bit 3 in `RegIrqFlags2`)
            while ((self.readReg(REG_IRQ_FLAGS_2) & IRQ2_PACKET_SENT) == 0):
                pass # FIXME: check timeout
            
            # switch to standby mode
            self.setMode(MODE_STDBY)

        self.collect()
        #self.aquire_lock(False) # unlock when done writing

    
    def onReceive(self, callback):
        """set callback on receive packet (Lora/FSK/OOK)"""
        self._onReceive = callback        
        if callback:
            self.pin_dio0.irq(trigger=Pin.IRQ_RISING, handler=self._handleOnReceive) 
        else:
            self.pin_dio0.irq(trigger=0, handler=None)
        

    def receive(self, size = 0):
        """go to RX mode; wait callback by interrupt (LoRa/FSK/OOK)"""
        if self.mode == 0: # LoRa mode
            self.implicitHeaderMode(size > 0)
            if size > 0: self.writeReg(REG_PAYLOAD_LENGTH, size & 0xFF)  
            
            # The last packet always starts at FIFO_RX_CURRENT_ADDR
            # no need to reset FIFO_ADDR_PTR
            self.setMode(MODE_RX_CONTINUOUS)
        else: # FSK/OOK mode
            pass # FIXME
                 
                 
    def collect(self):
        """garbage collection"""
        gc.collect()
        #if MICROPYTHON:
        #    print('[Memory - free: {}   allocated: {}]'.format(gc.mem_free(), gc.mem_alloc()))
            

    def _handleOnReceive(self, event_source):
        #self.aquire_lock(True)
        if self.mode == 0: # LoRa mode 
            irqFlags = self.readReg(REG_IRQ_FLAGS) # should be 0x50 ???
            self.writeReg(REG_IRQ_FLAGS, irqFlags)
            if (irqFlags & IRQ_PAYLOAD_CRC_ERROR):
                return # CRC error

            # set FIFO address to current RX address
            self.writeReg(REG_FIFO_ADDR_PTR, self.readReg(REG_FIFO_RX_CURRENT_ADDR))
            
            # read packet length
            packetLength = self.readReg(REG_PAYLOAD_LENGTH) if self._implicitHeaderMode else \
                           self.readReg(REG_RX_NB_BYTES)
                           
            # read FIFO
            payload = bytearray()
            for i in range(packetLength):
                payload.append(self.readReg(REG_FIFO))
            payload = bytes(payload)
            self.collect()

            # run callback
            if self._onReceive:
                self._onReceive(self, payload)
            self.collect()

        else: # FSK/OOK mode
            pass # FIXME

        #self.aquire_lock(False)
        
            

#*** end of "sx127x.py" module ***#
