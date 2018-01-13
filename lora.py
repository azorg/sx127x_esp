from machine import Pin, SPI
from time import sleep_ms

import gc
gc.collect()

MICROPYTHON = True

# constants
PA_OUTPUT_RFO_PIN      = 0
PA_OUTPUT_PA_BOOST_PIN = 1

# registers
REG_FIFO      = 0x00
REG_OP_MODE   = 0x01
REG_FRF_MSB   = 0x06
REG_FRF_MID   = 0x07
REG_FRF_LSB   = 0x08
REG_PA_CONFIG = 0x09
REG_LNA       = 0x0C

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
REG_DETECTION_THRESHOLD = 0x37
REG_SYNC_WORD      = 0x39
REG_DIO_MAPPING_1  = 0x40
REG_VERSION        = 0x42

# modes
MODE_LONG_RANGE_MODE = 0x80 # bit 7: 1 => LoRa mode
MODE_SLEEP           = 0x00
MODE_STDBY           = 0x01
MODE_TX              = 0x03
MODE_RX_CONTINUOUS   = 0x05
MODE_RX_SINGLE       = 0x06

# PA config
PA_BOOST = 0x80

# IRQ masks
IRQ_TX_DONE_MASK           = 0x08
IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20

# Buffer size
MAX_PKT_LENGTH = 255

FIFO_TX_BASE_ADDR = 0x00 # 0x80 FIXME
FIFO_RX_BASE_ADDR = 0x00 

class LORA:
    def __init__(self,
                 parameters = {'freq_kHz':         433000,
                               'freq_Hz':          0,
                               'tx_power_level':   10,
                               'signal_bandwidth': 125e3, 
                               'spreading_factor': 10,
                               'ldr' :             None, # Low Data Rate Optimize
                               'coding_rate':      5,
                               'preamble_length':  8, 
                               'implicit_header':  False,
                               'sync_word':        0x12,
                               'enable_crc':       False},
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
                 onReceive    = None): # callback

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
        self.init(parameters)


    def __exit__(self): 
        self.pin_dio0.irq(trigger=0, handler=None)
        self.spi.close()
    

    def led(self, on=True):
        self.pin_led.value(not self.led_on ^ on)


    def blink(self, times=1, on_ms=100, off_ms=20):
        for i in range(times):
            self.led(1)
            sleep_ms(on_ms)
            self.led(0)
            sleep_ms(off_ms) 
            

    def reset(self, low_ms=100, high_ms=100, times=1):
        if self.pin_reset:
            for i in range(times):
                self.pin_reset.value(1)
                sleep_ms(high_ms)
                self.pin_reset.value(0)
                sleep_ms(low_ms)
                self.pin_reset.value(1)
                sleep_ms(high_ms)

        
    def spi_transfer(self, address, value=0x00):
        response = bytearray(1)
        self.pin_cs.value(0)
        self.spi.write(bytes([address]))
        self.spi.write_readinto(bytes([value]), response)
        self.pin_cs.value(1)
        return response


    def readRegister(self, address, byteorder='big', signed=False):
        response = self.spi_transfer(address & 0x7F) 
        return int.from_bytes(response, byteorder)        
        

    def writeRegister(self, address, value):
        self.spi_transfer(address | 0x80, value)

    def version(self):
        return self.readRegister(REG_VERSION)


    def init(self, parameters=None):
        if parameters: self.parameters = parameters
            
        # check version
        version = self.version()
        print("SX127x version = 0x%02X" % version)
        if version != 0x12:
            raise Exception('Invalid SX127x version.')
        
        # put in LoRa and sleep mode
        self.sleep()
        
        # config
        self.setFrequency(self.parameters['freq_kHz'], self.parameters['freq_Hz'])
        self.setSignalBandwidth(self.parameters['signal_bandwidth'])

        # set LNA boost
        self.writeRegister(REG_LNA, self.readRegister(REG_LNA) | 0x03)

        self.setTxPower(self.parameters['tx_power_level'])
        self._implicitHeaderMode = None
        self.implicitHeaderMode(self.parameters['implicit_header'])      
        sf = self.parameters['spreading_factor']
        self.setSpreadingFactor(sf)
        ldr = self.parameters['ldr']
        if ldr == None:
            ldr = True if sf >= 10 else False
        self.setLDR(ldr)
        self.setCodingRate(self.parameters['coding_rate'])
        self.setPreambleLength(self.parameters['preamble_length'])
        self.setSyncWord(self.parameters['sync_word'])
        self.enableCRC(self.parameters['enable_crc'])
        
        # set base addresses
        self.writeRegister(REG_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR)
        self.writeRegister(REG_FIFO_RX_BASE_ADDR, FIFO_RX_BASE_ADDR)
        
        self.standby() 

        
    def beginPacket(self, implicitHeaderMode=False):
        self.standby()
        self.implicitHeaderMode(implicitHeaderMode)
 
        # reset FIFO address and paload length 
        self.writeRegister(REG_FIFO_ADDR_PTR, FIFO_TX_BASE_ADDR)
        self.writeRegister(REG_PAYLOAD_LENGTH, 0)
     

    def endPacket(self):
        # put in TX mode
        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)

        # wait for TX done, standby automatically on TX_DONE
        while (self.readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0:
            pass 
            
        # clear IRQ's
        self.writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)
        
        self.collect()
   

    def write(self, buffer):
        currentLength = self.readRegister(REG_PAYLOAD_LENGTH)
        size = len(buffer)

        # check size
        size = min(size, (MAX_PKT_LENGTH - FIFO_TX_BASE_ADDR - currentLength))

        # write data
        for i in range(size):
            self.writeRegister(REG_FIFO, buffer[i])
        
        # update length        
        self.writeRegister(REG_PAYLOAD_LENGTH, currentLength + size)
        return size

        
    #def aquire_lock(self, lock=False):        
    #    if not MICROPYTHON: # MicroPython is single threaded, doesn't need lock.
    #        if lock:
    #            while self._lock: pass
    #            self._lock = True
    #        else:
    #            self._lock = False
            
            
    def println(self, string, implicitHeader=False):
        #self.aquire_lock(True)  # wait until RX_Done, lock and begin writing.
        
        self.beginPacket(implicitHeader) 
        self.write(string.encode())
        self.endPacket()  

        #self.aquire_lock(False) # unlock when done writing

    
    def getIrqFlags(self):
        irqFlags = self.readRegister(REG_IRQ_FLAGS)
        self.writeRegister(REG_IRQ_FLAGS, irqFlags)
        return irqFlags

        
    def packetRssi(self):
        return (self.readRegister(REG_PKT_RSSI_VALUE) - (164 if self._frequency < 868000000 else 157))


    def packetSnr(self):
        return (self.readRegister(REG_PKT_SNR_VALUE)) * 0.25
        
       
    def standby(self):
        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)

        
    def sleep(self):
        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)
        
        
    def setTxPower(self, level, outputPin=PA_OUTPUT_PA_BOOST_PIN):
        if (outputPin == PA_OUTPUT_RFO_PIN):
            # Pout = 0...15 dBm
            # RFO => Pmax = 10.8 + 0.6 * 7 ~ 15 dBm (Pout = Pmax - (15 - level))
            level = min(max(level, 0), 14)
            self.writeRegister(REG_PA_CONFIG, 0x70 | level)
        else:
            # Pout = 2...17 dBm
            # PA BOOST => Pmax ~ 20 dBm (Pout = 17 - (15 - (level - 2)) dBm
            level = min(max(level, 2), 17)
            self.writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2))
            

    def setFrequency(self, freq_kHz, freq_Hz=0):
        self._frequency = int(freq_kHz) * 1000 + freq_Hz # Hz
        freq_code = self._frequency * 256 // 15625
        self.writeRegister(REG_FRF_MSB, (freq_code >> 16) & 0xFF)
        self.writeRegister(REG_FRF_MID, (freq_code >>  8) & 0xFF)
        self.writeRegister(REG_FRF_LSB,  freq_code        & 0xFF)
        

    def setSpreadingFactor(self, sf=10):
        sf = min(max(sf, 6), 12)
        self.writeRegister(REG_DETECTION_OPTIMIZE,  0xC5 if sf == 6 else 0xC3)
        self.writeRegister(REG_DETECTION_THRESHOLD, 0x0C if sf == 6 else 0x0A)
        self.writeRegister(REG_MODEM_CONFIG_2,
                           (self.readRegister(REG_MODEM_CONFIG_2) & 0x0F) | ((sf << 4) & 0xF0))

        # set AGC auto on (internal AGC loop)
        self.writeRegister(REG_MODEM_CONFIG_3,
                           self.readRegister(REG_MODEM_CONFIG_3) | 0x04)

    def setLDR(self, ldr):
        self.writeRegister(REG_MODEM_CONFIG_3,
                           (self.readRegister(REG_MODEM_CONFIG_3) & ~0x08) | 0x08 if ldr else 0)

    def setSignalBandwidth(self, sbw):        
        bins = (7.8e3, 10.4e3, 15.6e3, 20.8e3, 31.25e3, 41.7e3, 62.5e3, 125e3, 250e3, 500e3)
        
        bw = 9 # max 500kHz
        for i in range(len(bins)):
            if sbw <= bins[i]:
                bw = i
                break
        
        self.writeRegister(REG_MODEM_CONFIG_1, (self.readRegister(REG_MODEM_CONFIG_1) & 0x0F) | (bw << 4))


    def setCodingRate(self, denominator):
        denominator = min(max(denominator, 5), 8)        
        cr = denominator - 4
        self.writeRegister(REG_MODEM_CONFIG_1, (self.readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1))
        

    def setPreambleLength(self, length):
        self.writeRegister(REG_PREAMBLE_MSB,  (length >> 8) & 0xFF)
        self.writeRegister(REG_PREAMBLE_LSB,  (length >> 0) & 0xFF)
        
        
    def enableCRC(self, enable_CRC = False):
        modem_config_2 = self.readRegister(REG_MODEM_CONFIG_2)
        config = modem_config_2 | 0x04 if enable_CRC else modem_config_2 & 0xfb 
        self.writeRegister(REG_MODEM_CONFIG_2, config)
  
 
    def setSyncWord(self, sw):
        self.writeRegister(REG_SYNC_WORD, sw) 
         
    
    def enable_rx_irq(self, enable=True):
        if enable:
            self.writeRegister(REG_IRQ_FLAGS_MASK, self.readRegister(REG_IRQ_FLAGS_MASK) & ~IRQ_RX_DONE_MASK)
        else:
            self.writeRegister(REG_IRQ_FLAGS_MASK, self.readRegister(REG_IRQ_FLAGS_MASK) | IRQ_RX_DONE_MASK)
   
   
    def dumpRegisters(self):
        for i in range(128):
            print("0x{0:02X}: {1:02X}".format(i, self.readRegister(i)))

    
    def implicitHeaderMode(self, implicitHeaderMode = False):
        if self._implicitHeaderMode != implicitHeaderMode:  # set value only if different.
            self._implicitHeaderMode = implicitHeaderMode
            modem_config_1 = self.readRegister(REG_MODEM_CONFIG_1)
            config = modem_config_1 | 0x01 if implicitHeaderMode else modem_config_1 & 0xfe
            self.writeRegister(REG_MODEM_CONFIG_1, config)
       
        
    def onReceive(self, callback):
        self._onReceive = callback        
        if callback:
            self.writeRegister(REG_DIO_MAPPING_1, 0x00)
            self.pin_dio0.irq(trigger=Pin.IRQ_RISING, handler=self.handleOnReceive) 
        else:
            self.pin_dio0.irq(trigger=0, handler=None)
        

    def receive(self, size = 0):
        self.implicitHeaderMode(size > 0)
        if size > 0: self.writeRegister(REG_PAYLOAD_LENGTH, size & 0xff)  
        
        # The last packet always starts at FIFO_RX_CURRENT_ADDR
        # no need to reset FIFO_ADDR_PTR
        self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS) 
                 
                 
    # http://raspi.tv/2013/how-to-use-interrupts-with-python-on-the-raspberry-pi-and-rpi-gpio-part-2
    def handleOnReceive(self, event_source):
        #self.aquire_lock(True)              # lock until TX_Done 
        
        # irqFlags = self.getIrqFlags() should be 0x50
        if (self.getIrqFlags() & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0:
            if self._onReceive:
                payload = self.read_payload()                
                #self.aquire_lock(False)     # unlock when done reading  
                
                self._onReceive(self, payload)
                
        #self.aquire_lock(False)             # unlock in any case.
        
        
    def receivedPacket(self, size=0):
        irqFlags = self.getIrqFlags()
        
        self.implicitHeaderMode(size > 0)
        if size > 0: self.writeRegister(REG_PAYLOAD_LENGTH, size & 0xff) 

        # if (irqFlags & IRQ_RX_DONE_MASK) and \
           # (irqFlags & IRQ_RX_TIME_OUT_MASK == 0) and \
           # (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK == 0):
           
        if (irqFlags == IRQ_RX_DONE_MASK):  # RX_DONE only, irqFlags should be 0x40
            # automatically standby when RX_DONE
            return True
            
        elif self.readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE):
            # no packet received.            
            # reset FIFO address / # enter single RX mode
            self.writeRegister(REG_FIFO_ADDR_PTR, FIFO_RX_BASE_ADDR)
            self.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)
        
            
    def read_payload(self):
        # set FIFO address to current RX address
        # fifo_rx_current_addr = self.readRegister(REG_FIFO_RX_CURRENT_ADDR)
        self.writeRegister(REG_FIFO_ADDR_PTR, self.readRegister(REG_FIFO_RX_CURRENT_ADDR))
        
        # read packet length
        packetLength = self.readRegister(REG_PAYLOAD_LENGTH) if self._implicitHeaderMode else \
                       self.readRegister(REG_RX_NB_BYTES)
                       
        payload = bytearray()
        for i in range(packetLength):
            payload.append(self.readRegister(REG_FIFO))
        
        self.collect()
        return bytes(payload)
                        

    def collect(self):
        gc.collect()
        #if MICROPYTHON:
        #    print('[Memory - free: {}   allocated: {}]'.format(gc.mem_free(), gc.mem_alloc()))
            
        
