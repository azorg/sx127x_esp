#import esp
#esp.osdebug(None)
import gc
import time
#import utime
#import network
#import socket
#import framebuf
import sx127x
gc.collect()

# setup Wi-Fi network
import network
sta_if = network.WLAN(network.STA_IF)
sta_ap = network.WLAN(network.AP_IF)
sta_if.active(False)
sta_ap.active(False)

"""
if True:
    # create AP
    sta_if.active(False)
    sta_ap.active(True)
    sta_ap.config(essid="esp-net", authmode=network.AUTH_WPA_WPA2_PSK, password="********")
    # IP address, netmask, gateway, DNS
    sta_ap.ifconfig(['192.168.1.1', '255.255.255.0', '192.168.1.1', '8.8.8.8'])

else:
    # connnect to AP
    sta_ap.active(False)
    sta_if.active(True)
    sta_if.connect('veonet', '********')
    sta_if.ifconfig(['192.168.0.13', '255.255.255.0', '192.168.0.254', '192.168.0.254'])
    #sta_if.ifconfig()
"""

def on_receive(tr, payload):
    tr.blink()
            
    payload_string = payload.decode()
    rssi = tr.rssi()
    snr  = tr.snr()
    print("*** Received message ***\n{}".format(payload_string))
    print("RSSI={}, SNR={}\n".format(rssi, snr))


tr = sx127x.RADIO(mode=sx127x.FSK) # init SX127x RF module
tr.setFrequency(433000,000) # kHz, Hz
tr.setPower(10, True)       # power +10dBm (RFO pin if False or PA_BOOST pin if True)
#tr.setHighPower(False)     # add +3 dB (up to +20 dBm power on PA_BOOST pin)
tr.enableCRC(False)         # CRC off

if tr.isLora(): # LoRa mode
    tr.setBW(125e3)   # BW [7.8e3...500e3] Hz
    tr.setSF(10)      # SF 6...12
    tr.setLDR(False)  # Low Datarate Optimize
    tr.setCR(5)       # 5..8
    tr.setPreamble(8) # 6..65000 (8 by default)
    tr.setSW(0x12)    # allways 0x12

else: # FSK/OOK mode
    tr.bitrate(4800)  # bit/s
    tr.fdev(5000.)    # frequency deviation [Hz] 
    tr.rxBW(10.4)     # 2,6...250 kHz
    tr.afcBW(50.0)    # 2,6...250 kHz
    tr.fixedLen(False)
    tr.enableAFC(True)

tr.collect()

if 0:
    # reseiver
    tr.onReceive(on_receive) # set the receive callback
    tr.receive() # go into receive mode

else:
    # transmitter
    while True:
        tr.blink()
        tr.send("Hello!")
        time.sleep_ms(1000)


