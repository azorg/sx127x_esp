#import esp
#esp.osdebug(None)
import gc
import time
#import utime
#import network
#import socket
#import framebuf
from lora import LORA
gc.collect()

# setup Wi-Fi network
import network
sta_if = network.WLAN(network.STA_IF)
sta_ap = network.WLAN(network.AP_IF)

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


def on_receive(lora, payload):
    lora.blink()
            
    payload_string = payload.decode()
    rssi = lora.packetRssi()
    snr  = lora.packetSnr()
    print("*** Received message ***\n{}".format(payload_string))
    print("RSSI={}, SNR={}\n".format(rssi, snr))


lora = LORA() # init LoRa subsystem
lora.setFrequency(433000,000)  # kHz, Hz
lora.setTxPower(13, True)      # power +13dBm (RFO pin if False or PA_BOOST pin if True)
#lora.setHighPower(False)      # add +3 dB (up to +20 dBm power on PA_BOOST pin)
lora.setSignalBandwidth(125e3) # BW [7.8e3...500e3] Hz
lora.setSpreadingFactor(10)    # SF 6...12
lora.setLDR(False)             # Low Datarate Optimize
lora.setCodingRate(5)          # 5..8
lora.setPreambleLength(8)      # 6..65000 (8 by default)
lora.setSyncWord(0x12)         # allways 0x12
lora.enableCRC(False)          # CRC off
lora.collect()

if 1:
    # reseiver
    lora.onReceive(on_receive) # register the receive callback
    lora.receive() # go into receive mode

else:
    # transmitter
    while True:
        lora.blink()
        lora.println("Hello!")
        time.sleep_ms(1000)


