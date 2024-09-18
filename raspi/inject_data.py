from scapy.all import *
from scapy.layers.dot11 import Dot11, RadioTap, Dot11QoS
import time

# AP and STA MAC addresses
ap_mac = 'aa:bb:cc:11:22:33'  # Replace with the actual AP MAC address
sta_mac = '1c:b7:96:c6:6b:58'  # Replace with the actual STA MAC address
sta2_mac='dc:33:3d:7b:da:f0'
real_ap_mac = 'e8:9f:80:d4:a5:7a'
broadcast_address = 'ff:ff:ff:ff:ff:ff'

# Construct the QoS data frame from AP to STA
radio_tap = RadioTap()
dot11 = Dot11(type=2, subtype=4, addr1=sta2_mac, addr2=ap_mac, addr3=ap_mac)
dot11_qos = Dot11QoS()  # No LLC/SNAP header is added
payload = "Data Payload"  # Dummy payload
qos_frame = radio_tap / dot11 / dot11_qos / Raw(load=payload)

ssid = 'OpenWrt'  
beacon = RadioTap() / Dot11(type=0, subtype=8, addr1=sta2_mac, addr2=ap_mac, addr3=ap_mac) / Dot11Beacon(cap='ESS+privacy') / Dot11Elt(ID='SSID', info=ssid)
supported_rates = Dot11Elt(ID='Rates', info=b'\x82\x84\x8b\x96\x24\x30\x48\x6c')  # 1, 2, 5.5, 11, 18, 24, 36, 54 Mbps
#tim = Dot11Elt(ID='TIM', info=b'\x00\x01\xff\xff')
tim = Dot11Elt(ID='TIM', info=b'\x06\x00\x00\x01\x00\x01\x01')
beacon /= supported_rates / tim

# Set the wireless interface name
iface = "mon0"  # Replace with your actual wireless interface name

# Loop to send the data frame 100 times and then send the beacon frame once
while True:
    #for _ in range(1):
#  sendp(beacon, iface=iface, count=1, verbose=0)  # Then send beacon frame once
#  time.sleep(0.01)  
  # for _ in range(5000):  # Send qos_frame 100 times
  # while True:
       sendp(qos_frame, iface=iface, count=1, verbose=0)
       time.sleep(0.005)

