# WiNCDN 
## WiNCDN-Robot Usage Instructions
### Notes:
1. Only applicable to the source code of the yahboom X3 Plus.
2. The code here is for offline testing, using UDP for communication.

### Usage:
1. Execute `roslaunch yahboomcar_nav laser_usb_bringup.launch`
2. Execute `roslaunch yahboomcar_nav yahboomcar_navigation.launch use_rviz:=true map:=shuyuan_6_7` replacing it with your constructed map.
3. Execute `rec_new.py`. The robot will move to the target point after obtaining the target coordinates and rotate to capture images to send to the server.

## WiNCDN—Sensor Usage Instructions
### Notes:
1. All open-source code in this repository is for offline testing.
2. This repository will be continuously updated.

### Usage:
1. Refer to `raspi_nexmon.md` to configure the environment for Raspberry Pi 4B.
2. Compile and flash the source code from the `listen_ack/` directory to the ESP32-S3/C3 (the MAC address in `inject_data.py` needs to be changed to a fake MAC address).
3. Connect the ESP32 device to the Raspberry Pi via USB.
4. After setting the Raspberry Pi to monitor mode and the correct channel, use `inject_data.py` to inject fake packets into the target phone (the target phone's MAC address and the fake sending address need to be specified in the script).
5. Use `collect_data.py` to collect the CPU cycles and RSSI recorded by the ESP32, and calculate the CPU delay.
