[[_TOC_]]
# NEXMON Firmware Installation and Related Operations

## Model Requirements
* Raspberry Pi 4B
* Raspberry Pi 3B+

## Preparation
* balenaEtcher firmware flashing software
* Official firmware with Linux kernel version 5.10 series
* Kernel headers that match the kernel version

## Enable SSH and Wireless Network
* Use a card reader to enter the boot directory and create a file named `ssh`.
* In the same directory, create a file named `wpa_supplicant.conf` with the following configuration:
```
country=CN
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
ssid="shuyuan_5G"
psk="shuyuan1234"
}
```
## Installation Commands
* `sudo su`
* `apt update`
* `apt install -y git libgmp3-dev gawk qpdf bison flex make automake autoconf libtool texinfo`
* `apt install -y raspberrypi-kernel-headers` If the download fails, try downloading it to your computer, use SCP to upload it, and then use `dpkg` to install.
* `git clone https://github.com/seemoo-lab/nexmon.git` If using SCP, rename the directory to `nexmon`.
* `cd buildtools/isl-0.10; ./configure; make; make install; ln -s /usr/local/lib/libisl.so /usr/lib/arm-linux-gnueabihf/libisl.so.10`
* `cd buildtools/mpfr-3.1.4; autoreconf -f -i; ./configure; make; make install; ln -s /usr/local/lib/libmpfr.so /usr/lib/arm-linux-gnueabihf/libmpfr.so.4`
* `cd /home/pi/nexmon/`
* `source setup_env.sh; make`
* `cd patches/bcm43455c0/7_45_206/`
* `git clone https://github.com/seemoo-lab/nexmon_csi.git` If using SCP, rename the directory to `nexmon_csi`.
* `cd nexmon_csi; make install-firmware;`
* `cd brcmfmac_5.10.y-nexmon/` The driver file supporting CSI capture will be generated in this directory.
* `modinfo brcmfmac` The first line is the path to the original ko file; copy it as `<PATH TO THE DRIVER>`.
* `mv "<PATH TO THE DRIVER>/brcmfmac.ko" "<PATH TO THE DRIVER>/brcmfmac.ko.orig`
* `cd utils/makecsiparams/; make && make install;`
* `cd ../../../../../../utilities/nexutil/; make && make install;`
* `sudo nano /etc/dhcpcd.conf`, add `deny interfaces wlan0`, and make sure to connect the Raspberry Pi to a wired network before this step, as the Pi will lose wireless functionality after this.
* `depmod -a`, execute `iwlist` to check if phy0 supports monitor mode. If supported, the installation is successful.

## Usage
* Install tcpdump to capture the CSI transmitted from hardware UDP to the user. Run `apt install -y tcpdump`.
* Enable monitor mode on the wireless card:
1. `ifconfig wlan0 up`
2. `iw phy \`iw dev wlan0 info | gawk '/wiphy/ {printf "phy" $2}'\` interface add mon0 type monitor`
3. `ifconfig mon0 up`
4. `iwconfig` to check if it is enabled and in monitor mode.
5. `sudo iwconfig mon0 channel 10` to set the channel to 10 using iwconfig.

### Note
To achieve perception, it must meet the requirement of capturing 100 to 1000 packets per second. To address this issue, the target device can run two scripts. For more details, see the repository.
