[[_TOC_]]
# NEXMON 固件安装以及相关操作

## 型号需求
*raspiberry 4B*
*raspiberry 3B+*

## 准备工作
* balenaEtcher 固件烧录软件
* linux内核版本为5.10系列的官方固件
* 与内核版本匹配的内核头文件

## 开启ssh以及无线入网
* 使用读卡器进入boot目录，新建ssh文件
* 在同一个目录下新建wpa_suppliacant.conf文件，文件配置参考如下：
```
country=CN
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1

network={
ssid="shuyuan_5G"
psk="shuyuan1234"
}
```

## 安装指令
* `sudo su`
* `apt update`
* `apt install -y git libgmp3-dev gawk qpdf bison flex make automake autoconf libtool texinfo`
* `apt install -y raspberrypi-kernel-headers` 这里如果下载失败就尝试先下到自己电脑上，SCP 上传，之后使用 dpkg 进行安装。
* `git clone https://github.com/seemoo-lab/nexmon.git` ，通过scp上传的话要将目录名改为 nexmon。
* ``cd buildtools/isl-0.10; ./configure; make, make install; ln -s /usr/local/lib/libisl.so /usr/lib/arm-linux-gnueabihf/libisl.so.10``
* ``cd buildtools/mpfr-3.1.4,autoreconf -f -i; ./configure, make, make install;ln -s /usr/local/lib/libmpfr.so /usr/lib/arm-linux-gnueabihf/libmpfr.so.4``
* `cd /home/pi/nexmon/`
* `source setup_env.sh;make`
* `cd patches/bcm43455c0/7_45_206/`
* `git clone https://github.com/seemoo-lab/nexmon_csi.git `,通过scp上传的话要将目录名改为 nexmon_csi。
* `cd nexmon_csi;make install-firmware;`
* `cd brcmfmac_5.10.y-nexmon/`,支持捕获CSI的ko驱动文件在此目录下生成。
* `modinfo brcmfmac` 第一行是原始ko文件地址，copy下来作为<PATH TO THE DRIVER>。
* `mv "<PATH TO THE DRIVER>/brcmfmac.ko" "<PATH TO THE DRIVER>/brcmfmac.ko.orig`
* `cd utils/makecsiparams/;make & make instal ;`
* `cd ../../../../../../utilities/nexutil/;make && make install ;`
* `sudo nano/etc/dhcpcd.conf` ,添加 `denydinterfaces wlan0` ，再进行这一步之前需要网线接入树莓派，因为这一步过后树莓派将丧失无线功能。
* `depmod -a` ，执行 `iwlist` 查看phy0 是否支持monitor模式，若支持则安装成功。 

## 使用
* 安装tcpdumap 捕获硬件UDP向用户传输的CSI。执行`apt inatll -y tcpdump`
* 开启网卡的监控模式
1. `ifconfig wlan0 up`
2. ``iw phy `iw dev wlan0 info | gawk '/wiphy/ {printf "phy" $2}'` interface add mon0 type monitor ``
3. `ifconfig mon0 up`
4. `iwconfig` 检查是否开启以及是否在监控模式
5. `sudo iwconfig mon0 channel 10` 使用 iwconfig 将信道配置为10。

### 注意
实现感知必须满足每秒可以抓100到1000个包，针对此问题，目标设备可以跑两个脚本可以解决此问题，详见仓库。
