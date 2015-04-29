#!/bin/bash

cd /home/ubuntu

echo "nameserver 8.8.8.8" >> /etc/resolvconf/resolv.conf.d/tail

echo "#" >> .bashrc
echo "#" >> .bashrc
echo "#" >> .bashrc
echo "source /opt/ros/indigo/setup.bash" >> .bashrc
echo "source ~/workspace/catkin/devel/setup.bash" >> .bashrc
echo "export ROS_WORKSPACE=$ROS_WORKSPACE:~/workspace/catkin" >> .bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/workspace/catkin/src" >> .bashrc
echo "export ROS_HOSTNAME=arm" >> .bashrc
echo "export ROS_MASTER_URI=http://arm:11311" >> .bashrc
echo "export EDITOR='nano -w'" >> .bashrc
echo "export ROSLAUNCH_SSH_UNKNOWN=1" >> .bashrc

echo "#" >> /etc/network/interfaces
echo "#" >> /etc/network/interfaces
echo "#" >> /etc/network/interfaces
echo "# WiFi Example" >> /etc/network/interfaces
echo "auto wlan0" >> /etc/network/interfaces
echo "allow-hotplug wlan0" >> /etc/network/interfaces
echo "#ap-set" >> /etc/network/interfaces
echo "#iface wlan0 inet static" >> /etc/network/interfaces
echo "#    address 192.168.10.1" >> /etc/network/interfaces
echo "#    netmask 255.255.255.0" >> /etc/network/interfaces
echo "#up iptables-restore < /etc/iptables.ipv4.nat" >> /etc/network/interfaces
echo "#wpa-set" >> /etc/network/interfaces
echo "#iface wlan0 inet dhcp" >> /etc/network/interfaces
echo "#wpa-driver wext" >> /etc/network/interfaces
echo "#wpa-ssid finder-network" >> /etc/network/interfaces
echo "#wpa-psk a5b0c0e94a9975fc505a73a41f15b2409f30cbe1bafd6cdad9094dcbb3f886f6" >> 
/etc/network/interfaces
echo "#set-area" >> /etc/network/interfaces
echo "iface wlan0 inet dhcp" >> /etc/network/interfaces
echo "wpa-driver wext" >> /etc/network/interfaces
echo "wpa-ssid finder-network" >> /etc/network/interfaces
echo "wpa-psk a5b0c0e94a9975fc505a73a41f15b2409f30cbe1bafd6cdad9094dcbb3f886f6" >> 
/etc/network/interfaces

echo "optargs=quiet drm.debug=7 capemgr.enable_partno=BB-I2C1,BB-I2C0,BB-I2C2,BB-UART4,BB-UART5" >> /uEnv.txt
echo "cape_disable=capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN" >> /uEnv.txt