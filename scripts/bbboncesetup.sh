#!/bin/bash

echo "nameserver 8.8.8.8" >> /etc/resolv.conf

apt-get update
apt-get -y upgrade

apt-get install -y build-essential g++ python-setuptools python2.7-dev
wget -c https://raw.github.com/RobertCNelson/tools/master/pkgs/dtc.sh
chmod +x dtc.sh
./dtc.sh

update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
apt-get update
apt-get -y install ros-indigo-ros-comm
apt-get -y install ros-indigo-ros-base

apt-get -y install libspnav-dev
apt-get -y install libusb-dev
apt-get -y install aptitude

apt-get -y install ipython python-opencv python-scipy python-numpy python-pygame python-setuptools 
python-pip
pip install https://github.com/sightmachine/SimpleCV/zipball/develop

ntpdate pool.ntp.org
apt-get -y install build-essential python-dev python-setuptools python-pip python-smbus -y
pip install Adafruit_BBIO
pip install pyserial

apt-get -y install tightvncserver x11vnc git ssh python-numpy

apt-get -y install ros-indigo-tf2*
apt-get -y install ros-indigo-diagnostics*
apt-get -y install ros-indigo-spacenav*

git clone https://github.com/eigendreams/workspace
source /opt/ros/indigo/setup.bash
export ROS_WORKSPACE=$ROS_WORKSPACE:/home/ubuntu/workspace/catkin
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/workspace/catkin/src
cd /home/ubuntu/workspace/catkin
rm -rf src/hector*
rm -rf src/hokuyo*
rm -rf src/urg*
rm -rf src/audio*
rm -rf src/rosserial*
catkin_make

cd /home/ubuntu

chmod 777 -R *

apt-get update
apt-get dist-upgrade
apt-get -y install gcc build-essential linux-headers-generic linux-headers-`uname -r`

wget http://node-arm.herokuapp.com/node_latest_armhf.deb
sudo dpkg -i node_latest_armhf.deb

sudo usermod -a -G tty ubuntu
sudo usermod -a -G dialout ubuntu
