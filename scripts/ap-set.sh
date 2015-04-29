#!/bin/bash
sudo update-rc.d hostapd enable
sudo update-rc.d isc-dhcp-server enable

TLINEN=$( grep -n "#set-area" /etc/network/interfaces | cut -c1-2)
TLINE1="iface wlan0 inet static"
TLINE2="    address 192.168.10.1"
TLINE3="    netmask 255.255.255.0"
TLINE4="up iptables-restore < /etc/iptables.ipv4.nat"

TLINEN=$(($TLINEN + 1))
sed -i "$TLINEN"s@.*@"$TLINE1"@ /etc/network/interfaces
TLINEN=$(($TLINEN + 1))
sed -i "$TLINEN"s@.*@"$TLINE2"@ /etc/network/interfaces
TLINEN=$(($TLINEN + 1))
sed -i "$TLINEN"s@.*@"$TLINE3"@ /etc/network/interfaces
TLINEN=$(($TLINEN + 1))
sed -i "$TLINEN"s@.*@"$TLINE4"@ /etc/network/interfaces

service network-interface restart INTERFACE=wlan0
