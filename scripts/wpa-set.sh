#!/bin/bash
update-rc.d hostapd disable
update-rc.d isc-dhcp-server disable

LINEN=$( grep -n "#set-area" /etc/network/interfaces | cut -c1-2)
LINE1="iface wlan0 inet dhcp"
LINE2="wpa-driver wext"
LINE3="wpa-ssid finder-network"
LINE4="wpa-psk a5b0c0e94a9975fc505a73a41f15b2409f30cbe1bafd6cdad9094dcbb3f886f6"

LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE1"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE2"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE3"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE4"@ /etc/network/interfaces

service network-interface restart INTERFACE=wlan0

