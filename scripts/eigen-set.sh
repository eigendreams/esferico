#!/bin/bash
update-rc.d hostapd disable
update-rc.d isc-dhcp-server disable

LINEN=$( grep -n "#set-area" /etc/network/interfaces | cut -c1-2)
LINE1="iface wlan0 inet dhcp"
LINE2="wpa-driver wext"
LINE3="wpa-ssid eigen-ap"
LINE4="wpa-psk 5e5a17ac4af8d2318b602f02fdcec2f408776f9078405f9d460b5b84c8f7b23d"

LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE1"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE2"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE3"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE4"@ /etc/network/interfaces

service network-interface restart INTERFACE=wlan0
