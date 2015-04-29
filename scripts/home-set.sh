#!/bin/bash
update-rc.d hostapd disable
update-rc.d isc-dhcp-server disable

LINEN=$( grep -n "#set-area" /etc/network/interfaces | cut -c1-2)
LINE1="iface wlan0 inet dhcp"
LINE2="wpa-driver wext"
LINE3="wpa-ssid eigen"
LINE4="wpa-psk 08fdfc2e55d6ac85bbf3ca315709502d442b58a4f8eae43dcdaf0424275bd75d"

LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE1"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE2"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE3"@ /etc/network/interfaces
LINEN=$(($LINEN + 1))
sed -i "$LINEN"s@.*@"$LINE4"@ /etc/network/interfaces

service network-interface restart INTERFACE=wlan0

