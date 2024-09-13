#!/bin/bash
printf 'USER\nPASSWORD\ny' | /opt/cisco/anyconnect/bin/vpn -s connect vpn.uchile.cl

echo "Connected with IP:"
ipaddr=$(ip -4 addr show tun0 | grep -oP "(?<=inet ).*(?=/)")
echo $ipaddr

exit
