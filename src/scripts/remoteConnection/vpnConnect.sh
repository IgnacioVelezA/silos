#!/bin/bash
printf 'ignacio.velez\nNacho16nico5\ny' | /opt/cisco/anyconnect/bin/vpn -s connect vpn.uchile.cl

echo "Connected with IP:"
ipaddr=$(ip -4 addr show cscotun0 | grep -oP "(?<=inet ).*(?=/)")
echo $ipaddr

exit
