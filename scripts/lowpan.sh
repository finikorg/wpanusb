#!/bin/sh

PHY=`iwpan phy | grep wpan_phy | cut -d' ' -f2`

CHAN=${1:-20}

echo 'Using phy' $PHY 'channel' $CHAN

iwpan dev wpan0 set pan_id 0xabcd
iwpan dev wpan0 set short_addr 0xbeef
iwpan phy $PHY set channel 0 $CHAN
ip link add link wpan0 name lowpan0 type lowpan
ip link set wpan0 up
ip link set lowpan0 up
