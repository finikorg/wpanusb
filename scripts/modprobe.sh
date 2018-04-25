#!/bin/sh

modprobe mac802154
modprobe ieee802154_socket
insmod ./wpanusb.ko
