#!/bin/sh

sudo sh -c "echo 'module ieee802154_6lowpan +fp' > /sys/kernel/debug/dynamic_debug/control"
sudo sh -c "echo 'module mac802154 +fp' > /sys/kernel/debug/dynamic_debug/control"
sudo sh -c "echo 'module ieee802154 +fp' > /sys/kernel/debug/dynamic_debug/control"
sudo sh -c "echo 'module wpanusb +fp' > /sys/kernel/debug/dynamic_debug/control"
