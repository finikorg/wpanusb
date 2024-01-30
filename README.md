===========
QUICK HOWTO
===========

Prerequisites
-------------
QuarkSE devboard is connected to Linux PC with 2 USB cables and flashed
with wpanusb application.

Building Linux kernel wpanusb driver
------------------------------------
1. Make sure you have Linux kernel headers installed

   .. code-block:: console

     $ ls /lib/modules/`uname -r`/build

2. Type make, below is output for the latest Ubuntu

   .. code-block:: console

      $ make
      make -C /lib/modules/`uname -r`/build M=$PWD
      make[1]: Entering directory '/usr/src/linux-headers-4.4.0-38-generic'
        CC [M]  /usr/local/src/ieee802154/wpanusb/wpanusb.o
        Building modules, stage 2.
        MODPOST 1 modules
        CC      /usr/local/src/ieee802154/wpanusb/wpanusb.mod.o
        LD [M]  /usr/local/src/ieee802154/wpanusb/wpanusb.ko
      make[1]: Leaving directory '/usr/src/linux-headers-4.4.0-38-generic'

Loading wpanusb
---------------
You can load driver with insmod given that all dependency are loaded, otherwise use
provided modprobe.sh script

.. code-block:: console

  $ sudo scripts/modprobe.sh

wpan0 device should appear in the network devices list

Configuring 6lowpan address
---------------------------
There is a script helping to configure 6lowpan address

.. code-block:: console

  $ sudo scripts/lowpan.sh

lowpan0 network device should appear and IPv6 address can be checked with

.. code-block:: console

  $ ip addr show dev lowpan0
  59: lowpan0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1280 qdisc noqueue state UNKNOWN group default qlen 1
      link/[825] 92:05:91:9b:5a:2c:34:41 brd ff:ff:ff:ff:ff:ff:ff:ff
      inet6 fe80::9005:919b:5a2c:3441/64 scope link
         valid_lft forever preferred_lft forever

There is link local IPv6 address which can be used for ping6.
