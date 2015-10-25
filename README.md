MT7630E
=======
Easy installation package for the official driver at http://www.mediatek.com/en/downloads/mt7630-pcie/


***************************************************************************************************************************
                                  Mediatek MT7630E Combo Linux Driver User Guide
***************************************************************************************************************************

1. Component
------------

* rt2x00: Wi-Fi driver source code
* btloader: Bluetooth firmware loader source code
* firmware: Firmware binary code (MT7650E234.bin is for Wi-Fi, mt76x0.bin is for Bluetooth)


2. Installation
----------------

First give some file execution permission:

     chmod +x install test uninstall
     
Now to install it, run:

     ./install
     
To test it without installing, run:

     ./test
     
To uninstall, run:

      ./uninstall

To install with dkms:

    sudo make dkms
 
The driver will automatically load at startup.... 

Source:
-------

      The original source was taken from https://github.com/kuba-moo/mt7630e
      Some patches for extended kernel support is taken from https://github.com/benjarobin/MT7630E

<h2><a href="http://neurobin.github.io/MT7630E/">WebPage</a></h2>
