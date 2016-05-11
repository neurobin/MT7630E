MT7630E
=======
Easy installation package for the official driver at http://www.mediatek.com/en/downloads1/downloads/pcimpcicbrt2860rt2760rt2890/


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

3. Troubleshooting when upgrading kernel
----------------
###3.1 Install script

If you installed the driver with the `install` script, you will have to reinstall the drivers when you upgrade your kernel.

To do so, run: 
```sh
./uninstall
make clean
./install
```

If you don't uninstall and clean, you will face problems like this: 
```
modprobe: ERROR: could not insert 'mt7630e': Exec format error
modprobe: ERROR: could not insert 'mt76xx': Exec format error
```

###3.2 DKMS 

If you install with dkms then you won't need to uninstall/install for minor kernel updates. Major kernel updates may still need update/uninstall/install though.

Source:
-------

      The original source was taken from https://github.com/kuba-moo/mt7630e
      Some patches for extended kernel support are taken from https://github.com/benjarobin/MT7630E
      
Note: Even though the original source was taken from kuba-moo, it no longer resembles that of the original. If you want to apply a patch that works with other sources, be ware that the line number and content may or may not match i.e you will have to be careful applying patches.

<h2><a href="http://neurobin.github.io/MT7630E/">WebPage</a></h2>
