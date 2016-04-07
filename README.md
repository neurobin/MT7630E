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

3. Troubleshooting when upgrading kernel
----------------
When you upgrade your kernel, you will have to reinstall the drivers.

To do so, run: 
```sh
./uninstall
make clean
./install
```

If you don't uninstall and clean, you will problems like this: 
```
modprobe: ERROR: could not insert 'mt7630e': Exec format error
modprobe: ERROR: could not insert 'mt76xx': Exec format error
```


Source:
-------

      The original source was taken from https://github.com/kuba-moo/mt7630e
      Some patches for extended kernel support are taken from https://github.com/benjarobin/MT7630E
      
Note: Even though the original source was taken from kuba-moo, it no longer resembles that of the original. If you want to apply a patch that works with other sources, be ware that the line number and content may or may not match i.e you will have to be careful applying patches.

<h2><a href="http://neurobin.github.io/MT7630E/">WebPage</a></h2>
