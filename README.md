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

     chmod +x install test uninstall bpatch
     
Now to install it, run:

     ./install
     
To test it without installing, run:

     ./test
     
To uninstall, run:

      ./uninstall

To install with dkms:

    sudo make dkms
 
The driver will automatically load at startup...

### Bluetooth

####a. Test blocking state:
For kernel 3.13 - 3.16 bluetooth may work without any extra efforts, but for higher kernels it may not work at all. A kernel patch for the bluetooth driver may be needed to get it up and working.

If wifi works after installing the driver but bluetooth doesn't, you should first check if bluetooth is hard/soft blocked. If it is blocked then remove the block, but if that's not the case then we have a patch for you that you can apply to your kernel.

To check if bluetooth is hard/soft blocked, run `rfkill list`. It will show you something like this:

```sh
1: hci0: Bluetooth
	Soft blocked: yes
	Hard blocked: no
```

If it is soft blocked, you can unblock it with:

```sh
sudo rfkill unblock bluetooth
```
If it is hard blocked then check if your PC has some kind of switch to turn it on or may be a key combination or a little haggle in the BIOS settings (you will have to investigate).

If there is no block or unblocking from blocked state didn't do any good, then you should try the [patch](https://github.com/neurobin/MT7630E/wiki/Get-bluetooth-working-in-Linux-kernel--with-mt7630e). 

**Without running all those codes manually you can use the bpatch script to automatically patch it for you.**

####b. Try the bpatch script
The **bpatch** script downloads the kernel source and tries to patch the bluetooth driver and insert the compiled module in the right place automatically. To apply patch with the **bpatch** script all you need to do is run it with root privilege.

```sh
sudo ./bpatch
```
If you want to undo this patch:

```sh
sudo ./bpatch -u
```

If you have already downloaded the kernel source, then give the source directory path as an argument to the **bpatch** script with `-sd` option:

```sh
sudo ./bpatch -sd /path/to/kernel/source/dir
```

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


Secure Boot
---------------
Not being signed, this driver is not expected to work on secure boot.


Source:
-------

      The original source was taken from https://github.com/kuba-moo/mt7630e
      Some patches for extended kernel support are taken from https://github.com/benjarobin/MT7630E
      
Note: Even though the original source was taken from kuba-moo, it no longer resembles that of the original. If you want to apply a patch that works with other sources, be ware that the line number and content may or may not match i.e you will have to be careful applying patches.

<h2><a href="http://neurobin.github.io/MT7630E/">WebPage</a></h2>
