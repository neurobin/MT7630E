.PHONY: all clean install uninstall

KERNEL ?= `uname -r`
KDIR ?= /lib/modules/$(KERNEL)/build
DST_DIR ?= /lib/modules/$(KERNEL)/kernel/drivers/net/wireless/
PKG_VER ?= `sed -n 's/^[[:blank:]]*PACKAGE_VERSION=\([^[:blank:]]*\).*/\1/p' dkms.conf`

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR)/rt2x00 modules
	$(MAKE) -C $(KDIR) M=$(CURDIR)/btloader modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR)/rt2x00 clean
	$(MAKE) -C $(KDIR) M=$(CURDIR)/btloader clean

install:
	cp -v firmware/*/* /lib/firmware/
	cp rt2x00/mt7630e.ko $(DST_DIR)
	cp btloader/mt76xx.ko $(DST_DIR)
	depmod $(KERNEL)

uninstall:
	rm -vf /lib/firmware/mt76x0.bin /lib/firmware/MT7650E234.bin
	rm -vf $(DST_DIR)/mt7630e.ko
	rm -vf $(DST_DIR)/mt76xx.ko
	depmod $(KERNEL)

dkms:
	cp -v firmware/*/* /lib/firmware/
	cp -R . /usr/src/mt7630e-$(PKG_VER)
	dkms add -m mt7630e -v $(PKG_VER)
	dkms build -m mt7630e -v $(PKG_VER) -k $(KERNEL)
	dkms install -m mt7630e -v $(PKG_VER) -k $(KERNEL)

