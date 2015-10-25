/*
 *  MediaTek Bluetooth firmware loader
 *
 *  Copyright (C) 2013, MediaTek co.
 *
 *  <hugo.lee@mediatek.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *  or on the worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 */

#ifndef _MT76XX_H_
#define _MT76XX_H_

/* Memory map for MTK BT */

/* SYS Control */
#define SYSCTL		0x400000

/* WLAN */
#define WLAN		0x410000

/* MCUCTL */
#define INT_LEVEL	0x0718
#define COM_REG0	0x0730
#define SEMAPHORE_00	0x07B0
#define SEMAPHORE_01	0x07B4
#define SEMAPHORE_02	0x07B8
#define SEMAPHORE_03	0x07BC

/* Chip definition */

#define CONTROL_TIMEOUT_JIFFIES ((300 * HZ) / 100)
#define DEVICE_VENDOR_REQUEST_OUT	0x40
#define DEVICE_VENDOR_REQUEST_IN	0xc0
#define DEVICE_CLASS_REQUEST_OUT	0x20

#define BTUSB_MAX_ISOC_FRAMES	10
#define BTUSB_INTR_RUNNING	0
#define BTUSB_BULK_RUNNING	1
#define BTUSB_ISOC_RUNNING	2
#define BTUSB_SUSPENDING	3
#define BTUSB_DID_ISO_RESUME	4

/* ROM Patch */
#define PATCH_HCI_HEADER_SIZE 4
#define PATCH_WMT_HEADER_SIZE 5
#define PATCH_HEADER_SIZE (PATCH_HCI_HEADER_SIZE + PATCH_WMT_HEADER_SIZE)
#define UPLOAD_PATCH_UNIT 2048
#define PATCH_INFO_SIZE 30
#define PATCH_PHASE1 1
#define PATCH_PHASE2 2
#define PATCH_PHASE3 3

struct mtk_hci_tci_cmd {
	unsigned short	opcode;
	unsigned char	len;
	unsigned char	subcode;
	unsigned int	pc_time_stamp_l;
	unsigned int	pc_time_stamp_h;
	unsigned int	fwRegistry[10];
} __attribute__ ((packed));

struct mtk_loader_date {
	struct usb_device    *udev;
	struct usb_interface *intf;
	
	struct usb_endpoint_descriptor *intr_ep;
	struct usb_endpoint_descriptor *bulk_tx_ep;
	struct usb_endpoint_descriptor *bulk_rx_ep;
	struct usb_endpoint_descriptor *isoc_tx_ep;
	struct usb_endpoint_descriptor *isoc_rx_ep;
	
	/* request for different io operation */
	u8 w_request;
	u8 r_request;

	/* io buffer for usb control transfer */
	char *io_buf;

	struct semaphore fw_upload_sem;

	const struct firmware *firmware;
	u32 chip_id;
	u8 need_load_fw;
	u8 need_load_rom_patch;
	u32 rom_patch_offset;
	u32 rom_patch_len;
};

static inline int is_mt7630(struct mtk_loader_date *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76300000);
}

static inline int is_mt7650(struct mtk_loader_date *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76500000);
}

static inline int is_mt7632(struct mtk_loader_date *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76320000);
}

static inline int is_mt7662(struct mtk_loader_date *data)
{
	return ((data->chip_id & 0xffff0000) == 0x76620000);
}

#define MT_REV_LT(_data, _chip, _rev)	\
	is_##_chip(_data) && (((_data)->chip_id & 0x0000ffff) < (_rev))

#define MT_REV_GTE(_data, _chip, _rev)	\
	is_##_chip(_data) && (((_data)->chip_id & 0x0000ffff) >= (_rev))

#endif /* _MT76XX_H_ */
