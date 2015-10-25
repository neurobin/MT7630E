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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "mt76xx.h"

#define VERSION "1.0.0"
#define MT76x0_FIRMWARE	"mt76x0.bin"
#define MT76x2_FIRMWARE	"mt76x2.bin"

static struct usb_driver mtk_bt_loader;


static int load_rom_patch(struct mtk_loader_date *);
static int load_fw(struct mtk_loader_date *);


static int mtk_usb_reset(struct usb_device *udev)
{
	int ret;

	printk("%s\n", __FUNCTION__);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x01, 
			DEVICE_VENDOR_REQUEST_OUT, 0x01, 0x00, NULL, 0x00, 
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0)
	{
		printk("%s error(%d)\n", __FUNCTION__, ret);
		return ret;
	}

	if (ret > 0)
		ret = 0;

	return ret;
}

static int mtk_io_read32(struct mtk_loader_date *data, u32 reg, u32 *val)
{
	u8 request = data->r_request;
	struct usb_device *udev = data->udev;
	int ret;

	ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), request, 
			DEVICE_VENDOR_REQUEST_IN,0, reg, data->io_buf, 4, 
			CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0) {
		*val = 0xffffffff;
		printk("%s error(%d), reg=%x, value=%x\n",
				__FUNCTION__, ret, reg, *val);
		return ret;
	}

	memmove(val, data->io_buf, 4);

	*val = le32_to_cpu(*val);

	if (ret > 0)
		ret = 0;

	return ret;
}

static int mtk_io_write32(struct mtk_loader_date *data, u32 reg, u32 val)
{
	u16 value, index;
	u8 request = data->w_request;
	struct usb_device *udev = data->udev;
	int ret;

	index = (u16)reg;
	value = val & 0x0000ffff;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), request, 
			DEVICE_VENDOR_REQUEST_OUT, value, index, NULL, 0, 
			CONTROL_TIMEOUT_JIFFIES);

	if (ret < 0) {
		printk("%s error(%d), reg=%x, value=%x\n", 
				__FUNCTION__, ret, reg, val);
		return ret;
	}

	index = (u16)(reg + 2);
	value = (val & 0xffff0000) >> 16;

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), request, 
			DEVICE_VENDOR_REQUEST_OUT, value, index, NULL, 0,
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0) {
		printk("%s error(%d), reg=%x, value=%x\n", 
				__FUNCTION__, ret, reg, val);
		return ret;
	}
	else {
		ret = 0;
	}

	return ret;
}

static int mtk_io_base(struct mtk_loader_date *data, int base)
{
	int ret = 0;

	switch (base) {
	case SYSCTL:
		data->w_request = 0x42;
		data->r_request = 0x47;
		break;
	case WLAN:
		data->w_request = 0x02;
		data->r_request = 0x07;
		break;

	default:
		return -EINVAL;
	}

	return ret;
}

static void mtk_loader(struct mtk_loader_date *data)
{
	const struct firmware	*firmware;
	struct usb_device   *udev = data->udev;
	int ret;

	mtk_io_read32(data, 0x00, &data->chip_id);

	printk ("chip id = %x\n", data->chip_id);

	if (is_mt7630(data) || is_mt7650(data)) {
		data->need_load_fw = 1;
		data->need_load_rom_patch = 0;
		ret = request_firmware (&firmware, MT76x0_FIRMWARE, 
				&udev->dev);
		if (ret < 0) {
		    if (ret == -ENOENT) {
			printk ("Firmware file \"%s\" not found \n", 
					MT76x0_FIRMWARE);
		    }
		    else {
			printk ("Firmware file \"%s\" request failed (err=%d) \n", 
					MT76x0_FIRMWARE, ret);
		    }
		}
		else {
		    printk ("Firmware file \"%s\" Found \n", 
				    MT76x0_FIRMWARE);
		    /* load firmware here */
		    data->firmware = firmware;
		    load_fw(data);
		}
		release_firmware (firmware);
	} 
	else if (is_mt7632(data) || is_mt7662(data)) {
		data->need_load_fw = 0;
		data->need_load_rom_patch = 1;
		data->rom_patch_offset = 0x90000;
		ret = request_firmware (&firmware, MT76x2_FIRMWARE, 
				&udev->dev);
		if (ret < 0) {
		    if (ret == -ENOENT) {
			printk ("Firmware file \"%s\" not found \n", 
					MT76x2_FIRMWARE);
		    }
		    else {
			printk ("Firmware file \"%s\" request failed (err=%d) \n", 
					MT76x2_FIRMWARE, ret);
		    }
		}
		else {
		    printk ("Firmware file \"%s\" Found \n", 
				    MT76x2_FIRMWARE);
		    /* load rom patch here */
		    data->firmware = firmware;
		    data->rom_patch_len = firmware->size;
		    load_rom_patch(data);
		}
		release_firmware (firmware);
	}
	else {
		printk("unknow chip(%x)\n", data->chip_id);
	}
	printk ("%s: done\n", __FUNCTION__);
}

u16 checksume16(u8 *pData, int len)
{
	int sum = 0;

	while (len > 1) {
		sum += *((u16*)pData);

		pData = pData + 2;
		
		if (sum & 0x80000000) 
			sum = (sum & 0xFFFF) + (sum >> 16);

		len -= 2;
	}

	if (len) {
		sum += *((u8*)pData);
	}

	while (sum >> 16) {
		sum = (sum & 0xFFFF) + (sum >> 16);
	}

	return ~sum;
}

static int btmtk_usb_chk_crc(struct mtk_loader_date *data, u32 checksum_len)
{
	int ret = 0;
	struct usb_device *udev = data->udev;
	
	printk("%s\n", __FUNCTION__);

	memmove(data->io_buf, &data->rom_patch_offset, 4);
	memmove(&data->io_buf[4], &checksum_len, 4);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x1, 
			DEVICE_VENDOR_REQUEST_IN, 0x20, 0, data->io_buf, 8,
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0) {
		printk("%s error(%d)\n", __FUNCTION__, ret);
	}

	return ret;
}

static u16 btmtk_usb_get_crc(struct mtk_loader_date *data)
{
	int ret = 0;
	struct usb_device *udev = data->udev;
	u16 crc, count = 0;

	printk("%s\n", __FUNCTION__);

	while (1) {
		ret = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0), 0x01,
				DEVICE_VENDOR_REQUEST_IN, 0x21, 0, 
				data->io_buf, 2, CONTROL_TIMEOUT_JIFFIES);
		if (ret < 0) {
			crc = 0xFFFF;
			printk("%s error(%d)\n", __FUNCTION__, ret);
		}

		memmove(&crc, data->io_buf, 2);

		crc = le16_to_cpu(crc);
	
		if (crc != 0xFFFF)
			break;

		mdelay(100);
	
		if (count++ > 100) {
			printk("Query CRC over %d times\n", count);
			break;
		}
	}

	return crc;
}

static int mtk_usb_reset_wmt(struct mtk_loader_date *data)
{
	int ret = 0;
	
	/* reset command */
	u8 cmd[8] = {0x6F, 0xFC, 0x05, 0x01, 0x07, 0x01, 0x00, 0x04};

	memmove(data->io_buf, cmd, 8);

	printk("%s\n", __FUNCTION__);

	ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0), 
			0x01, DEVICE_CLASS_REQUEST_OUT, 0x12, 0x00, 
			data->io_buf, 8, CONTROL_TIMEOUT_JIFFIES);

	if (ret) {
		printk("%s:(%d)\n", __FUNCTION__, ret);
	}

	return ret;
}

static void load_rom_patch_complete(struct urb *urb)
{

	struct completion *sent_to_mcu_done = (struct completion *)urb->context;

	complete(sent_to_mcu_done);
}

static int load_rom_patch(struct mtk_loader_date *data)
{
	u32 loop = 0;
	u32 value;
	s32 sent_len;
	int ret = 0, total_checksum = 0;
	struct urb *urb;
	u32 patch_len = 0;
	u32 cur_len = 0;
	dma_addr_t data_dma;
	struct completion sent_to_mcu_done;
	int first_block = 1;
	unsigned char phase;
	void *buf;
	char *pos; 
	unsigned int pipe = usb_sndbulkpipe(data->udev, 
			data->bulk_tx_ep->bEndpointAddress);

	if (!data->firmware) {
		printk("%s:please assign a rom patch\n", __FUNCTION__);
		return -1;
	}

load_patch_protect:
	mtk_io_base(data, WLAN);
	mtk_io_read32(data, SEMAPHORE_03, &value);
	loop++;

	if (((value & 0x01) == 0x00) && (loop < 600)) {
		mdelay(1);
		goto load_patch_protect;
	}
	
	mtk_io_write32(data, 0x1004, 0x2c);

	mtk_io_base(data, SYSCTL);

	mtk_io_write32(data, 0x1c, 0x30);
	
	/* Enable USB_DMA_CFG */
	mtk_io_write32(data, 0x9018, 0x00c00020);

	mtk_io_base(data, WLAN);
	
	/* check ROM patch if upgrade */
	mtk_io_read32(data, COM_REG0, &value);

	if ((value & 0x02) == 0x02)
		goto error0;
	
	urb = usb_alloc_urb(0, GFP_ATOMIC);

	if (!urb) {
		ret = -ENOMEM;
		goto error0;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	buf = usb_alloc_coherent(data->udev, UPLOAD_PATCH_UNIT, GFP_ATOMIC, &data_dma);
#else
	buf = usb_buffer_alloc(data->udev, UPLOAD_PATCH_UNIT, GFP_ATOMIC, &data_dma);
#endif

	if (!buf) {
		ret = -ENOMEM;
		goto error1;
	}

	pos = buf;
	printk("loading rom patch");

	init_completion(&sent_to_mcu_done);

	cur_len = 0x00;
	patch_len = data->rom_patch_len - PATCH_INFO_SIZE;
	
	/* loading rom patch */
	while (1) {
		s32 sent_len_max = UPLOAD_PATCH_UNIT - PATCH_HEADER_SIZE;
		sent_len = (patch_len - cur_len) >= sent_len_max ? sent_len_max : (patch_len - cur_len);

		printk("patch_len = %d\n", patch_len);
		printk("cur_len = %d\n", cur_len);
		printk("sent_len = %d\n", sent_len);

		if (sent_len > 0) {
			if (first_block == 1) {
				if (sent_len < sent_len_max) {
					phase = PATCH_PHASE3;
				}
				else {
					phase = PATCH_PHASE1;
				}
				first_block = 0;
			} 
			else if (sent_len == sent_len_max) {
				phase = PATCH_PHASE2;
			}
			else {
				phase = PATCH_PHASE3;
			}

			/* prepare HCI header */
			pos[0] = 0x6F;
			pos[1] = 0xFC;
			pos[2] = (sent_len + 5) & 0xFF;
			pos[3] = ((sent_len + 5) >> 8) & 0xFF;

			/* prepare WMT header */
			pos[4] = 0x01;
			pos[5] = 0x01;
			pos[6] = (sent_len + 1) & 0xFF;
			pos[7] = ((sent_len + 1) >> 8) & 0xFF;

			pos[8] = phase;

			memcpy(&pos[9], data->firmware->data + PATCH_INFO_SIZE + cur_len, sent_len);
			
			printk("sent_len + PATCH_HEADER_SIZE = %d, phase = %d\n", 
					sent_len + PATCH_HEADER_SIZE, phase);

			usb_fill_bulk_urb(urb, 
					data->udev,
					pipe,
					buf,
					sent_len + PATCH_HEADER_SIZE,
					load_rom_patch_complete,
					&sent_to_mcu_done);

			urb->transfer_dma = data_dma;
			urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

			ret = usb_submit_urb(urb, GFP_ATOMIC);
			
			if (ret)
				goto error2;

			if (!wait_for_completion_timeout(&sent_to_mcu_done, msecs_to_jiffies(1000))) {
				usb_kill_urb(urb);
				printk("upload rom_patch timeout\n");
				goto error2;
			}

			printk(".");

			mdelay(200);
			
			cur_len += sent_len;
		} 
		else {
			break;
		}
	}
	
	total_checksum = checksume16((u8 *)data->firmware->data + PATCH_INFO_SIZE, patch_len);
	 
	printk("Send checksum req..\n");

	btmtk_usb_chk_crc(data, patch_len);
	
	mdelay(20);

	if (total_checksum != btmtk_usb_get_crc(data)) {
		printk("checksum fail!, local(0x%x) <> fw(0x%x)\n", 
				total_checksum, btmtk_usb_get_crc(data));
		ret = -1;
		goto error2;
	}

	mdelay(20);

	ret = mtk_usb_reset_wmt(data);

	mdelay(20);

error2:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	usb_free_coherent(data->udev, UPLOAD_PATCH_UNIT, buf, data_dma);
#else
	usb_buffer_free(data->udev, UPLOAD_PATCH_UNIT, buf, data_dma);
#endif
error1:
	usb_free_urb(urb);
error0:
	mtk_io_write32(data, SEMAPHORE_03, 0x1);
	return ret;
}


static int load_fw_iv(struct mtk_loader_date *data)
{
	int ret;
	struct usb_device *udev = data->udev;
	char *buf = kmalloc(64, GFP_ATOMIC);

	memmove(buf, data->firmware->data + 32, 64);

	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0), 0x01,
			DEVICE_VENDOR_REQUEST_OUT, 0x12, 0x0, buf, 64,
			CONTROL_TIMEOUT_JIFFIES);
	if (ret < 0) {
		printk("%s error(%d) step4\n", __FUNCTION__, ret);
		kfree(buf);
		return ret;
	}
	else {
		ret = 0;
	}

	kfree(buf);

	return ret;
}

static void load_fw_complete(struct urb *urb)
{
	struct completion *sent_to_mcu_done = (struct completion *)urb->context;

	complete(sent_to_mcu_done);
}

static int load_fw(struct mtk_loader_date *data)
{
	struct usb_device *udev = data->udev;
	struct urb *urb;
	void *buf;
	u32 cur_len = 0;
	u32 packet_header = 0;
	u32 value;
	u32 ilm_len = 0, dlm_len = 0; 
	u16 fw_ver, build_ver;
	u32 loop = 0;
	dma_addr_t data_dma;
	int ret = 0, sent_len;
	struct completion sent_to_mcu_done;
	unsigned int pipe = usb_sndbulkpipe(data->udev, data->bulk_tx_ep->bEndpointAddress);

	if (!data->firmware) {
		printk("%s:please assign a fw\n", __FUNCTION__);
		return -1;
	}

	printk("bulk_tx_ep = %x\n", data->bulk_tx_ep->bEndpointAddress);


#if 0
loadfw_protect:
	mtk_io_base(data, WLAN);
	mtk_io_read32(data, SEMAPHORE_00, &value);
	loop++;

	if (((value & 0x1) == 0) && (loop < 10000))
		goto loadfw_protect;
#endif


	for(loop = 0; loop < 6; loop++)
	{
		value = 0x00;
		mtk_io_base(data, WLAN);
		mtk_io_read32(data, SEMAPHORE_00, &value);
		//DBGPRINT(RT_DEBUG_TRACE, ("%s: Check FW loading Semaphore, MCUCTL:SEMAPHORE_00 = 0x%08X\n", __FUNCTION__, MacValue));
		if ( (value & 0x1) == 0x01)
		{
			// Continue to check COM_REG0[0]
			break;
		}
		else if ( (value & 0x1) == 0x00)
		{
			// WIFI deiver in FW loading stage, wait 500ms
			 mdelay(500);
		}
	}
		
	

	/* check MCU if ready */
	mtk_io_read32(data, COM_REG0, &value);

	if ((value & 0x01)== 0x01)
		goto error0;
	
	/* Enable MPDMA TX and EP2 load FW mode */
	mtk_io_write32(data, 0x238, 0x1c000000);

	mtk_usb_reset(udev);
	mdelay(100);

	ilm_len = (*(data->firmware->data + 3) << 24) 
		    | (*(data->firmware->data + 2) << 16) 
		    | (*(data->firmware->data +1) << 8) 
		    | (*data->firmware->data);

	dlm_len = (*(data->firmware->data + 7) << 24) 
		    | (*(data->firmware->data + 6) << 16) 
		    | (*(data->firmware->data + 5) << 8) 
		    | (*(data->firmware->data + 4));

	fw_ver = (*(data->firmware->data + 11) << 8) | (*(data->firmware->data + 10));

	build_ver = (*(data->firmware->data + 9) << 8) | (*(data->firmware->data + 8));

	printk ("fw version:%d.%d.%02d ", 
			(fw_ver & 0xf000) >> 8,
			(fw_ver & 0x0f00) >> 8,
			(fw_ver & 0x00ff));

	printk ("build:%x\n", build_ver);

	printk ("build Time =");

	for (loop = 0; loop < 16; loop++) {
		printk ("%c", *(data->firmware->data + 16 + loop));
	}

	printk ("\n");

	printk ("ILM length = %d(bytes)\n", ilm_len);
	printk ("DLM length = %d(bytes)\n", dlm_len);

	mtk_io_base(data, SYSCTL);

	/* U2M_PDMA rx_ring_base_ptr */
	mtk_io_write32(data, 0x790, 0x400230);

	/* U2M_PDMA rx_ring_max_cnt */
	mtk_io_write32(data, 0x794, 0x1);

	/* U2M_PDMA cpu_idx */
	mtk_io_write32(data, 0x798, 0x1);

	/* U2M_PDMA enable */
	mtk_io_write32(data, 0x704, 0x44);

	urb = usb_alloc_urb(0, GFP_ATOMIC);

	if (!urb) {
		ret = -ENOMEM;
		printk ("can not allocate urb\n");
		goto error1;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	buf = usb_alloc_coherent(udev, 14592, GFP_ATOMIC, &data_dma);
#else
	buf = usb_buffer_alloc(udev, 14592, GFP_ATOMIC, &data_dma);
#endif

	if (!buf) {
		ret = -ENOMEM;
		printk ("can not allocate buf\n");
		goto error2;
	}

	printk("loading fw");

	init_completion(&sent_to_mcu_done);

	mtk_io_base(data, SYSCTL);

	cur_len = 0x40;

	/* Loading ILM */
	while (1) {
		sent_len = (ilm_len - cur_len) >= 14336 ? 14336 : (ilm_len - cur_len);

		if (sent_len > 0) {
			packet_header &= ~(0xffffffff);
			packet_header |= (sent_len << 16);
			packet_header = cpu_to_le32(packet_header);

			memmove(buf, &packet_header, 4);
			memmove(buf + 4, data->firmware->data + 32 + cur_len, sent_len);

			/* U2M_PDMA descriptor */
			mtk_io_write32(data, 0x230, cur_len);

			while ((sent_len % 4) != 0) {
				sent_len++;
			}

			/* U2M_PDMA length */
			mtk_io_read32(data, 0x234, &value);
			value &= 0xffff;
			value |= (sent_len<<16);
			mtk_io_write32(data, 0x234, value);

			usb_fill_bulk_urb(urb, 
					udev,
					pipe,
					buf,
					sent_len + 4,
					load_fw_complete,
					&sent_to_mcu_done);

			urb->transfer_dma = data_dma;
			urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

			ret = usb_submit_urb(urb, GFP_ATOMIC);
			
			if (ret)
				goto error3;

			if (!wait_for_completion_timeout(&sent_to_mcu_done, msecs_to_jiffies(1000))) {
				usb_kill_urb(urb);
				printk("upload ilm fw timeout\n");
				goto error3;
			}
			
			/* Check DDONE after bulk transfer */
			loop = 0;

			mtk_io_read32(data, 0x234, &value);
			while (!(value & 0x80000000)) {
				printk ("DDone is not ready, wait for 2ms\n");
				mdelay(2);
				if (loop > 100) {
					printk ("DDone is still not ready, wait for timeout\n");
					break;
				}
				mtk_io_read32(data, 0x234, &value);
				loop++;
			}

			printk(".");

			//mdelay(200);

			cur_len += sent_len;
		} else {
			break;
		}
	}
	printk ("loading ILM done\n");
	
	init_completion(&sent_to_mcu_done);
	cur_len = 0x00;
	
	/* Loading DLM */
	while (1) {
		sent_len = (dlm_len - cur_len) >= 14336 ? 14336 : (dlm_len - cur_len);

		if (sent_len > 0) {
			packet_header &= ~(0xffffffff);
			packet_header |= (sent_len << 16);
			packet_header = cpu_to_le32(packet_header);

			memmove(buf, &packet_header, 4);
			memmove(buf + 4, data->firmware->data + 32 + ilm_len + cur_len, sent_len);
			
			/* U2M_PDMA descriptor */
			mtk_io_write32(data, 0x230, 0x80000 + cur_len);

			while ((sent_len % 4) != 0) {
				printk("sent_len is not divided by 4\n");
				sent_len++;
			}
			
			/* U2M_PDMA length */
			mtk_io_read32(data, 0x234, &value);
			value &= 0xffff;
			value |= (sent_len<<16);
			mtk_io_write32(data, 0x234, sent_len << 16);

			usb_fill_bulk_urb(urb, 
					udev,
					pipe,
					buf,
					sent_len + 4,
					load_fw_complete,
					&sent_to_mcu_done);

			urb->transfer_dma = data_dma;
			urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

			ret = usb_submit_urb(urb, GFP_ATOMIC);
			
			if (ret)
				goto error3;

			if (!wait_for_completion_timeout(&sent_to_mcu_done, msecs_to_jiffies(1000))) {
				usb_kill_urb(urb);
				printk("upload dlm fw timeout\n");
				goto error3;
			}
			
			/* Check DDONE after bulk transfer */
			loop = 0;

			mtk_io_read32(data, 0x234, &value);
			while (!(value & 0x80000000)) {
				printk ("DDone is not ready, wait for 2ms\n");
				mdelay(2);
				if (loop > 250) {
					printk ("DDone is still not ready, wait for timeout\n");
					break;
				}
				mtk_io_read32(data, 0x234, &value);
				loop++;
			}

			printk(".");

			//mdelay(500);

			cur_len += sent_len;

		} 
		else {
			break;
		}
	}
	printk ("loading DLM done\n");
	
	/* upload 64bytes interrupt vector */
	ret = load_fw_iv(data);
	mdelay(100);

	mtk_io_base(data, WLAN);

	/* check MCU if ready */
	loop = 0;

	do {
		mtk_io_read32(data, COM_REG0, &value);

		if (value == 0x01) {
			printk ("loading fw success\n");
			break;
		}
		
		mdelay(10);
		loop++;
	} while (loop <= 100);

	if (loop > 1000)
	{
		printk("wait for 100 times\n");
		ret = -ENODEV;
	}

error3:
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
	usb_free_coherent(udev, 14592, buf, data_dma);
#else
	usb_buffer_free(udev, 14592, buf, data_dma);
#endif
error2:
	usb_free_urb(urb);
error1:
	/* Disbale load fw mode */
	mtk_io_read32(data, 0x238, &value);
	value = value & ~(0x10000000);
	mtk_io_write32(data,  0x238, value);
error0:
	mtk_io_write32(data, SEMAPHORE_00, 0x1);
	return ret;
}

static void usb_vendor_hci_cmd_complete(struct urb *urb)
{
	printk ("%s \n", __FUNCTION__);
	kfree(urb->setup_packet);
	kfree(urb->transfer_buffer);
}

static int usb_send_vendor_hci_cmd(struct usb_device *udev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
#endif /* version 3.4.0 */
	struct urb *urb;
	struct usb_ctrlrequest	*class_request;
	struct mtk_hci_tci_cmd	*vendor_hci_cmd;
	unsigned int pipe;
	int i, err;

	urb = usb_alloc_urb (0, GFP_ATOMIC);
	if (!urb) {
		printk ("%s: allocate usb urb failed!\n",
				__FUNCTION__);
		return -ENOMEM;
	}

	class_request = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (!class_request) {
		printk ("%s: allocate class request failed!\n",
				__FUNCTION__);
		return -ENOMEM;
	}
	
	vendor_hci_cmd = kmalloc(sizeof(struct mtk_hci_tci_cmd), GFP_ATOMIC);
	if (!vendor_hci_cmd) {
		printk ("%s: allocate vendor_hci_cmd failed!\n",
				__FUNCTION__);
		return -ENOMEM;
	}

	vendor_hci_cmd->opcode = 0xfcc3;
	vendor_hci_cmd->len = 0x31; // dec: 49
	vendor_hci_cmd->subcode = 0x01;
	vendor_hci_cmd->pc_time_stamp_l = 0x0;
	vendor_hci_cmd->pc_time_stamp_h = 0x0;
	vendor_hci_cmd->fwRegistry[0] = 0x3fcff;
	vendor_hci_cmd->fwRegistry[1] = 0x80000000;
	vendor_hci_cmd->fwRegistry[2] = 0x763f0e8d;
	//vendor_hci_cmd->fwRegistry[2] = 0x0e8d763f;
	vendor_hci_cmd->fwRegistry[3] = 0x88b805;

	class_request->bRequestType = USB_TYPE_CLASS;
	class_request->bRequest = 0;
	class_request->wIndex = 0;
	class_request->wValue = 0;
	class_request->wLength = __cpu_to_le16(sizeof(struct mtk_hci_tci_cmd));
	
	pipe = usb_sndctrlpipe(udev, 0x0);

	usb_fill_control_urb(urb, udev, pipe, (void *)class_request, 
			vendor_hci_cmd, sizeof(struct mtk_hci_tci_cmd), 
			usb_vendor_hci_cmd_complete, udev);
	/* debug print urb */
	printk ("setup_packet in urb: size=%lu\n", sizeof(struct usb_ctrlrequest));
	for (i=0; i<sizeof(struct usb_ctrlrequest); i++) {
		printk ("%02x ", *((unsigned char *)urb->setup_packet + i));
	}
	printk ("\ntransfer_buffer in urb: size=%lu\n", sizeof(struct mtk_hci_tci_cmd));
	for (i=0; i<sizeof(struct mtk_hci_tci_cmd); i++) {
		printk ("%02x ", *((unsigned char *)urb->transfer_buffer + i));
	}

	printk ("\nsubmit urb:\n");
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		printk ("%s: return=%d\n", __FUNCTION__, err);
		kfree(urb->setup_packet);
		kfree(urb->transfer_buffer);
	}
	else {
		printk ("%s: return=%d\n", __FUNCTION__, err);
		usb_mark_last_busy(udev);
	}

	usb_free_urb(urb);
	return err;
}

static int mtk_bt_loader_probe(struct usb_interface *intf,
			    const struct usb_device_id *id)
{
	struct mtk_loader_date *data;
	struct usb_endpoint_descriptor *ep_desc;
#if 0
	u8 	request;
	u16	value, index;
#endif
	int i, ret;

	/* interface numbers are hardcoded in the spec */
	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->intr_ep && usb_endpoint_is_int_in(ep_desc)) {
			data->intr_ep = ep_desc;
			continue;
		}

		if (!data->bulk_tx_ep && usb_endpoint_is_bulk_out(ep_desc)) {
			data->bulk_tx_ep = ep_desc;
			continue;
		}

		if (!data->bulk_rx_ep && usb_endpoint_is_bulk_in(ep_desc)) {
			data->bulk_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->intr_ep || !data->bulk_tx_ep || !data->bulk_rx_ep) {
		kfree(data);
		return -ENODEV;
	}

	data->udev = interface_to_usbdev(intf);
	data->intf = intf;

	data->io_buf = kmalloc(256, GFP_ATOMIC);

	mtk_io_base(data, WLAN);

	/* load fw or rom_patch */
	mtk_loader(data);

	usb_set_intfdata(intf, data);

#if 1
	/* send vendor specific hci tci command */
	ret = usb_send_vendor_hci_cmd(data->udev);
#endif

#if 0
	/* re-enumerate usb */
	request = 0x63;
	value = 0x763e;
	index = 0x0e8d;
	ret = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0), request, 
			DEVICE_VENDOR_REQUEST_OUT, value, index, NULL, 0, 
			CONTROL_TIMEOUT_JIFFIES);
#endif
	if (ret < 0) {
		printk("%s: ret = %d\n", __FUNCTION__, ret);
		return 0;
	}
	printk("%s: ret = %d\n", __FUNCTION__, ret);

	return 0;
}

static void mtk_bt_loader_disconnect(struct usb_interface *intf)
{
	struct mtk_loader_date *data = usb_get_intfdata(intf);
	//struct hci_dev *hdev;

	printk("%s\n", __FUNCTION__);

	if (!data)
		return;

	usb_set_intfdata(data->intf, NULL);

	kfree(data->io_buf);
	kfree(data);
}

static struct usb_device_id mtk_bt_loader_tbl[] = {
	/* Mediatek MT7650 */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7650, 0xe0, 0x01, 0x01) },
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7650, 0xff, 0xff, 0xff) },
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7630, 0xe0, 0x01, 0x01) },
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7630, 0xff, 0xff, 0xff) },
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x763e, 0xe0, 0x01, 0x01) },
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x763e, 0xff, 0xff, 0xff) },
	/* Mediatek MT662 */
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7662, 0xe0, 0x01, 0x01) },
	{ USB_DEVICE_AND_INTERFACE_INFO(0x0e8d, 0x7632, 0xe0, 0x01, 0x01) },
	{ }	/* Terminating entry */
};

static struct usb_driver mtk_bt_loader = {
	.name		= "mtk_bt_loader",
	.probe		= mtk_bt_loader_probe,
	.disconnect	= mtk_bt_loader_disconnect,
	.id_table	= mtk_bt_loader_tbl,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
module_usb_driver(mtk_bt_loader);
#else
static int __init mtk_bt_loader_init(void)
{
	printk("mtk bt fw loader ver %s", VERSION);

	return usb_register(&mtk_bt_loader);
}

static void __exit mtk_bt_loader_exit(void)
{
	usb_deregister(&mtk_bt_loader);
}

module_init(mtk_bt_loader_init);
module_exit(mtk_bt_loader_exit);
#endif

MODULE_DESCRIPTION("Mediatek Bluetooth USB driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE(MT76x0_FIRMWARE);
MODULE_FIRMWARE(MT76x2_FIRMWARE);
