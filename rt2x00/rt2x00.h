/*
	Copyright (C) 2010 Willow Garage <http://www.willowgarage.com>
	Copyright (C) 2004 - 2010 Ivo van Doorn <IvDoorn@gmail.com>
	Copyright (C) 2004 - 2009 Gertjan van Wingerde <gwingerde@gmail.com>
	<http://rt2x00.serialmonkey.com>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the
	Free Software Foundation, Inc.,
	59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
	Module: rt2x00
	Abstract: rt2x00 global information.
 */

#ifndef RT2X00_H
#define RT2X00_H

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <linux/firmware.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/etherdevice.h>
#include <linux/input-polldev.h>
#include <linux/kfifo.h>
#include <linux/hrtimer.h>
#include <linux/version.h>

#include <net/mac80211.h>

#include "rt2x00debug.h"
#include "rt2x00dump.h"
#include "rt2x00leds.h"
#include "rt2x00reg.h"
#include "rt2x00queue.h"

/*
 * Module information.
 */
#define DRV_VERSION	"2.3.4"
#define DRV_PROJECT	"http://rt2x00.serialmonkey.com"


#ifndef GNU_PACKED
#define GNU_PACKED	__attribute__ ((packed))
#endif // GNU_PACKED //

typedef void VOID;
typedef signed long LONG;
typedef signed char CHAR;
typedef int INT32;
typedef unsigned int UINT32;
typedef unsigned char BOOLEAN;
typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned char UCHAR;
typedef unsigned char BOOLEAN;
typedef unsigned short USHORT;
typedef unsigned int UINT;
typedef unsigned long ULONG;
typedef char *PSTRING;
typedef VOID *PVOID;
typedef CHAR *PCHAR;
typedef UCHAR *PUCHAR;
typedef USHORT *PUSHORT;
typedef LONG *PLONG;
typedef ULONG *PULONG;
typedef UINT *PUINT;
typedef signed int INT;
typedef long long INT64;

#define ra_dma_addr_t					dma_addr_t
typedef void				* PNDIS_PACKET;
typedef char				NDIS_PACKET;
typedef PNDIS_PACKET		* PPNDIS_PACKET;
typedef	ra_dma_addr_t			NDIS_PHYSICAL_ADDRESS;
typedef	ra_dma_addr_t			* PNDIS_PHYSICAL_ADDRESS;
typedef void				* NDIS_HANDLE;
typedef char 				* PNDIS_BUFFER;
typedef struct pci_dev 		* PPCI_DEV;

#undef __inline
#define __inline		static inline
#define IN
#define OUT
#define INOUT
#define NDIS_STATUS		INT

#ifndef TRUE
#define TRUE						1
#endif
#ifndef FALSE
#define FALSE						0
#endif


#define RF_MISC	0x0518
#define TX_ALC_CFG_0	0x13B0
#define NDIS_STATUS_SUCCESS                     0x00
#define NDIS_STATUS_FAILURE                     0x01
#define NDIS_STATUS_INVALID_DATA				0x02
#define NDIS_STATUS_RESOURCES                   0x03

#define MGMT_RING_SIZE          32
#define RTMP_PKT_TAIL_PADDING 	11 /* 3(max 4 byte padding) + 4 (last packet padding) + 4 (MaxBulkOutsize align padding) */
#define NUM_OF_LOCAL_TXBUF      2
#define TXD_SIZE		16	/* TXD_SIZE = TxD + TxInfo */
#define RXD_SIZE		16	

#define TypeTFSwitch						0x1
#define TypeProtectionFrame					0x2
#define TypeAFH 							0x3	
#define TypeWiFiStatus                                  0x4
#define TypeHostLoopBackTFSwitch 							0xFFF1	
#define TypeCoexCCUForceMode 							0xFFF2

#define COEX_MODE_RESET           0
#define COEX_MODE_TDD               1
#define COEX_MODE_HYBRID          2
#define COEX_MODE_FDD               3

#define COEX_OPMODE_STA                  0
#define COEX_OPMODE_AP                   1
#define COEX_OPMODE_GC           2
#define COEX_OPMODE_GO               3     
#define COEX_OPMODE_BT                   4
#define COEX_OPMODE_AD_HOC            5


#define WLAN_NO_BSSID               0x0
#define WLAN_Device_OFF                 0x1
#define WLAN_Device_ON    0x2
#define WLAN_SCANREQEST                         0x3
#define WLAN_SCANDONE                              0x4
#define WLAN_SCANCANCEL                             0x5
#define WLAN_SCAN2GREQUET                             0x6
#define WLAN_SCAN5GREQUET                             0x7
#define WLAN_CONNECTION_START		0x14
#define WLAN_CONNECTION_COMPLETION	0x15
#define WLAN_CONNECTION_CANCEL_IND	0x16
#define WLAN_CONNECTION_JOIN_FAIL	0x17
#define WLAN_CONNECTION_AUTH_FAIL	0x18
#define WLAN_CONNECTION_ASSOC_FAIL	0x19
#define WLAN_DISCONNECT                      	0x1A
#define WLAN_ROAMING_START                  	0x1B
#define WLAN_ROAMING_COMPLETE             0x1C
#define WLAN_DeAuthConfirm	0x8
#define WLAN_AssocRequest	0x9
#define WLAN_AssocConfirm	0xA
#define WLAN_ReAssocRequest	0xB
#define WLAN_ReAssocConfirm	0xC
#define WLAN_DisAssocRequest	0xD
#define BT_HCI_Create_Physical_Link     0xB0
#define BT_HCI_Accept_Physical_Link     0xB1
#define BT_HCI_Disconnect_Physical_Link 0xB2
#define BT_Connection_Accept_Timeout    0xB3
#define BT_MAC_Start_Completed	0xB4
#define BT_MAC_Start_Failed	0xB5
#define BT_MAC_Connect_Completed	0xB6
#define BT_MAC_Connect_Failed	0xB7
#define BT_MAC_Media_Disconnection_Ind	0xB8
#define BT_MAC_Connection_Cancel_Ind	0xB9
#define BT_Four_Way_Handshake_Fail	0xBA
#define BT_Four_Way_Handshake_Success	0xBB
#define WLAN_INVINCIBLE_REQ 0xF1
#define WLAN_INVINCIBLE_ABORT 0xF2
#define WLAN_BEACON_SERVICE_REQ 0xF3
#define WLAN_BEACON_SERVICE_CONF 0xF4

#define NULLFRAMESPACE                  10
#define COEXNOZEROSHIFT                1
#define CTSTOSELF                  0
#define CFEND                         1
#define POWERSAVE0               2
#define POWERSAVE1               3
#define PROTECTIONFRAMEREADY  1
#define PROTECTIONFRAMECANCEL 2

#define COEX_WIFI_LINK_UP              1
#define COEX_WIFI_LINK_DOWN         2

// value domain of 802.11 header FC.Tyte, which is b3..b2 of the 1st-byte of MAC header
#define BTYPE_MGMT                  0
#define BTYPE_CNTL                  1
#define BTYPE_DATA                  2
// value domain of 802.11 MGMT frame's FC.subtype, which is b7..4 of the 1st-byte of MAC header
#define SUBTYPE_ASSOC_REQ           0
#define SUBTYPE_ASSOC_RSP           1
#define SUBTYPE_REASSOC_REQ         2
#define SUBTYPE_REASSOC_RSP         3
#define SUBTYPE_PROBE_REQ           4
#define SUBTYPE_PROBE_RSP           5
#define SUBTYPE_BEACON              8
#define SUBTYPE_ATIM                9
#define SUBTYPE_DISASSOC            10
#define SUBTYPE_AUTH                11
#define SUBTYPE_DEAUTH              12
#define SUBTYPE_ACTION              13
#define SUBTYPE_ACTION_NO_ACK              14

// value domain of 802.11 CNTL frame's FC.subtype, which is b7..4 of the 1st-byte of MAC header
#define SUBTYPE_WRAPPER       	7
#define SUBTYPE_BLOCK_ACK_REQ       8
#define SUBTYPE_BLOCK_ACK           9
#define SUBTYPE_PS_POLL             10
#define SUBTYPE_RTS                 11
#define SUBTYPE_CTS                 12
#define SUBTYPE_ACK                 13
#define SUBTYPE_CFEND               14
#define SUBTYPE_CFEND_CFACK         15

// value domain of 802.11 DATA frame's FC.subtype, which is b7..4 of the 1st-byte of MAC header
#define SUBTYPE_DATA                0
#define SUBTYPE_DATA_CFACK          1
#define SUBTYPE_DATA_CFPOLL         2
#define SUBTYPE_DATA_CFACK_CFPOLL   3
#define SUBTYPE_NULL_FUNC           4
#define SUBTYPE_CFACK               5
#define SUBTYPE_CFPOLL              6
#define SUBTYPE_CFACK_CFPOLL        7
#define SUBTYPE_QDATA               8
#define SUBTYPE_QDATA_CFACK         9
#define SUBTYPE_QDATA_CFPOLL        10
#define SUBTYPE_QDATA_CFACK_CFPOLL  11
#define SUBTYPE_QOS_NULL            12
#define SUBTYPE_QOS_CFACK           13
#define SUBTYPE_QOS_CFPOLL          14
#define SUBTYPE_QOS_CFACK_CFPOLL    15

// ACK policy of QOS Control field bit 6:5
#define NORMAL_ACK                  0x00  // b6:5 = 00
#define NO_ACK                      0x20  // b6:5 = 01
#define NO_EXPLICIT_ACK             0x40  // b6:5 = 10
#define BLOCK_ACK                   0x60  // b6:5 = 11

// power status related definitions
#define PWR_ACTIVE                      0
#define PWR_SAVE                        1
#define PWR_MMPS                        2			//MIMO power save
//#define PWR_UNKNOWN                   2

#define MODE_CCK	0
#define MODE_OFDM   1
#define MODE_HTMIX	2
#define MODE_HTGREENFIELD	3
#define MODE_VHT	4

/* BW */
#define BW_20		0
#define BW_40		1
#define BW_80		2
#define BW_10		4	/* 802.11j has 10MHz. This definition is for internal usage. doesn't fill in the IE or other field. */

/*
	RF bank
*/
#define RF_BANK0	0
#define RF_BANK1	1
#define RF_BANK2	2
#define RF_BANK3	3
#define RF_BANK4	4
#define RF_BANK5	5
#define RF_BANK6	6
#define RF_BANK7	7
#define RF_BANK8	8
#define RF_BANK9	9
#define RF_BANK10	10
#define RF_BANK11	11
#define RF_BANK12	12
#define RF_BANK13	13
#define RF_BANK14	14
#define RF_BANK15	15

/*
	RF sections
*/
#define RF_R00			0
#define RF_R01			1
#define RF_R02			2
#define RF_R03			3
#define RF_R04			4
#define RF_R05			5
#define RF_R06			6
#define RF_R07			7
#define RF_R08			8
#define RF_R09			9
#define RF_R10			10
#define RF_R11			11
#define RF_R12			12
#define RF_R13			13
#define RF_R14			14
#define RF_R15			15
#define RF_R16			16
#define RF_R17			17
#define RF_R18			18
#define RF_R19			19
#define RF_R20			20
#define RF_R21			21
#define RF_R22			22
#define RF_R23			23
#define RF_R24			24
#define RF_R25			25
#define RF_R26			26
#define RF_R27			27
#define RF_R28			28
#define RF_R29			29
#define RF_R30			30
#define RF_R31			31
#define	RF_R32			32
#define	RF_R33			33
#define	RF_R34			34
#define	RF_R35			35
#define	RF_R36			36
#define	RF_R37			37
#define	RF_R38			38
#define	RF_R39			39
#define	RF_R40			40
#define	RF_R41			41
#define	RF_R42			42
#define	RF_R43			43
#define	RF_R44			44
#define	RF_R45			45
#define	RF_R46			46
#define	RF_R47			47
#define	RF_R48			48
#define	RF_R49			49
#define	RF_R50			50
#define	RF_R51			51
#define	RF_R52			52
#define	RF_R53			53
#define	RF_R54			54
#define	RF_R55			55
#define	RF_R56			56
#define	RF_R57			57
#define	RF_R58			58
#define	RF_R59			59
#define	RF_R60			60
#define	RF_R61			61
#define	RF_R62			62
#define	RF_R63			63
#define	RF_R64			64
#define	RF_R65			65
#define	RF_R66			66
#define	RF_R67			67
#define	RF_R68			68
#define	RF_R69			69
#define	RF_R70			70
#define	RF_R71			71
#define	RF_R72			72
#define	RF_R73			73
#define	RF_R74			74
#define	RF_R75			75
#define	RF_R76			76
#define	RF_R77			77
#define	RF_R78			78
#define	RF_R79			79
#define	RF_R126			126
#define	RF_R127			127

#define	MT7630_RF_CSR_CFG			0x0500
#define 	PCIE_REMAP_BASE4 			(0x1024C)

// on-chip NULL frame sapace - base address = 0x7700
// here stores QOS-NULL frame . 
//PBF_CFG(0x408) bit15 use Qos-NULL to enable STA mode early terminate txop.
#define HW_NULL_BASE            0x7700
// 2nd NULL FRAME put CF-END frame. 
//PBF_CFG(0x408) bit14 to enable AP mode early terminate txop requires CF-END
#define HW_NULL2_BASE            0x7780

#define INT_HCCA_DLY	(0x00001000)
#define INT_MGMT_DLY	(0x00002000)
#define MAC_ADDR_LEN                    6
#define BSSID_WCID      1	// in infra mode, always put bssid with this WCID 
//woody 
#define BT_FUN_INFO		0xC4

#define RINGREG_DIFF	0x10
#define TX_RING_BASE	0x0300
#define TX_RING_NUM		10
#define TX_RING_PTR		0x0300
#define TX_RING_CNT		0x0304
#define TX_RING_CIDX	0x0308
#define TX_RING_DIDX	0x030c
#define TX_MGMT_BASE	(TX_RING_BASE  + RINGREG_DIFF * 9)
#define TX_MGMT_CNT	(TX_MGMT_BASE + 0x04)
#define TX_MGMT_CIDX	(TX_MGMT_BASE + 0x08)
#define TX_MGMT_DIDX	(TX_MGMT_BASE + 0x0c)
#define TX_CTRL_BASE	(TX_RING_BASE  + RINGREG_DIFF * 8)
#define TX_CTRL_CNT		(TX_CTRL_BASE + 0x04)
#define TX_CTRL_CIDX	(TX_CTRL_BASE + 0x08)
#define TX_CTRL_DIDX	(TX_CTRL_BASE + 0x0c)

#define RX_RING_BASE	0x03c0
#define RX_RING_PTR		RX_RING_BASE
#define RX_RING_CNT		(RX_RING_BASE + 0x04)
#define RX_RING_CIDX	(RX_RING_BASE + 0x08)
#define RX_RING_DIDX	(RX_RING_BASE + 0x0c)

#define RX_RING1_BASE	0x03d0
#define RX_RING1_PTR	RX_RING1_BASE
#define RX_RING1_CNT	(RX_RING1_BASE + 0x04)
#define RX_RING1_CIDX	(RX_RING1_BASE + 0x08)
#define RX_RING1_DIDX	(RX_RING1_BASE + 0x0c)

#define TxWIMPDUByteCnt	TXWI_N.MPDUtotalByteCnt
#define TxWIWirelessCliID		TXWI_N.wcid
#define TxWIFRAG			TXWI_N.FRAG
#define TxWICFACK			TXWI_N.CFACK
#define TxWITS				TXWI_N.TS
#define TxWIAMPDU			TXWI_N.AMPDU
#define TxWIACK				TXWI_N.ACK
#define TxWITXOP			TXWI_N.txop
#define TxWINSEQ			TXWI_N.NSEQ
#define TxWIBAWinSize		TXWI_N.BAWinSize
#define TxWIShortGI			TXWI_N.ShortGI
#define TxWISTBC			TXWI_N.STBC
#define TxWIPacketId			TXWI_N.TxPktId
#define TxWIBW				TXWI_N.BW
#define TxWIMCS				TXWI_N.MCS
#define TxWIPHYMODE		TXWI_N.PHYMODE
#define TxWIMIMOps			TXWI_N.MIMOps
#define TxWIMpduDensity		TXWI_N.MpduDensity
#define TxWILutEn			TXWI_N.lut_en
#define TxWIIV				TXWI_N.IV
#define TxWIEIV				TXWI_N.EIV

#define TxInfoWIV			txinfo_nmac_pkt.wiv
#define TxInfoQSEL			txinfo_nmac_pkt.QSEL
#define TxInfoPktLen			txinfo_nmac_pkt.pkt_len
#define TxInfoSwLstRnd		txinfo_nmac_pkt.rsv0
#define TxInfoUDMATxburst	txinfo_nmac_pkt.tx_burst
#define TxInfoUDMANextVld	txinfo_nmac_pkt.next_vld
#define TxInfoPkt80211		txinfo_nmac_pkt.pkt_80211


#define RTMP_PCI_DMA_TODEVICE		0xFF00
#define RTMP_PCI_DMA_FROMDEVICE		0xFF01


enum CALIBRATION_ID {
	R_CALIBRATION = 1,
	RXDCOC_CALIBRATION,
	LC_CALIBRATION,
	LOFT_CALIBRATION,
	TXIQ_CALIBRATION,
	BW_CALIBRATION,
	DPD_CALIBRATION,
	RXIQ_CALIBRATION,
	TXDCOC_CALIBRATION,
	RX_GROUP_DELAY_CALIBRATION,
	TX_GROUP_DELAY_CALIBRATION,
};


#define TX_STA_FIFO_EXT		0x1798		/* Only work after RT53xx */
typedef	union _TX_STA_FIFO_EXT_STRUC {
	struct
	{
		u32 TX_RTY_CNT:8; // Tx retry count
		u32 TX_PKT_ID:8; // Tx packet ID (copied from per-packet TXWI)
		u32 Reserved:16;
	} field;

	u32 word;
} TX_STA_FIFO_EXT_STRUC;


typedef union  _MCU_LEDCS_STRUC {
	struct	{
		UCHAR		LedMode:7;		
		UCHAR		Polarity:1;
	} field;
	UCHAR			word;
} MCU_LEDCS_STRUC, *PMCU_LEDCS_STRUC;

typedef struct _LED_CONTROL
{
	MCU_LEDCS_STRUC		MCULedCntl; /* LED Mode EEPROM 0x3b */
	USHORT				LedAGCfg;	/* LED A/G Configuration EEPROM 0x3c */
	USHORT				LedACTCfg;	/* LED ACT Configuration EEPROM 0x3e */
	USHORT				LedPolarity;/* LED A/G/ACT polarity EEPROM 0x40 */
	UCHAR				LedIndicatorStrength;
	UCHAR				RssiSingalstrengthOffet;
	BOOLEAN				bLedOnScanning;
	UCHAR				LedStatus;
}LED_CONTROL, *PLED_CONTROL;



typedef	union _RLT_RF_CSR_CFG {
	struct {
		unsigned int RF_CSR_DATA:8;
		unsigned int RF_CSR_REG_ID:7;
		unsigned int RF_CSR_REG_BANK:3;
		unsigned int rsv:12;
		unsigned int RF_CSR_WR:1;
		unsigned int RF_CSR_KICK:1;
	} field;
	unsigned int word;
}RLT_RF_CSR_CFG;


typedef struct GNU_PACKED _RXFCE_INFO{
	UINT32 pkt_len:14;
	UINT32 rsv:2;

	UINT32 udp_err:1;
	UINT32 tcp_err:1;
	UINT32 ip_err:1;
	UINT32 pkt_80211:1;
	UINT32 l3l4_done:1;
	UINT32 mac_len:3;

	UINT32 pcie_intr:1;
	UINT32 qsel:2;
	UINT32 s_port:3;
	UINT32 info_type:2;
}RXFCE_INFO;

typedef	struct GNU_PACKED _RXINFO_STRUC {
	UINT32		BA:1;
	UINT32		DATA:1;
	UINT32		NULLDATA:1;
	UINT32		FRAG:1;
	UINT32		U2M:1;
	UINT32		Mcast:1;
	UINT32		Bcast:1;
	UINT32		MyBss:1;
	UINT32		Crc:1;
	UINT32		CipherErr:2;
	UINT32		AMSDU:1;
	UINT32		HTC:1;
	UINT32		RSSI:1;
	UINT32		L2PAD:1;
	UINT32		AMPDU:1;
	UINT32		Decrypted:1;
	UINT32		BssIdx3:1;
	UINT32		wapi_kidx:1;
	UINT32		pn_len:3;
	UINT32		rsv:6;
	UINT32		tcp_sum_bypass:1;
	UINT32		ip_sum_bypass:1;
	UINT32		tcp_sum_err:1;
	UINT32		ip_sum_err:1;
}RXINFO_STRUC, *PRXINFO_STRUC;

typedef union _LARGE_INTEGER {
	struct {

		UINT LowPart;
		INT32 HighPart;
	} u;
	INT64 QuadPart;
} LARGE_INTEGER;

typedef struct _COUNTER_802_3 {
	/* General Stats */
	ULONG GoodTransmits;
	ULONG GoodReceives;
	ULONG TxErrors;
	ULONG RxErrors;
	ULONG RxNoBuffer;

	/* Ethernet Stats */
	ULONG RcvAlignmentErrors;
	ULONG OneCollision;
	ULONG MoreCollisions;

} COUNTER_802_3, *PCOUNTER_802_3;

typedef struct _COUNTER_802_11 {
	ULONG Length;
/*	LARGE_INTEGER   LastTransmittedFragmentCount; */
	LARGE_INTEGER TransmittedFragmentCount;
	LARGE_INTEGER MulticastTransmittedFrameCount;
	LARGE_INTEGER FailedCount;
	LARGE_INTEGER RetryCount;
	LARGE_INTEGER MultipleRetryCount;
	LARGE_INTEGER RTSSuccessCount;
	LARGE_INTEGER RTSFailureCount;
	LARGE_INTEGER ACKFailureCount;
	LARGE_INTEGER FrameDuplicateCount;
	LARGE_INTEGER ReceivedFragmentCount;
	LARGE_INTEGER MulticastReceivedFrameCount;
	LARGE_INTEGER FCSErrorCount;
	LARGE_INTEGER TransmittedFrameCount;
	LARGE_INTEGER WEPUndecryptableCount;
	LARGE_INTEGER TransmitCountFrmOs;
} COUNTER_802_11, *PCOUNTER_802_11;

typedef struct _COUNTER_RALINK {
	UINT32 OneSecStart;	/* for one sec count clear use */
	UINT32 OneSecBeaconSentCnt;
	UINT32 OneSecFalseCCACnt;	/* CCA error count, for debug purpose, might move to global counter */
	UINT32 OneSecRxFcsErrCnt;	/* CRC error */
	UINT32 OneSecRxOkCnt;	/* RX without error */
	UINT32 OneSecTxFailCount;
	UINT32 OneSecTxNoRetryOkCount;
	UINT32 OneSecTxRetryOkCount;
	UINT32 OneSecRxOkDataCnt;	/* unicast-to-me DATA frame count */
	UINT32 OneSecTransmittedByteCount;	/* both successful and failure, used to calculate TX throughput */

	ULONG OneSecOsTxCount[5];
	ULONG OneSecDmaDoneCount[5];
	UINT32 OneSecTxDoneCount;
	ULONG OneSecRxCount;
	UINT32 OneSecReceivedByteCount;
	UINT32 OneSecTxAggregationCount;
	UINT32 OneSecRxAggregationCount;
	UINT32 OneSecEnd;	/* for one sec count clear use */

	ULONG TransmittedByteCount;	/* both successful and failure, used to calculate TX throughput */
	ULONG ReceivedByteCount;	/* both CRC okay and CRC error, used to calculate RX throughput */
	ULONG LastReceivedByteCount;
	ULONG BadCQIAutoRecoveryCount;
	ULONG PoorCQIRoamingCount;
	ULONG MgmtRingFullCount;
	ULONG RxCountSinceLastNULL;
	ULONG RxCount;
	ULONG KickTxCount;
	LARGE_INTEGER RealFcsErrCount;
	ULONG PendingNdisPacketCount;
	ULONG FalseCCACnt;                    /* CCA error count */

	UINT32 LastOneSecTotalTxCount;	/* OneSecTxNoRetryOkCount + OneSecTxRetryOkCount + OneSecTxFailCount */
	UINT32 LastOneSecRxOkDataCnt;	/* OneSecRxOkDataCnt */
	ULONG DuplicateRcv;
	ULONG TxAggCount;
	ULONG TxNonAggCount;
	ULONG TxAgg1MPDUCount;
	ULONG TxAgg2MPDUCount;
	ULONG TxAgg3MPDUCount;
	ULONG TxAgg4MPDUCount;
	ULONG TxAgg5MPDUCount;
	ULONG TxAgg6MPDUCount;
	ULONG TxAgg7MPDUCount;
	ULONG TxAgg8MPDUCount;
	ULONG TxAgg9MPDUCount;
	ULONG TxAgg10MPDUCount;
	ULONG TxAgg11MPDUCount;
	ULONG TxAgg12MPDUCount;
	ULONG TxAgg13MPDUCount;
	ULONG TxAgg14MPDUCount;
	ULONG TxAgg15MPDUCount;
	ULONG TxAgg16MPDUCount;

	LARGE_INTEGER TransmittedOctetsInAMSDU;
	LARGE_INTEGER TransmittedAMSDUCount;
	LARGE_INTEGER ReceivedOctesInAMSDUCount;
	LARGE_INTEGER ReceivedAMSDUCount;
	LARGE_INTEGER TransmittedAMPDUCount;
	LARGE_INTEGER TransmittedMPDUsInAMPDUCount;
	LARGE_INTEGER TransmittedOctetsInAMPDUCount;
	LARGE_INTEGER MPDUInReceivedAMPDUCount;

	ULONG PhyErrCnt;
	ULONG PlcpErrCnt;
} COUNTER_RALINK, *PCOUNTER_RALINK;

#define RX_STA_CNT0		0x1700
typedef	union _RX_STA_CNT0_STRUC {
	struct {
	    UINT16  CrcErr;
	    UINT16  PhyErr;
	} field;
	UINT32 word;
} RX_STA_CNT0_STRUC;

#define RX_STA_CNT1		0x1704
typedef	union _RX_STA_CNT1_STRUC {
	struct {
	    UINT16  FalseCca;
	    UINT16  PlcpErr;
	} field;
	UINT32 word;
} RX_STA_CNT1_STRUC;

#define RX_STA_CNT2		0x1708
typedef	union _RX_STA_CNT2_STRUC {
	struct {
	    UINT16  RxDupliCount;
	    UINT16  RxFifoOverflowCount;
	} field;
	UINT32 word;
} RX_STA_CNT2_STRUC;

/* STA_CSR3: TX Beacon count */
#define TX_STA_CNT0		0x170C
typedef	union _TX_STA_CNT0_STRUC {
	struct {
	    UINT16  TxFailCount;
	    UINT16  TxBeaconCount;
	} field;
	UINT32 word;
} TX_STA_CNT0_STRUC;

#define TX_STA_CNT1		0x1710
typedef	union _TX_STA_CNT1_STRUC {
	struct {
	    UINT16  TxSuccess;
	    UINT16  TxRetransmit;
	} field;
	UINT32 word;
} TX_STA_CNT1_STRUC;
/* TX_STA_CNT2: TX tx count */
#define TX_STA_CNT2		0x1714
typedef	union _TX_STA_CNT2_STRUC {
	struct {
	    UINT16  TxZeroLenCount;
	    UINT16  TxUnderFlowCount;
	} field;
	UINT32 word;
} TX_STA_CNT2_STRUC;

#define TX_STA_FIFO		0x1718
typedef	union _TX_STA_FIFO_STRUC {
	struct {
		UINT32       	bValid:1;
		UINT32       	PidType:4;
		UINT32       	TxSuccess:1;
		UINT32       	TxAggre:1;
		UINT32       	TxAckRequired:1;
		UINT32		wcid:8;
		UINT32		SuccessRate:11;
		UINT32		eTxBF:1;
		UINT32		Sounding:1;
		UINT32		iTxBF:1;
		UINT32		Reserve:2;
	} field;
	UINT32 word;
} TX_STA_FIFO_STRUC;

#define INC_COUNTER64(Val)          (Val.QuadPart++)

typedef spinlock_t			OS_NDIS_SPIN_LOCK;
#define NDIS_SPIN_LOCK							OS_NDIS_SPIN_LOCK

#define OS_NdisAllocateSpinLock(__lock)			\
{                                       		\
    spin_lock_init((spinlock_t *)(__lock));		\
}

#define OS_NdisFreeSpinLock(lock)				\
	do{}while(0)


#define NdisAllocateSpinLock(__pReserved, __pLock)	OS_NdisAllocateSpinLock(__pLock)

#define OS_SEM_LOCK(__lock)						\
{												\
	spin_lock_bh((spinlock_t *)(__lock));		\
}

#define OS_SEM_UNLOCK(__lock)					\
{												\
	spin_unlock_bh((spinlock_t *)(__lock));		\
}


#define RTMP_IRQ_LOCK(__lock, __irqflags)			\
{													\
	__irqflags = 0;									\
	spin_lock_irqsave((spinlock_t *)(__lock), __irqflags);			\
}

#define RTMP_IRQ_UNLOCK(__lock, __irqflag)			\
{													\
	spin_unlock_irqrestore((spinlock_t *)(__lock), __irqflag);			\
}




#define RELEASE_NDIS_PACKET(_pAd, _pPacket, _Status)                    \
{                                                                       \
        RTMPFreeNdisPacket(_pAd, _pPacket);                             \
}

#define NdisAllocateSpinLock(__pReserved, __pLock)	OS_NdisAllocateSpinLock(__pLock)
#define NdisFreeSpinLock						OS_NdisFreeSpinLock

#define RTMP_SEM_LOCK							OS_SEM_LOCK
#define RTMP_SEM_UNLOCK							OS_SEM_UNLOCK
#define NdisAcquireSpinLock						RTMP_SEM_LOCK
#define NdisReleaseSpinLock						RTMP_SEM_UNLOCK


#define RTMP_IO_WRITE8(_A, _R, _V)							\
{															\
	writeb((_V), (PUCHAR)((_A)->csr.base + (_R)));		\
}

#define RTMP_IO_WRITE32(_A, _R, _V) \
do{ \
		writel((_V), (void *)((_A)->csr.base + (_R))); \
}while(0)

#define RTMP_IO_READ8(_A, _R, _pV)								\
{																\
	(*(_pV) = readb((void *)((_A)->csr.base + (_R))));				\
}


#define RTMP_IO_READ32(_A, _R, _pV)								\
{																\
(*(_pV) = readl((void *)((_A)->csr.base + (_R))));			\
											\
}


#define COPY_MAC_ADDR(Addr1, Addr2)             memcpy((Addr1), (Addr2), MAC_ADDR_LEN)

#define GET_CTRLRING_FREENO(_pAd) \
	(_pAd->CtrlRing.TxSwFreeIdx > _pAd->CtrlRing.TxCpuIdx)	? \
			(_pAd->CtrlRing.TxSwFreeIdx - _pAd->CtrlRing.TxCpuIdx - 1) \
			 :	\
			(_pAd->CtrlRing.TxSwFreeIdx + MGMT_RING_SIZE - _pAd->CtrlRing.TxCpuIdx - 1);

#define RTPKT_TO_OSPKT(_p)		((struct sk_buff *)(_p))
#define OSPKT_TO_RTPKT(_p)		((PNDIS_PACKET)(_p))

#define GET_OS_PKT_DATAPTR(_pkt) \
		(RTPKT_TO_OSPKT(_pkt)->data)
#define SET_OS_PKT_DATAPTR(_pkt, _dataPtr)	\
		(RTPKT_TO_OSPKT(_pkt)->data) = (_dataPtr)

#define GET_OS_PKT_LEN(_pkt) \
		(RTPKT_TO_OSPKT(_pkt)->len)
#define SET_OS_PKT_LEN(_pkt, _len)	\
		(RTPKT_TO_OSPKT(_pkt)->len) = (_len)
		
#define GET_OS_PKT_DATATAIL(_pkt) \
		(RTPKT_TO_OSPKT(_pkt)->tail)
#define SET_OS_PKT_DATATAIL(_pkt, _start, _len)	\
		((RTPKT_TO_OSPKT(_pkt))->tail) = (PUCHAR)((_start) + (_len))
		
#define GET_OS_PKT_HEAD(_pkt) \
		(RTPKT_TO_OSPKT(_pkt)->head)

#define GET_OS_PKT_END(_pkt) \
		(RTPKT_TO_OSPKT(_pkt)->end)

#define GET_OS_PKT_NETDEV(_pkt) \
		(RTPKT_TO_OSPKT(_pkt)->dev)
#define SET_OS_PKT_NETDEV(_pkt, _pNetDev)	\
		(RTPKT_TO_OSPKT(_pkt)->dev) = (_pNetDev)
		
#define GET_OS_PKT_TYPE(_pkt) \
		(RTPKT_TO_OSPKT(_pkt))

#define GET_OS_PKT_NEXT(_pkt) \
		(RTPKT_TO_OSPKT(_pkt)->next)


#define INC_RING_INDEX(_idx, _RingSize)    \
{                                          \
    (_idx) = (_idx+1) % (_RingSize);       \
}


#define NdisMoveMemory(Destination, Source, Length) memmove(Destination, Source, Length)
#define NdisCopyMemory(Destination, Source, Length) memcpy(Destination, Source, Length)
#define NdisZeroMemory(Destination, Length)         memset(Destination, 0, Length)
#define NdisFillMemory(Destination, Length, Fill)   memset(Destination, Fill, Length)
#define NdisCmpMemory(Destination, Source, Length)  memcmp(Destination, Source, Length)
#define NdisEqualMemory(Source1, Source2, Length)   (!memcmp(Source1, Source2, Length))
#define RTMPEqualMemory(Source1, Source2, Length)	(!memcmp(Source1, Source2, Length))

/*
 * ULONG
 * RTMP_GetPhysicalAddressLow(
 *   IN NDIS_PHYSICAL_ADDRESS  PhysicalAddress);
 */
#define RTMP_GetPhysicalAddressLow(PhysicalAddress)		(PhysicalAddress)

/*
 * ULONG
 * RTMP_GetPhysicalAddressHigh(
 *   IN NDIS_PHYSICAL_ADDRESS  PhysicalAddress);
 */
#define RTMP_GetPhysicalAddressHigh(PhysicalAddress)		(0)


/*
 * VOID
 * RTMP_SetPhysicalAddressLow(
 *   IN NDIS_PHYSICAL_ADDRESS  PhysicalAddress,
 *   IN ULONG  Value);
 */
#define RTMP_SetPhysicalAddressLow(PhysicalAddress, Value)	\
			PhysicalAddress = Value;

/*
 * VOID
 * RTMP_SetPhysicalAddressHigh(
 *   IN NDIS_PHYSICAL_ADDRESS  PhysicalAddress,
 *   IN ULONG  Value);
 */
#define RTMP_SetPhysicalAddressHigh(PhysicalAddress, Value)

#define RxWIMPDUByteCnt	RXWI_N.MPDUtotalByteCnt
#define RxWIWirelessCliID	RXWI_N.wcid
#define RxWIKeyIndex		RXWI_N.key_idx
#define RxWIMCS				RXWI_N.mcs
#define RxWIBW				RXWI_N.bw
#define RxWIBSSID			RXWI_N.bss_idx
#define RxWISGI				RXWI_N.sgi
#define RxWIPhyMode		RXWI_N.phy_mode
#define RxWISTBC			RXWI_N.stbc
#define RxWITID				RXWI_N.tid
#define RxWIRSSI0			RXWI_N.rssi[0]
#define RxWIRSSI1			RXWI_N.rssi[1]
#define RxWIRSSI2			RXWI_N.rssi[2]
#define RxWIRSSI3			RXWI_N.rssi[3]
#define RxWISNR0			RXWI_N.bbp_rxinfo[0]
#define RxWISNR1			RXWI_N.bbp_rxinfo[1]
#define RxWISNR2			RXWI_N.bbp_rxinfo[2]
#define RxWIFOFFSET			RXWI_N.bbp_rxinfo[3]

enum INFO_TYPE {
	NORMAL_PACKET,
	CMD_PACKET,
};

enum D_PORT {
	WLAN_PORT,
	CPU_RX_PORT,
	CPU_TX_PORT,
	HOST_PORT,
	VIRTUAL_CPU_RX_PORT,
	VIRTUAL_CPU_TX_PORT,
	DISCARD,
};


typedef struct  _PACKET_INFO    {
	UINT PhysicalBufferCount;    /* Physical breaks of buffer descripor chained */
	UINT BufferCount;           /* Number of Buffer descriptor chained */
	UINT TotalPacketLength ;     /* Self explained */
	PNDIS_BUFFER pFirstBuffer;   /* Pointer to first buffer descriptor */
} PACKET_INFO, *PPACKET_INFO;

typedef union _INT_SOURCE_CSR_STRUC {
	struct {
		UINT32 RxDone:1;
		UINT32 RxDone1:1;
		UINT32 rsv4:2;
		UINT32 Ac0DmaDone:1;
		UINT32 Ac1DmaDone:1;
		UINT32 Ac2DmaDone:1;
		UINT32 Ac3DmaDone:1;
		UINT32 HccaDmaDone:1;
		UINT32 MgmtDmaDone:1;
		UINT32 TxDone6:1;
		UINT32 TxDone7:1;
		UINT32 TxDone8:1;
		UINT32 TxDone9:1;
		UINT32 rsv3:2;
		UINT32 rxCoherent:1;
		UINT32 txCoherent:1;
		UINT32 trCoherent:1;
		UINT32 MCUCommandINT:1;
		UINT32 tbttInt:1;
		UINT32 PreTBTT:1;
		UINT32 TXFifoStatusInt:1;
		UINT32 AutoWakeup:1;
		UINT32 GPTimer:1;
		UINT32 rsv2:1;
		UINT32 RxDelayINT:1;
		UINT32 TxDelayINT:1;
		UINT32 rsv1:4;
	}field;
	UINT32 word;
}INT_SOURCE_CSR_STRUC;


typedef	union _SHAREDKEY_MODE_STRUC {
	struct {
		UINT32       Bss0Key0CipherAlg:4;
		UINT32       Bss0Key1CipherAlg:4;
		UINT32       Bss0Key2CipherAlg:4;
		UINT32       Bss0Key3CipherAlg:4;
		UINT32       Bss1Key0CipherAlg:4;
		UINT32       Bss1Key1CipherAlg:4;
		UINT32       Bss1Key2CipherAlg:4;
		UINT32       Bss1Key3CipherAlg:4;
	} field;
	UINT32 word;
} SHAREDKEY_MODE_STRUC;





typedef struct _DL_LIST
{
	struct _DL_LIST *Next;
	struct _DL_LIST *Prev;
}DL_LIST, *PDL_LIST;

struct MCU_CTRL {
	UCHAR CmdSeq;
	BOOLEAN IsFWReady;
	NDIS_SPIN_LOCK CmdRspEventListLock;
	DL_LIST CmdRspEventList;
};

typedef struct _COEX_MODE_STRUCTURE {
	BOOLEAN     bForce;
       BOOLEAN  FDDRequest;
       BOOLEAN  TDDRequest;
       ULONG      TDD_Power;
       ULONG      FDD_Power;
	ULONG      DefaultMode; 
	ULONG      CurrentMode;  
       ULONG      UpdateMode;  
       CHAR        CoexTDDRSSITreshold;
       CHAR        CoexFDDRSSITreshold;   
} COEX_MODE_STRUCTURE, *PCOEX_MODE_STRUCTURE;

typedef struct _COMMON_CONFIG {
	UCHAR BBPCurrentBW;
	UCHAR  Channel;
	UCHAR  CentralChannel;
} COMMON_CONFIG, *PCOMMON_CONFIG;	

typedef	union	_BT_FUN_INFO_STRUC	{
	struct	{
		ULONG		AFH_START_CH:8;
		ULONG		AFH_END_CH:8;
		ULONG		YIELD_WIFI_LOW_PRIORITY_TX:1;	
		ULONG		Reserve:13;	
		ULONG		WLAN_ACTIVE:1;
		ULONG		WLAN_EEP_BUSY:1;				
	}field;
	ULONG			word;
} BT_FUN_INFO_STRUC, *PBT_FUN_INFO_STRUC;

struct rt2x00_dev;
typedef void (*CMD_RSP_HANDLER)(struct rt2x00_dev *pAd, UCHAR *Data);
struct CMD_UNIT {
	union {
		struct {
			UCHAR Command;
			UCHAR Token;
			UCHAR Arg0;
			UCHAR Arg1;
		} MCU51;
		struct {
			UINT8 Type;
			USHORT CmdPayloadLen;
			PUCHAR CmdPayload;
			USHORT RspPayloadLen;
			PUCHAR RspPayload;
			ULONG Timeout;
			BOOLEAN NeedRsp;
			BOOLEAN NeedWait;
			CMD_RSP_HANDLER CmdRspHdler;
		} ANDES;
	} u;
};



struct CMD_RSP_EVENT {
	DL_LIST List;
	UCHAR CmdSeq;	
	UINT32 Timeout;
	BOOLEAN NeedWait;
	struct completion AckDone; 
	UCHAR **RspPayload;
	USHORT *RspPayloadLen;
};

typedef struct _COEX_AFH
{
	UINT32					CoexOperation;
    UINT32                   LinkStatus;   
    UINT32                   BW;
    UINT32                   Channel;
    UINT32	            BssHashID;
 } COEX_AFH, *PCOEX_AFH;

typedef struct _COEX_TF_SWITCH
{
    UINT32					CoexOperation;
    UINT32                   CoexMode;   

 } COEX_TF_SWITCH, *PCOEX_TF_SWITCH;


typedef struct _COEX_WLAN_STATUS
{
    UINT32		     CoexOperation;
    UINT32                   WLANStatus;   
    UINT32                   PrivilegeTime;
    UINT32	            BssHashID;

 } COEX_WLAN_STATUS, *PCOEX_WLAN_STATUS;

enum CMD_TYPE {
	CMD_FUN_SET_OP = 1,
	CMD_BURST_WRITE = 8,
	CMD_READ_MODIFY_WRITE,
	CMD_RANDOM_READ,
	CMD_BURST_READ,
	CMD_RANDOM_WRITE = 12,
	CMD_LED_MODE_OP = 16,
	CMD_POWER_SAVING_OP = 20,
	CMD_WOW_ENTRY,
	CMD_WOW_QUERY,
	CMD_CARRIER_DETECT_OP = 28,
	CMD_RADOR_DETECT_OP,
	CMD_SWITCH_CHANNEL_OP,
	CMD_CALIBRATION_OP,
};

#define PKT_CMD_TYPE_COEX_OP			(17)

typedef union _CMB_CTRL_STRUC{
	struct{
		u32		AUX_OPT_Bit0_InterfaceClk_40Mhz:1;
		u32		AUX_OPT_Bit1_PCIePhyClkOn_L1:1;	
		u32		AUX_OPT_Bit2_PCIeCoreClkOn_L1:1;
		u32		AUX_OPT_Bit3_PLLOn_L1:1;
		u32		AUX_OPT_Bit4_RemovePCIePhyClk_WLANOff:1;
		u32		AUX_OPT_Bit5_RemovePCIePhyClk_BTOff:1;
		u32		AUX_OPT_Bit6_KeepXtal_On:1;
		u32		AUX_OPT_Bit7_KeepInterfaceClk:1;
		u32		AUX_OPT_Bit8_AuxPower_Exists:1;
		u32		AUX_OPT_Bit9_GPIO3_as_GPIO:1;
		u32		AUX_OPT_Bit10_NotSwap_WL_LED_ACT_RDY:1;	
		u32		AUX_OPT_Bit11_Rsv:1;
		u32		AUX_OPT_Bit12_TRSW0_as_WLAN_ANT_SEL:1;
		u32		AUX_OPT_Bit13_GPIO7_as_GPIO:1;
		u32		AUX_OPT_Bit14_TRSW1_as_GPIO:1;
		u32		AUX_OPT_Bit15_Two_AntennaMode:1;
		u32		CsrUartMode:1;
		u32		GPIOModeLed1:1;
		u32		GPIOModeLed2:1;
		u32		Rsv:3;
		u32       	XTAL_RDY_CMB:1;
		u32       	PLL_LD_CMB:1;
		u32       	LDO_CORE_LEVEL_CMB:4;
		u32       	LDO_BGSEL_CMB:2;
		u32       	LDO3_EN_CMB:1;
		u32       	LDO0_EN_CMB:1;
	}field;
	u32 word;
}CMB_CTRL_STRUC, *PCMB_CTRL_STRUC;
typedef	union _WLAN_FUN_CTRL_STRUC	{
	struct {
		u32 WLAN_EN_MT7630:1;
		u32 WLAN_CLK_EN_MT7630:1;
		u32 WLAN_RESET_RF_MT7630:1;
		u32 WLAN_RESET_MT7630:1;
		u32 PCIE_APP0_CLK_REQ_MT7630:1;
		u32 FRC_WL_ANT_SET_MT7630:1;
		u32 INV_TR_SW0_MT7630:1;
		u32 WLAN_ACC_BT_MT7630:1;
		u32 GPIO0_IN_MT7630:8;
		u32 GPIO0_OUT_MT7630:8;
		u32 GPIO0_OUT_OE_N_MT7630:8;
	} field;
	u32 word;
} WLAN_FUN_CTRL_STRUC, *_WLAN_FUN_CTRL_STRUC;


typedef struct GNU_PACKED _TXINFO_NMAC_CMD{
	UINT32 pkt_len:16;
	UINT32 cmd_seq:4;
	UINT32 cmd_type:7;
	UINT32 d_port:3;	
	UINT32 info_type:2;
}TXINFO_NMAC_CMD;

typedef struct GNU_PACKED _TXINFO_NMAC_PKT {
	UINT32 pkt_len:16;
	UINT32 next_vld:1;
	UINT32 tx_burst:1;
	UINT32 rsv0:1;
	UINT32 pkt_80211:1;
	UINT32 tso:1;
	UINT32 cso:1;
	UINT32 rsv1:2;
	UINT32 wiv:1;
	UINT32 QSEL:2;
	UINT32 d_port:3;
	UINT32 info_type:2;
}TXINFO_NMAC_PKT;

#define TXINFO_SIZE			4
typedef union GNU_PACKED _TXINFO_STRUC{
	struct _TXINFO_NMAC_PKT txinfo_nmac_pkt;
	struct _TXINFO_NMAC_CMD txinfo_nmac_cmd;
	UINT32 word;
}TXINFO_STRUC;
typedef	struct GNU_PACKED _TXD_STRUC {
	/* Word	0 */
	UINT32		SDPtr0;
	/* Word	1 */
	UINT32		SDLen1:14;
	UINT32		LastSec1:1;
	UINT32		Burst:1;
	UINT32		SDLen0:14;
	UINT32		LastSec0:1;
	UINT32		DMADONE:1;
	/*Word2 */
	UINT32		SDPtr1;
} TXD_STRUC, *PTXD_STRUC;


typedef struct _RTMP_DMABUF {
	ULONG AllocSize;
	PVOID AllocVa;		/* TxBuf virtual address */
	NDIS_PHYSICAL_ADDRESS AllocPa;	/* TxBuf physical address */
} RTMP_DMABUF, *PRTMP_DMABUF;

typedef struct _RTMP_DMACB {
	ULONG AllocSize;	/* Control block size */
	PVOID AllocVa;		/* Control block virtual address */
	NDIS_PHYSICAL_ADDRESS AllocPa;	/* Control block physical address */
	PNDIS_PACKET pNdisPacket;
	PNDIS_PACKET pNextNdisPacket;

	RTMP_DMABUF DmaBuf;	/* Associated DMA buffer structure */
} RTMP_DMACB, *PRTMP_DMACB;

typedef struct _RTMP_CTRL_RING {
	RTMP_DMACB Cell[MGMT_RING_SIZE];
	UINT32 TxCpuIdx;
	UINT32 TxDmaIdx;
	UINT32 TxSwFreeIdx;	/* software next free tx index */
} RTMP_CTRL_RING, *PRTMP_CTRL_RING;

typedef struct PROTECTION_FRAME_STRUCT {
	BOOLEAN     Occupied;
       ULONG         Triggernumber;
       ULONG         Valid;  
       ULONG         NodeType; 
       ULONG         BssHashID; 
       ULONG         FrameType; 
} PROTECTION_FRAME_STRUCT, *PPROTECTION_FRAME_STRUCT;


typedef	struct	{
	USHORT		Ver:2;				// Protocol version
	USHORT		Type:2;				// MSDU type
	USHORT		SubType:4;			// MSDU subtype
	USHORT		ToDs:1;				// To DS indication
	USHORT		FrDs:1;				// From DS indication
	USHORT		MoreFrag:1;			// More fragment bit
	USHORT		Retry:1;			// Retry status bit
	USHORT		PwrMgmt:1;			// Power management bit
	USHORT		MoreData:1;			// More data bit
	USHORT		Wep:1;				// Wep data
	USHORT		Order:1;			// Strict order expected
}	FRAME_CONTROL, *PFRAME_CONTROL;

typedef	struct	_HEADER_802_11	{
    FRAME_CONTROL   FC;
    USHORT          Duration;
    UCHAR           Addr1[MAC_ADDR_LEN];
    UCHAR           Addr2[MAC_ADDR_LEN];
	UCHAR			Addr3[MAC_ADDR_LEN];
	USHORT			Frag:4;
	USHORT			Sequence:12;
}	HEADER_802_11, *PHEADER_802_11;


typedef	struct GNU_PACKED _TXWI_NMAC {
	/* Word	0 */
	/* ex: 00 03 00 40 means txop = 3, PHYMODE = 1 */
	UINT32		FRAG:1;		/* 1 to inform TKIP engine this is a fragment. */
	UINT32		MIMOps:1;	/* the remote peer is in dynamic MIMO-PS mode */
	UINT32		CFACK:1;
	UINT32		TS:1;
	UINT32		AMPDU:1;
	UINT32		MpduDensity:3;

	UINT32		txop:2;
	UINT32		NDPSndRate:2; /* 0 : MCS0, 1: MCS8, 2: MCS16, 3: reserved */
	UINT32		NDPSndBW:1; /* NDP sounding BW */
	UINT32		Sounding:1;
	UINT32		rsv0:1;
	UINT32		lut_en:1;
	
	UINT32		MCS:7;
	UINT32		BW:2;		/*channel bandwidth 20/40/80 MHz */
	UINT32		ShortGI:1;
	UINT32		STBC:1;
	UINT32		eTxBF:1;
	UINT32		iTxBF:1;
	UINT32		PHYMODE:3;  

	/* Word1 */
	/* ex:  1c ff 38 00 means ACK=0, BAWinSize=7, MPDUtotalByteCnt = 0x38 */
	UINT32		ACK:1;
	UINT32		NSEQ:1;
	UINT32		BAWinSize:6;
	UINT32		wcid:8;
	UINT32		MPDUtotalByteCnt:14;
	UINT32		Rsv1:2;
	
	/*Word2 */
	UINT32		IV;
	
	/*Word3 */
	UINT32		EIV;

	/* Word 4 */
	UINT32		TxEAPId:8;
	UINT32		TxStreamMode:8;
	UINT32		TxPwrAdj:4;
	UINT32		Rsv4:4;	
	UINT32		TxPktId:8;
}	TXWI_NMAC, *PTXWI_NMAC;

typedef	union GNU_PACKED _TXWI_STRUC {
	struct _TXWI_NMAC TXWI_N;
	UINT32 word;
}TXWI_STRUC;

typedef struct _COEX_PROTECTION_FRAME_INFO
{
	ULONG					CoexOperation;
    ULONG                   Triggernumber;   
    ULONG                   Valid;
    ULONG                   NodeType;
    ULONG	            BssHashID;
    ULONG                   FrameType;    
} COEX_PROTECTION_FRAME_INFO, *PCOEX_PROTECTION_FRAME_INFO;

typedef	struct GNU_PACKED _RXWI_NMAC {
	/* Word 0 */
	UINT32 wcid:8;
	UINT32 key_idx:2;
	UINT32 bss_idx:3;
	UINT32 udf:3;
	UINT32 MPDUtotalByteCnt:14; /* mpdu_total_byte = rxfceinfo_len - rxwi_len- rxinfo_len - l2pad */
	UINT32 rsv:1;
	UINT32 eof:1;

	/* Word 1 */
	UINT32 tid:4;
	UINT32 sn:12;
	UINT32 mcs:7;
	UINT32 bw:2;
	UINT32 sgi:1;
	UINT32 stbc:1;
	UINT32 e_txbf:1;
	UINT32 i_txbf:1;
	UINT32 phy_mode:3;

	/* Word 2 */
	UINT8 rssi[4];

	/* Word 3~6 */
	UINT8 bbp_rxinfo[16];
}	RXWI_NMAC;

typedef	union GNU_PACKED _RXWI_STRUC {
	struct _RXWI_NMAC RXWI_N;
}RXWI_STRUC;
////////////////////////woody//////////////////////


/* Debug definitions.
 * Debug output has to be enabled during compile time.
 */
#ifdef CONFIG_RT2X00_DEBUG
#define DEBUG
#endif /* CONFIG_RT2X00_DEBUG */

/* Utility printing macros
 * rt2x00_probe_err is for messages when rt2x00_dev is uninitialized
 */
#define rt2x00_probe_err(fmt, ...)					\
	printk(KERN_ERR KBUILD_MODNAME ": %s: Error - " fmt,		\
	       __func__, ##__VA_ARGS__)
#define rt2x00_err(dev, fmt, ...)					\
	wiphy_err((dev)->hw->wiphy, "%s: Error - " fmt,			\
		  __func__, ##__VA_ARGS__)
#define rt2x00_warn(dev, fmt, ...)					\
	wiphy_warn((dev)->hw->wiphy, "%s: Warning - " fmt,		\
		   __func__, ##__VA_ARGS__)
#define rt2x00_info(dev, fmt, ...)					\
	wiphy_info((dev)->hw->wiphy, "%s: Info - " fmt,			\
		   __func__, ##__VA_ARGS__)

/* Various debug levels */
#define rt2x00_dbg(dev, fmt, ...)					\
	wiphy_dbg((dev)->hw->wiphy, "%s: Debug - " fmt,			\
		  __func__, ##__VA_ARGS__)
#define rt2x00_eeprom_dbg(dev, fmt, ...)				\
	wiphy_dbg((dev)->hw->wiphy, "%s: EEPROM recovery - " fmt,	\
		  __func__, ##__VA_ARGS__)

#define DEBUG_PRINTK_MSG(__dev, __kernlvl, __lvl, __msg, __args...)	\
	printk(__kernlvl "%s -> %s: %s - " __msg,			\
	       wiphy_name((__dev)->hw->wiphy), __func__, __lvl, ##__args)
/*
 * Various debug levels.
 * The debug levels PANIC and ERROR both indicate serious problems,
 * for this reason they should never be ignored.
 * The special ERROR_PROBE message is for messages that are generated
 * when the rt2x00_dev is not yet initialized.
 */
#define PANIC(__dev, __msg, __args...) \
	DEBUG_PRINTK_MSG(__dev, KERN_CRIT, "Panic", __msg, ##__args)
#define ERROR(__dev, __msg, __args...)	\
	DEBUG_PRINTK_MSG(__dev, KERN_ERR, "Error", __msg, ##__args)
#define ERROR_PROBE(__msg, __args...) \
	DEBUG_PRINTK_PROBE(KERN_ERR, "Error", __msg, ##__args)

#ifdef CONFIG_RT2X00_DEBUG
#define DEBUG_PRINTK(__dev, __kernlvl, __lvl, __msg, __args...)	\
	DEBUG_PRINTK_MSG(__dev, __kernlvl, __lvl, __msg, ##__args)
#else
#define DEBUG_PRINTK(__dev, __kernlvl, __lvl, __msg, __args...)	\
	do { } while (0)
#endif /* CONFIG_RT2X00_DEBUG */


#define WARNING(__dev, __msg, __args...) \
	DEBUG_PRINTK(__dev, KERN_WARNING, "Warning", __msg, ##__args)
#define NOTICE(__dev, __msg, __args...) \
	DEBUG_PRINTK(__dev, KERN_NOTICE, "Notice", __msg, ##__args)
#define INFO(__dev, __msg, __args...) \
	DEBUG_PRINTK(__dev, KERN_INFO, "Info", __msg, ##__args)
#define DEBUG(__dev, __msg, __args...) \
	DEBUG_PRINTK(__dev, KERN_DEBUG, "Debug", __msg, ##__args)
#define EEPROM(__dev, __msg, __args...) \
	DEBUG_PRINTK(__dev, KERN_DEBUG, "EEPROM recovery", __msg, ##__args)
	
/*
 * Duration calculations
 * The rate variable passed is: 100kbs.
 * To convert from bytes to bits we multiply size with 8,
 * then the size is multiplied with 10 to make the
 * real rate -> rate argument correction.
 */
#define GET_DURATION(__size, __rate)	(((__size) * 8 * 10) / (__rate))
#define GET_DURATION_RES(__size, __rate)(((__size) * 8 * 10) % (__rate))

/*
 * Determine the number of L2 padding bytes required between the header and
 * the payload.
 */
#define L2PAD_SIZE(__hdrlen)	(-(__hdrlen) & 3)

/*
 * Determine the alignment requirement,
 * to make sure the 802.11 payload is padded to a 4-byte boundrary
 * we must determine the address of the payload and calculate the
 * amount of bytes needed to move the data.
 */
#define ALIGN_SIZE(__skb, __header) \
	(  ((unsigned long)((__skb)->data + (__header))) & 3 )

/*
 * Constants for extra TX headroom for alignment purposes.
 */
#define RT2X00_ALIGN_SIZE	4 /* Only whole frame needs alignment */
#define RT2X00_L2PAD_SIZE	8 /* Both header & payload need alignment */

/*
 * Standard timing and size defines.
 * These values should follow the ieee80211 specifications.
 */
#define ACK_SIZE		14
#define IEEE80211_HEADER	24
#define PLCP			48
#define BEACON			100
#define PREAMBLE		144
#define SHORT_PREAMBLE		72
#define SLOT_TIME		20
#define SHORT_SLOT_TIME		9
#define SIFS			10
#define PIFS			( SIFS + SLOT_TIME )
#define SHORT_PIFS		( SIFS + SHORT_SLOT_TIME )
#define DIFS			( PIFS + SLOT_TIME )
#define SHORT_DIFS		( SHORT_PIFS + SHORT_SLOT_TIME )
#define EIFS			( SIFS + DIFS + \
				  GET_DURATION(IEEE80211_HEADER + ACK_SIZE, 10) )
#define SHORT_EIFS		( SIFS + SHORT_DIFS + \
				  GET_DURATION(IEEE80211_HEADER + ACK_SIZE, 10) )

/*
 * Structure for average calculation
 * The avg field contains the actual average value,
 * but avg_weight is internally used during calculations
 * to prevent rounding errors.
 */
struct avg_val {
	int avg;
	int avg_weight;
};

enum rt2x00_chip_intf {
	RT2X00_CHIP_INTF_PCI,
	RT2X00_CHIP_INTF_PCIE,
	RT2X00_CHIP_INTF_USB,
	RT2X00_CHIP_INTF_SOC,
};

/*
 * Chipset identification
 * The chipset on the device is composed of a RT and RF chip.
 * The chipset combination is important for determining device capabilities.
 */
struct rt2x00_chip {
	u16 rt;
#define RT2460		0x2460
#define RT2560		0x2560
#define RT2570		0x2570
#define RT2661		0x2661
#define RT2573		0x2573
#define RT2860		0x2860	/* 2.4GHz */
#define RT2872		0x2872	/* WSOC */
#define RT2883		0x2883	/* WSOC */
#define RT3070		0x3070
#define RT3071		0x3071
#define RT3090		0x3090	/* 2.4GHz PCIe */
#define RT3290		0x3290
#define RT3352		0x3352  /* WSOC */
#define RT3390		0x3390
#define RT3572		0x3572
#define RT3593		0x3593
#define RT3883		0x3883	/* WSOC */
#define RT5390		0x5390  /* 2.4GHz */
#define RT5392		0x5392  /* 2.4GHz */
#define RT5592		0x5592
#define MT7630		0x7630

	u16 rf;
	u16 rev;

	enum rt2x00_chip_intf intf;
};

/*
 * RF register values that belong to a particular channel.
 */
struct rf_channel {
	int channel;
	u32 rf1;
	u32 rf2;
	u32 rf3;
	u32 rf4;
};

/*
 * Channel information structure
 */
struct channel_info {
	unsigned int flags;
#define GEOGRAPHY_ALLOWED	0x00000001

	short max_power;
	short default_power1;
	short default_power2;
};

/*
 * Antenna setup values.
 */
struct antenna_setup {
	enum antenna rx;
	enum antenna tx;
	u8 rx_chain_num;
	u8 tx_chain_num;
};

/*
 * Quality statistics about the currently active link.
 */
struct link_qual {
	/*
	 * Statistics required for Link tuning by driver
	 * The rssi value is provided by rt2x00lib during the
	 * link_tuner() callback function.
	 * The false_cca field is filled during the link_stats()
	 * callback function and could be used during the
	 * link_tuner() callback function.
	 */
	int rssi;
	int false_cca;

	/*
	 * VGC levels
	 * Hardware driver will tune the VGC level during each call
	 * to the link_tuner() callback function. This vgc_level is
	 * is determined based on the link quality statistics like
	 * average RSSI and the false CCA count.
	 *
	 * In some cases the drivers need to differentiate between
	 * the currently "desired" VGC level and the level configured
	 * in the hardware. The latter is important to reduce the
	 * number of BBP register reads to reduce register access
	 * overhead. For this reason we store both values here.
	 */
	u8 vgc_level;
	u8 vgc_level_reg;

	/*
	 * Statistics required for Signal quality calculation.
	 * These fields might be changed during the link_stats()
	 * callback function.
	 */
	int rx_success;
	int rx_failed;
	int tx_success;
	int tx_failed;
};

/*
 * Antenna settings about the currently active link.
 */
struct link_ant {
	/*
	 * Antenna flags
	 */
	unsigned int flags;
#define ANTENNA_RX_DIVERSITY	0x00000001
#define ANTENNA_TX_DIVERSITY	0x00000002
#define ANTENNA_MODE_SAMPLE	0x00000004

	/*
	 * Currently active TX/RX antenna setup.
	 * When software diversity is used, this will indicate
	 * which antenna is actually used at this time.
	 */
	struct antenna_setup active;

	/*
	 * RSSI history information for the antenna.
	 * Used to determine when to switch antenna
	 * when using software diversity.
	 */
	int rssi_history;

	/*
	 * Current RSSI average of the currently active antenna.
	 * Similar to the avg_rssi in the link_qual structure
	 * this value is updated by using the walking average.
	 */
	struct avg_val rssi_ant;
};

/*
 * To optimize the quality of the link we need to store
 * the quality of received frames and periodically
 * optimize the link.
 */
struct link {
	/*
	 * Link tuner counter
	 * The number of times the link has been tuned
	 * since the radio has been switched on.
	 */
	u32 count;

	/*
	 * Quality measurement values.
	 */
	struct link_qual qual;

	/*
	 * TX/RX antenna setup.
	 */
	struct link_ant ant;

	/*
	 * Currently active average RSSI value
	 */
	struct avg_val avg_rssi;

	/*
	 * Work structure for scheduling periodic link tuning.
	 */
	struct delayed_work work;

	/*
	 * Work structure for scheduling periodic watchdog monitoring.
	 * This work must be scheduled on the kernel workqueue, while
	 * all other work structures must be queued on the mac80211
	 * workqueue. This guarantees that the watchdog can schedule
	 * other work structures and wait for their completion in order
	 * to bring the device/driver back into the desired state.
	 */
	struct delayed_work watchdog_work;

	/*
	 * Work structure for scheduling periodic AGC adjustments.
	 */
	struct delayed_work agc_work;

	/*
	 * Work structure for scheduling periodic VCO calibration.
	 */
	struct delayed_work vco_work;
};

enum rt2x00_delayed_flags {
	DELAYED_UPDATE_BEACON,
};

/*
 * Interface structure
 * Per interface configuration details, this structure
 * is allocated as the private data for ieee80211_vif.
 */
struct rt2x00_intf {
	/*
	 * beacon->skb must be protected with the mutex.
	 */
	struct mutex beacon_skb_mutex;

	/*
	 * Entry in the beacon queue which belongs to
	 * this interface. Each interface has its own
	 * dedicated beacon entry.
	 */
	struct queue_entry *beacon;
	bool enable_beacon;

	/*
	 * Actions that needed rescheduling.
	 */
	unsigned long delayed_flags;

	/*
	 * Software sequence counter, this is only required
	 * for hardware which doesn't support hardware
	 * sequence counting.
	 */
	atomic_t seqno;
};

static inline struct rt2x00_intf* vif_to_intf(struct ieee80211_vif *vif)
{
	return (struct rt2x00_intf *)vif->drv_priv;
}

/**
 * struct hw_mode_spec: Hardware specifications structure
 *
 * Details about the supported modes, rates and channels
 * of a particular chipset. This is used by rt2x00lib
 * to build the ieee80211_hw_mode array for mac80211.
 *
 * @supported_bands: Bitmask contained the supported bands (2.4GHz, 5.2GHz).
 * @supported_rates: Rate types which are supported (CCK, OFDM).
 * @num_channels: Number of supported channels. This is used as array size
 *	for @tx_power_a, @tx_power_bg and @channels.
 * @channels: Device/chipset specific channel values (See &struct rf_channel).
 * @channels_info: Additional information for channels (See &struct channel_info).
 * @ht: Driver HT Capabilities (See &ieee80211_sta_ht_cap).
 */
struct hw_mode_spec {
	unsigned int supported_bands;
#define SUPPORT_BAND_2GHZ	0x00000001
#define SUPPORT_BAND_5GHZ	0x00000002

	unsigned int supported_rates;
#define SUPPORT_RATE_CCK	0x00000001
#define SUPPORT_RATE_OFDM	0x00000002

	unsigned int num_channels;
	const struct rf_channel *channels;
	const struct channel_info *channels_info;

	struct ieee80211_sta_ht_cap ht;
};

/*
 * Configuration structure wrapper around the
 * mac80211 configuration structure.
 * When mac80211 configures the driver, rt2x00lib
 * can precalculate values which are equal for all
 * rt2x00 drivers. Those values can be stored in here.
 */
struct rt2x00lib_conf {
	struct ieee80211_conf *conf;

	struct rf_channel rf;
	struct channel_info channel;
};

/*
 * Configuration structure for erp settings.
 */
struct rt2x00lib_erp {
	int short_preamble;
	int cts_protection;

	u32 basic_rates;

	int slot_time;

	short sifs;
	short pifs;
	short difs;
	short eifs;

	u16 beacon_int;
	u16 ht_opmode;
};

/*
 * Configuration structure for hardware encryption.
 */
struct rt2x00lib_crypto {
	enum cipher cipher;

	enum set_key_cmd cmd;
	const u8 *address;

	u32 bssidx;

	u8 key[16];
	u8 tx_mic[8];
	u8 rx_mic[8];

	int wcid;
};

/*
 * Configuration structure wrapper around the
 * rt2x00 interface configuration handler.
 */
struct rt2x00intf_conf {
	/*
	 * Interface type
	 */
	enum nl80211_iftype type;

	/*
	 * TSF sync value, this is dependent on the operation type.
	 */
	enum tsf_sync sync;

	/*
	 * The MAC and BSSID addresses are simple array of bytes,
	 * these arrays are little endian, so when sending the addresses
	 * to the drivers, copy the it into a endian-signed variable.
	 *
	 * Note that all devices (except rt2500usb) have 32 bits
	 * register word sizes. This means that whatever variable we
	 * pass _must_ be a multiple of 32 bits. Otherwise the device
	 * might not accept what we are sending to it.
	 * This will also make it easier for the driver to write
	 * the data to the device.
	 */
	__le32 mac[2];
	__le32 bssid[2];
};

/*
 * Private structure for storing STA details
 * wcid: Wireless Client ID
 */
struct rt2x00_sta {
	int wcid;
};

static inline struct rt2x00_sta* sta_to_rt2x00_sta(struct ieee80211_sta *sta)
{
	return (struct rt2x00_sta *)sta->drv_priv;
}

/*
 * rt2x00lib callback functions.
 */
struct rt2x00lib_ops {
	/*
	 * Interrupt handlers.
	 */
	irq_handler_t irq_handler;

	/*
	 * TX status tasklet handler.
	 */
	void (*tx8damdone_tasklet) (unsigned long data); 
	void (*txstatus_tasklet) (unsigned long data);
	void (*pretbtt_tasklet) (unsigned long data);
	void (*tbtt_tasklet) (unsigned long data);
	void (*rxdone_tasklet) (unsigned long data);
	void (*autowake_tasklet) (unsigned long data);

	/*
	 * Device init handlers.
	 */
	int (*probe_hw) (struct rt2x00_dev *rt2x00dev);
	char *(*get_firmware_name) (struct rt2x00_dev *rt2x00dev);
	int (*check_firmware) (struct rt2x00_dev *rt2x00dev,
			       const u8 *data, const size_t len);
	int (*load_firmware) (struct rt2x00_dev *rt2x00dev,
			      const u8 *data, const size_t len);

	/*
	 * Device initialization/deinitialization handlers.
	 */
	int (*initialize) (struct rt2x00_dev *rt2x00dev);
	void (*uninitialize) (struct rt2x00_dev *rt2x00dev);

	/*
	 * queue initialization handlers
	 */
	bool (*get_entry_state) (struct queue_entry *entry);
	void (*clear_entry) (struct queue_entry *entry);

	/*
	 * Radio control handlers.
	 */
	int (*set_device_state) (struct rt2x00_dev *rt2x00dev,
				 enum dev_state state);
	int (*rfkill_poll) (struct rt2x00_dev *rt2x00dev);
	void (*link_stats) (struct rt2x00_dev *rt2x00dev,
			    struct link_qual *qual);
	void (*reset_tuner) (struct rt2x00_dev *rt2x00dev,
			     struct link_qual *qual);
	void (*link_tuner) (struct rt2x00_dev *rt2x00dev,
			    struct link_qual *qual, const u32 count);
	void (*gain_calibration) (struct rt2x00_dev *rt2x00dev);
	void (*vco_calibration) (struct rt2x00_dev *rt2x00dev);

	/*
	 * Data queue handlers.
	 */
	void (*watchdog) (struct rt2x00_dev *rt2x00dev);
	void (*start_queue) (struct data_queue *queue);
	void (*kick_queue) (struct data_queue *queue);
	void (*stop_queue) (struct data_queue *queue);
	void (*flush_queue) (struct data_queue *queue, bool drop);
	void (*tx_dma_done) (struct queue_entry *entry);

	/*
	 * TX control handlers
	 */
	void (*write_tx_desc) (struct queue_entry *entry,
			       struct txentry_desc *txdesc);
	void (*write_tx_data) (struct queue_entry *entry,
			       struct txentry_desc *txdesc);
	void (*write_beacon) (struct queue_entry *entry,
			      struct txentry_desc *txdesc);
	void (*clear_beacon) (struct queue_entry *entry);
	int (*get_tx_data_len) (struct queue_entry *entry);

	/*
	 * RX control handlers
	 */
	void (*fill_rxdone) (struct queue_entry *entry,
			     struct rxdone_entry_desc *rxdesc);

	/*
	 * Configuration handlers.
	 */
	int (*config_shared_key) (struct rt2x00_dev *rt2x00dev,
				  struct rt2x00lib_crypto *crypto,
				  struct ieee80211_key_conf *key);
	int (*config_pairwise_key) (struct rt2x00_dev *rt2x00dev,
				    struct rt2x00lib_crypto *crypto,
				    struct ieee80211_key_conf *key);
	void (*config_filter) (struct rt2x00_dev *rt2x00dev,
			       const unsigned int filter_flags);
	void (*config_intf) (struct rt2x00_dev *rt2x00dev,
			     struct rt2x00_intf *intf,
			     struct rt2x00intf_conf *conf,
			     const unsigned int flags);
#define CONFIG_UPDATE_TYPE		( 1 << 1 )
#define CONFIG_UPDATE_MAC		( 1 << 2 )
#define CONFIG_UPDATE_BSSID		( 1 << 3 )

	void (*config_erp) (struct rt2x00_dev *rt2x00dev,
			    struct rt2x00lib_erp *erp,
			    u32 changed);
	void (*config_ant) (struct rt2x00_dev *rt2x00dev,
			    struct antenna_setup *ant);
	void (*config) (struct rt2x00_dev *rt2x00dev,
			struct rt2x00lib_conf *libconf,
			const unsigned int changed_flags);
	int (*sta_add) (struct rt2x00_dev *rt2x00dev,
			struct ieee80211_vif *vif,
			struct ieee80211_sta *sta);
	int (*sta_remove) (struct rt2x00_dev *rt2x00dev,
			   int wcid);
};

/*
 * rt2x00 driver callback operation structure.
 */
struct rt2x00_ops {
	const char *name;
	const unsigned int drv_data_size;
	const unsigned int max_ap_intf;
	const unsigned int eeprom_size;
	const unsigned int rf_size;
	const unsigned int tx_queues;
	const unsigned int extra_tx_headroom;
	const struct data_queue_desc *rx;
	const struct data_queue_desc *tx;
	const struct data_queue_desc *bcn;
	const struct data_queue_desc *atim;
	const struct rt2x00lib_ops *lib;
	const void *drv;
	const struct ieee80211_ops *hw;
#ifdef CONFIG_RT2X00_LIB_DEBUGFS
	const struct rt2x00debug *debugfs;
#endif /* CONFIG_RT2X00_LIB_DEBUGFS */
};

/*
 * rt2x00 state flags
 */
enum rt2x00_state_flags {
	/*
	 * Device flags
	 */
	DEVICE_STATE_PRESENT,
	DEVICE_STATE_REGISTERED_HW,
	DEVICE_STATE_INITIALIZED,
	DEVICE_STATE_STARTED,
	DEVICE_STATE_ENABLED_RADIO,
	DEVICE_STATE_SCANNING,

	/*
	 * Driver configuration
	 */
	CONFIG_CHANNEL_HT40,
	CONFIG_POWERSAVING,
	CONFIG_HT_DISABLED,
	CONFIG_QOS_DISABLED,

	/*
	 * Mark we currently are sequentially reading TX_STA_FIFO register
	 * FIXME: this is for only rt2800usb, should go to private data
	 */
	TX_STATUS_READING,
};

/*
 * rt2x00 capability flags
 */
enum rt2x00_capability_flags {
	/*
	 * Requirements
	 */
	REQUIRE_FIRMWARE,
	REQUIRE_BEACON_GUARD,
	REQUIRE_ATIM_QUEUE,
	REQUIRE_DMA,
	REQUIRE_COPY_IV,
	REQUIRE_L2PAD,
	REQUIRE_TXSTATUS_FIFO,
	REQUIRE_TASKLET_CONTEXT,
	REQUIRE_SW_SEQNO,
	REQUIRE_HT_TX_DESC,
	REQUIRE_PS_AUTOWAKE,

	/*
	 * Capabilities
	 */
	CAPABILITY_HW_BUTTON,
	CAPABILITY_HW_CRYPTO,
	CAPABILITY_POWER_LIMIT,
	CAPABILITY_CONTROL_FILTERS,
	CAPABILITY_CONTROL_FILTER_PSPOLL,
	CAPABILITY_PRE_TBTT_INTERRUPT,
	CAPABILITY_LINK_TUNING,
	CAPABILITY_FRAME_TYPE,
	CAPABILITY_RF_SEQUENCE,
	CAPABILITY_EXTERNAL_LNA_A,
	CAPABILITY_EXTERNAL_LNA_BG,
	CAPABILITY_DOUBLE_ANTENNA,
	CAPABILITY_BT_COEXIST,
	CAPABILITY_VCO_RECALIBRATION,
};

/*
 * Interface combinations
 */
enum {
	IF_COMB_AP = 0,
	NUM_IF_COMB,
};

/*
 * rt2x00 device structure.
 */
struct rt2x00_dev {
	/*
	 * Device structure.
	 * The structure stored in here depends on the
	 * system bus (PCI or USB).
	 * When accessing this variable, the rt2x00dev_{pci,usb}
	 * macros should be used for correct typecasting.
	 */

// CONFIG_ANDES_SUPPORT
	u8 bssid[6];
	u8 addr[6];
	u16 ht_cap;
	struct MCU_CTRL MCUCtrl;
	COMMON_CONFIG    CommonCfg;
	COEX_MODE_STRUCTURE                CoexMode;
	PROTECTION_FRAME_STRUCT          NullFrameSpace[NULLFRAMESPACE];
	UINT8 TXWISize;
	RTMP_DMABUF CtrlDescRing;	/* Shared memory for CTRL descriptors */
	RTMP_CTRL_RING CtrlRing;
	NDIS_SPIN_LOCK CtrlRingLock;	/* Ctrl Ring spinlock */
	NDIS_SPIN_LOCK  CalLock;
	UCHAR connected;
	UCHAR connect_channel;
	COUNTER_RALINK RalinkCounters;
	COUNTER_802_11 WlanCounters;
	COUNTER_802_3 Counters8023;
	ULONG TxCount;
/* CONFIG_ANDES_SUPPORT */
	BOOLEAN PollIdle;
	 unsigned char alloc_len;
	 int bprint;
	int bscan; 
	unsigned int int_enable_reg;
	unsigned int int_disable_mask;
	unsigned int int_pending;
	u8 TxDmaIdx[4];	
	struct device *dev;

	/*
	 * Callback functions.
	 */
	const struct rt2x00_ops *ops;

	/*
	 * Driver data.
	 */
	void *drv_data;

	/*
	 * IEEE80211 control structure.
	 */
	struct ieee80211_hw *hw;
	struct ieee80211_supported_band bands[IEEE80211_NUM_BANDS];
	enum ieee80211_band curr_band;
	int curr_freq;

	/*
	 * If enabled, the debugfs interface structures
	 * required for deregistration of debugfs.
	 */
#ifdef CONFIG_RT2X00_LIB_DEBUGFS
	struct rt2x00debug_intf *debugfs_intf;
#endif /* CONFIG_RT2X00_LIB_DEBUGFS */

	/*
	 * LED structure for changing the LED status
	 * by mac8011 or the kernel.
	 */
#ifdef CONFIG_RT2X00_LIB_LEDS
	struct rt2x00_led led_radio;
	struct rt2x00_led led_assoc;
	struct rt2x00_led led_qual;
	u16 led_mcu_reg;
#endif /* CONFIG_RT2X00_LIB_LEDS */

	/*
	 * Device state flags.
	 * In these flags the current status is stored.
	 * Access to these flags should occur atomically.
	 */
	unsigned long flags;

	/*
	 * Device capabiltiy flags.
	 * In these flags the device/driver capabilities are stored.
	 * Access to these flags should occur non-atomically.
	 */
	unsigned long cap_flags;

	/*
	 * Device information, Bus IRQ and name (PCI, SoC)
	 */
	int irq;
	const char *name;

	/*
	 * Chipset identification.
	 */
	struct rt2x00_chip chip;

	/*
	 * hw capability specifications.
	 */
	struct hw_mode_spec spec;

	/*
	 * This is the default TX/RX antenna setup as indicated
	 * by the device's EEPROM.
	 */
	struct antenna_setup default_ant;

	/*
	 * Register pointers
	 * csr.base: CSR base register address. (PCI)
	 * csr.cache: CSR cache for usb_control_msg. (USB)
	 */
	union csr {
		void __iomem *base;
		void *cache;
	} csr;

	/*
	 * Mutex to protect register accesses.
	 * For PCI and USB devices it protects against concurrent indirect
	 * register access (BBP, RF, MCU) since accessing those
	 * registers require multiple calls to the CSR registers.
	 * For USB devices it also protects the csr_cache since that
	 * field is used for normal CSR access and it cannot support
	 * multiple callers simultaneously.
	 */
	struct mutex csr_mutex;

	/*
	 * Current packet filter configuration for the device.
	 * This contains all currently active FIF_* flags send
	 * to us by mac80211 during configure_filter().
	 */
	unsigned int packet_filter;

	/*
	 * Interface details:
	 *  - Open ap interface count.
	 *  - Open sta interface count.
	 *  - Association count.
	 *  - Beaconing enabled count.
	 */
	unsigned int intf_ap_count;
	unsigned int intf_sta_count;
	unsigned int intf_associated;
	unsigned int intf_beaconing;

	/*
	 * Interface combinations
	 */
	struct ieee80211_iface_limit if_limits_ap;
	struct ieee80211_iface_combination if_combinations[NUM_IF_COMB];

	/*
	 * Link quality
	 */
	struct link link;

	/*
	 * EEPROM data.
	 */
	__le16 *eeprom;

	/*
	 * Active RF register values.
	 * These are stored here so we don't need
	 * to read the rf registers and can directly
	 * use this value instead.
	 * This field should be accessed by using
	 * rt2x00_rf_read() and rt2x00_rf_write().
	 */
	u32 *rf;

	/*
	 * LNA gain
	 */
	short lna_gain;

	/*
	 * Current TX power value.
	 */
	u16 tx_power;

	/*
	 * Current retry values.
	 */
	u8 short_retry;
	u8 long_retry;

	/*
	 * Rssi <-> Dbm offset
	 */
	u8 rssi_offset;

	/*
	 * Frequency offset.
	 */
	u8 freq_offset;

	/*
	 * Association id.
	 */
	u16 aid;

	/*
	 * Beacon interval.
	 */
	u16 beacon_int;

	/**
	 * Timestamp of last received beacon
	 */
	unsigned long last_beacon;

	/*
	 * Low level statistics which will have
	 * to be kept up to date while device is running.
	 */
	struct ieee80211_low_level_stats low_level_stats;

	/**
	 * Work queue for all work which should not be placed
	 * on the mac80211 workqueue (because of dependencies
	 * between various work structures).
	 */
	struct workqueue_struct *workqueue;

	/*
	 * Scheduled work.
	 * NOTE: intf_work will use ieee80211_iterate_active_interfaces()
	 * which means it cannot be placed on the hw->workqueue
	 * due to RTNL locking requirements.
	 */
	struct work_struct intf_work;

	/**
	 * Scheduled work for TX/RX done handling (USB devices)
	 */
	struct work_struct rxdone_work;
	struct work_struct txdone_work;
	work_func_t txdone_workfn;

	/*
	 * Powersaving work
	 */
	struct delayed_work autowakeup_work;
	struct work_struct sleep_work;

	/*
	 * Data queue arrays for RX, TX, Beacon and ATIM.
	 */
	unsigned int data_queues;
	struct data_queue *rx;
	struct data_queue *tx;
	struct data_queue *bcn;
	struct data_queue *atim;

	/*
	 * Firmware image.
	 */
	const struct firmware *fw;

	/*
	 * FIFO for storing tx status reports between isr and tasklet.
	 */
	DECLARE_KFIFO_PTR(txstatus_fifo, u32);

	/*
	 * Timer to ensure tx status reports are read (rt2800usb).
	 */
	struct hrtimer txstatus_timer;

	/*
	 * Tasklet for processing tx status reports (rt2800pci).
	 */
	struct tasklet_struct tx8damdone_tasklet;	 
	struct tasklet_struct txstatus_tasklet;
	struct tasklet_struct pretbtt_tasklet;
	struct tasklet_struct tbtt_tasklet;
	struct tasklet_struct rxdone_tasklet;
	struct tasklet_struct autowake_tasklet;

	/*
	 * Used for VCO periodic calibration.
	 */
	int rf_channel;

	/*
	 * Protect the interrupt mask register.
	 */
	spinlock_t irqmask_lock;
	spinlock_t LockInterrupt;
	spinlock_t Ctrl_LockInterrupt;
	/*
	 * List of BlockAckReq TX entries that need driver BlockAck processing.
	 */
	struct list_head bar_list;
	spinlock_t bar_list_lock;
};

struct rt2x00_bar_list_entry {
	struct list_head list;
	struct rcu_head head;

	struct queue_entry *entry;
	int block_acked;

	/* Relevant parts of the IEEE80211 BAR header */
	__u8 ra[6];
	__u8 ta[6];
	__le16 control;
	__le16 start_seq_num;
};

/*
 * Register defines.
 * Some registers require multiple attempts before success,
 * in those cases REGISTER_BUSY_COUNT attempts should be
 * taken with a REGISTER_BUSY_DELAY interval.
 */
#define REGISTER_BUSY_COUNT	100
#define REGISTER_BUSY_DELAY	100

/*
 * Generic RF access.
 * The RF is being accessed by word index.
 */
static inline void rt2x00_rf_read(struct rt2x00_dev *rt2x00dev,
				  const unsigned int word, u32 *data)
{
	BUG_ON(word < 1 || word > rt2x00dev->ops->rf_size / sizeof(u32));
	*data = rt2x00dev->rf[word - 1];
}

static inline void rt2x00_rf_write(struct rt2x00_dev *rt2x00dev,
				   const unsigned int word, u32 data)
{
	BUG_ON(word < 1 || word > rt2x00dev->ops->rf_size / sizeof(u32));
	rt2x00dev->rf[word - 1] = data;
}

/*
 * Generic EEPROM access. The EEPROM is being accessed by word or byte index.
 */
static inline void *rt2x00_eeprom_addr(struct rt2x00_dev *rt2x00dev,
				       const unsigned int word)
{
	return (void *)&rt2x00dev->eeprom[word];
}

static inline void rt2x00_eeprom_read(struct rt2x00_dev *rt2x00dev,
				      const unsigned int word, u16 *data)
{
	*data = le16_to_cpu(rt2x00dev->eeprom[word]);
}

static inline void rt2x00_eeprom_write(struct rt2x00_dev *rt2x00dev,
				       const unsigned int word, u16 data)
{
	rt2x00dev->eeprom[word] = cpu_to_le16(data);
}

static inline u8 rt2x00_eeprom_byte(struct rt2x00_dev *rt2x00dev,
				    const unsigned int byte)
{
	return *(((u8 *)rt2x00dev->eeprom) + byte);
}

/*
 * Chipset handlers
 */
static inline void rt2x00_set_chip(struct rt2x00_dev *rt2x00dev,
				   const u16 rt, const u16 rf, const u16 rev)
{
	rt2x00dev->chip.rt = rt;
	rt2x00dev->chip.rf = rf;
	rt2x00dev->chip.rev = rev;

	rt2x00_info(rt2x00dev, "Chipset detected - rt: %04x, rf: %04x, rev: %04x\n",
		    rt2x00dev->chip.rt, rt2x00dev->chip.rf,
		    rt2x00dev->chip.rev);
}

static inline void rt2x00_set_rt(struct rt2x00_dev *rt2x00dev,
				 const u16 rt, const u16 rev)
{
	rt2x00dev->chip.rt = rt;
	rt2x00dev->chip.rev = rev;

	rt2x00_info(rt2x00dev, "RT chipset %04x, rev %04x detected\n",
		    rt2x00dev->chip.rt, rt2x00dev->chip.rev);
}

static inline void rt2x00_set_rf(struct rt2x00_dev *rt2x00dev, const u16 rf)
{
	rt2x00dev->chip.rf = rf;

	rt2x00_info(rt2x00dev, "RF chipset %04x detected\n",
		    rt2x00dev->chip.rf);
}

static inline bool rt2x00_rt(struct rt2x00_dev *rt2x00dev, const u16 rt)
{
	return (rt2x00dev->chip.rt == rt);
}

static inline bool rt2x00_rf(struct rt2x00_dev *rt2x00dev, const u16 rf)
{
	return (rt2x00dev->chip.rf == rf);
}

static inline u16 rt2x00_rev(struct rt2x00_dev *rt2x00dev)
{
	return rt2x00dev->chip.rev;
}

static inline bool rt2x00_rt_rev(struct rt2x00_dev *rt2x00dev,
				 const u16 rt, const u16 rev)
{
	return (rt2x00_rt(rt2x00dev, rt) && rt2x00_rev(rt2x00dev) == rev);
}

static inline bool rt2x00_rt_rev_lt(struct rt2x00_dev *rt2x00dev,
				    const u16 rt, const u16 rev)
{
	return (rt2x00_rt(rt2x00dev, rt) && rt2x00_rev(rt2x00dev) < rev);
}

static inline bool rt2x00_rt_rev_gte(struct rt2x00_dev *rt2x00dev,
				     const u16 rt, const u16 rev)
{
	return (rt2x00_rt(rt2x00dev, rt) && rt2x00_rev(rt2x00dev) >= rev);
}

static inline void rt2x00_set_chip_intf(struct rt2x00_dev *rt2x00dev,
					enum rt2x00_chip_intf intf)
{
	rt2x00dev->chip.intf = intf;
}

static inline bool rt2x00_intf(struct rt2x00_dev *rt2x00dev,
			       enum rt2x00_chip_intf intf)
{
	return (rt2x00dev->chip.intf == intf);
}

static inline bool rt2x00_is_pci(struct rt2x00_dev *rt2x00dev)
{
	return rt2x00_intf(rt2x00dev, RT2X00_CHIP_INTF_PCI) ||
	       rt2x00_intf(rt2x00dev, RT2X00_CHIP_INTF_PCIE);
}

static inline bool rt2x00_is_pcie(struct rt2x00_dev *rt2x00dev)
{
	return rt2x00_intf(rt2x00dev, RT2X00_CHIP_INTF_PCIE);
}

static inline bool rt2x00_is_usb(struct rt2x00_dev *rt2x00dev)
{
	return rt2x00_intf(rt2x00dev, RT2X00_CHIP_INTF_USB);
}

static inline bool rt2x00_is_soc(struct rt2x00_dev *rt2x00dev)
{
	return rt2x00_intf(rt2x00dev, RT2X00_CHIP_INTF_SOC);
}

/**
 * rt2x00queue_map_txskb - Map a skb into DMA for TX purposes.
 * @entry: Pointer to &struct queue_entry
 *
 * Returns -ENOMEM if mapping fail, 0 otherwise.
 */
int rt2x00queue_map_txskb(struct queue_entry *entry);

/**
 * rt2x00queue_unmap_skb - Unmap a skb from DMA.
 * @entry: Pointer to &struct queue_entry
 */
void rt2x00queue_unmap_skb(struct queue_entry *entry);

/**
 * rt2x00queue_get_tx_queue - Convert tx queue index to queue pointer
 * @rt2x00dev: Pointer to &struct rt2x00_dev.
 * @queue: rt2x00 queue index (see &enum data_queue_qid).
 *
 * Returns NULL for non tx queues.
 */
static inline struct data_queue *
rt2x00queue_get_tx_queue(struct rt2x00_dev *rt2x00dev,
			 const enum data_queue_qid queue)
{
	if (queue < rt2x00dev->ops->tx_queues && rt2x00dev->tx)
		return &rt2x00dev->tx[queue];

	if (queue == QID_ATIM)
		return rt2x00dev->atim;

	return NULL;
}

/**
 * rt2x00queue_get_entry - Get queue entry where the given index points to.
 * @queue: Pointer to &struct data_queue from where we obtain the entry.
 * @index: Index identifier for obtaining the correct index.
 */
struct queue_entry *rt2x00queue_get_entry(struct data_queue *queue,
					  enum queue_index index);

/**
 * rt2x00queue_pause_queue - Pause a data queue
 * @queue: Pointer to &struct data_queue.
 *
 * This function will pause the data queue locally, preventing
 * new frames to be added to the queue (while the hardware is
 * still allowed to run).
 */
void rt2x00queue_pause_queue(struct data_queue *queue);

/**
 * rt2x00queue_unpause_queue - unpause a data queue
 * @queue: Pointer to &struct data_queue.
 *
 * This function will unpause the data queue locally, allowing
 * new frames to be added to the queue again.
 */
void rt2x00queue_unpause_queue(struct data_queue *queue);

/**
 * rt2x00queue_start_queue - Start a data queue
 * @queue: Pointer to &struct data_queue.
 *
 * This function will start handling all pending frames in the queue.
 */
void rt2x00queue_start_queue(struct data_queue *queue);

/**
 * rt2x00queue_stop_queue - Halt a data queue
 * @queue: Pointer to &struct data_queue.
 *
 * This function will stop all pending frames in the queue.
 */
void rt2x00queue_stop_queue(struct data_queue *queue);

/**
 * rt2x00queue_flush_queue - Flush a data queue
 * @queue: Pointer to &struct data_queue.
 * @drop: True to drop all pending frames.
 *
 * This function will flush the queue. After this call
 * the queue is guaranteed to be empty.
 */
void rt2x00queue_flush_queue(struct data_queue *queue, bool drop);

/**
 * rt2x00queue_start_queues - Start all data queues
 * @rt2x00dev: Pointer to &struct rt2x00_dev.
 *
 * This function will loop through all available queues to start them
 */
void rt2x00queue_start_queues(struct rt2x00_dev *rt2x00dev);

/**
 * rt2x00queue_stop_queues - Halt all data queues
 * @rt2x00dev: Pointer to &struct rt2x00_dev.
 *
 * This function will loop through all available queues to stop
 * any pending frames.
 */
void rt2x00queue_stop_queues(struct rt2x00_dev *rt2x00dev);

/**
 * rt2x00queue_flush_queues - Flush all data queues
 * @rt2x00dev: Pointer to &struct rt2x00_dev.
 * @drop: True to drop all pending frames.
 *
 * This function will loop through all available queues to flush
 * any pending frames.
 */
void rt2x00queue_flush_queues(struct rt2x00_dev *rt2x00dev, bool drop);

/*
 * Debugfs handlers.
 */
/**
 * rt2x00debug_dump_frame - Dump a frame to userspace through debugfs.
 * @rt2x00dev: Pointer to &struct rt2x00_dev.
 * @type: The type of frame that is being dumped.
 * @skb: The skb containing the frame to be dumped.
 */
#ifdef CONFIG_RT2X00_LIB_DEBUGFS
void rt2x00debug_dump_frame(struct rt2x00_dev *rt2x00dev,
			    enum rt2x00_dump_type type, struct sk_buff *skb);
#else
static inline void rt2x00debug_dump_frame(struct rt2x00_dev *rt2x00dev,
					  enum rt2x00_dump_type type,
					  struct sk_buff *skb)
{
}
#endif /* CONFIG_RT2X00_LIB_DEBUGFS */

/*
 * Utility functions.
 */
u32 rt2x00lib_get_bssidx(struct rt2x00_dev *rt2x00dev,
			 struct ieee80211_vif *vif);

/*
 * Interrupt context handlers.
 */
void rt2x00lib_beacondone(struct rt2x00_dev *rt2x00dev);
void rt2x00lib_pretbtt(struct rt2x00_dev *rt2x00dev);
void rt2x00lib_dmastart(struct queue_entry *entry);
void rt2x00lib_dmadone(struct queue_entry *entry);
void rt2x00lib_txdone(struct queue_entry *entry,
		      struct txdone_entry_desc *txdesc);
void rt2x00lib_txdone_noinfo(struct queue_entry *entry, u32 status);
void rt2x00lib_rxdone(struct queue_entry *entry, gfp_t gfp);

/*
 * mac80211 handlers.
 */
void rt2x00mac_tx(struct ieee80211_hw *hw,
		  struct ieee80211_tx_control *control,
		  struct sk_buff *skb);
int rt2x00mac_start(struct ieee80211_hw *hw);
void rt2x00mac_stop(struct ieee80211_hw *hw);
int rt2x00mac_add_interface(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif);
void rt2x00mac_remove_interface(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif);
int rt2x00mac_config(struct ieee80211_hw *hw, u32 changed);
void rt2x00mac_configure_filter(struct ieee80211_hw *hw,
				unsigned int changed_flags,
				unsigned int *total_flags,
				u64 multicast);
int rt2x00mac_set_tim(struct ieee80211_hw *hw, struct ieee80211_sta *sta,
		      bool set);
#ifdef CONFIG_RT2X00_LIB_CRYPTO
int rt2x00mac_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		      struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		      struct ieee80211_key_conf *key);
#else
#define rt2x00mac_set_key	NULL
#endif /* CONFIG_RT2X00_LIB_CRYPTO */
int rt2x00mac_sta_add(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		      struct ieee80211_sta *sta);
int rt2x00mac_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			 struct ieee80211_sta *sta);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
void rt2x00mac_sw_scan_start(struct ieee80211_hw *hw);
void rt2x00mac_sw_scan_complete(struct ieee80211_hw *hw);
#else
void rt2x00mac_sw_scan_start(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			     const u8 *mac_addr);
void rt2x00mac_sw_scan_complete(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif);
#endif
int rt2x00mac_get_stats(struct ieee80211_hw *hw,
			struct ieee80211_low_level_stats *stats);
void rt2x00mac_bss_info_changed(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_bss_conf *bss_conf,
				u32 changes);
int rt2x00mac_conf_tx(struct ieee80211_hw *hw,
		      struct ieee80211_vif *vif, u16 queue,
		      const struct ieee80211_tx_queue_params *params);
void rt2x00mac_rfkill_poll(struct ieee80211_hw *hw);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 16, 0)
void rt2x00mac_flush(struct ieee80211_hw *hw, u32 queues, bool drop);
#else
void rt2x00mac_flush(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		     u32 queues, bool drop);
#endif
int rt2x00mac_set_antenna(struct ieee80211_hw *hw, u32 tx_ant, u32 rx_ant);
int rt2x00mac_get_antenna(struct ieee80211_hw *hw, u32 *tx_ant, u32 *rx_ant);
void rt2x00mac_get_ringparam(struct ieee80211_hw *hw,
			     u32 *tx, u32 *tx_max, u32 *rx, u32 *rx_max);
bool rt2x00mac_tx_frames_pending(struct ieee80211_hw *hw);

/*
 * Driver allocation handlers.
 */
int rt2x00lib_probe_dev(struct rt2x00_dev *rt2x00dev);
void rt2x00lib_remove_dev(struct rt2x00_dev *rt2x00dev);
#ifdef CONFIG_PM
int rt2x00lib_suspend(struct rt2x00_dev *rt2x00dev, pm_message_t state);
int rt2x00lib_resume(struct rt2x00_dev *rt2x00dev);
#endif /* CONFIG_PM */


int WaitForAsicReady(struct rt2x00_dev *rt2x00dev);

void AsicRemoveSharedKeyEntry(
	struct rt2x00_dev *rt2x00dev,
	unsigned char		 BssIndex,
	unsigned char	 KeyIdx);

VOID MCUCtrlInit(struct rt2x00_dev *rt2x00dev);

NDIS_STATUS os_alloc_mem(
	IN VOID *pReserved,
	OUT UCHAR **mem,
	IN ULONG size);

NDIS_STATUS os_free_mem(
	IN VOID *pReserved,
	IN PVOID mem);

INT AsicSendCmdToAndes(struct rt2x00_dev *rt2x00dev, struct CMD_UNIT *CmdUnit);

NDIS_STATUS RTMPAllocateNdisPacket(
	IN VOID *pReserved,
	OUT PNDIS_PACKET *ppPacket,
	IN UCHAR *pHeader,
	IN UINT HeaderLen,
	IN UCHAR *pData,
	IN UINT DataLen);

INT PCIKickOutCmd(
	struct rt2x00_dev *rt2x00dev, 
	UCHAR *Buf, 
	UINT32 Len);

void RTMP_QueryPacketInfo(
	IN PNDIS_PACKET pPacket,
	OUT PACKET_INFO *info,
	OUT UCHAR **pSrcBufVA,
	OUT UINT *pSrcBufLen);

NDIS_STATUS	RTMPAllocTxRxRingMemory(struct rt2x00_dev *rt2x00dev);
NDIS_STATUS RTMPInitTxRxRingMemory(struct rt2x00_dev *rt2x00dev);
VOID AsicInitTxRxRing(struct rt2x00_dev *rt2x00dev);
VOID RTMPFreeNdisPacket(
	IN VOID *pReserved,
	IN PNDIS_PACKET pPacket);
int	RTMPHandleTxRing8DmaDoneInterrupt(
	IN struct rt2x00_dev *rt2x00dev);

VOID SendAndesTFSWITCH(
	IN struct rt2x00_dev *rt2x00dev,
	IN UCHAR			CoexMode
	);

ra_dma_addr_t linux_pci_map_single(void *pPciDev, void *ptr, size_t size, int sd_idx, int direction);
ra_dma_addr_t RtmpDrvPciMapSingle(
	IN struct rt2x00_dev *rt2x00dev,
	IN VOID *ptr,
	IN size_t size,
	IN INT sd_idx,
	IN INT direction);
#define PCI_MAP_SINGLE					RtmpDrvPciMapSingle
VOID TDDFDDExclusiveRequest(
        IN struct rt2x00_dev *rt2x00dev, 
	UCHAR CoexMode 
	);

VOID SendAndesAFH(
	IN struct rt2x00_dev *rt2x00dev,
	IN UCHAR			BBPCurrentBW,
	IN UCHAR			Channel,
	IN UCHAR			CentralChannel,
	IN BOOLEAN			Disable,
	IN ULONG                     BssHashID);

VOID BtAFHCtl(
		IN struct rt2x00_dev *rt2x00dev,
		IN UCHAR			BBPCurrentBW,
		IN UCHAR			Channel,
		IN UCHAR			CentralChannel,
		IN BOOLEAN			Disable);

VOID UpdateAndesNullFrameSpace(
	IN struct rt2x00_dev *rt2x00dev);

VOID SendAndesCoexFrameInfo(
	IN struct rt2x00_dev *rt2x00dev, 
	IN ULONG TriggerNumber) ;

VOID EstablishFrameBundle(
	IN	 struct rt2x00_dev *rt2x00dev,
	IN      PUCHAR  pAddr,
	IN      ULONG  OPMode,
	IN      INT  WCID
);

INT AndesFunSetOP(IN struct rt2x00_dev *rt2x00dev, UINT32 FunID, UINT32 Param);
void RTMPusecDelay(unsigned long usec);
void MT76x0_VCO_CalibrationMode3(
	struct rt2x00_dev *rt2x00dev);
VOID MT76x0_Calibration(
	IN struct rt2x00_dev *rt2x00dev,
	IN UCHAR Channel,
	IN BOOLEAN bPowerOn,
	IN BOOLEAN bDoTSSI,
	IN BOOLEAN bFullCal);

VOID NICUpdateRawCounters(
	struct rt2x00_dev *rt2x00dev);

void RtmpAllocDescBuf(
	IN struct rt2x00_dev *rt2x00dev,
	IN UINT Index,
	IN ULONG Length,
	IN BOOLEAN Cached,
	OUT VOID **VirtualAddress,
	OUT PNDIS_PHYSICAL_ADDRESS	phy_addr);

VOID dumpTxWI(struct rt2x00_dev *rt2x00dev, TXWI_STRUC *pTxWI);
void rt2x00_hex_dump(char *str, unsigned char *pSrcBufVA, u32 SrcBufLen);

VOID SendAndesWLANStatus(
	IN struct rt2x00_dev *rt2x00dev,
	IN UCHAR			WlanStatus,
	IN ULONG			PrivilegeTime, 
	IN UCHAR                     BssHashID
	);

VOID SendAndesCCUForceMode(
	struct rt2x00_dev *rt2x00dev,
	IN UCHAR			CoexMode
	);

void Set_BtDump_Proc(
	IN 	struct rt2x00_dev *rt2x00dev,
	IN int index);

VOID MLMEHook(
	IN 	struct rt2x00_dev *rt2x00dev,
	IN UCHAR		WlanStatus,
	IN UCHAR              BssHashID
	);

void RTMPusecDelay(unsigned long usec);
#endif /* RT2X00_H */
