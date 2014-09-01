#include "rt2x00.h"
#include <linux/dma-mapping.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>

UCHAR MULTICAST_ADDR[MAC_ADDR_LEN] = {0x1,  0x00, 0x00, 0x00, 0x00, 0x00};
UCHAR BROADCAST_ADDR[MAC_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
UCHAR ZERO_MAC_ADDR[MAC_ADDR_LEN]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

INT LED_Array[16][12]={
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{ 0,   2,     1,    0,   -1,  -1,  0, -1,   5, -1, -1, 17},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{  3,  2,   -1,   -1,   -1, -1, 16,   1,  5,  -1, -1, 17},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{-1, -1,   -1,   -1,   -1, -1, -1, -1, -1, -1, -1, -1},
	{ 1,   2,     1,   -1,   -1, -1,  3, -1,   6, -1, -1,   0},
	{ 1,   2,     1,   -1,   -1, -1, -1,  1,   4, -1, -1, 18}
};

static inline UCHAR GetCmdSeq(struct rt2x00_dev *rt2x00dev)
{
	struct MCU_CTRL *MCtrl = &rt2x00dev->MCUCtrl;

	MCtrl->CmdSeq == 0xf ? MCtrl->CmdSeq = 1 : MCtrl->CmdSeq++;
	return MCtrl->CmdSeq;
}

static inline void DlListInit(struct _DL_LIST *List)
{
	List->Next = List;
	List->Prev = List;
}

static inline void DlListAdd(struct _DL_LIST *List, struct _DL_LIST *Item)
{
	Item->Next = List->Next;
	Item->Prev = List;
	List->Next->Prev = Item;
	List->Next = Item; 	
}

static inline void DlListAddTail(struct _DL_LIST *List, struct _DL_LIST *Item)
{
	DlListAdd(List->Prev, Item);
}

static inline void DlListDel(struct _DL_LIST *Item)
{
	Item->Next->Prev = Item->Prev;
	Item->Prev->Next = Item->Next;
	Item->Next = NULL;
	Item->Prev = NULL;
}

static inline int DlListEmpty(struct _DL_LIST *List)
{
	return List->Next == List;
}

static inline unsigned int DlListLen(struct _DL_LIST *List)
{
	struct _DL_LIST *Item;
	unsigned int Count = 0;
	
	for (Item = List->Next; Item != List; Item = Item->Next)
		Count++;
	
	return Count;
}

VOID MCUCtrlInit(struct rt2x00_dev *rt2x00dev)
{
	struct MCU_CTRL *MCtrl = &rt2x00dev->MCUCtrl;

	rt2x00dev->PollIdle = FALSE;
	NdisZeroMemory(MCtrl, sizeof(*MCtrl));
	MCtrl->CmdSeq = 0;
	MCtrl->IsFWReady = TRUE;
	NdisAllocateSpinLock(rt2x00dev, &MCtrl->CmdRspEventListLock);
	DlListInit(&MCtrl->CmdRspEventList);
}

NDIS_STATUS os_alloc_mem(
	IN VOID *pReserved,
	OUT UCHAR **mem,
	IN ULONG size)
{
	*mem = (PUCHAR) kmalloc(size, GFP_ATOMIC);
	if (*mem) {
		return NDIS_STATUS_SUCCESS;
	} else
		return NDIS_STATUS_FAILURE;
}


NDIS_STATUS os_free_mem(
	IN VOID *pReserved,
	IN PVOID mem)
{
	kfree(mem);
	return NDIS_STATUS_SUCCESS;
}

NDIS_STATUS RTMPAllocateNdisPacket(
	IN VOID *pReserved,
	OUT PNDIS_PACKET *ppPacket,
	IN UCHAR *pHeader,
	IN UINT HeaderLen,
	IN UCHAR *pData,
	IN UINT DataLen)
{
	struct sk_buff *pPacket;


	pPacket = dev_alloc_skb(HeaderLen + DataLen + RTMP_PKT_TAIL_PADDING);
	if (pPacket == NULL) {
		*ppPacket = NULL;
		printk(KERN_ERR "RTMPAllocateNdisPacket Fail\n\n");
		return NDIS_STATUS_FAILURE;
	}

	/* Clone the frame content and update the length of packet */
	if (HeaderLen > 0)
		NdisMoveMemory(pPacket->data, pHeader, HeaderLen);
	if (DataLen > 0)
		NdisMoveMemory(pPacket->data + HeaderLen, pData, DataLen);
	
	skb_put(pPacket, HeaderLen + DataLen);

	*ppPacket = (PNDIS_PACKET)pPacket;

	return NDIS_STATUS_SUCCESS;
}


void RTMP_QueryPacketInfo(
	IN PNDIS_PACKET pPacket,
	OUT PACKET_INFO *info,
	OUT UCHAR **pSrcBufVA,
	OUT UINT *pSrcBufLen)
{
	info->BufferCount = 1;
	info->pFirstBuffer = (PNDIS_BUFFER) GET_OS_PKT_DATAPTR(pPacket);
	info->PhysicalBufferCount = 1;
	info->TotalPacketLength = GET_OS_PKT_LEN(pPacket);

	*pSrcBufVA = GET_OS_PKT_DATAPTR(pPacket);
	*pSrcBufLen = GET_OS_PKT_LEN(pPacket);
}

INT PCIKickOutCmd(
	struct rt2x00_dev *rt2x00dev, 
	UCHAR *Buf, 
	UINT32 Len)
{
	NDIS_STATUS Status = NDIS_STATUS_SUCCESS;
	ULONG	IrqFlags = 0;
	BOOLEAN bIntContext = FALSE;
	ULONG FreeNum;
	UINT32 SwIdx = 0, SrcBufPA;
	UCHAR *pSrcBufVA;
	UINT SrcBufLen = 0;
	PACKET_INFO PacketInfo;
	TXD_STRUC *pTxD;
	TXINFO_STRUC *pTxInfo;
	PNDIS_PACKET pPacket;
	
	FreeNum = GET_CTRLRING_FREENO(rt2x00dev);	

	if (FreeNum == 0)
	{
		printk("%s FreeNum == 0 (TxCpuIdx = %d, TxDmaIdx = %d, TxSwFreeIdx = %d)\n", 
			__FUNCTION__, rt2x00dev->CtrlRing.TxCpuIdx, rt2x00dev->CtrlRing.TxDmaIdx, rt2x00dev->CtrlRing.TxSwFreeIdx);
		return NDIS_STATUS_FAILURE;
	}

	hex_dump("Buf", Buf, Len);
	RTMP_IRQ_LOCK(&rt2x00dev->CtrlRingLock, IrqFlags);

	Status = RTMPAllocateNdisPacket(rt2x00dev, &pPacket, NULL, 0, Buf+TXINFO_SIZE, Len-TXINFO_SIZE);

	hex_dump("pPacket->data", GET_OS_PKT_DATAPTR(pPacket), Len-TXINFO_SIZE);
	if (Status != NDIS_STATUS_SUCCESS)
	{
		printk("PCIKickOutCmd (error:: can't allocate NDIS PACKET)\n");
		return NDIS_STATUS_FAILURE;
	}

	RTMP_QueryPacketInfo(pPacket, &PacketInfo, &pSrcBufVA, &SrcBufLen);
	if (pSrcBufVA == NULL)
		return NDIS_STATUS_FAILURE;
	SwIdx = rt2x00dev->CtrlRing.TxCpuIdx;
	

	pTxD  = (PTXD_STRUC) rt2x00dev->CtrlRing.Cell[SwIdx].AllocVa;

	pTxInfo = (TXINFO_STRUC *)((UCHAR *)pTxD + sizeof(TXD_STRUC));
	NdisMoveMemory(pTxInfo, Buf, TXINFO_SIZE);

	rt2x00dev->CtrlRing.Cell[SwIdx].pNdisPacket = pPacket;
	rt2x00dev->CtrlRing.Cell[SwIdx].pNextNdisPacket = NULL;

	SrcBufPA = PCI_MAP_SINGLE(rt2x00dev, (pSrcBufVA), (SrcBufLen), 0, RTMP_PCI_DMA_TODEVICE);
	//SrcBufPA =dma_map_single(rt2x00dev->dev,  (pSrcBufVA), (SrcBufLen), DMA_TO_DEVICE);
	pTxD->LastSec0 = 1;
	pTxD->LastSec1 = 0;
	pTxD->SDLen0 = SrcBufLen;
	pTxD->SDLen1 = 0;
	pTxD->SDPtr0 = SrcBufPA;
	pTxD->DMADONE = 0;


	/* flush dcache if no consistent memory is supported */
	//RTMP_DCACHE_FLUSH(SrcBufPA, Len);
	//RTMP_DCACHE_FLUSH(pAd->CtrlRing.Cell[SwIdx].AllocPa, TXD_SIZE);

   	/* Increase TX_CTX_IDX, but write to register later.*/
	INC_RING_INDEX(rt2x00dev->CtrlRing.TxCpuIdx, MGMT_RING_SIZE);

	RTMP_IO_WRITE32(rt2x00dev, TX_CTRL_CIDX,  rt2x00dev->CtrlRing.TxCpuIdx);
	RTMP_IRQ_UNLOCK(&rt2x00dev->CtrlRingLock, IrqFlags);
	printk("PCIKickOutCmd (TxCpuIdx = %d)\n",rt2x00dev->CtrlRing.TxCpuIdx);
	return Status;
}


INT AsicSendCmdToAndes(struct rt2x00_dev *rt2x00dev, struct CMD_UNIT *CmdUnit)
{
	UINT32 VarLen;
	UCHAR *Pos, *Buf;
	TXINFO_NMAC_CMD *TxInfoCmd;
	INT32 Ret = NDIS_STATUS_SUCCESS;
	struct MCU_CTRL *MCtrl = &rt2x00dev->MCUCtrl;
	struct CMD_RSP_EVENT *CmdRspEvent;
	ULONG Expire;
	unsigned long IrqFlags;
	//return Ret;
	if (!MCtrl->IsFWReady)
	{
		printk("22222\n");
		return NDIS_STATUS_FAILURE;
	}


	VarLen = sizeof(*TxInfoCmd) + CmdUnit->u.ANDES.CmdPayloadLen;

	os_alloc_mem(rt2x00dev, (UCHAR **)&Buf, VarLen);
	
	NdisZeroMemory(Buf, VarLen);

	Pos = Buf;
	TxInfoCmd = (TXINFO_NMAC_CMD *)Pos;
	TxInfoCmd->info_type = CMD_PACKET;
	TxInfoCmd->d_port = CPU_TX_PORT;
	TxInfoCmd->cmd_type = CmdUnit->u.ANDES.Type;

	if (CmdUnit->u.ANDES.NeedRsp)
	{
		TxInfoCmd->cmd_seq = GetCmdSeq(rt2x00dev);

		//printk("cmd seq = %d\n", TxInfoCmd->cmd_seq);

		os_alloc_mem(NULL, (UCHAR **)&CmdRspEvent, sizeof(*CmdRspEvent));

		if (!CmdRspEvent)
		{
			printk("%s Not available memory\n", __FUNCTION__);
			Ret = NDIS_STATUS_RESOURCES;
			goto error;
		}

		NdisZeroMemory(CmdRspEvent, sizeof(*CmdRspEvent));

		CmdRspEvent->CmdSeq = TxInfoCmd->cmd_seq;
		CmdRspEvent->Timeout = CmdUnit->u.ANDES.Timeout;
		CmdRspEvent->RspPayload = &CmdUnit->u.ANDES.RspPayload;
		CmdRspEvent->RspPayloadLen = &CmdUnit->u.ANDES.RspPayloadLen;

		if (CmdUnit->u.ANDES.NeedWait)
		{
			CmdRspEvent->NeedWait = TRUE;
			init_completion(&CmdRspEvent->AckDone);
		}

		RTMP_IRQ_LOCK(&MCtrl->CmdRspEventListLock, IrqFlags);
		DlListAddTail(&MCtrl->CmdRspEventList, &CmdRspEvent->List);
		RTMP_IRQ_UNLOCK(&MCtrl->CmdRspEventListLock, IrqFlags);
	}
	else
	{	
		TxInfoCmd->cmd_seq = 0;
		printk("AsicSendCmdToAndes not need  Rsp!!!\n");
	}

	TxInfoCmd->pkt_len = CmdUnit->u.ANDES.CmdPayloadLen;


	Pos += sizeof(*TxInfoCmd);
	
	NdisMoveMemory(Pos, CmdUnit->u.ANDES.CmdPayload, CmdUnit->u.ANDES.CmdPayloadLen);
	hex_dump("CmdUnit->u.ANDES.CmdPayload", CmdUnit->u.ANDES.CmdPayload, CmdUnit->u.ANDES.CmdPayloadLen);
	hex_dump("AsicSendCmdToAndes", Buf, VarLen);

	PCIKickOutCmd(rt2x00dev, Buf, VarLen);

	/* Wait for Command Rsp */
	if (CmdUnit->u.ANDES.NeedWait) {
		ULONG Timeout = CmdUnit->u.ANDES.Timeout;
		Expire = Timeout ? msecs_to_jiffies(Timeout) : msecs_to_jiffies(300);
		RTMP_IRQ_LOCK(&MCtrl->CmdRspEventListLock, IrqFlags);
		DlListDel(&CmdRspEvent->List);
		os_free_mem(NULL, CmdRspEvent);
		RTMP_IRQ_UNLOCK(&MCtrl->CmdRspEventListLock, IrqFlags);
	}
error:
	os_free_mem(NULL, Buf);

	return Ret;
}


void RtmpAllocDescBuf(
	IN struct rt2x00_dev *rt2x00dev,
	IN UINT Index,
	IN ULONG Length,
	IN BOOLEAN Cached,
	OUT VOID **VirtualAddress,
	OUT PNDIS_PHYSICAL_ADDRESS	phy_addr)
{
	dma_addr_t DmaAddr = (dma_addr_t)(*phy_addr);

	*VirtualAddress = (PVOID)dma_alloc_coherent(rt2x00dev->dev,sizeof(char)*Length, &DmaAddr,GFP_KERNEL);
	*phy_addr = (NDIS_PHYSICAL_ADDRESS)DmaAddr;
}
EXPORT_SYMBOL_GPL(RtmpAllocDescBuf);

static INT desc_ring_alloc(struct rt2x00_dev *rt2x00dev, RTMP_DMABUF *pDescRing, INT size)
{
	pDescRing->AllocSize = size;
	RtmpAllocDescBuf(rt2x00dev,
				0,
				pDescRing->AllocSize,
				FALSE,
				&pDescRing->AllocVa,
				&pDescRing->AllocPa);

	if (pDescRing->AllocVa == NULL)
	{
		printk("Failed to allocate a big buffer\n");
		return 0x00000402L;
	}

	/* Zero init this memory block*/
	NdisZeroMemory(pDescRing->AllocVa, size);

	return 0;
}



NDIS_STATUS	RTMPAllocTxRxRingMemory(struct rt2x00_dev *rt2x00dev)
{
	NDIS_STATUS Status = NDIS_STATUS_SUCCESS;
	INT num;
	ULONG ErrorValue = 0;
	
	printk("-->RTMPAllocTxRxRingMemory\n");
	do
	{
		/* Alloc CTRL ring desc buffer except Tx ring allocated eariler */
		desc_ring_alloc(rt2x00dev, &rt2x00dev->CtrlDescRing,
							MGMT_RING_SIZE * TXD_SIZE);
		if (rt2x00dev->CtrlDescRing.AllocVa == NULL) {
			Status = NDIS_STATUS_RESOURCES;
			break;
		}
		printk("CTRL Ring: total %d bytes allocated\n",
					(INT)rt2x00dev->CtrlDescRing.AllocSize);
	}while (FALSE);


	printk("<-- RTMPAllocTxRxRingMemory, Status=%x\n", Status);
	return Status;
}
EXPORT_SYMBOL_GPL(RTMPAllocTxRxRingMemory);

NDIS_STATUS RTMPInitTxRxRingMemory(struct rt2x00_dev *rt2x00dev)
{
	INT num, index;
	ULONG RingBasePaHigh, RingBasePaLow;
	VOID *RingBaseVa;
	RTMP_DMABUF *pDmaBuf, *pDescRing;
	PNDIS_PACKET pPacket;
	TXD_STRUC *pTxD;
	ULONG ErrorValue = 0;
	NDIS_STATUS Status = NDIS_STATUS_SUCCESS;




	/* Initialize All Tx Ring Descriptors and associated buffer memory*/
	/* (5 TX rings = 4 ACs + 1 HCCA)*/

	/* Initialize CTRL Ring and associated buffer memory */
	pDescRing = &rt2x00dev->CtrlDescRing;
	RingBasePaHigh = RTMP_GetPhysicalAddressHigh(pDescRing->AllocPa);
	RingBasePaLow = RTMP_GetPhysicalAddressLow (pDescRing->AllocPa);
	RingBaseVa = pDescRing->AllocVa;
	NdisZeroMemory(pDescRing->AllocVa, pDescRing->AllocSize);
	for (index = 0; index < MGMT_RING_SIZE; index++)
	{
		rt2x00dev->CtrlRing.Cell[index].pNdisPacket = NULL;
		rt2x00dev->CtrlRing.Cell[index].pNextNdisPacket = NULL;
		/* Init Ctrl Ring Size, Va, Pa variables */
		rt2x00dev->CtrlRing.Cell[index].AllocSize = TXD_SIZE;
		rt2x00dev->CtrlRing.Cell[index].AllocVa = RingBaseVa;
		RTMP_SetPhysicalAddressHigh(rt2x00dev->CtrlRing.Cell[index].AllocPa, RingBasePaHigh);
		RTMP_SetPhysicalAddressLow (rt2x00dev->CtrlRing.Cell[index].AllocPa, RingBasePaLow);

		/* Offset to next ring descriptor address */
		RingBasePaLow += TXD_SIZE;
		RingBaseVa = (PUCHAR) RingBaseVa + TXD_SIZE;

		/* link the pre-allocated TxBuf to TXD */
		pTxD = (PTXD_STRUC) rt2x00dev->CtrlRing.Cell[index].AllocVa;
		pTxD->DMADONE = 1;
		/* no pre-allocated buffer required in CtrlRing for scatter-gather case */
	}


	/* init CTRL ring index pointer */
	rt2x00dev->CtrlRing.TxSwFreeIdx = 0;
	rt2x00dev->CtrlRing.TxCpuIdx = 0;

		
	return Status;

}

VOID AsicInitTxRxRing(struct rt2x00_dev *rt2x00dev)
{
	UINT32 addr;
	INT i, offset;
	
	/*
		Write Tx Ring base address registers 
		
		1. RT85592
		The Tx Ring arrangement:
		RingIdx	SwRingIdx	AsicPriority	WMM QID
		0 		TxSw0		L			QID_AC_BE
		1		TxSw1		L			QID_AC_BK
		2		TxSw2		L			QID_AC_VI
		3		TxSw3		L			QID_AC_VO

		4		CTRL		M			-
		5		MGMT		H			-

		6		-			L			QID_AC_BE
		7		-			L			QID_AC_BK
		8		-			L			QID_AC_VI
		9		-			L			QID_AC_VO

		Ring 0~3 for TxChannel 0
			Ring 6~9 for TxChannel 1		

		2. MT7650
			TxRing 0~3: for TxQ Channel 1 with AC_BK/BE/VI/VO
			TxRing 4~7: for TxQ Channel 2 with AC_BK/BE/VI/VO
			TxRing 8    : for TxQ CTRL (In-band command)
			TxRing 9    : for TxQ MGMT
	*/

	


	/* init CTRL ring index pointer */
	printk("AsicInitTxRxRing\n");
	addr = RTMP_GetPhysicalAddressLow(rt2x00dev->CtrlRing.Cell[0].AllocPa);
	RTMP_IO_WRITE32(rt2x00dev, TX_CTRL_BASE, addr);
	RTMP_IO_WRITE32(rt2x00dev, TX_CTRL_CNT, MGMT_RING_SIZE);
	rt2x00dev->CtrlRing.TxSwFreeIdx = 0;
	rt2x00dev->CtrlRing.TxCpuIdx = 0;
	RTMP_IO_WRITE32(rt2x00dev, TX_CTRL_CIDX,  rt2x00dev->CtrlRing.TxCpuIdx);
	printk("-->TX_RING_CTRL: Base=0x%x, Cnt=%d!\n",
					addr, MGMT_RING_SIZE);

}
EXPORT_SYMBOL_GPL(AsicInitTxRxRing);

int	RTMPHandleTxRing8DmaDoneInterrupt(
	IN struct rt2x00_dev *rt2x00dev)
{
	PTXD_STRUC	 pTxD;
	PNDIS_PACKET pPacket;
/*	int 		 i;*/
	UCHAR	FREE = 0;
	int ret = 0;
	RTMP_CTRL_RING *pCtrlRing = &rt2x00dev->CtrlRing;
	UINT8 TXWISize = rt2x00dev->TXWISize;
	
	RTMP_IO_READ32(rt2x00dev, TX_CTRL_DIDX, &pCtrlRing->TxDmaIdx);
	while (pCtrlRing->TxSwFreeIdx!= pCtrlRing->TxDmaIdx)
	{
		FREE++;
		pTxD = (PTXD_STRUC) (pCtrlRing->Cell[pCtrlRing->TxSwFreeIdx].AllocVa);
		//pTxD->DMADONE = 0;
		pPacket = pCtrlRing->Cell[pCtrlRing->TxSwFreeIdx].pNdisPacket;

		if (pPacket == NULL)
		{
			INC_RING_INDEX(pCtrlRing->TxSwFreeIdx, MGMT_RING_SIZE);
			continue;
		}

		if (pPacket)
		{
#if 0
		dma_unmap_single(dev, skbdesc->skb_dma, entry->skb->len,
				 DMA_TO_DEVICE);
#endif
			dma_unmap_single(rt2x00dev->dev, pTxD->SDPtr0, pTxD->SDLen0, DMA_TO_DEVICE);
			RELEASE_NDIS_PACKET(rt2x00dev, pPacket, NDIS_STATUS_SUCCESS);
		}
		pCtrlRing->Cell[pCtrlRing->TxSwFreeIdx].pNdisPacket = NULL;

		pPacket = pCtrlRing->Cell[pCtrlRing->TxSwFreeIdx].pNextNdisPacket;
		if (pPacket)
		{
			dma_unmap_single(rt2x00dev->dev, pTxD->SDPtr1, pTxD->SDLen1, DMA_TO_DEVICE);
			RELEASE_NDIS_PACKET(rt2x00dev, pPacket, NDIS_STATUS_SUCCESS);
		}
		pCtrlRing->Cell[pCtrlRing->TxSwFreeIdx].pNextNdisPacket = NULL;

		/* flush dcache if no consistent memory is supported */
		//RTMP_DCACHE_FLUSH(pCtrlRing->Cell[pCtrlRing->TxSwFreeIdx].AllocPa, TXD_SIZE);

		INC_RING_INDEX(pCtrlRing->TxSwFreeIdx, MGMT_RING_SIZE);
		RTMP_IO_READ32(rt2x00dev, TX_CTRL_DIDX, &pCtrlRing->TxDmaIdx);

	}
}
EXPORT_SYMBOL_GPL(RTMPHandleTxRing8DmaDoneInterrupt);

VOID RTMPFreeNdisPacket(
	IN VOID *pReserved,
	IN PNDIS_PACKET pPacket)
{
	dev_kfree_skb_any(RTPKT_TO_OSPKT(pPacket));
}



VOID SendAndesTFSWITCH(
	IN struct rt2x00_dev *rt2x00dev,
	IN UCHAR			CoexMode
	)
{
    
	COEX_TF_SWITCH coexTF = {0};
	USHORT coexTFLength = 0;
	INT ret;
	struct CMD_UNIT CmdUnit;
	
	printk("%s: -->\n", __FUNCTION__);
	coexTF.CoexOperation = TypeTFSwitch;
       coexTF.CoexMode = CoexMode;
       
	coexTFLength = sizeof(coexTF);

	printk("%s: CoexOperation = %d, CoexMode = %d\n, PktLength = %d\n", 
		__FUNCTION__, 
		coexTF.CoexOperation, 
		coexTF.CoexMode,
		coexTFLength
		);

		NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
		CmdUnit.u.ANDES.Type = PKT_CMD_TYPE_COEX_OP;
		CmdUnit.u.ANDES.CmdPayloadLen = coexTFLength;
		CmdUnit.u.ANDES.CmdPayload = &coexTF;

		CmdUnit.u.ANDES.NeedRsp = FALSE;
		CmdUnit.u.ANDES.NeedWait = FALSE;
		CmdUnit.u.ANDES.Timeout = 0;

		ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);
#if 0		
	TxPktCmd(pAd, PKT_CMD_TYPE_COEX_OP, NO_PKT_CMD_RSP_EVENT, 
		&coexTF, coexTFLength, 0);
#endif
	TDDFDDExclusiveRequest(rt2x00dev, CoexMode);
	printk("%s: <--\n", __FUNCTION__);
       
	
}
EXPORT_SYMBOL_GPL(SendAndesTFSWITCH);

VOID RTMPWriteTxWI(
	IN struct rt2x00_dev *rt2x00dev,
	IN TXWI_STRUC *pOutTxWI,
	IN BOOLEAN FRAG,
	IN BOOLEAN CFACK,
	IN BOOLEAN InsTimestamp,
	IN BOOLEAN AMPDU,
	IN BOOLEAN Ack,
	IN BOOLEAN NSeq,		/* HW new a sequence.*/
	IN UCHAR BASize,
	IN UCHAR WCID,
	IN ULONG Length,
	IN UCHAR PID,
	IN UCHAR TID,
	IN UCHAR TxRate,
	IN UCHAR Txopmode,
	IN BOOLEAN CfAck)
{
	//PMAC_TABLE_ENTRY pMac = NULL;
	TXWI_STRUC TxWI, *pTxWI;
	UINT8 TXWISize = rt2x00dev->TXWISize;
	//UINT32 MaxWcidNum = MAX_LEN_OF_MAC_TABLE;

	/* 
		Always use Long preamble before verifiation short preamble functionality works well.
		Todo: remove the following line if short preamble functionality works
	*/
	NdisZeroMemory(&TxWI, TXWISize);
	pTxWI = &TxWI;
	pTxWI->TxWIFRAG= FRAG;
	pTxWI->TxWICFACK = CFACK;
	pTxWI->TxWITS= InsTimestamp;
	pTxWI->TxWIAMPDU = AMPDU;
	pTxWI->TxWIACK = Ack;
	pTxWI->TxWITXOP= Txopmode;
	
	pTxWI->TxWINSEQ = NSeq;
	/* John tune the performace with Intel Client in 20 MHz performance*/
	BASize = 0;


	pTxWI->TxWIBAWinSize = BASize;
	pTxWI->TxWIShortGI = 0;
	pTxWI->TxWISTBC = 0;

		
	pTxWI->TxWIWirelessCliID = WCID;
	pTxWI->TxWIMPDUByteCnt = Length;
	pTxWI->TxWIPacketId = PID;
	
	/* If CCK or OFDM, BW must be 20*/
	pTxWI->TxWIBW =  (BW_20);
	
	pTxWI->TxWIMCS = 0;
	pTxWI->TxWIPHYMODE = MODE_HTMIX;
	pTxWI->TxWICFACK = CfAck;
	pTxWI->TxWIPacketId = pTxWI->TxWIMCS;
	NdisMoveMemory(pOutTxWI, &TxWI, TXWISize);
}


VOID PrepareProtectionFrame(
	IN	struct rt2x00_dev *rt2x00dev,
	IN    ULONG           Type,
	IN    ULONG           Number,
	IN	ULONG		NAV,
	IN    ULONG           OPMode,
	IN    INT wcid
)
{
       HEADER_802_11	    ProtectionFrame ={0};
	TXWI_STRUC		    TxWI;
	UCHAR				*ptr = NULL;
	UINT				i = 0;
	//PHY_CFG PhyCfg = {0};
       UCHAR			Wcid = 0;
       UCHAR                  Length = 0;
       ULONG                  FrameAddress = 0;
       BOOLEAN 		Ack = FALSE;

	TXINFO_STRUC *pTxInfo;
	TXWI_STRUC *pTxWI;


	printk("==>PrepareProtectionFrame\n");	

	NdisZeroMemory(&TxWI,rt2x00dev->TXWISize);
	NdisZeroMemory(&ProtectionFrame, sizeof(HEADER_802_11));
	pTxWI = &TxWI;
	//ProtectionFrame.FC.Type = BTYPE_DATA;
        switch (Type){
            
            case CTSTOSELF:
	        ProtectionFrame.FC.Type = BTYPE_CNTL;
                ProtectionFrame.FC.SubType = SUBTYPE_CTS;
                break;
                
            case CFEND:
	        ProtectionFrame.FC.Type = BTYPE_CNTL;
                ProtectionFrame.FC.SubType = SUBTYPE_CFEND;
                break;
                
            case POWERSAVE1:
	        ProtectionFrame.FC.Type = BTYPE_DATA;
                ProtectionFrame.FC.SubType = SUBTYPE_NULL_FUNC; 
                ProtectionFrame.FC.PwrMgmt = PWR_SAVE;
                ProtectionFrame.FC.ToDs = 1;
                break;
                
            case POWERSAVE0:
	        ProtectionFrame.FC.Type = BTYPE_DATA;
                ProtectionFrame.FC.SubType = SUBTYPE_NULL_FUNC;      
                ProtectionFrame.FC.PwrMgmt = PWR_ACTIVE;
                ProtectionFrame.FC.ToDs = 1;
                break;                         
        }

       if (Type == CTSTOSELF)  
       {
            COPY_MAC_ADDR(ProtectionFrame.Addr1, rt2x00dev->addr);
            ProtectionFrame.Duration = (USHORT)NAV;
            Wcid = 0xff;
            Length = 10;
       }
       else if (Type == CFEND)
       {
             COPY_MAC_ADDR(ProtectionFrame.Addr1, BROADCAST_ADDR);
             COPY_MAC_ADDR(ProtectionFrame.Addr2, rt2x00dev->addr);
             ProtectionFrame.Duration = 0;
             Wcid = 0xff;
             Length = 16;
       }
       else
       {
 #if 0     
        	pAd->Sequence       = ((pAd->Sequence) + 1) & (MAX_SEQ_NUMBER);
	       ProtectionFrame.Sequence = pAd->Sequence;
              ProtectionFrame.Duration = RTMPCalcDuration(pAd, RATE_1, 14);				
#endif

        	COPY_MAC_ADDR(ProtectionFrame.Addr1, rt2x00dev->bssid);
        	COPY_MAC_ADDR(ProtectionFrame.Addr2, rt2x00dev->addr);
        	COPY_MAC_ADDR(ProtectionFrame.Addr3, rt2x00dev->bssid);

              Ack =TRUE;
              Wcid = wcid;
              Length =  sizeof(HEADER_802_11) ;
       }
#if 0	
	WRITE_PHY_CFG_DOUBLE_WORD(pAd, &PhyCfg, 0);
	WRITE_PHY_CFG_MODE(pAd, &PhyCfg, MODE_CCK);
	WRITE_PHY_CFG_MCS(pAd, &PhyCfg, RATE_1);
		
	RTMPWriteTxWI(pAd, pPort, &ProtectionFrameTxWI, FALSE, FALSE, FALSE, Ack, FALSE, 0, Wcid, Length,
		0, 0, IFS_HTTXOP, FALSE, PhyCfg, FALSE, TRUE, FALSE);
#endif
	RTMPWriteTxWI(rt2x00dev, pTxWI, FALSE, FALSE, FALSE, FALSE, TRUE, FALSE, 0,
		      Wcid, Length, 0, 0,
		      (UCHAR)0,
		      IFS_BACKOFF, FALSE);


      if (Number == 0)
      {
            FrameAddress = HW_NULL_BASE + 0x4000;
      }
      else if (Number == 1)
      {
            FrameAddress = HW_NULL2_BASE + 0x4000;
      }
      else
      {
		//trigger number 0/1 is belong to null number address 
		//Beacon address from D000
            FrameAddress = 0xD000 + (0x200*(Number-2));
      }
      printk("Protection FrameAddress =%x \n",FrameAddress);
	//
	// Move TXWI and frame content to on-chip memory
	//
	ptr = (PUCHAR)pTxWI;
	for (i=0; i<rt2x00dev->TXWISize; i++)  // 24-byte TXINFO field
	{
		RTMP_IO_WRITE8(rt2x00dev, FrameAddress + i, *ptr);
		ptr ++;
	}

	ptr = (PUCHAR)&ProtectionFrame;
	for (i = 0; i < Length; i++)
	{
		RTMP_IO_WRITE8(rt2x00dev, FrameAddress + rt2x00dev->TXWISize + i, *ptr);
		ptr ++;
	}
}
EXPORT_SYMBOL_GPL(PrepareProtectionFrame);

UCHAR CheckAvailableNullFrameSpace(
	struct rt2x00_dev *rt2x00dev)
{
    char iter = 0;
    for (iter=0; iter < NULLFRAMESPACE; iter++ )
    {
        if ((rt2x00dev->NullFrameSpace[iter].Occupied) == 0 ) 
        {
            return iter;
        }
        
    }

    return NULLFRAMESPACE;

}
EXPORT_SYMBOL_GPL(CheckAvailableNullFrameSpace);

VOID FillProtectionFrameSpace(
    IN	struct rt2x00_dev *rt2x00dev,
    IN	ULONG	Triggernumber,
    IN      ULONG	Valid,
    IN      ULONG	NodeType,
    IN      PUCHAR  pAddr,
    IN      ULONG	FrameType
)
{
    rt2x00dev->NullFrameSpace[Triggernumber].Occupied = TRUE;
    rt2x00dev->NullFrameSpace[Triggernumber].Triggernumber = Triggernumber;
    rt2x00dev->NullFrameSpace[Triggernumber].Valid = Valid;
    rt2x00dev->NullFrameSpace[Triggernumber].NodeType = NodeType;
    rt2x00dev->NullFrameSpace[Triggernumber].BssHashID = 0;
    rt2x00dev->NullFrameSpace[Triggernumber].FrameType= FrameType;

}
EXPORT_SYMBOL_GPL(FillProtectionFrameSpace);

VOID EstablishFrameBundle(
	IN	 struct rt2x00_dev *rt2x00dev,
	IN      PUCHAR  pAddr,
	IN      ULONG  OPMode,
	IN      INT  WCID
)
{
    
    UCHAR n0, n1, n2, n3 = 0;
    n0 = CheckAvailableNullFrameSpace(rt2x00dev);
	printk("COEX: Protection FrameBaseNumber=%d\n",n0);
	if (n0 != NULLFRAMESPACE)
	{
        PrepareProtectionFrame(rt2x00dev, CTSTOSELF, n0, 2500, OPMode,WCID);
	 FillProtectionFrameSpace(rt2x00dev, n0, PROTECTIONFRAMEREADY  , OPMode, pAddr, CTSTOSELF);
	}
    n1 = CheckAvailableNullFrameSpace(rt2x00dev);
	printk("COEX: Protection FrameBaseNumber=%d\n",n1);
	if (n1 != NULLFRAMESPACE)
	{
        PrepareProtectionFrame(rt2x00dev, POWERSAVE1, n1, 0, OPMode,WCID);
        FillProtectionFrameSpace(rt2x00dev, n1, PROTECTIONFRAMEREADY  , OPMode, pAddr, POWERSAVE1);
	}
    n2 = CheckAvailableNullFrameSpace(rt2x00dev);
	printk("COEX: Protection FrameBaseNumber=%d\n",n2);
	if (n2 != NULLFRAMESPACE)
	{
        PrepareProtectionFrame(rt2x00dev, CFEND, n2, 0, OPMode,WCID);
        FillProtectionFrameSpace(rt2x00dev, n2, PROTECTIONFRAMEREADY  , OPMode, pAddr, CFEND);
	}
    n3 = CheckAvailableNullFrameSpace(rt2x00dev);
	printk("COEX: Protection FrameBaseNumber=%d\n",n3);
	if (n3 != NULLFRAMESPACE)
	{
        PrepareProtectionFrame(rt2x00dev, POWERSAVE0, n3, 0, OPMode,WCID);
        FillProtectionFrameSpace(rt2x00dev, n3, PROTECTIONFRAMEREADY  , OPMode, pAddr, POWERSAVE0);
	}
  
    
}
EXPORT_SYMBOL_GPL(EstablishFrameBundle);

ra_dma_addr_t linux_pci_map_single(void *pPciDev, void *ptr, size_t size, int sd_idx, int direction)
{
	if (direction == RTMP_PCI_DMA_TODEVICE)
		direction = PCI_DMA_TODEVICE;

	if (direction == RTMP_PCI_DMA_FROMDEVICE)
		direction = PCI_DMA_FROMDEVICE;

	/* 
		------ Porting Information ------
		> For Tx Alloc:
			mgmt packets => sd_idx = 0
			SwIdx: pAd->MgmtRing.TxCpuIdx
			pTxD : pAd->MgmtRing.Cell[SwIdx].AllocVa;
	 
			data packets => sd_idx = 1
	 		TxIdx : pAd->TxRing[pTxBlk->QueIdx].TxCpuIdx 
	 		QueIdx: pTxBlk->QueIdx 
	 		pTxD  : pAd->TxRing[pTxBlk->QueIdx].Cell[TxIdx].AllocVa;

	 	> For Rx Alloc:
	 		sd_idx = -1
	*/

/*	pAd = (PRTMP_ADAPTER)handle; */
/*	pObj = (POS_COOKIE)pAd->OS_Cookie; */
	
	{
		return (ra_dma_addr_t)pci_map_single(pPciDev, ptr, size, direction);
	}

}

void linux_pci_unmap_single(void *pPciDev, ra_dma_addr_t radma_addr, size_t size, int direction)
{
	dma_addr_t DmaAddr = (dma_addr_t)radma_addr;


	if (direction == RTMP_PCI_DMA_TODEVICE)
		direction = PCI_DMA_TODEVICE;

	if (direction == RTMP_PCI_DMA_FROMDEVICE)
		direction = PCI_DMA_FROMDEVICE;
	
	if (size > 0)
		pci_unmap_single(pPciDev, DmaAddr, size, direction);
	
}

ra_dma_addr_t RtmpDrvPciMapSingle(
	IN struct rt2x00_dev *rt2x00dev,
	IN VOID *ptr,
	IN size_t size,
	IN INT sd_idx,
	IN INT direction)
{
		struct pci_dev *pci_dev = to_pci_dev(rt2x00dev->dev);
		return linux_pci_map_single(pci_dev,
					ptr, size, 0, direction);
}


ra_dma_addr_t RtmpDrvPciUnMapSingle(
	IN struct rt2x00_dev *rt2x00dev,
	IN VOID *ptr,
	IN size_t size,
	IN INT direction)
{
		struct pci_dev *pci_dev = to_pci_dev(rt2x00dev->dev);
		linux_pci_unmap_single(pci_dev,
					ptr, size, direction);
}

VOID TDDFDDExclusiveRequest(
        IN struct rt2x00_dev *rt2x00dev, 
	UCHAR CoexMode 
	) 
{
    if (CoexMode == COEX_MODE_FDD)
    {
        rt2x00dev->CoexMode.FDDRequest = TRUE;
        rt2x00dev->CoexMode.TDDRequest = FALSE;
    }
    else if  (CoexMode == COEX_MODE_TDD)
    {
        rt2x00dev->CoexMode.FDDRequest = FALSE;
        rt2x00dev->CoexMode.TDDRequest = TRUE;
    }
    else if (CoexMode == COEX_MODE_RESET)
    {
        rt2x00dev->CoexMode.FDDRequest = FALSE;
        rt2x00dev->CoexMode.TDDRequest = FALSE;
    }
    
}
EXPORT_SYMBOL_GPL(TDDFDDExclusiveRequest);


VOID BtAFHCtl(
		IN struct rt2x00_dev *rt2x00dev,
		IN UCHAR			BBPCurrentBW,
		IN UCHAR			Channel,
		IN UCHAR			CentralChannel,
		IN BOOLEAN			Disable)
{
	UCHAR Kstart = 0, Kend = 0;
	BT_FUN_INFO_STRUC btFunInfo={0};
	
	if (!((rt2x00_rt(rt2x00dev, MT7630)) || Channel>14))
		return;

	if (BBPCurrentBW == BW_40)
	{
		if (CentralChannel <= 4 )
		{
			Kstart = 0;
			Kend = 53;
		}
		else if (CentralChannel >=10)
		{
			Kstart = 25;
			Kend = 78;		
		}
		else
		{
			Kstart = 2 + (CentralChannel-5)*5;
			Kend = Kstart + 55;
		}
	}
	else
	{
		if (Channel <= 3 )
		{
			Kstart = 0;
			Kend = 47;
		}
		else if (Channel >=10)
		{
			Kstart = 31;
			Kend = 78;		
		}
		else
		{
			Kstart = 1 + (Channel-4)*5;
			Kend = Kstart + 48;
		}
	}

	RTMP_IO_READ32(rt2x00dev, BT_FUN_INFO, &btFunInfo.word);
	btFunInfo.word &= ~(0x3FFFFFFF); //Clear Power and AFH but keep active bit 
	if (!Disable)
	{
		btFunInfo.field.AFH_START_CH = Kstart;
		btFunInfo.field.AFH_END_CH = Kend;
		/* These Code and Definition are gone
		if (pAd->CommonCfg.BBPCurrentBW == BW_40)
			//0x04 // -14
			btFunInfo.field.BTPower0 = 0x0c;	//-8
		else
			btFunInfo.field.BTPower0 = 0x1c;	//-2
		*/
	}

	printk("%s: COEX AFH Start Ch = %d, AFH End Ch = %d, Channel = %d, CentralChannel = %d\n",
		__FUNCTION__, 
		btFunInfo.field.AFH_START_CH,
		btFunInfo.field.AFH_END_CH,
		Channel,
		CentralChannel);
	
	RTMP_IO_WRITE32(rt2x00dev, BT_FUN_INFO, btFunInfo.word);
	// High BT Priority Mode
	//RTMP_IO_WRITE32(pAd, 0x5c, 0x8000);		
}
EXPORT_SYMBOL_GPL(BtAFHCtl);

VOID SendAndesCoexFrameInfo(
	IN struct rt2x00_dev *rt2x00dev, 
	IN ULONG TriggerNumber) 
{
       INT ret;
	struct CMD_UNIT CmdUnit;
	COEX_PROTECTION_FRAME_INFO coexProtectionFrameInfo = {0};
	USHORT coexProtectionFrameInfoLength = 0;

	printk("%s: -->\n", __FUNCTION__);

	coexProtectionFrameInfo.CoexOperation = TypeProtectionFrame;
	coexProtectionFrameInfo.Triggernumber = rt2x00dev->NullFrameSpace[TriggerNumber].Triggernumber;
	coexProtectionFrameInfo.Valid = rt2x00dev->NullFrameSpace[TriggerNumber].Valid;
	coexProtectionFrameInfo.NodeType = rt2x00dev->NullFrameSpace[TriggerNumber].NodeType;
	coexProtectionFrameInfo.BssHashID = rt2x00dev->NullFrameSpace[TriggerNumber].BssHashID;
	coexProtectionFrameInfo.FrameType = rt2x00dev->NullFrameSpace[TriggerNumber].FrameType;
    
	coexProtectionFrameInfoLength = sizeof(coexProtectionFrameInfo);

	printk("%s: Triggernumber = %d, Valid = %d, NodeType = %d, BssHashID = %d, , FrameType = %d, CmdParametersLength = %d\n", 
		__FUNCTION__, 
		coexProtectionFrameInfo.Triggernumber, 
		coexProtectionFrameInfo.Valid, 
		coexProtectionFrameInfo.NodeType,
              coexProtectionFrameInfo.BssHashID,
              coexProtectionFrameInfo.FrameType,
              coexProtectionFrameInfoLength
              );

#if 0
	TxPktCmd(pAd, PKT_CMD_TYPE_COEX_OP, NO_PKT_CMD_RSP_EVENT, 
		&coexProtectionFrameInfo, coexProtectionFrameInfoLength, 0);
#endif
		NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
		CmdUnit.u.ANDES.Type = PKT_CMD_TYPE_COEX_OP;
		CmdUnit.u.ANDES.CmdPayloadLen = coexProtectionFrameInfoLength;
		CmdUnit.u.ANDES.CmdPayload = &coexProtectionFrameInfo;

		CmdUnit.u.ANDES.NeedRsp = FALSE;
		CmdUnit.u.ANDES.NeedWait = FALSE;
		CmdUnit.u.ANDES.Timeout = 0;

		ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);

	printk("%s: <--\n", __FUNCTION__);
	
}
EXPORT_SYMBOL_GPL(SendAndesCoexFrameInfo);

VOID UpdateAndesNullFrameSpace(
	IN struct rt2x00_dev *rt2x00dev)
{
    char iter = 0;
    for (iter=0; iter < NULLFRAMESPACE; iter++ )
    {
        if (rt2x00dev->NullFrameSpace[iter].Occupied != 0 ) 
        {
     	   	printk("Coex: Send protection frame %d\n",iter);		
           	SendAndesCoexFrameInfo(rt2x00dev, iter);
        }
        
    }

}
EXPORT_SYMBOL_GPL(UpdateAndesNullFrameSpace);

INT AndesFunSetOP(IN struct rt2x00_dev *rt2x00dev, UINT32 FunID, UINT32 Param)
{
	struct CMD_UNIT CmdUnit;
	CHAR *Pos, *Buf;
	UINT32 VarLen;
	UINT32 Value;
	INT32 Ret;

	/* Function ID and Parameter */
	VarLen = 8;

	os_alloc_mem(rt2x00dev, (UCHAR **)&Buf, VarLen);

	Pos = Buf;
	
	/* Function ID */
	Value = FunID;//cpu2le32(FunID);
	NdisMoveMemory(Pos, &Value, 4);
	Pos += 4;

	/* Parameter */
	Value = Param;//cpu2le32(Param);
	NdisMoveMemory(Pos, &Value, 4);
	Pos += 4;
	
	NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
	
	CmdUnit.u.ANDES.Type = CMD_FUN_SET_OP;
	CmdUnit.u.ANDES.CmdPayloadLen = VarLen;
	CmdUnit.u.ANDES.CmdPayload = Buf;

	Ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);

	os_free_mem(NULL, Buf);

	return NDIS_STATUS_SUCCESS;
}
EXPORT_SYMBOL_GPL(AndesFunSetOP);

INT AndesCalibrationOP(IN struct rt2x00_dev *rt2x00dev, UINT32 CalibrationID, UINT32 Param)
{

	struct CMD_UNIT CmdUnit;
	CHAR *Pos, *Buf;
	UINT32 VarLen;
	UINT32 Value;
	INT32 Ret;
	return 0;
	/* Calibration ID and Parameter */
	VarLen = 8;

	os_alloc_mem(rt2x00dev, (UCHAR **)&Buf, VarLen);

	Pos = Buf;
	
	/* Calibration ID */
	Value = (CalibrationID);
	NdisMoveMemory(Pos, &Value, 4);
	Pos += 4;

	/* Parameter */
	Value = (Param);
	NdisMoveMemory(Pos, &Value, 4);
	Pos += 4;
	
	NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
	
	CmdUnit.u.ANDES.Type = CMD_CALIBRATION_OP;
	CmdUnit.u.ANDES.CmdPayloadLen = VarLen;
	CmdUnit.u.ANDES.CmdPayload = Buf;
	
	CmdUnit.u.ANDES.NeedRsp = FALSE;
	CmdUnit.u.ANDES.NeedWait = FALSE;
	CmdUnit.u.ANDES.Timeout = 0;

	Ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);

	os_free_mem(NULL, Buf);

	return NDIS_STATUS_SUCCESS;
}
EXPORT_SYMBOL_GPL(AndesCalibrationOP);

VOID MT76x0_Calibration(
	IN struct rt2x00_dev *rt2x00dev,
	IN UCHAR Channel,
	IN BOOLEAN bPowerOn,
	IN BOOLEAN bDoTSSI,
	IN BOOLEAN bFullCal)
{
	UINT32 MacReg = 0, reg_val = 0, reg_tx_alc = 0;
	UINT32 Value = 0;
	
	printk("%s - Channel = %d, bPowerOn = %d, bFullCal = %d\n", __FUNCTION__, Channel, bPowerOn, bFullCal);

//#ifdef RTMP_MAC_PCI
	RTMP_SEM_LOCK(&rt2x00dev->CalLock);
//#endif /* RTMP_MAC_PCI */


	if (bPowerOn)
	{
		UCHAR RFValue = 0;
		
		/*
			Do Power on calibration.
			The calibration sequence is very important, please do NOT change it.
			1 XTAL Setup (already done in AsicRfInit)
			2 R-calibration
			3 VCO calibration
		*/

		/*
			2 R-calibration 
		*/
		AndesCalibrationOP(rt2x00dev, R_CALIBRATION, 0x0);


		MT76x0_VCO_CalibrationMode3(rt2x00dev);
		RTMPusecDelay(500);

#ifdef MT76x0_TSSI_CAL_COMPENSATION
		/* TSSI Calibration */
		if (bPowerOn && pAd->chipCap.bInternalTxALC)
		{
			RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0x8);
			MT76x0_TSSI_DC_Calibration(pAd);
			RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0xc);
		}
#endif /* MT76x0_TSSI_CAL_COMPENSATION */
	}

	RTMP_IO_READ32(rt2x00dev, TX_ALC_CFG_0, &reg_tx_alc); /* We need to restore 0x13b0 after calibration. */
	RTMP_IO_WRITE32(rt2x00dev, TX_ALC_CFG_0, 0x0);
	RTMPusecDelay(500);
	
	RTMP_IO_READ32(rt2x00dev, 0x2124, &reg_val); /* We need to restore 0x2124 after calibration. */
	MacReg = 0xFFFFFF7E; /* Disable 0x2704, 0x2708 controlled by MAC. */
	RTMP_IO_WRITE32(rt2x00dev, 0x2124, MacReg);

	/*
		RF_MISC (offset: 0x0518)
		[2]1'b1: enable external A band PA, 1'b0: disable external A band PA
		[3]1'b1: enable external G band PA, 1'b0: disable external G band PA

		Disable external PA before calibration.
	*/
	RTMP_IO_WRITE32(rt2x00dev, RF_MISC, 0x1);
	
	/*
		Do calibration.
		The calibration sequence is very important, please do NOT change it.
		1  RX DCOC calibration
		2  LC tank calibration
		3  TX Filter BW --> not ready yet @20121003
		4  RX Filter BW --> not ready yet @20121003
		5  TX RF LOFT 
		6  TX I/Q
		7  TX Group Delay		
		8  RX I/Q
		9  RX Group Delay
		10 TX 2G DPD
		11 TX 2G IM3 --> not ready yet @20121016
		12 TSSI Zero Reference --> not ready yet @20121016
		13 RX DCOC calibration
	*/
	if (bFullCal)
	{
		/*
			1. RXDC Calibration parameter
				0:Back Ground Disable
		*/
		AndesCalibrationOP(rt2x00dev, RXDCOC_CALIBRATION, 0);

		/*
			2. LC-Calibration parameter
				Bit[0:7]
					0: 2G
					1: 5G + External PA
					2: 5G + Internal PA
				Bit[8:15]
					0: Full Calibration
					1: Partial Calibration
					2: G-Band Full Calibration + Save
					3: A-Band (Low) Full Calibration + Save
					4: A-Band (Mid) Full Calibration + Save
					5: A-Band (High) Full Calibration + Save
					6: G-Band Restore Calibration
					7: A-Band (Low) Restore Calibration
					8: A-Band (Mid) Restore Calibration
					9: A-Band (High) Restore Calibration
		*/
		if (Channel > 14)
		{
			// TODO: check PA setting from EEPROM @20121016
			AndesCalibrationOP(rt2x00dev, LC_CALIBRATION, 0x1);
		}
		else
			AndesCalibrationOP(rt2x00dev, LC_CALIBRATION, 0x0);

		/*
			3,4. BW-Calibration
				Bit[0:7] (0:RX, 1:TX)
				Bit[8:15] (0:BW20, 1:BW40, 2:BW80)
				Bit[16:23]
					0: Full Calibration
					1: Partial Calibration
					2: G-Band Full Calibration + Save
					3: A-Band (Low) Full Calibration + Save
					4: A-Band (Mid) Full Calibration + Save
					5: A-Band (High) Full Calibration + Save
					6: G-Band Restore Calibration
					7: A-Band (Low) Restore Calibration
					8: A-Band (Mid) Restore Calibration
					9: A-Band (High) Restore Calibration
		*/

		/*
			5. RF LOFT-Calibration parameter
				Bit[0:7] (0:G-Band, 1: A-Band)
				Bit[8:15] 
					0: Full Calibration
					1: Partial Calibration
					2: G-Band Full Calibration + Save
					3: A-Band (Low) Full Calibration + Save
					4: A-Band (Mid) Full Calibration + Save
					5: A-Band (High) Full Calibration + Save
					6: G-Band Restore Calibration
					7: A-Band (Low) Restore Calibration
					8: A-Band (Mid) Restore Calibration
					9: A-Band (High) Restore Calibration

		*/
		if (Channel > 14)
		{
			AndesCalibrationOP(rt2x00dev, LOFT_CALIBRATION, 0x1);
		}
		else
			AndesCalibrationOP(rt2x00dev, LOFT_CALIBRATION, 0x0);

		/*
			6. TXIQ-Calibration parameter
				Bit[0:7] (0:G-Band, 1: A-Band)
				Bit[8:15] 
					0: Full Calibration
					1: Partial Calibration
					2: G-Band Full Calibration + Save
					3: A-Band (Low) Full Calibration + Save
					4: A-Band (Mid) Full Calibration + Save
					5: A-Band (High) Full Calibration + Save
					6: G-Band Restore Calibration
					7: A-Band (Low) Restore Calibration
					8: A-Band (Mid) Restore Calibration
					9: A-Band (High) Restore Calibration
		*/
		if (Channel > 14)
		{
			AndesCalibrationOP(rt2x00dev, TXIQ_CALIBRATION, 0x1);
		}
		else
		{
			AndesCalibrationOP(rt2x00dev, TXIQ_CALIBRATION, 0x0);
		}
		/*			
			7. TX Group-Delay Calibation parameter
				Bit[0:7] (0:G-Band, 1: A-Band)
				Bit[8:15] 
					0: Full Calibration
					1: Partial Calibration
					2: G-Band Full Calibration + Save
					3: A-Band (Low) Full Calibration + Save
					4: A-Band (Mid) Full Calibration + Save
					5: A-Band (High) Full Calibration + Save
					6: G-Band Restore Calibration
					7: A-Band (Low) Restore Calibration
					8: A-Band (Mid) Restore Calibration
					9: A-Band (High) Restore Calibration
		*/
		if (Channel > 14)
		{
			AndesCalibrationOP(rt2x00dev, TX_GROUP_DELAY_CALIBRATION, 0x1);
		}
		else
		{
			AndesCalibrationOP(rt2x00dev, TX_GROUP_DELAY_CALIBRATION, 0x0);
		}

		/*
			8. RXIQ-Calibration parameter
				Bit[0:7] (0:G-Band, 1: A-Band)
				Bit[8:15] 
					0: Full Calibration
					1: Partial Calibration
					2: G-Band Full Calibration + Save
					3: A-Band (Low) Full Calibration + Save
					4: A-Band (Mid) Full Calibration + Save
					5: A-Band (High) Full Calibration + Save
					6: G-Band Restore Calibration
					7: A-Band (Low) Restore Calibration
					8: A-Band (Mid) Restore Calibration
					9: A-Band (High) Restore Calibration
					
			9. RX Group-Delay Calibation parameter
				Bit[0:7] (0:G-Band, 1: A-Band)
				Bit[8:15] 
					0: Full Calibration
					1: Partial Calibration
					2: G-Band Full Calibration + Save
					3: A-Band (Low) Full Calibration + Save
					4: A-Band (Mid) Full Calibration + Save
					5: A-Band (High) Full Calibration + Save
					6: G-Band Restore Calibration
					7: A-Band (Low) Restore Calibration
					8: A-Band (Mid) Restore Calibration
					9: A-Band (High) Restore Calibration
		*/
		if (Channel > 14)
		{
			AndesCalibrationOP(rt2x00dev, RXIQ_CALIBRATION, 0x1);
			AndesCalibrationOP(rt2x00dev, RX_GROUP_DELAY_CALIBRATION, 0x1);			
		}
		else
		{
			AndesCalibrationOP(rt2x00dev, RXIQ_CALIBRATION, 0x0);
			AndesCalibrationOP(rt2x00dev, RX_GROUP_DELAY_CALIBRATION, 0x0);			
		}

		/* 
			10. TX 2G DPD - Only 2.4G needs to do DPD Calibration. 
		*/
		if (Channel <= 14)
			AndesCalibrationOP(rt2x00dev, DPD_CALIBRATION, 0x0);
	}
	else
	{
	}

	/*
		RF_MISC (offset: 0x0518)
		[2]1'b1: enable external A band PA, 1'b0: disable external A band PA
		[3]1'b1: enable external G band PA, 1'b0: disable external G band PA
	*/
	if (Channel > 14)
	{
		RTMP_IO_READ32(rt2x00dev, RF_MISC, &MacReg);
		MacReg |= (0x4);
		RTMP_IO_WRITE32(rt2x00dev, RF_MISC, MacReg);
	}
	else
	{
		RTMP_IO_READ32(rt2x00dev, RF_MISC, &MacReg);
		MacReg |= (0x8);
		RTMP_IO_WRITE32(rt2x00dev, RF_MISC, MacReg);
	}

	/* Restore 0x2124 & TX_ALC_CFG_0 after calibration completed */
	RTMP_IO_WRITE32(rt2x00dev, 0x2124, reg_val);
	RTMP_IO_WRITE32(rt2x00dev, TX_ALC_CFG_0, reg_tx_alc);
	RTMPusecDelay(100000); // TODO: check response packet from FW

	/*
		14. RXDC Calibration parameter
			1:Back Ground Enable
	*/
	AndesCalibrationOP(rt2x00dev, RXDCOC_CALIBRATION, 1);

	//RTMPusecDelay(100000); // TODO: check response packet from FW

//#ifdef RTMP_MAC_PCI
	RTMP_SEM_UNLOCK(&rt2x00dev->CalLock);
//#endif /* RTMP_MAC_PCI */
}
EXPORT_SYMBOL_GPL(MT76x0_Calibration);

void RTMPusecDelay(unsigned long usec)
{
	unsigned long i;

	for (i = 0; i < (usec / 50); i++)
		udelay(50);

	if (usec % 50)
		udelay(usec % 50);
}
EXPORT_SYMBOL_GPL(RTMPusecDelay);

void MT7630_rfcsr_read(struct rt2x00_dev *rt2x00dev,
			       const u8 word, u8 *value,const u8 bank)
{
	RLT_RF_CSR_CFG rfcsr = { { 0 } };
	unsigned int i=0, k=0;
	int	 ret = 1;



	//ASSERT((word <= 127));

	for (i=0; i<100; i++)
	{
			
		RTMP_IO_READ32(rt2x00dev, MT7630_RF_CSR_CFG, &rfcsr.word);

		if (rfcsr.field.RF_CSR_KICK == 1)
				continue;
		
		rfcsr.word = 0;
		rfcsr.field.RF_CSR_WR = 0;
		rfcsr.field.RF_CSR_KICK = 1;
		rfcsr.field.RF_CSR_REG_ID = word;
		rfcsr.field.RF_CSR_REG_BANK = bank;
		RTMP_IO_WRITE32(rt2x00dev, MT7630_RF_CSR_CFG, rfcsr.word);
		
		for (k=0; k<100; k++)
		{		
			RTMP_IO_READ32(rt2x00dev, MT7630_RF_CSR_CFG, &rfcsr.word);

			if (rfcsr.field.RF_CSR_KICK == 0)
				break;
		}
		
		if ((rfcsr.field.RF_CSR_KICK == 0) &&
			(rfcsr.field.RF_CSR_REG_ID == word) &&
			(rfcsr.field.RF_CSR_REG_BANK == bank))
		{
			*value = (u8)(rfcsr.field.RF_CSR_DATA);
			break;
		}
	}

	if (rfcsr.field.RF_CSR_KICK == 1)
	{																	
		printk("RF read R%d=0x%X fail, i[%d], k[%d]\n", word, rfcsr.word,i,k);
		goto done;
	}
	ret = 0;

done:
	return ret;
}

void MT7630_rfcsr_write(struct rt2x00_dev *rt2x00dev,
			       const u8 word, const u8 value,const u8 bank)
{
	RLT_RF_CSR_CFG rfcsr = { { 0 } };
	unsigned int i = 1;
	int	 ret;



	//ASSERT((word <= 127));

	ret = 0;
	do
	{
		RTMP_IO_READ32(rt2x00dev, MT7630_RF_CSR_CFG, &rfcsr.word);

		if (!rfcsr.field.RF_CSR_KICK)
			break;
		i++;
	}
	while ((i < 100));

	if ((i == 100))
	{
		printk("rt2800_MT7630_rfcsr_write Retry count exhausted or device removed!!!\n");
		goto done;
	}

	rfcsr.field.RF_CSR_WR = 1;
	rfcsr.field.RF_CSR_KICK = 1;
	rfcsr.field.RF_CSR_REG_BANK = bank;
	rfcsr.field.RF_CSR_REG_ID = word;

	rfcsr.field.RF_CSR_DATA = value;
	RTMP_IO_WRITE32(rt2x00dev, MT7630_RF_CSR_CFG, rfcsr.word);
	//printk("rlt_rf_write bank=0x%x ID=0x%x value=0x%x\n",bank, word, value);
	ret = 0;

done:
	return ret;
}

void MT76x0_VCO_CalibrationMode3(
	struct rt2x00_dev *rt2x00dev)
{
	unsigned char RFValue = 0, Mode = 0;

	MT7630_rfcsr_read(rt2x00dev, RF_R04, &RFValue,RF_BANK0);
	Mode = (RFValue & 0xF0);	
	if (Mode == 0x30)
	{
		printk("%s - Calibration Mode: Open loop, closed loop, and amplitude\n", __FUNCTION__);
		/*
			Calibration Mode - Open loop, closed loop, and amplitude:
			B0.R06.[0]: 0
			B0.R06.[3:1] bp_close_code: 100
			B0.R05.[7:0] bp_open_code: 00
			B0.R04.[2:0] cal_bits: 000
			B0.R03.[2:0] startup_time: 011
			B0.R03.[6:4] settle_time: 011

		*/
		MT7630_rfcsr_read(rt2x00dev, RF_R06, &RFValue, RF_BANK0);
		RFValue &= ~(0x0F);
		RFValue |= (0x08);
		MT7630_rfcsr_write(rt2x00dev, RF_R06, RFValue, RF_BANK0);

		MT7630_rfcsr_read(rt2x00dev, RF_R05, &RFValue, RF_BANK0);
		if (RFValue != 0)
		{
			RFValue = 0;
			MT7630_rfcsr_write(rt2x00dev, RF_R05, RFValue, RF_BANK0);
		}

		MT7630_rfcsr_read(rt2x00dev, RF_R04, &RFValue, RF_BANK0);
		RFValue &= ~(0x07);
		MT7630_rfcsr_write(rt2x00dev, RF_R04, RFValue, RF_BANK0);

		MT7630_rfcsr_read(rt2x00dev, RF_R03, &RFValue, RF_BANK0);
		RFValue &= ~(0x77);
		RFValue |= (0x33);
		MT7630_rfcsr_write(rt2x00dev, RF_R03, RFValue, RF_BANK0);

		MT7630_rfcsr_read(rt2x00dev, RF_R04, &RFValue, RF_BANK0);
		RFValue = ((RFValue & ~0x80) | 0x80); 
		MT7630_rfcsr_write(rt2x00dev, RF_R04, RFValue, RF_BANK0);
	}
	
	return;
}
EXPORT_SYMBOL_GPL(MT76x0_VCO_CalibrationMode3);

VOID NicGetTxRawCounters(
	struct rt2x00_dev *rt2x00dev,
	IN TX_STA_CNT0_STRUC *pStaTxCnt0,
	IN TX_STA_CNT1_STRUC *pStaTxCnt1)
{

	RTMP_IO_READ32(rt2x00dev, TX_STA_CNT0, &pStaTxCnt0->word);
	RTMP_IO_READ32(rt2x00dev, TX_STA_CNT1, &pStaTxCnt1->word);

	//pAd->bUpdateBcnCntDone = TRUE;	/* not appear in Rory's code */
	rt2x00dev->RalinkCounters.OneSecBeaconSentCnt += pStaTxCnt0->field.TxBeaconCount;
	rt2x00dev->RalinkCounters.OneSecTxRetryOkCount += pStaTxCnt1->field.TxRetransmit;
	rt2x00dev->RalinkCounters.OneSecTxNoRetryOkCount += pStaTxCnt1->field.TxSuccess;
	rt2x00dev->RalinkCounters.OneSecTxFailCount += pStaTxCnt0->field.TxFailCount;

	rt2x00dev->WlanCounters.TransmittedFragmentCount.u.LowPart += pStaTxCnt1->field.TxSuccess;
	rt2x00dev->WlanCounters.RetryCount.u.LowPart += pStaTxCnt1->field.TxRetransmit;
	rt2x00dev->WlanCounters.FailedCount.u.LowPart += pStaTxCnt0->field.TxFailCount;
}
EXPORT_SYMBOL_GPL(NicGetTxRawCounters);

VOID NICUpdateRawCounters(
	struct rt2x00_dev *rt2x00dev)
{
	UINT32	OldValue;/*, Value2;*/
	/*ULONG	PageSum, OneSecTransmitCount;*/
	/*ULONG	TxErrorRatio, Retry, Fail;*/
	RX_STA_CNT0_STRUC	 RxStaCnt0;
	RX_STA_CNT1_STRUC   RxStaCnt1;
	RX_STA_CNT2_STRUC   RxStaCnt2;
	TX_STA_CNT0_STRUC 	 TxStaCnt0;
	TX_STA_CNT1_STRUC	 StaTx1;
	TX_STA_CNT2_STRUC	 StaTx2;
#ifdef STATS_COUNT_SUPPORT
	TX_NAG_AGG_CNT_STRUC	TxAggCnt;
	TX_AGG_CNT0_STRUC	TxAggCnt0;
	TX_AGG_CNT1_STRUC	TxAggCnt1;
	TX_AGG_CNT2_STRUC	TxAggCnt2;
	TX_AGG_CNT3_STRUC	TxAggCnt3;
	TX_AGG_CNT4_STRUC	TxAggCnt4;
	TX_AGG_CNT5_STRUC	TxAggCnt5;
	TX_AGG_CNT6_STRUC	TxAggCnt6;
	TX_AGG_CNT7_STRUC	TxAggCnt7;
#endif /* STATS_COUNT_SUPPORT */
	COUNTER_RALINK		*pRalinkCounters;


	pRalinkCounters = &rt2x00dev->RalinkCounters;



	RTMP_IO_READ32(rt2x00dev, RX_STA_CNT0, &RxStaCnt0.word);
	RTMP_IO_READ32(rt2x00dev, RX_STA_CNT2, &RxStaCnt2.word);

	rt2x00dev->RalinkCounters.PhyErrCnt += RxStaCnt0.field.PhyErr;
	{
		RTMP_IO_READ32(rt2x00dev, RX_STA_CNT1, &RxStaCnt1.word);
		rt2x00dev->RalinkCounters.PlcpErrCnt += RxStaCnt1.field.PlcpErr;
	    /* Update RX PLCP error counter*/
	    //rt2x00dev->PrivateInfo.PhyRxErrCnt += RxStaCnt1.field.PlcpErr;
		/* Update False CCA counter*/
		rt2x00dev->RalinkCounters.OneSecFalseCCACnt += RxStaCnt1.field.FalseCca;
		rt2x00dev->RalinkCounters.FalseCCACnt += RxStaCnt1.field.FalseCca;
	}

	/* Update FCS counters*/
	OldValue= rt2x00dev->WlanCounters.FCSErrorCount.u.LowPart;
	rt2x00dev->WlanCounters.FCSErrorCount.u.LowPart += (RxStaCnt0.field.CrcErr); /* >> 7);*/
	if (rt2x00dev->WlanCounters.FCSErrorCount.u.LowPart < OldValue)
		rt2x00dev->WlanCounters.FCSErrorCount.u.HighPart++;

	/* Add FCS error count to private counters*/
	pRalinkCounters->OneSecRxFcsErrCnt += RxStaCnt0.field.CrcErr;
	OldValue = pRalinkCounters->RealFcsErrCount.u.LowPart;
	pRalinkCounters->RealFcsErrCount.u.LowPart += RxStaCnt0.field.CrcErr;
	if (pRalinkCounters->RealFcsErrCount.u.LowPart < OldValue)
		pRalinkCounters->RealFcsErrCount.u.HighPart++;

	/* Update Duplicate Rcv check*/
	pRalinkCounters->DuplicateRcv += RxStaCnt2.field.RxDupliCount;
	rt2x00dev->WlanCounters.FrameDuplicateCount.u.LowPart += RxStaCnt2.field.RxDupliCount;
	/* Update RX Overflow counter*/
	rt2x00dev->Counters8023.RxNoBuffer += (RxStaCnt2.field.RxFifoOverflowCount);
	
	/*pAd->RalinkCounters.RxCount = 0;*/

	
	/*if (!OPSTATUS_TEST_FLAG(pAd, fOP_STATUS_TX_RATE_SWITCH_ENABLED) || */
	/*	(OPSTATUS_TEST_FLAG(pAd, fOP_STATUS_TX_RATE_SWITCH_ENABLED) && (pAd->MacTab.Size != 1)))*/
	//if (!pAd->bUpdateBcnCntDone)
	{
		/* Update BEACON sent count*/
		NicGetTxRawCounters(rt2x00dev, &TxStaCnt0, &StaTx1);
		RTMP_IO_READ32(rt2x00dev, TX_STA_CNT2, &StaTx2.word);
	}


	/*if (pAd->bStaFifoTest == TRUE)*/
#ifdef STATS_COUNT_SUPPORT
	{
		RTMP_IO_READ32(pAd, TX_AGG_CNT, &TxAggCnt.word);
	RTMP_IO_READ32(pAd, TX_AGG_CNT0, &TxAggCnt0.word);
	RTMP_IO_READ32(pAd, TX_AGG_CNT1, &TxAggCnt1.word);
	RTMP_IO_READ32(pAd, TX_AGG_CNT2, &TxAggCnt2.word);
	RTMP_IO_READ32(pAd, TX_AGG_CNT3, &TxAggCnt3.word);
		RTMP_IO_READ32(pAd, TX_AGG_CNT4, &TxAggCnt4.word);
		RTMP_IO_READ32(pAd, TX_AGG_CNT5, &TxAggCnt5.word);
		RTMP_IO_READ32(pAd, TX_AGG_CNT6, &TxAggCnt6.word);
		RTMP_IO_READ32(pAd, TX_AGG_CNT7, &TxAggCnt7.word);
		pRalinkCounters->TxAggCount += TxAggCnt.field.AggTxCount;
		pRalinkCounters->TxNonAggCount += TxAggCnt.field.NonAggTxCount;
		pRalinkCounters->TxAgg1MPDUCount += TxAggCnt0.field.AggSize1Count;
		pRalinkCounters->TxAgg2MPDUCount += TxAggCnt0.field.AggSize2Count;
		
		pRalinkCounters->TxAgg3MPDUCount += TxAggCnt1.field.AggSize3Count;
		pRalinkCounters->TxAgg4MPDUCount += TxAggCnt1.field.AggSize4Count;
		pRalinkCounters->TxAgg5MPDUCount += TxAggCnt2.field.AggSize5Count;
		pRalinkCounters->TxAgg6MPDUCount += TxAggCnt2.field.AggSize6Count;
	
		pRalinkCounters->TxAgg7MPDUCount += TxAggCnt3.field.AggSize7Count;
		pRalinkCounters->TxAgg8MPDUCount += TxAggCnt3.field.AggSize8Count;
		pRalinkCounters->TxAgg9MPDUCount += TxAggCnt4.field.AggSize9Count;
		pRalinkCounters->TxAgg10MPDUCount += TxAggCnt4.field.AggSize10Count;

		pRalinkCounters->TxAgg11MPDUCount += TxAggCnt5.field.AggSize11Count;
		pRalinkCounters->TxAgg12MPDUCount += TxAggCnt5.field.AggSize12Count;
		pRalinkCounters->TxAgg13MPDUCount += TxAggCnt6.field.AggSize13Count;
		pRalinkCounters->TxAgg14MPDUCount += TxAggCnt6.field.AggSize14Count;

		pRalinkCounters->TxAgg15MPDUCount += TxAggCnt7.field.AggSize15Count;
		pRalinkCounters->TxAgg16MPDUCount += TxAggCnt7.field.AggSize16Count;

		/* Calculate the transmitted A-MPDU count*/
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += TxAggCnt0.field.AggSize1Count;
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt0.field.AggSize2Count >> 1);

		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt1.field.AggSize3Count / 3);
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt1.field.AggSize4Count >> 2);

		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt2.field.AggSize5Count / 5);
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt2.field.AggSize6Count / 6);

		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt3.field.AggSize7Count / 7);
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt3.field.AggSize8Count >> 3);

		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt4.field.AggSize9Count / 9);
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt4.field.AggSize10Count / 10);

		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt5.field.AggSize11Count / 11);
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt5.field.AggSize12Count / 12);

		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt6.field.AggSize13Count / 13);
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt6.field.AggSize14Count / 14);

		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt7.field.AggSize15Count / 15);
		pRalinkCounters->TransmittedAMPDUCount.u.LowPart += (TxAggCnt7.field.AggSize16Count >> 4);	
	}
#endif /* STATS_COUNT_SUPPORT */			
}


VOID SendLEDCmd(
	struct rt2x00_dev *rt2x00dev,
	IN ULONG	LEDMode,
	IN ULONG	Para)
{
	CHAR *Pos, *pBuf;
	ULONG	LEDParameter[2] = {0};
	INT ret;
	struct CMD_UNIT CmdUnit;
	
	if (!rt2x00_rt(rt2x00dev, MT7630))
	{
		//DBGPRINT(RT_DEBUG_ERROR, ("%s: Incorrect NIC\n", __FUNCTION__));
		
		return;
	}

	os_alloc_mem(rt2x00dev, (UCHAR **)&pBuf, 8);
	if (pBuf == NULL)
	{
		return NDIS_STATUS_RESOURCES;
	}
	
        // workaround patch
	
	printk("%s: Mode:%d, Para: %d-->\n", __FUNCTION__, LEDMode, Para);

	LEDParameter[0] = LEDMode;
	LEDParameter[1] = Para;	

	//hex_dump("SendLEDCmd: ", LEDParameter, sizeof(LEDParameter));
	NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
	
	CmdUnit.u.ANDES.Type = CMD_LED_MODE_OP;
	CmdUnit.u.ANDES.CmdPayloadLen = sizeof(LEDParameter);
	CmdUnit.u.ANDES.CmdPayload = LEDParameter;
	
	CmdUnit.u.ANDES.NeedRsp = FALSE;
	CmdUnit.u.ANDES.NeedWait = FALSE;
	CmdUnit.u.ANDES.Timeout = 0;

	ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);

	RTMPusecDelay(500);
	return ret;

}


static char *phy_mode_str[]={"CCK", "OFDM", "HTMIX", "GF", "VHT"};
char* get_phymode_str(int Mode)
{
	if (Mode >= MODE_CCK && Mode <= MODE_VHT)
		return phy_mode_str[Mode];
	else
		return "N/A";
}

static UCHAR *phy_bw_str[] = {"20M", "40M", "80M", "10M"};
char* get_bw_str(int bandwidth)
{
	if (bandwidth >= BW_20 && bandwidth <= BW_10)
		return phy_bw_str[bandwidth];
	else
		return "N/A";
}

static UCHAR *txwi_txop_str[]={"HT_TXOP", "PIFS", "SIFS", "BACKOFF", "Invalid"};
#define TXWI_TXOP_STR(_x)	((_x) <= 3 ? txwi_txop_str[(_x)]: txwi_txop_str[4])
VOID dumpTxWI(struct rt2x00_dev *rt2x00dev, TXWI_STRUC *pTxWI)
{
	hex_dump("TxWI Raw Data: ", (UCHAR *)pTxWI, sizeof(TXWI_STRUC));

	printk("TxWI Fields:\n");
	printk("\tPHYMODE=%d(%s)\n", pTxWI->TxWIPHYMODE,  get_phymode_str(pTxWI->TxWIPHYMODE));
	printk("\tSTBC=%d\n", pTxWI->TxWISTBC);
	printk("\tShortGI=%d\n", pTxWI->TxWIShortGI);
	printk("\tBW=%d(%sMHz)\n", pTxWI->TxWIBW, get_bw_str(pTxWI->TxWIBW));
	printk("\tMCS=%d\n", pTxWI->TxWIMCS);
	printk("\tTxOP=%d(%s)\n", pTxWI->TxWITXOP, TXWI_TXOP_STR(pTxWI->TxWITXOP));
	printk("\tMpduDensity=%d\n", pTxWI->TxWIMpduDensity);	
	printk("\tAMPDU=%d\n", pTxWI->TxWIAMPDU);
	printk("\tTS=%d\n", pTxWI->TxWITS);
	printk("\tCF-ACK=%d\n", pTxWI->TxWICFACK);
	printk("\tMIMO-PS=%d\n", pTxWI->TxWIMIMOps);
	printk("\tNSEQ=%d\n", pTxWI->TxWINSEQ);
	printk("\tACK=%d\n", pTxWI->TxWIACK);
	printk("\tFRAG=%d\n", pTxWI->TxWIFRAG);
	printk("\tWCID=%d\n", pTxWI->TxWIWirelessCliID);
	printk("\tBAWinSize=%d\n", pTxWI->TxWIBAWinSize);
	printk("\tMPDUtotalByteCnt=%d\n", pTxWI->TxWIMPDUByteCnt);	
	printk("\tPID=%d\n", pTxWI->TxWIPacketId);	
}
EXPORT_SYMBOL_GPL(dumpTxWI);


void hex_dump(char *str, unsigned char *pSrcBufVA, u32 SrcBufLen)
{
	unsigned char *pt;
	int x;
	pt = pSrcBufVA;
	printk("%s: %p, len = %d\n", str, pSrcBufVA, SrcBufLen);
	for (x = 0; x < SrcBufLen; x++) {
		if (x % 16 == 0)
			printk("0x%04x : ", x);
		printk("%02x ", ((unsigned char)pt[x]));
		if (x % 16 == 15)
			printk("\n");
	}
	printk("\n");
}
EXPORT_SYMBOL_GPL(hex_dump);

//
// SendAndesWLANStatus
//
VOID SendAndesWLANStatus(
	struct rt2x00_dev *rt2x00dev,
	IN UCHAR			WlanStatus,
	IN ULONG			PrivilegeTime, 
	IN UCHAR                     BssHashID
	)
{
    
	COEX_WLAN_STATUS wlanStatus = {0};
	USHORT wlanStatusLength = 0;
	struct CMD_UNIT CmdUnit;
	INT ret = NDIS_STATUS_SUCCESS;

	if (!rt2x00_rt(rt2x00dev, MT7630))
	{	
		return;
	}
	
	printk("%s: -->\n", __FUNCTION__);
	wlanStatus.CoexOperation = TypeWiFiStatus;
       wlanStatus.WLANStatus= WlanStatus;
       wlanStatus.PrivilegeTime= PrivilegeTime;
       wlanStatus.BssHashID = BssHashID;
	wlanStatusLength = sizeof(wlanStatus);

	printk("%s: CoexOperation = %d, WlanStatus = %X, PrivilegeTime = %d, BssHashID = %d, PktLength = %d\n", 
		__FUNCTION__, 
		wlanStatus.CoexOperation, 
		wlanStatus.WLANStatus,
		wlanStatus.PrivilegeTime,
		wlanStatus.BssHashID,
		wlanStatusLength
		);

	//hex_dump("SendLEDCmd: ", LEDParameter, sizeof(LEDParameter));
	NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
	
	CmdUnit.u.ANDES.Type = PKT_CMD_TYPE_COEX_OP;
	CmdUnit.u.ANDES.CmdPayloadLen = wlanStatusLength;
	CmdUnit.u.ANDES.CmdPayload = &wlanStatus;
	
	CmdUnit.u.ANDES.NeedRsp = FALSE;
	CmdUnit.u.ANDES.NeedWait = FALSE;
	CmdUnit.u.ANDES.Timeout = 0;

	ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);

	RTMPusecDelay(500);
	return ret;
	
}
EXPORT_SYMBOL_GPL(SendAndesWLANStatus);


//
// SendAndesCCUForceMode
//
VOID SendAndesCCUForceMode(
	struct rt2x00_dev *rt2x00dev,
	IN UCHAR			CoexMode
	)
{
    
	COEX_TF_SWITCH coexTF = {0};
	USHORT coexTFLength = 0;
	struct CMD_UNIT CmdUnit;
	INT ret = NDIS_STATUS_SUCCESS;
	
	printk("%s: -->\n", __FUNCTION__);
	coexTF.CoexOperation = TypeCoexCCUForceMode;
       coexTF.CoexMode = CoexMode;
       
	coexTFLength = sizeof(coexTF);

	NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
	
	CmdUnit.u.ANDES.Type = PKT_CMD_TYPE_COEX_OP;
	CmdUnit.u.ANDES.CmdPayloadLen = coexTFLength;
	CmdUnit.u.ANDES.CmdPayload = &coexTF;
	
	CmdUnit.u.ANDES.NeedRsp = FALSE;
	CmdUnit.u.ANDES.NeedWait = FALSE;
	CmdUnit.u.ANDES.Timeout = 0;
	
	printk("%s: CoexOperation = %d, CoexMode = %d\n, PktLength = %d\n", 
		__FUNCTION__, 
		coexTF.CoexOperation, 
		coexTF.CoexMode,
		coexTFLength
		);

	ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);

	printk("%s: <--\n", __FUNCTION__);
       TDDFDDExclusiveRequest(rt2x00dev, COEX_MODE_RESET);
	
}
EXPORT_SYMBOL_GPL(SendAndesCCUForceMode);


//
// SendAndesAFH
//
VOID SendAndesAFH(
	IN struct rt2x00_dev *rt2x00dev,
	IN UCHAR			BBPCurrentBW,
	IN UCHAR			Channel,
	IN UCHAR			CentralChannel,
	IN BOOLEAN			Disable,
	IN ULONG                     BssHashID)
{
    
	COEX_AFH coexAFH = {0};
	USHORT coexAFHLength = 0;
	struct CMD_UNIT CmdUnit;
	INT ret = NDIS_STATUS_SUCCESS;
	
        if ( !(rt2x00_rt(rt2x00dev, MT7630)))
        {
            return;
        }
	printk("%s: -->\n", __FUNCTION__);
	coexAFH.CoexOperation = TypeAFH;

        if (BBPCurrentBW == BW_40)
        {
            coexAFH.BW= BW_40 + COEXNOZEROSHIFT ;
            coexAFH.Channel= CentralChannel;
        }
        else if (BBPCurrentBW == BW_20)
        {
            coexAFH.BW= BW_20 + COEXNOZEROSHIFT;
            coexAFH.Channel= Channel;
        }
         else if (BBPCurrentBW == BW_80)
        {
            coexAFH.BW= BW_80 + COEXNOZEROSHIFT;
            coexAFH.Channel= Channel;
        }

        if (Channel > 14)
        {
            coexAFH.BW= 0;
        }
        if (Disable == FALSE)
        {
    	    coexAFH.LinkStatus = COEX_WIFI_LINK_UP;
        }
        else
        {
	    coexAFH.LinkStatus = COEX_WIFI_LINK_DOWN;
        }
        coexAFH.BssHashID = BssHashID;

    
	coexAFHLength = sizeof(coexAFH);


	NdisZeroMemory(&CmdUnit, sizeof(CmdUnit));
	
	CmdUnit.u.ANDES.Type = PKT_CMD_TYPE_COEX_OP;
	CmdUnit.u.ANDES.CmdPayloadLen = coexAFHLength;
	CmdUnit.u.ANDES.CmdPayload = &coexAFH;
	
	CmdUnit.u.ANDES.NeedRsp = FALSE;
	CmdUnit.u.ANDES.NeedWait = FALSE;
	CmdUnit.u.ANDES.Timeout = 0;
	
	printk("%s: LinkStatus = %d, BW = %d, Channel = %d, BssHashID = %d, PktLength = %d\n", 
		__FUNCTION__, 
		coexAFH.LinkStatus, 
		coexAFH.BW, 
		coexAFH.Channel,
		coexAFH.BssHashID, 
		coexAFHLength
		);

	ret = AsicSendCmdToAndes(rt2x00dev, &CmdUnit);

	printk("%s: <--\n", __FUNCTION__);
	
}
EXPORT_SYMBOL_GPL(SendAndesAFH);

void Set_BtDump_Proc(
	IN 	struct rt2x00_dev *rt2x00dev,
	IN int index)
{
    mm_segment_t old_fs;
    struct file *file = NULL;

    UINT16 BaseOffset;
    UINT16 ReadOffset;
    UINT32 offset,buf;
	//unsigned char buf[4] = {0};

    old_fs = get_fs();
    set_fs(KERNEL_DS);

	if (index == 0)
	file = filp_open("/tmp/bt_log_0x00080000_to_0x000A7FFF_before", O_RDWR | O_APPEND | O_CREAT, 0644);
	else
	file = filp_open("/tmp/bt_log_0x00080000_to_0x000A7FFF_after", O_RDWR | O_APPEND | O_CREAT, 0644);
	
    if (IS_ERR(file)) 
    {
        printk("error occured while opening file /tmp/bt_log_0x00080000_to_0x000A7FFF, exiting...\n");
        set_fs(old_fs);
        return 0;
    }	

	RTMP_IO_WRITE32(rt2x00dev, PCIE_REMAP_BASE4, 0x80000);
	for ( offset = 0x0 ; offset < 0xFFFF ; offset+=4 )
    	{
    		//memset(buf, 0, 4);
    		buf = 0;
		RTMP_IO_READ32(rt2x00dev, 0x80000+offset, &buf);	
		//printk("0x%X:= 0x%x :\n",0x80000+offset, buf); 
		file->f_op->write(file, &buf, 4, &file->f_pos);
	}

	RTMP_IO_WRITE32(rt2x00dev, PCIE_REMAP_BASE4, 0x90000);
	for ( offset = 0x0 ; offset < 0xFFFF ; offset+=4 )
    	{
    		//memset(buf, 0, 4);
    		buf = 0;
		RTMP_IO_READ32(rt2x00dev, 0x90000+offset, &buf);	
		//printk("0x%X:= 0x%x :\n",0x80000+offset, buf); 
		file->f_op->write(file, &buf, 4, &file->f_pos);
	}

	RTMP_IO_WRITE32(rt2x00dev, PCIE_REMAP_BASE4, 0xa0000);
	for ( offset = 0x0 ; offset < 0x2FFF ; offset+=4 )
    	{
    		//memset(buf, 0, 4);
    		buf = 0;
		RTMP_IO_READ32(rt2x00dev, 0xa0000+offset, &buf);	
		//printk("0x%X:= 0x%x :\n",0x80000+offset, buf); 
		file->f_op->write(file, &buf, 4, &file->f_pos);
	}
	
	RTMP_IO_WRITE32(rt2x00dev, PCIE_REMAP_BASE4, 0x00);
	filp_close(file, NULL);
}
EXPORT_SYMBOL_GPL(Set_BtDump_Proc);


VOID MLMEHook(
	IN 	struct rt2x00_dev *rt2x00dev,
	IN UCHAR		WlanStatus,
	IN UCHAR              BssHashID
	) 
	{
    //ULONG unit = ((rt2x00dev->CommonCfg.CoexWLANPrivilegeTime & 0xF0000000)>>28)*100; //5ms*100 = 500
    //ULONG times = 0;
   
    if (!(rt2x00_rt(rt2x00dev, MT7630)))
    {
        return;
    }

    switch (WlanStatus)
    {
        case WLAN_Device_ON :
            SendAndesWLANStatus(rt2x00dev, WlanStatus, 0, 0);        
            break;

        case WLAN_CONNECTION_START :
           // times = (pAd->CommonCfg.CoexWLANPrivilegeTime & 0xF0)>>4;
            SendAndesWLANStatus(rt2x00dev, WlanStatus, 0, BssHashID);
            break;                    
        default:
            SendAndesWLANStatus(rt2x00dev, WlanStatus, 0, BssHashID);
    }

 }
EXPORT_SYMBOL_GPL(MLMEHook);
EXPORT_SYMBOL_GPL(NICUpdateRawCounters);
MODULE_AUTHOR(DRV_PROJECT);
MODULE_VERSION(DRV_VERSION);
MODULE_DESCRIPTION("rt2x00 7630 library");
MODULE_LICENSE("GPL");

