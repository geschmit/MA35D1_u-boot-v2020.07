// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2020 Nuvoton Technology Corp.
 *
 */


#include <cpu_func.h>
#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/uclass.h>
#include <dm/pinctrl.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <malloc.h>
#include <linux/delay.h>
#include <nand.h>
#include <clk.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>

#include <dt-bindings/clock/ma35d1-clk.h>

/* NFI Registers */
#define REG_NFI_BUFFER0	(0x000)	/* NFI Embedded Buffer Word 0 */
#define REG_NFI_DMACTL		(0x400)	/* NFI DMA Control and Status Register */
#define REG_NFI_DMASA		(0x408)	/* NFI DMA Transfer Starting Address Register */
#define REG_NFI_DMABCNT	(0x40C)	/* NFI DMA Transfer Byte Count Register */
#define REG_NFI_DMAINTEN	(0x410)	/* NFI DMA Interrupt Enable Control Register */
#define REG_NFI_DMAINTSTS	(0x414)	/* NFI DMA Interrupt Status Register */

#define REG_NFI_GCTL		(0x800)	/* NFI Global Control and Status Register */
#define REG_NFI_GINTEN		(0x804)	/* NFI Global Interrupt Control Register */
#define REG_NFI_GINTSTS	(0x808)	/* NFI Global Interrupt Status Register */

/* NAND-type Flash Registers */
#define REG_NFI_NANDCTL	(0x8A0)	/* NAND Flash Control Register */
#define REG_NFI_NANDTMCTL	(0x8A4)	/* NAND Flash Timing Control Register */
#define REG_NFI_NANDINTEN	(0x8A8)	/* NAND Flash Interrupt Enable Register */
#define REG_NFI_NANDINTSTS	(0x8AC)	/* NAND Flash Interrupt Status Register */
#define REG_NFI_NANDCMD	(0x8B0)	/* NAND Flash Command Port Register */
#define REG_NFI_NANDADDR	(0x8B4)	/* NAND Flash Address Port Register */
#define REG_NFI_NANDDATA	(0x8B8)	/* NAND Flash Data Port Register */
#define REG_NFI_NANDRACTL	(0x8BC)	/* NAND Flash Redundant Area Control Register */
#define REG_NFI_NANDECTL	(0x8C0)	/* NAND Flash Extend Control Register */
#define REG_NFI_NANDECCES0	(0x8D0)	/* NAND Flash ECC Error Status 0 Register */
#define REG_NFI_NANDECCES1	(0x8D4)	/* NAND Flash ECC Error Status 1 Register */
#define REG_NFI_NANDECCES2	(0x8D8)	/* NAND Flash ECC Error Status 2 Register */
#define REG_NFI_NANDECCES3	(0x8DC)	/* NAND Flash ECC Error Status 3 Register */

/* NAND-type Flash BCH Error Address Registers */
#define REG_NFI_NANDECCEA0	(0x900)	/* NAND Flash ECC Error Byte Address 0 Register */
#define REG_NFI_NANDECCEA1	(0x904)	/* NAND Flash ECC Error Byte Address 1 Register */
#define REG_NFI_NANDECCEA2	(0x908)	/* NAND Flash ECC Error Byte Address 2 Register */
#define REG_NFI_NANDECCEA3	(0x90C)	/* NAND Flash ECC Error Byte Address 3 Register */
#define REG_NFI_NANDECCEA4	(0x910)	/* NAND Flash ECC Error Byte Address 4 Register */
#define REG_NFI_NANDECCEA5	(0x914)	/* NAND Flash ECC Error Byte Address 5 Register */
#define REG_NFI_NANDECCEA6	(0x918)	/* NAND Flash ECC Error Byte Address 6 Register */
#define REG_NFI_NANDECCEA7	(0x91C)	/* NAND Flash ECC Error Byte Address 7 Register */
#define REG_NFI_NANDECCEA8	(0x920)	/* NAND Flash ECC Error Byte Address 8 Register */
#define REG_NFI_NANDECCEA9	(0x924)	/* NAND Flash ECC Error Byte Address 9 Register */
#define REG_NFI_NANDECCEA10	(0x928)	/* NAND Flash ECC Error Byte Address 10 Register */
#define REG_NFI_NANDECCEA11	(0x92C)	/* NAND Flash ECC Error Byte Address 11 Register */

/* NAND-type Flash BCH Error Data Registers */
#define REG_NFI_NANDECCED0	(0x960)	/* NAND Flash ECC Error Data Register 0 */
#define REG_NFI_NANDECCED1	(0x964)	/* NAND Flash ECC Error Data Register 1 */
#define REG_NFI_NANDECCED2	(0x968)	/* NAND Flash ECC Error Data Register 2 */
#define REG_NFI_NANDECCED3	(0x96C)	/* NAND Flash ECC Error Data Register 3 */
#define REG_NFI_NANDECCED4	(0x970)	/* NAND Flash ECC Error Data Register 4 */
#define REG_NFI_NANDECCED5	(0x974)	/* NAND Flash ECC Error Data Register 5 */

/* NAND-type Flash Redundant Area Registers */
#define REG_NFI_NANDRA0	(0xA00)	/* NAND Flash Redundant Area Word 0 */
#define REG_NFI_NANDRA1	(0xA04)	/* NAND Flash Redundant Area Word 1 */

/*******************************************/
#define NAND_EN     0x08
#define ENDADDR     (0x01 << 31)

/*-------------------------------*/
/* Define some constants for BCH */
/*-------------------------------*/
// define the total padding bytes for 512/1024 data segment
#define BCH_PADDING_LEN_512     32
#define BCH_PADDING_LEN_1024    64
// define the BCH parity code length for 512 bytes data pattern
#define BCH_PARITY_LEN_T8  15
#define BCH_PARITY_LEN_T12 23
// define the BCH parity code length for 1024 bytes data pattern
#define BCH_PARITY_LEN_T24 45

#define BCH_T8    0x00100000
#define BCH_T12   0x00200000
#define BCH_T24   0x00040000


struct ma35d1_nand_info {
	struct udevice   *dev;
	struct mtd_info  mtd;
	struct nand_chip chip;
	void __iomem     *reg;
	int eBCHAlgo;
	int m_i32SMRASize;
};
struct ma35d1_nand_info *ma35d1_nand;

static struct nand_ecclayout ma35d1_nand_oob;

static const int g_i32BCHAlgoIdx[4] = { BCH_T8, BCH_T8, BCH_T12, BCH_T24 };
static const int g_i32ParityNum[3][4] = {
	{ 0,  60,  92,  90 },  // for 2K
	{ 0, 120, 184, 180 },  // for 4K
	{ 0, 240, 368, 360 },  // for 8K
};

static void ma35d1_layout_oob_table(struct nand_ecclayout *pNandOOBTbl,
					int oobsize, int eccbytes)
{
	pNandOOBTbl->eccbytes = eccbytes;

	pNandOOBTbl->oobavail = oobsize - 4 - eccbytes;

	pNandOOBTbl->oobfree[0].offset = 4;  // Bad block marker size

	pNandOOBTbl->oobfree[0].length = oobsize - eccbytes - pNandOOBTbl->oobfree[0].offset;
}


static void ma35d1_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);

	if (ctrl & NAND_CTRL_CHANGE) {
		ulong IO_ADDR_W = (ulong)REG_NFI_NANDDATA;

		if ((ctrl & NAND_CLE))
			IO_ADDR_W = REG_NFI_NANDCMD;
		if ((ctrl & NAND_ALE))
			IO_ADDR_W = REG_NFI_NANDADDR;

		nand->IO_ADDR_W = (void __iomem *)(nand_info->reg + IO_ADDR_W);
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, nand->IO_ADDR_W);
}


/* select chip */
static void ma35d1_nand_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);

	if (chip == 0)
		writel(readl(nand_info->reg+REG_NFI_NANDCTL) & (~0x02000000), nand_info->reg+REG_NFI_NANDCTL);
	else
		writel(readl(nand_info->reg+REG_NFI_NANDCTL) | 0x02000000, nand_info->reg+REG_NFI_NANDCTL);
}


static int ma35d1_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(chip);
	int ready;

	ready = (readl(nand_info->reg+REG_NFI_NANDINTSTS) & 0x40000) ? 1 : 0;

	return ready;
}

static int ma35d1_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct ma35d1_nand_info *nand = nand_get_controller_data(chip);
	unsigned long time;
	int status = -1;

	time = get_timer(0);
	while (1) {
		if (readl(nand->reg+REG_NFI_NANDINTSTS) & 0x400) {	/* check r/b# flag */
			writel(0x400, nand->reg+REG_NFI_NANDINTSTS);
			status = 0;
			break;
		}
		if (get_timer(time) > 3000) {
			pr_err("R/B# timeout!\n");
			break;
		}
	}
	return status;
}

static void ma35d1_nand_command(struct mtd_info *mtd, unsigned int command,
				int column, int page_addr)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(chip);

	writel(0x400, nand_info->reg+REG_NFI_NANDINTSTS);

	if (command == NAND_CMD_READOOB) {
		column += mtd->writesize;
		command = NAND_CMD_READ0;
	}

	switch (command) {

	case NAND_CMD_RESET:
		writel(command, nand_info->reg+REG_NFI_NANDCMD);
		ma35d1_waitfunc(mtd, chip);
		break;

	case NAND_CMD_READID:
		writel(command, nand_info->reg+REG_NFI_NANDCMD);
		writel(ENDADDR|column, nand_info->reg+REG_NFI_NANDADDR);
		break;

	case NAND_CMD_PARAM:
		writel(command, nand_info->reg+REG_NFI_NANDCMD);
		writel(ENDADDR|column, nand_info->reg+REG_NFI_NANDADDR);
		ma35d1_waitfunc(mtd, chip);
		break;

	case NAND_CMD_READ0:
		writel(command, nand_info->reg+REG_NFI_NANDCMD);
		if (column != -1) {
			writel(column & 0xff, nand_info->reg+REG_NFI_NANDADDR);
			writel((column >> 8) & 0xff, nand_info->reg+REG_NFI_NANDADDR);
		}
		if (page_addr != -1) {
			writel(page_addr & 0xff, nand_info->reg+REG_NFI_NANDADDR);
			if (chip->options & NAND_ROW_ADDR_3) {
				writel((page_addr >> 8) & 0xff, nand_info->reg+REG_NFI_NANDADDR);
				writel(((page_addr >> 16) & 0xff)|ENDADDR, nand_info->reg+REG_NFI_NANDADDR);
			} else
				writel(((page_addr >> 8) & 0xff)|ENDADDR, nand_info->reg+REG_NFI_NANDADDR);
		}
		writel(NAND_CMD_READSTART, nand_info->reg+REG_NFI_NANDCMD);
		ma35d1_waitfunc(mtd, chip);
		break;


	case NAND_CMD_ERASE1:
		writel(command, nand_info->reg+REG_NFI_NANDCMD);
		writel(page_addr & 0xff, nand_info->reg+REG_NFI_NANDADDR);
		if (chip->options & NAND_ROW_ADDR_3) {
			writel((page_addr >> 8) & 0xff, nand_info->reg+REG_NFI_NANDADDR);
			writel(((page_addr >> 16) & 0xff)|ENDADDR, nand_info->reg+REG_NFI_NANDADDR);
		} else
			writel(((page_addr >> 8) & 0xff)|ENDADDR, nand_info->reg+REG_NFI_NANDADDR);
		break;


	case NAND_CMD_SEQIN:
		writel(command, nand_info->reg+REG_NFI_NANDCMD);
		writel(column & 0xff, nand_info->reg+REG_NFI_NANDADDR);
		writel(column >> 8, nand_info->reg+REG_NFI_NANDADDR);
		writel(page_addr & 0xff, nand_info->reg+REG_NFI_NANDADDR);
		if (chip->options & NAND_ROW_ADDR_3) {
			writel((page_addr >> 8) & 0xff, nand_info->reg+REG_NFI_NANDADDR);
			writel(((page_addr >> 16) & 0xff)|ENDADDR, nand_info->reg+REG_NFI_NANDADDR);
		} else
			writel(((page_addr >> 8) & 0xff)|ENDADDR, nand_info->reg+REG_NFI_NANDADDR);
		break;

	default:
		writel(command, nand_info->reg+REG_NFI_NANDCMD);
	}
}

/*
 * ma35d1_nand_read_byte - read a byte from NAND controller into buffer
 * @mtd: MTD device structure
 */
static unsigned char ma35d1_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);

	return ((unsigned char)readl(nand_info->reg+REG_NFI_NANDDATA));
}

/*
 * ma35d1_nand_write_buf - write data from buffer into NAND controller
 * @mtd: MTD device structure
 * @buf: virtual address in RAM of source
 * @len: number of data bytes to be transferred
 */

static void ma35d1_nand_write_buf(struct mtd_info *mtd, const unsigned char *buf, int len)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);
	int i;

	for (i = 0; i < len; i++)
		writel(buf[i], nand_info->reg+REG_NFI_NANDDATA);
}

/*
 * ma35d1_nand_read_buf - read data from NAND controller into buffer
 * @mtd: MTD device structure
 * @buf: virtual address in RAM of source
 * @len: number of data bytes to be transferred
 */
static void ma35d1_nand_read_buf(struct mtd_info *mtd, unsigned char *buf, int len)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);
	int i;

	for (i = 0; i < len; i++)
		buf[i] = (unsigned char)readl(nand_info->reg+REG_NFI_NANDDATA);
}


/*
 * Enable HW ECC : unused on most chips
 */
void ma35d1_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
}

/*
 * Calculate HW ECC
 * function called after a write
 * mtd:        MTD block structure
 * dat:        raw data (unused)
 * ecc_code:   buffer for ECC
 */
static int ma35d1_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	return 0;
}

/*
 * HW ECC Correction
 * function called after a read
 * mtd:        MTD block structure
 * dat:        raw data read from the chip
 * read_ecc:   ECC from the chip (unused)
 * isnull:     unused
 */
static int ma35d1_nand_correct_data(struct mtd_info *mtd, u_char *dat,
					u_char *read_ecc, u_char *calc_ecc)
{
	return 0;
}


/*-------------------------------------------------------*/
/* Correct data by BCH alrogithm.                        */
/*      Support 8K page size NAND and BCH T4/8/12/15/24. */
/*-------------------------------------------------------*/
void fmiSM_CorrectData_BCH(struct mtd_info *mtd, u8 ucFieidIndex, u8 ucErrorCnt, u8 *pDAddr)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);
	u32 uaData[24], uaAddr[24];
	u32 uaErrorData[6];
	u8  ii, jj;
	u32 uPageSize;
	u32 field_len, padding_len, parity_len;
	u32 total_field_num;
	u8  *smra_index;

	//--- assign some parameters for different BCH and page size
	switch (readl(nand_info->reg+REG_NFI_NANDCTL) & 0x007C0000) {
	case BCH_T24:
		field_len   = 1024;
		padding_len = BCH_PADDING_LEN_1024;
		parity_len  = BCH_PARITY_LEN_T24;
		break;
	case BCH_T12:
		field_len   = 512;
		padding_len = BCH_PADDING_LEN_512;
		parity_len  = BCH_PARITY_LEN_T12;
		break;
	case BCH_T8:
		field_len   = 512;
		padding_len = BCH_PADDING_LEN_512;
		parity_len  = BCH_PARITY_LEN_T8;
		break;
	default:
		return;
	}

	uPageSize = readl(nand_info->reg+REG_NFI_NANDCTL) & 0x00030000;
	switch (uPageSize) {
	case 0x30000:
		total_field_num = 8192 / field_len;
		break;
	case 0x20000:
		total_field_num = 4096 / field_len;
		break;
	case 0x10000:
		total_field_num = 2048 / field_len;
		break;
	case 0x00000:
		total_field_num =  512 / field_len;
		break;
	default:
		return;
	}

	// got valid BCH_ECC_DATAx and parse them to uaData[]
	// got the valid register number of BCH_ECC_DATAx since one register include 4 error bytes
	jj = ucErrorCnt / 4;
	jj++;
	if (jj > 6)
		jj = 6;     // there are 6 BCH_ECC_DATAx registers to support BCH T24

	for (ii = 0; ii < jj; ii++)
		uaErrorData[ii] = readl(nand_info->reg+REG_NFI_NANDECCED0 + ii*4);

	for (ii = 0; ii < jj; ii++) {
		uaData[ii*4+0] = uaErrorData[ii] & 0xff;
		uaData[ii*4+1] = (uaErrorData[ii]>>8) & 0xff;
		uaData[ii*4+2] = (uaErrorData[ii]>>16) & 0xff;
		uaData[ii*4+3] = (uaErrorData[ii]>>24) & 0xff;
	}

	// got valid REG_BCH_ECC_ADDRx and parse them to uaAddr[]
	// got the valid register number of REG_BCH_ECC_ADDRx since one register include 2 error addresses
	jj = ucErrorCnt / 2;
	jj++;
	if (jj > 12)
		jj = 12;    // there are 12 REG_BCH_ECC_ADDRx registers to support BCH T24

	for (ii = 0; ii < jj; ii++) {
		// 11 bits for error address
		uaAddr[ii*2+0] = readl(nand_info->reg+REG_NFI_NANDECCEA0 + ii*4) & 0x07ff;
		uaAddr[ii*2+1] = (readl(nand_info->reg+REG_NFI_NANDECCEA0 + ii*4)>>16) & 0x07ff;
	}

	//--- pointer to begin address of field that with data error
	pDAddr += (ucFieidIndex-1) * field_len;

	//--- correct each error bytes
	for (ii = 0; ii < ucErrorCnt; ii++) {
		// for wrong data in field
		if (uaAddr[ii] < field_len)
			*(pDAddr+uaAddr[ii]) ^= uaData[ii];

		// for wrong first-3-bytes in redundancy area
		else if (uaAddr[ii] < (field_len+3)) {
			uaAddr[ii] -= field_len;
			uaAddr[ii] += (parity_len*(ucFieidIndex-1));    // field offset
			*((u8 *)(nand_info->reg+REG_NFI_NANDRA0) + uaAddr[ii]) ^= uaData[ii];
		}
		// for wrong parity code in redundancy area
		else {
			// BCH_ERR_ADDRx = [data in field] + [3 bytes] + [xx] + [parity code]
			//                                   |<--     padding bytes      -->|
			// The BCH_ERR_ADDRx for last parity code always = field size + padding size.
			// So, the first parity code = field size + padding size - parity code length.
			// For example, for BCH T12, the first parity code = 512 + 32 - 23 = 521.
			// That is, error byte address offset within field is
			uaAddr[ii] = uaAddr[ii] - (field_len + padding_len - parity_len);

			// smra_index point to the first parity code of first field in register SMRA0~n
			smra_index = (u8 *)(nand_info->reg+REG_NFI_NANDRA0 +
					(readl(nand_info->reg+REG_NFI_NANDRACTL) & 0x1ff) -
					(parity_len * total_field_num));

			// final address = first parity code of first field +
			//                 offset of fields +
			//                 offset within field
			*((u8 *)smra_index + (parity_len * (ucFieidIndex-1)) + uaAddr[ii]) ^= uaData[ii];
		}
	}   // end of for (ii < ucErrorCnt)
}

int fmiSMCorrectData(struct mtd_info *mtd, unsigned long uDAddr)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);
	int uStatus, ii, jj, i32FieldNum = 0;
	int uErrorCnt = 0;

	if (readl(nand_info->reg+REG_NFI_NANDINTSTS) & 0x4) {
		if ((readl(nand_info->reg+REG_NFI_NANDCTL) & 0x7C0000) == BCH_T24)
			i32FieldNum = mtd->writesize / 1024;    // Block=1024 for BCH
		else
			i32FieldNum = mtd->writesize / 512;

		if (i32FieldNum < 4)
			i32FieldNum = 1;
		else
			i32FieldNum /= 4;

		for (jj = 0; jj < i32FieldNum; jj++) {
			uStatus = readl(nand_info->reg+REG_NFI_NANDECCES0 + jj*4);
			if (!uStatus)
				continue;

			for (ii = 1; ii < 5; ii++) {
				if (!(uStatus & 0x03)) { // No error

					uStatus >>= 8;
					continue;

				} else if ((uStatus & 0x03) == 0x01) { // Correctable error

					uErrorCnt = (uStatus >> 2) & 0x1F;
					fmiSM_CorrectData_BCH(mtd, jj*4+ii, uErrorCnt, (u8 *)uDAddr);

				} else { // uncorrectable error or ECC error
					pr_err("uncorrectable!\n");
					return -1;
				}
				uStatus >>= 8;
			}
		}
	}
	return uErrorCnt;
}


static inline int ma35d1_nand_dma_transfer(struct mtd_info *mtd,
	const u_char *addr, unsigned int len, int is_write)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);

	// For save, wait DMAC to ready
	while (readl(nand_info->reg+REG_NFI_DMACTL) & 0x200)
		;

	// Reinitial dmac
	// DMAC enable
	writel(readl(nand_info->reg+REG_NFI_DMACTL) | 0x3, nand_info->reg+REG_NFI_DMACTL);
	while (readl(nand_info->reg+REG_NFI_DMACTL) & 0x2)
		;

	// Fill dma_addr
	flush_dcache_range((unsigned long)addr, (unsigned long)addr + len);
	writel((unsigned long)addr, nand_info->reg+REG_NFI_DMASA);

	writel(nand_info->m_i32SMRASize, nand_info->reg+REG_NFI_NANDRACTL);

	// Enable SM_CS0
	writel((readl(nand_info->reg+REG_NFI_NANDCTL) & (~0x02000000)), nand_info->reg+REG_NFI_NANDCTL);
	/* setup and start DMA using dma_addr */

	if (is_write) {
		register char *ptr = (char *)(nand_info->reg+REG_NFI_NANDRA0);
		// To mark this page as dirty.
		if (ptr[3] == 0xFF)
			ptr[3] = 0;
		if (ptr[2] == 0xFF)
			ptr[2] = 0;

		writel(readl(nand_info->reg+REG_NFI_NANDCTL) | 0x4, nand_info->reg+REG_NFI_NANDCTL);
		while (!(readl(nand_info->reg+REG_NFI_NANDINTSTS) & 0x1))
			;

	} else {
		// Blocking for reading
		// Enable DMA Read
		writel(readl(nand_info->reg+REG_NFI_NANDCTL) | 0x2, nand_info->reg+REG_NFI_NANDCTL);

		if (readl(nand_info->reg+REG_NFI_NANDCTL) & 0x80) {
			do {
				int stat = 0;

				stat = fmiSMCorrectData(mtd, (unsigned long)addr);
				if (stat < 0) {
					mtd->ecc_stats.failed++;
					writel(0x4, nand_info->reg+REG_NFI_NANDINTSTS);
					writel(0x3, nand_info->reg+REG_NFI_DMACTL);          // reset DMAC
					writel(readl(nand_info->reg+REG_NFI_NANDCTL)|0x1, nand_info->reg+REG_NFI_NANDCTL);
					break;
				} else if (stat > 0) {
					//mtd->ecc_stats.corrected += stat; //Occurred: MLC UBIFS mount error
					writel(0x4, nand_info->reg+REG_NFI_NANDINTSTS);
				}

			} while (!(readl(nand_info->reg+REG_NFI_NANDINTSTS) & 0x1) ||
				(readl(nand_info->reg+REG_NFI_NANDINTSTS) & 0x4));
		} else
			while (!(readl(nand_info->reg+REG_NFI_NANDINTSTS) & 0x1))
				;
	}
	invalidate_dcache_range((unsigned long)addr, (unsigned long)addr + len);

	// Clear DMA finished flag
	writel(0x1, nand_info->reg+REG_NFI_NANDINTSTS);

	return 0;
}

/**
 * Raw page write function
 *
 * @param mtd	mtd info structure
 * @param chip	nand chip info structure
 * @param buf	data buffer
 */
static int ma35d1_nand_write_page_raw(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf, int oob_required, int page)
{
	ma35d1_nand_command(mtd, NAND_CMD_SEQIN, 0, page);
	ma35d1_nand_write_buf(mtd, buf, mtd->writesize);
	ma35d1_nand_command(mtd, NAND_CMD_PAGEPROG, -1, -1);
	ma35d1_waitfunc(mtd, chip);

	return 0;
}

/**
 * nand_write_page_hwecc - hardware ecc based page write function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @buf:        data buffer
 */
static int ma35d1_nand_write_page_hwecc(struct mtd_info *mtd,
	struct nand_chip *chip, const uint8_t *buf, int oob_required, int page)
{
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(chip);
	uint8_t *ecc_calc = chip->buffers->ecccalc;
	register char *ptr = (char *)(nand_info->reg+REG_NFI_NANDRA0);

	memset((void *)ptr, 0xFF, mtd->oobsize);
	memcpy((void *)ptr, (void *)chip->oob_poi, mtd->oobsize - chip->ecc.total);

	ma35d1_nand_command(mtd, NAND_CMD_SEQIN, 0, page);
	ma35d1_nand_dma_transfer(mtd, buf, mtd->writesize, 0x1);
	ma35d1_nand_command(mtd, NAND_CMD_PAGEPROG, -1, -1);
	ma35d1_waitfunc(mtd, chip);

	// Copy parity code in SMRA to calc
	memcpy((void *)ecc_calc, (void *)((long)ptr + (mtd->oobsize - chip->ecc.total)), chip->ecc.total);

	// Copy parity code in calc to oob_poi
	memcpy((void *)(chip->oob_poi+(mtd->oobsize-chip->ecc.total)), (void *)ecc_calc, chip->ecc.total);

	return 0;
}

/**
 * Read raw page data without ecc
 *
 * @param mtd	mtd info structure
 * @param chip	nand chip info structure
 * @param buf	buffer to store read data
 * @param page	page number to read
 * @return	0 when successfully completed
 *		-EINVAL when chip->oob_poi is not double-word aligned
 *		-EIO when command timeout
 */
static int ma35d1_nand_read_page_raw(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	// read data from nand
	ma35d1_nand_command(mtd, NAND_CMD_READ0, 0, page);
	ma35d1_nand_read_buf(mtd, buf, mtd->writesize);

	return 0;
}


/**
 * ma35d1_nand_read_page_hwecc_oob_first - hardware ecc based page write function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @buf:        buffer to store read data
 * @page:       page number to read
 */
static int ma35d1_nand_read_page_hwecc_oob_first(struct mtd_info *mtd,
	struct nand_chip *chip, uint8_t *buf, int oob_required, int page)
{
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(chip);
	uint8_t *p = buf;
	char *ptr = (char *)(nand_info->reg+REG_NFI_NANDRA0);

	/* At first, read the OOB area  */
	ma35d1_nand_command(mtd, NAND_CMD_READOOB, 0, page);
	ma35d1_nand_read_buf(mtd, chip->oob_poi, mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy((void *)ptr, (void *)chip->oob_poi, mtd->oobsize);

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
		memset((void *)p, 0xff, mtd->writesize);
	else {
		// Third, read data from nand
		ma35d1_nand_command(mtd, NAND_CMD_READ0, 0, page);
		ma35d1_nand_dma_transfer(mtd, p, mtd->writesize, 0x0);

		// Fouth, restore OOB data from SMRA
		memcpy((void *)chip->oob_poi, (void *)ptr, mtd->oobsize);
	}

	return 0;
}

/**
 * ma35d1_nand_read_oob_hwecc - the most common OOB data read function
 * @mtd:        mtd info structure
 * @chip:       nand chip info structure
 * @page:       page number to read
 * @sndcmd:     flag whether to issue read command or not
 */
static int ma35d1_nand_read_oob_hwecc(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct ma35d1_nand_info *nand_info = nand_get_controller_data(nand);
	char *ptr = (char *)(nand_info->reg+REG_NFI_NANDRA0);

	/* At first, read the OOB area  */
	ma35d1_nand_command(mtd, NAND_CMD_READOOB, 0, page);
	ma35d1_nand_read_buf(mtd, chip->oob_poi, mtd->oobsize);

	// Second, copy OOB data to SMRA for page read
	memcpy((void *)ptr, (void *)chip->oob_poi, mtd->oobsize);

	if ((*(ptr+2) != 0) && (*(ptr+3) != 0))
		memset((void *)chip->oob_poi, 0xff, mtd->oobsize);

	return 0;
}

int ma35d1_nand_init(struct ma35d1_nand_info *nand_info)
{
	struct nand_chip *nand = &nand_info->chip;
	struct mtd_info *mtd = nand_to_mtd(nand);
	unsigned int reg;
	int ret = 0;
#ifdef CONFIG_OF_LIVE
	ofnode node;
	const char *str;
#endif

	nand_set_controller_data(nand, nand_info);
	nand->options |= NAND_NO_SUBPAGE_WRITE;
#ifdef CONFIG_OF_LIVE
	node = dev_ofnode(nand_info->dev);
	if (ofnode_read_bool(node, "nand-on-flash-bbt"))
		nand->bbt_options |= NAND_BBT_USE_FLASH;
	nand->ecc.strength = ofnode_read_u32_default(node, "nand-ecc-strength", -1);
	nand->ecc.size  = ofnode_read_u32_default(node, "nand-ecc-step-size", -1);
	str = ofnode_read_string(node, "nand-ecc-mode");
	if (str) {
		if (!strcmp(str, "none"))
			nand->ecc.mode = NAND_ECC_NONE;
		else if (!strcmp(str, "soft"))
			nand->ecc.mode = NAND_ECC_SOFT;
		else if (!strcmp(str, "hw"))
			nand->ecc.mode = NAND_ECC_HW;
		else if (!strcmp(str, "hw_syndrome"))
			nand->ecc.mode = NAND_ECC_HW_SYNDROME;
		else if (!strcmp(str, "hw_oob_first"))
			nand->ecc.mode = NAND_ECC_HW_OOB_FIRST;
		else if (!strcmp(str, "soft_bch"))
			nand->ecc.mode = NAND_ECC_SOFT_BCH;
	}
#else
	nand->flash_node = dev_of_offset(nand_info->dev);
#endif

	/* hwcontrol always must be implemented */
	nand->cmd_ctrl = ma35d1_hwcontrol;
	nand->cmdfunc = ma35d1_nand_command;
	nand->waitfunc = ma35d1_waitfunc;
	nand->dev_ready = ma35d1_dev_ready;
	nand->select_chip = ma35d1_nand_select_chip;

	nand->read_byte = ma35d1_nand_read_byte;
	nand->write_buf = ma35d1_nand_write_buf;
	nand->read_buf = ma35d1_nand_read_buf;
	nand->chip_delay = 50;

	nand->ecc.mode       = NAND_ECC_HW_OOB_FIRST;
	nand->ecc.options    = NAND_ECC_CUSTOM_PAGE_ACCESS;
	nand->ecc.hwctl      = ma35d1_nand_enable_hwecc;
	nand->ecc.calculate  = ma35d1_nand_calculate_ecc;
	nand->ecc.correct    = ma35d1_nand_correct_data;
	nand->ecc.write_page = ma35d1_nand_write_page_hwecc;
	nand->ecc.read_page  = ma35d1_nand_read_page_hwecc_oob_first;
	nand->ecc.read_oob   = ma35d1_nand_read_oob_hwecc;
	nand->ecc.layout     = &ma35d1_nand_oob;
	nand->ecc.write_page_raw = ma35d1_nand_write_page_raw;
	nand->ecc.read_page_raw  = ma35d1_nand_read_page_raw;

	mtd->priv = nand;

	// Enable SM_EN
	writel(NAND_EN, nand_info->reg+REG_NFI_GCTL);

	// Enable SM_CS0
	writel((readl(nand_info->reg+REG_NFI_NANDCTL) & (~0x02000000))|0x4000000,
		nand_info->reg+REG_NFI_NANDCTL);
	writel(0x1, nand_info->reg+REG_NFI_NANDECTL); /* un-lock write protect */

	// NAND Reset
	writel(readl(nand_info->reg+REG_NFI_NANDCTL) | 0x1,
		nand_info->reg+REG_NFI_NANDCTL);    // software reset
	while (readl(nand_info->reg+REG_NFI_NANDCTL) & 0x1)
		;

	/* Detect NAND chips */
	ret = nand_scan(mtd, 1);
	if (ret)
		return ret;

	//Set PSize bits of SMCSR register to select NAND card page size
	reg = readl(nand_info->reg+REG_NFI_NANDCTL) & (~0x30000);
	if (mtd->writesize == 2048)
		writel(reg | 0x10000, nand_info->reg+REG_NFI_NANDCTL);
	else if (mtd->writesize == 4096)
		writel(reg | 0x20000, nand_info->reg+REG_NFI_NANDCTL);
	else if (mtd->writesize == 8192)
		writel(reg | 0x30000, nand_info->reg+REG_NFI_NANDCTL);

	if (nand->ecc.strength == 0) {
		nand_info->eBCHAlgo = 0; /* No ECC */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
			g_i32ParityNum[mtd->writesize>>12][nand_info->eBCHAlgo]);

	} else if (nand->ecc.strength <= 8) {
		nand_info->eBCHAlgo = 1; /* T8 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
			g_i32ParityNum[mtd->writesize>>12][nand_info->eBCHAlgo]);

	} else if (nand->ecc.strength <= 12) {
		nand_info->eBCHAlgo = 2; /* T12 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
			g_i32ParityNum[mtd->writesize>>12][nand_info->eBCHAlgo]);

	} else if (nand->ecc.strength <= 24) {
		nand_info->eBCHAlgo = 3; /* T24 */
		ma35d1_layout_oob_table(&ma35d1_nand_oob, mtd->oobsize,
			g_i32ParityNum[mtd->writesize>>12][nand_info->eBCHAlgo]);

	} else {
		pr_warn("NAND Controller is not support this flash. (%d, %d)\n",
			mtd->writesize, mtd->oobsize);
	}

	nand_info->m_i32SMRASize  = mtd->oobsize;
	nand->ecc.steps = mtd->writesize / nand->ecc.size;
	nand->ecc.bytes = ma35d1_nand_oob.eccbytes / nand->ecc.steps;
	nand->ecc.total = ma35d1_nand_oob.eccbytes;

	// Redundant area size
	writel(nand_info->m_i32SMRASize, nand_info->reg+REG_NFI_NANDRACTL);

	// Protect redundant 3 bytes
	// because we need to implement write_oob function to partial data to oob available area.
	// Please note we skip 4 bytes
	writel(readl(nand_info->reg+REG_NFI_NANDCTL) | 0x100, nand_info->reg+REG_NFI_NANDCTL);

	// To read/write the ECC parity codes automatically from/to NAND Flash after data area field written.
	writel(readl(nand_info->reg+REG_NFI_NANDCTL) | 0x10, nand_info->reg+REG_NFI_NANDCTL);
	// Set BCH algorithm
	writel((readl(nand_info->reg+REG_NFI_NANDCTL) & (~0x007C0000)) |
		g_i32BCHAlgoIdx[nand_info->eBCHAlgo], nand_info->reg+REG_NFI_NANDCTL);
	// Enable H/W ECC, ECC parity check enable bit during read page
	writel(readl(nand_info->reg+REG_NFI_NANDCTL) | 0x00800080, nand_info->reg+REG_NFI_NANDCTL);

	nand_register(0, mtd);

	return 0;
}

static int ma35d1_nand_probe(struct udevice *dev)
{
	struct ma35d1_nand_info *info = dev_get_priv(dev);
	struct resource res;
	int ret;

	info->dev = dev;

	/* get nand info */
	ret = dev_read_resource_byname(dev, "nand", &res);
	if (ret)
		return ret;

	info->reg = devm_ioremap(dev, res.start, resource_size(&res));

	return ma35d1_nand_init(info);
}

static const struct udevice_id ma35d1_nand_ids[] = {
	{ .compatible = "nuvoton,ma35d1-nand" },
	{ }
};

U_BOOT_DRIVER(ma35d1_nand) = {
	.name = "ma35d1-nand",
	.id = UCLASS_MTD,
	.of_match = ma35d1_nand_ids,
	.probe = ma35d1_nand_probe,
	.priv_auto_alloc_size = sizeof(struct ma35d1_nand_info),
};

void board_nand_init(void)
{
	struct udevice *dev;
	struct clk clk;
	int ret;

	/* enable nand clock */
	ret = uclass_get_device_by_driver(UCLASS_CLK, DM_GET_DRIVER(ma35d1_clk), &dev);
	if (ret)
		pr_err("Failed to get nand clock. (error %d)\n", ret);

	clk.id = nand_gate;
	ret = clk_request(dev, &clk);
	if (ret < 0)
		dev_err(dev, "%s clk_request() failed: %d\n", __func__, ret);

	ret = clk_enable(&clk);
	if (ret == -ENOTSUPP)
		dev_err(dev, "clk not supported yet\n");

	ret = uclass_get_device_by_driver(UCLASS_MTD,
					  DM_GET_DRIVER(ma35d1_nand),
					  &dev);
	if (ret && ret != -ENODEV)
		pr_err("Failed to initialize ma35d1 NAND controller. (error %d)\n", ret);
}
