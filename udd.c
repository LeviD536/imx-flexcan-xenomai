/*
 * This file is part of the Xenomai project.
 *
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <rtdm/cobalt.h>
#include <rtdm/driver.h>
#include <rtdm/udd.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <rtdm/can.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>


#define CANFD_BRS 0x01 /* bit rate switch (second bitrate for payload data) */
#define CANFD_ESI 0x02 /* error state indicator of the transmitting node */


#define DEVICE_NAME_0	"udd_can0"
#define DEVICE_NAME_1	"udd_can1"
#define DRV_NAME	"udd_can_fd"
#define RTDM_SUBCLASS_UDD_CAN	0

/* 8 for RX fifo and 2 error handling */
#define FLEXCAN_NAPI_WEIGHT		(8 + 2)

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS		BIT(31)
#define FLEXCAN_MCR_FRZ			BIT(30)
#define FLEXCAN_MCR_FEN			BIT(29)
#define FLEXCAN_MCR_HALT		BIT(28)
#define FLEXCAN_MCR_NOT_RDY		BIT(27)
#define FLEXCAN_MCR_WAK_MSK		BIT(26)
#define FLEXCAN_MCR_SOFTRST		BIT(25)
#define FLEXCAN_MCR_FRZ_ACK		BIT(24)
#define FLEXCAN_MCR_SUPV		BIT(23)
#define FLEXCAN_MCR_SLF_WAK		BIT(22)
#define FLEXCAN_MCR_WRN_EN		BIT(21)
#define FLEXCAN_MCR_LPM_ACK		BIT(20)
#define FLEXCAN_MCR_WAK_SRC		BIT(19)
#define FLEXCAN_MCR_DOZE		BIT(18)
#define FLEXCAN_MCR_SRX_DIS		BIT(17)
#define FLEXCAN_MCR_IRMQ		BIT(16)
#define FLEXCAN_MCR_LPRIO_EN		BIT(13)
#define FLEXCAN_MCR_AEN			BIT(12)
#define FLEXCAN_MCR_FDEN		BIT(11)
/* MCR_MAXMB: maximum used MBs is MAXMB + 1 */
#define FLEXCAN_MCR_MAXMB(x)		((x) & 0x7f)
#define FLEXCAN_MCR_IDAM_A		(0x0 << 8)
#define FLEXCAN_MCR_IDAM_B		(0x1 << 8)
#define FLEXCAN_MCR_IDAM_C		(0x2 << 8)
#define FLEXCAN_MCR_IDAM_D		(0x3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x)		(((x) & 0xff) << 24)
#define FLEXCAN_CTRL_RJW(x)		(((x) & 0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x)		(((x) & 0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x)		(((x) & 0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK		BIT(15)
#define FLEXCAN_CTRL_ERR_MSK		BIT(14)
#define FLEXCAN_CTRL_CLK_SRC		BIT(13)
#define FLEXCAN_CTRL_LPB		BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK		BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK		BIT(10)
#define FLEXCAN_CTRL_SMP		BIT(7)
#define FLEXCAN_CTRL_BOFF_REC		BIT(6)
#define FLEXCAN_CTRL_TSYN		BIT(5)
#define FLEXCAN_CTRL_LBUF		BIT(4)
#define FLEXCAN_CTRL_LOM		BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x)		((x) & 0x07)
#define FLEXCAN_CTRL_ERR_BUS		(FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE \
	(FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | \
	 FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL \
	(FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

/* FLEXCAN control register 2 (CTRL2) bits */
#define FLEXCAN_CTRL2_ECRWRE		BIT(29)
#define FLEXCAN_CTRL2_WRMFRZ		BIT(28)
#define FLEXCAN_CTRL2_RFFN(x)		(((x) & 0x0f) << 24)
#define FLEXCAN_CTRL2_TASD(x)		(((x) & 0x1f) << 19)
#define FLEXCAN_CTRL2_MRP		BIT(18)
#define FLEXCAN_CTRL2_RRS		BIT(17)
#define FLEXCAN_CTRL2_EACEN		BIT(16)
#define FLEXCAN_CTRL2_ISOCANFDEN	BIT(12)

/* FLEXCAN memory error control register (MECR) bits */
#define FLEXCAN_MECR_ECRWRDIS		BIT(31)
#define FLEXCAN_MECR_HANCEI_MSK		BIT(19)
#define FLEXCAN_MECR_FANCEI_MSK		BIT(18)
#define FLEXCAN_MECR_CEI_MSK		BIT(16)
#define FLEXCAN_MECR_HAERRIE		BIT(15)
#define FLEXCAN_MECR_FAERRIE		BIT(14)
#define FLEXCAN_MECR_EXTERRIE		BIT(13)
#define FLEXCAN_MECR_RERRDIS		BIT(9)
#define FLEXCAN_MECR_ECCDIS		BIT(8)
#define FLEXCAN_MECR_NCEFAFRZ		BIT(7)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT		BIT(17)
#define FLEXCAN_ESR_RWRN_INT		BIT(16)
#define FLEXCAN_ESR_BIT1_ERR		BIT(15)
#define FLEXCAN_ESR_BIT0_ERR		BIT(14)
#define FLEXCAN_ESR_ACK_ERR		BIT(13)
#define FLEXCAN_ESR_CRC_ERR		BIT(12)
#define FLEXCAN_ESR_FRM_ERR		BIT(11)
#define FLEXCAN_ESR_STF_ERR		BIT(10)
#define FLEXCAN_ESR_TX_WRN		BIT(9)
#define FLEXCAN_ESR_RX_WRN		BIT(8)
#define FLEXCAN_ESR_IDLE		BIT(7)
#define FLEXCAN_ESR_TXRX		BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT	(4)
#define FLEXCAN_ESR_FLT_CONF_MASK	(0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE	(0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE	(0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT		BIT(2)
#define FLEXCAN_ESR_ERR_INT		BIT(1)
#define FLEXCAN_ESR_WAK_INT		BIT(0)
#define FLEXCAN_ESR_ERR_BUS \
	(FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | \
	 FLEXCAN_ESR_ACK_ERR | FLEXCAN_ESR_CRC_ERR | \
	 FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL \
	(FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)
#define FLEXCAN_ESR_ALL_INT \
	(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | \
	 FLEXCAN_ESR_BOFF_INT | FLEXCAN_ESR_ERR_INT)

/* FLEXCAN Bit Timing register (CBT) bits */
#define FLEXCAN_CBT_BTF			BIT(31)
#define FLEXCAN_CBT_EPRESDIV(x)		(((x) & 0x3ff) << 21)
#define FLEXCAN_CBT_ERJW(x)		(((x) & 0x0f) << 16)
#define FLEXCAN_CBT_EPROPSEG(x)		(((x) & 0x3f) << 10)
#define FLEXCAN_CBT_EPSEG1(x)		(((x) & 0x1f) << 5)
#define FLEXCAN_CBT_EPSEG2(x)		((x) & 0x1f)

/* FLEXCAN FD control register (FDCTRL) bits */
#define FLEXCAN_FDCTRL_FDRATE		BIT(31)
#define FLEXCAN_FDCTRL_TDCEN		BIT(15)
#define FLEXCAN_FDCTRL_TDCFAIL		BIT(14)
#define FLEXCAN_FDCTRL_MBDSR1(x)	(((x) & 0x3) << 19)
#define FLEXCAN_FDCTRL_MBDSR0(x)	(((x) & 0x3) << 16)
#define FLEXCAN_FDCTRL_TDCOFF(x)	(((x) & 0x1f) << 8)

/* FLEXCAN FD Bit Timing register (FDCBT) bits */
#define FLEXCAN_FDCBT_FPRESDIV(x)	(((x) & 0x3ff) << 20)
#define FLEXCAN_FDCBT_FRJW(x)		(((x) & 0x07) << 16)
#define FLEXCAN_FDCBT_FPROPSEG(x)	(((x) & 0x1f) << 10)
#define FLEXCAN_FDCBT_FPSEG1(x)		(((x) & 0x07) << 5)
#define FLEXCAN_FDCBT_FPSEG2(x)		((x) & 0x07)

/* FLEXCAN interrupt flag register (IFLAG) bits */
/* Errata ERR005829 step7: Reserve first valid MB */
#define FLEXCAN_TX_MB_RESERVED_OFF_FIFO		8
#define FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP	0
#define FLEXCAN_RX_MB_OFF_TIMESTAMP_FIRST	(FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP + 1)
#define FLEXCAN_IFLAG_MB(x)		BIT_ULL(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW	BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN	BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE	BIT(5)

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CODE_MASK		(0xf << 24)
#define FLEXCAN_MB_CODE_RX_BUSY_BIT	(0x1 << 24)
#define FLEXCAN_MB_CODE_RX_INACTIVE	(0x0 << 24)
#define FLEXCAN_MB_CODE_RX_EMPTY	(0x4 << 24)
#define FLEXCAN_MB_CODE_RX_FULL		(0x2 << 24)
#define FLEXCAN_MB_CODE_RX_OVERRUN	(0x6 << 24)
#define FLEXCAN_MB_CODE_RX_RANSWER	(0xa << 24)

#define FLEXCAN_MB_CODE_TX_INACTIVE	(0x8 << 24)
#define FLEXCAN_MB_CODE_TX_ABORT	(0x9 << 24)
#define FLEXCAN_MB_CODE_TX_DATA		(0xc << 24)
#define FLEXCAN_MB_CODE_TX_TANSWER	(0xe << 24)

#define FLEXCAN_MB_CNT_EDL		BIT(31)
#define FLEXCAN_MB_CNT_BRS		BIT(30)
#define FLEXCAN_MB_CNT_ESI		BIT(29)
#define FLEXCAN_MB_CNT_SRR		BIT(22)
#define FLEXCAN_MB_CNT_IDE		BIT(21)
#define FLEXCAN_MB_CNT_RTR		BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)	(((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)	((x) & 0xffff)

#define FLEXCAN_TIMEOUT_US		(250)

/* FLEXCAN hardware feature flags
 *
 * Below is some version info we got:
 *    SOC   Version   IP-Version  Glitch- [TR]WRN_INT IRQ Err Memory err RTR rece-   FD Mode
 *                                Filter? connected?  Passive detection  ption in MB Supported?
 *   MX25  FlexCAN2  03.00.00.00     no        no        no       no        no           no
 *   MX28  FlexCAN2  03.00.04.00    yes       yes        no       no        no           no
 *   MX35  FlexCAN2  03.00.00.00     no        no        no       no        no           no
 *   MX53  FlexCAN2  03.00.00.00    yes        no        no       no        no           no
 *   MX6s  FlexCAN3  10.00.12.00    yes       yes        no       no       yes           no
 *  MX8QM  FlexCAN3  03.00.23.00    yes       yes        no       no       yes          yes
 *   VF610 FlexCAN3  ?               no       yes        no      yes       yes?          no
 * LS1021A FlexCAN2  03.00.04.00     no       yes        no       no       yes           no
 * LX2160A FlexCAN3  03.00.23.00     no       yes        no       no       yes          yes
 *
 * Some SOCs do not have the RX_WARN & TX_WARN interrupt line connected.
 */
#define FLEXCAN_QUIRK_BROKEN_WERR_STATE	BIT(1) /* [TR]WRN_INT not connected */
#define FLEXCAN_QUIRK_DISABLE_RXFG	BIT(2) /* Disable RX FIFO Global mask */
#define FLEXCAN_QUIRK_ENABLE_EACEN_RRS	BIT(3) /* Enable EACEN and RRS bit in ctrl2 */
#define FLEXCAN_QUIRK_DISABLE_MECR	BIT(4) /* Disable non-correctable errors interrupt and freeze mode */
#define FLEXCAN_QUIRK_USE_OFF_TIMESTAMP	BIT(5) /* Use timestamp based offloading */
#define FLEXCAN_QUIRK_BROKEN_PERR_STATE	BIT(6) /* No interrupt for error passive */
#define FLEXCAN_QUIRK_DEFAULT_BIG_ENDIAN	BIT(7) /* default to BE register access */
#define FLEXCAN_QUIRK_SETUP_STOP_MODE		BIT(8) /* Setup stop mode to support wakeup */
#define FLEXCAN_QUIRK_TIMESTAMP_SUPPORT_FD	BIT(9) /* Use timestamp then support can fd mode */
#define FLEXCAN_QUIRK_USE_SCFW			BIT(10) /* Use System Controller Firmware */

#define rtcan_priv(dev) (dev)->priv

static const u8 len2dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,		/* 0 - 8 */
			     9, 9, 9, 9,			/* 9 - 12 */
			     10, 10, 10, 10,			/* 13 - 16 */
			     11, 11, 11, 11,			/* 17 - 20 */
			     12, 12, 12, 12,			/* 21 - 24 */
			     13, 13, 13, 13, 13, 13, 13, 13,	/* 25 - 32 */
			     14, 14, 14, 14, 14, 14, 14, 14,	/* 33 - 40 */
			     14, 14, 14, 14, 14, 14, 14, 14,	/* 41 - 48 */
			     15, 15, 15, 15, 15, 15, 15, 15,	/* 49 - 56 */
			     15, 15, 15, 15, 15, 15, 15, 15};	/* 57 - 64 */

static const u8 dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7,
			     8, 12, 16, 20, 24, 32, 48, 64};

/* Structure of the message buffer */
struct flexcan_mb {
	u32 can_ctrl;
	u32 can_id;
	u32 data[];
};

/* Structure of the hardware registers */
struct flexcan_regs {
	u32 mcr;		/* 0x00 */
	u32 ctrl;		/* 0x04 */
	u32 timer;		/* 0x08 */
	u32 _reserved1;		/* 0x0c */
	u32 rxgmask;		/* 0x10 */
	u32 rx14mask;		/* 0x14 */
	u32 rx15mask;		/* 0x18 */
	u32 ecr;		/* 0x1c */
	u32 esr;		/* 0x20 */
	u32 imask2;		/* 0x24 */
	u32 imask1;		/* 0x28 */
	u32 iflag2;		/* 0x2c */
	u32 iflag1;		/* 0x30 */
	union {			/* 0x34 */
		u32 gfwr_mx28;	/* MX28, MX53 */
		u32 ctrl2;	/* MX6, VF610 */
	};
	u32 esr2;		/* 0x38 */
	u32 imeur;		/* 0x3c */
	u32 lrfr;		/* 0x40 */
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c */
	u32 cbt;		/* 0x50 */
	u32 _reserved3[11];	/* 0x54 */
	u8 mb[2][512];		/* 0x80 */
	/* FIFO-mode:
	 *			MB
	 * 0x080...0x08f	0	RX message buffer
	 * 0x090...0x0df	1-5	reserverd
	 * 0x0e0...0x0ff	6-7	8 entry ID table
	 *				(mx25, mx28, mx35, mx53)
	 * 0x0e0...0x2df	6-7..37	8..128 entry ID table
	 *				size conf'ed via ctrl2::RFFN
	 *				(mx6, vf610)
	 */
	u32 _reserved4[256];	/* 0x480 */
	u32 rximr[64];		/* 0x880 */
	u32 _reserved5[24];	/* 0x980 */
	u32 gfwr_mx6;		/* 0x9e0 - MX6 */
	u32 _reserved6[39];	/* 0x9e4 */
	u32 _rxfir[6];		/* 0xa80 */
	u32 _reserved8[2];	/* 0xa98 */
	u32 _rxmgmask;		/* 0xaa0 */
	u32 _rxfgmask;		/* 0xaa4 */
	u32 _rx14mask;		/* 0xaa8 */
	u32 _rx15mask;		/* 0xaac */
	u32 tx_smb[4];		/* 0xab0 */
	u32 rx_smb0[4];		/* 0xac0 */
	u32 rx_smb1[4];		/* 0xad0 */
	u32 mecr;		/* 0xae0 */
	u32 erriar;		/* 0xae4 */
	u32 erridpr;		/* 0xae8 */
	u32 errippr;		/* 0xaec */
	u32 rerrar;		/* 0xaf0 */
	u32 rerrdr;		/* 0xaf4 */
	u32 rerrsynr;		/* 0xaf8 */
	u32 errsr;		/* 0xafc */
	u32 _reserved7[64];	/* 0xb00 */
	u32 fdctrl;		/* 0xc00 */
	u32 fdcbt;		/* 0xc04 */
	u32 fdcrc;		/* 0xc08 */
	u32 _reserved9[199];	/* 0xc0c */
	u32 tx_smb_fd[18];	/* 0xf28 */
	u32 rx_smb0_fd[18];	/* 0xf70 */
	u32 rx_smb1_fd[18];	/* 0xfb8 */
};

struct flexcan_devtype_data {
	u32 quirks;		/* quirks needed for different IP cores */
};

struct flexcan_stop_mode {
	struct regmap *gpr;
	u8 req_gpr;
	u8 req_bit;
	u8 ack_gpr;
	u8 ack_bit;
};

/*
 * CAN hardware-dependent bit-timing constant
 *
 * Used for calculating and checking bit-timing parameters
 */
struct can_bittiming_const {
	char name[16];		/* Name of the CAN controller hardware */
	__u32 tseg1_min;	/* Time segment 1 = prop_seg + phase_seg1 */
	__u32 tseg1_max;
	__u32 tseg2_min;	/* Time segment 2 = phase_seg2 */
	__u32 tseg2_max;
	__u32 sjw_max;		/* Synchronisation jump width */
	__u32 brp_min;		/* Bit-rate prescaler */
	__u32 brp_max;
	__u32 brp_inc;
};

/*
 * CAN bit-timing parameters
 *
 * For further information, please read chapter "8 BIT TIMING
 * REQUIREMENTS" of the "Bosch CAN Specification version 2.0"
 * at http://www.semiconductors.bosch.de/pdf/can2spec.pdf.
 */
struct can_bittiming {
	__u32 bitrate;		/* Bit-rate in bits/second */
	__u32 sample_point;	/* Sample point in one-tenth of a percent */
	__u32 tq;		/* Time quanta (TQ) in nanoseconds */
	__u32 prop_seg;		/* Propagation segment in TQs */
	__u32 phase_seg1;	/* Phase buffer segment 1 in TQs */
	__u32 phase_seg2;	/* Phase buffer segment 2 in TQs */
	__u32 sjw;		/* Synchronisation jump width in TQs */
	__u32 brp;		/* Bit-rate prescaler */
};

/*
 * CAN device statistics
 */
struct can_device_stats {
	__u32 bus_error;	/* Bus errors */
	__u32 error_warning;	/* Changes to error warning state */
	__u32 error_passive;	/* Changes to error passive state */
	__u32 bus_off;		/* Changes to bus off state */
	__u32 arbitration_lost; /* Arbitration lost errors */
	__u32 restarts;		/* CAN controller re-starts */
};

/*
 * CAN clock parameters
 */
struct can_clock {
	__u32 freq;		/* CAN system clock frequency in Hz */
};

/*
 * CAN common private data
 */
struct can_priv {
	//struct net_device *dev;
	struct can_device_stats can_stats;

	struct can_bittiming bittiming, data_bittiming;
	const struct can_bittiming_const *bittiming_const,
		*data_bittiming_const;
	const u16 *termination_const;
	unsigned int termination_const_cnt;
	u16 termination;
	const u32 *bitrate_const;
	unsigned int bitrate_const_cnt;
	const u32 *data_bitrate_const;
	unsigned int data_bitrate_const_cnt;
	u32 bitrate_max;
	struct can_clock clock;

	//enum can_state state;

	/* CAN controller features - see include/uapi/linux/can/netlink.h */
	u32 ctrlmode;		/* current options setting */
	u32 ctrlmode_supported;	/* options that can be modified by netlink */
	u32 ctrlmode_static;	/* static enabled options for driver/hardware */

	int restart_ms;
	struct delayed_work restart_work;
	
	/*
	int (*do_set_bittiming)(struct rtcan_device *dev);
	int (*do_set_data_bittiming)(struct rtcan_device *dev);
	int (*do_set_mode)(struct rtcan_device *dev, enum can_mode mode);
	int (*do_set_termination)(struct rtcan_device *dev, u16 term);
	int (*do_get_state)(const struct rtcan_device *dev,
			    enum can_state *state);
	int (*do_get_berr_counter)(const struct rtcan_device *dev,
				   struct can_berr_counter *bec);
	*/
};

struct flexcan_priv {
	struct can_priv can;
	//struct can_rx_offload offload;
	struct device *dev;

	struct flexcan_regs __iomem *regs;
	struct flexcan_mb __iomem *tx_mb;
	struct flexcan_mb __iomem *tx_mb_reserved;
	u8 tx_mb_idx;
	unsigned int mb_first;
	unsigned int mb_last;
	u8 mb_count;
	u8 mb_size;
	u8 clk_src;	/* clock source of CAN Protocol Engine */

	u64 rx_mask;
	u64 tx_mask;
	u32 reg_ctrl_default;
	u64 iflags_rx;

	struct clk *clk_ipg;
	struct clk *clk_per;
	const struct flexcan_devtype_data *devtype_data;
	struct regulator *reg_xceiver;
	struct flexcan_stop_mode stm;

	/* Read and Write APIs */
	u32 (*read)(void __iomem *addr);
	void (*write)(u32 val, void __iomem *addr);

	unsigned int irq;
};

struct rtcan_device {
	struct udd_device udd_dev;

	void                *priv;      /* pointer to chip private data */

	rtdm_irq_t          irq_handle; /* RTDM IRQ handle */
	
	/* Acts as a mutex allowing only one sender to write to the MSCAN
    	* simultaneously. Created when the controller goes into operating mode,
     	* destroyed if it goes into reset mode. */
    	rtdm_sem_t          tx_sem;

    	/* Spinlock for all devices (but not for all attributes) and also for HW
     	* access to all CAN controllers
     	*/
    	rtdm_lock_t         device_lock;
};



static const struct can_bittiming_const flexcan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

static const struct can_bittiming_const flexcan_fd_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 96,
	.tseg2_min = 2,
	.tseg2_max = 32,
	.sjw_max = 16,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static const struct can_bittiming_const flexcan_fd_data_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 39,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 1024,
	.brp_inc = 1,
};

static struct flexcan_devtype_data fsl_imx8mp_devtype_data_fd = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
		FLEXCAN_QUIRK_USE_OFF_TIMESTAMP | FLEXCAN_QUIRK_BROKEN_PERR_STATE |
		FLEXCAN_QUIRK_TIMESTAMP_SUPPORT_FD | FLEXCAN_QUIRK_SETUP_STOP_MODE |
		FLEXCAN_QUIRK_DISABLE_MECR,
};

static struct flexcan_devtype_data fsl_imx8mp_devtype_data = {
	.quirks = FLEXCAN_QUIRK_DISABLE_RXFG | FLEXCAN_QUIRK_ENABLE_EACEN_RRS |
		FLEXCAN_QUIRK_USE_OFF_TIMESTAMP | FLEXCAN_QUIRK_BROKEN_PERR_STATE |
		FLEXCAN_QUIRK_DISABLE_MECR,
};

/* Bit-timing calculation derived from:
 *
 * Code based on LinCAN sources and H8S2638 project
 * Copyright 2004-2006 Pavel Pisa - DCE FELK CVUT cz
 * Copyright 2005      Stanislav Marek
 * email: pisa@cmp.felk.cvut.cz
 *
 * Calculates proper bit-timing parameters for a specified bit-rate
 * and sample-point, which can then be used to set the bit-timing
 * registers of the CAN controller. You can find more information
 * in the header file linux/can/netlink.h.
 */

#define CAN_CALC_MAX_ERROR 50 /* in one-tenth of a percent */
#define CAN_CALC_SYNC_SEG 1

static int can_update_sample_point(const struct can_bittiming_const *btc,
			unsigned int sample_point_nominal, unsigned int tseg,
			unsigned int *tseg1_ptr, unsigned int *tseg2_ptr,
			unsigned int *sample_point_error_ptr)
{
	unsigned int sample_point_error, best_sample_point_error = UINT_MAX;
	unsigned int sample_point, best_sample_point = 0;
	unsigned int tseg1, tseg2;
	int i;

	for (i = 0; i <= 1; i++) {
		tseg2 = tseg + CAN_CALC_SYNC_SEG -
			(sample_point_nominal * (tseg + CAN_CALC_SYNC_SEG)) /
			1000 - i;
		tseg2 = clamp(tseg2, btc->tseg2_min, btc->tseg2_max);
		tseg1 = tseg - tseg2;
		if (tseg1 > btc->tseg1_max) {
			tseg1 = btc->tseg1_max;
			tseg2 = tseg - tseg1;
		}

		sample_point = 1000 * (tseg + CAN_CALC_SYNC_SEG - tseg2) /
			(tseg + CAN_CALC_SYNC_SEG);
		sample_point_error = abs(sample_point_nominal - sample_point);

		if (sample_point <= sample_point_nominal &&
		    sample_point_error < best_sample_point_error) {
			best_sample_point = sample_point;
			best_sample_point_error = sample_point_error;
			*tseg1_ptr = tseg1;
			*tseg2_ptr = tseg2;
		}
	}

	if (sample_point_error_ptr)
		*sample_point_error_ptr = best_sample_point_error;

	return best_sample_point;
}

static int can_calc_bittiming(struct rtcan_device *dev, struct can_bittiming *bt,
			      const struct can_bittiming_const *btc)
{
	struct flexcan_priv *fcpriv = rtcan_priv(dev);	
	struct can_priv *priv = &fcpriv->can;
	unsigned int bitrate;			/* current bitrate */
	unsigned int bitrate_error;		/* difference between current and nominal value */
	unsigned int best_bitrate_error = UINT_MAX;
	unsigned int sample_point_error;	/* difference between current and nominal value */
	unsigned int best_sample_point_error = UINT_MAX;
	unsigned int sample_point_nominal;	/* nominal sample point */
	unsigned int best_tseg = 0;		/* current best value for tseg */
	unsigned int best_brp = 0;		/* current best value for brp */
	unsigned int brp, tsegall, tseg, tseg1 = 0, tseg2 = 0;
	u64 v64;

	/* Use CiA recommended sample points */
	if (bt->sample_point) {
		sample_point_nominal = bt->sample_point;
	} else {
		if (bt->bitrate > 800000)
			sample_point_nominal = 750;
		else if (bt->bitrate > 500000)
			sample_point_nominal = 800;
		else
			sample_point_nominal = 875;
	}

	/* tseg even = round down, odd = round up */
	for (tseg = (btc->tseg1_max + btc->tseg2_max) * 2 + 1;
	     tseg >= (btc->tseg1_min + btc->tseg2_min) * 2; tseg--) {
		tsegall = CAN_CALC_SYNC_SEG + tseg / 2;

		/* Compute all possible tseg choices (tseg=tseg1+tseg2) */
		brp = priv->clock.freq / (tsegall * bt->bitrate) + tseg % 2;

		/* choose brp step which is possible in system */
		brp = (brp / btc->brp_inc) * btc->brp_inc;
		if (brp < btc->brp_min || brp > btc->brp_max)
			continue;

		bitrate = priv->clock.freq / (brp * tsegall);
		bitrate_error = abs(bt->bitrate - bitrate);

		/* tseg brp biterror */
		if (bitrate_error > best_bitrate_error)
			continue;

		/* reset sample point error if we have a better bitrate */
		if (bitrate_error < best_bitrate_error)
			best_sample_point_error = UINT_MAX;

		can_update_sample_point(btc, sample_point_nominal, tseg / 2,
					&tseg1, &tseg2, &sample_point_error);
		if (sample_point_error > best_sample_point_error)
			continue;

		best_sample_point_error = sample_point_error;
		best_bitrate_error = bitrate_error;
		best_tseg = tseg / 2;
		best_brp = brp;

		if (bitrate_error == 0 && sample_point_error == 0)
			break;
	}

	if (best_bitrate_error) {
		/* Error in one-tenth of a percent */
		v64 = (u64)best_bitrate_error * 1000;
		do_div(v64, bt->bitrate);
		bitrate_error = (u32)v64;
		if (bitrate_error > CAN_CALC_MAX_ERROR) {
			printk("bitrate error %d.%d%% too high\n",
				   bitrate_error / 10, bitrate_error % 10);
			return -EDOM;
		}
		printk("bitrate error %d.%d%%\n",
			    bitrate_error / 10, bitrate_error % 10);
	}

	/* real sample point */
	bt->sample_point = can_update_sample_point(btc, sample_point_nominal,
						   best_tseg, &tseg1, &tseg2,
						   NULL);

	v64 = (u64)best_brp * 1000 * 1000 * 1000;
	do_div(v64, priv->clock.freq);
	bt->tq = (u32)v64;
	bt->prop_seg = tseg1 / 2;
	bt->phase_seg1 = tseg1 - bt->prop_seg;
	bt->phase_seg2 = tseg2;

	/* check for sjw user settings */
	if (!bt->sjw || !btc->sjw_max) {
		bt->sjw = 1;
	} else {
		/* bt->sjw is at least 1 -> sanitize upper bound to sjw_max */
		if (bt->sjw > btc->sjw_max)
			bt->sjw = btc->sjw_max;
		/* bt->sjw must not be higher than tseg2 */
		if (tseg2 < bt->sjw)
			bt->sjw = tseg2;
	}

	bt->brp = best_brp;

	/* real bitrate */
	bt->bitrate = priv->clock.freq /
		(bt->brp * (CAN_CALC_SYNC_SEG + tseg1 + tseg2));

	return 0;
}

struct rtcan_device *canfd_dev_alloc(int sizeof_priv, int sizeof_board_priv)
{
    struct rtcan_device *dev;
    int alloc_size;

    alloc_size = sizeof(*dev) + sizeof_priv + sizeof_board_priv;

    dev = (struct rtcan_device *)kmalloc(alloc_size, GFP_KERNEL);
    if (dev == NULL) {
	printk(KERN_ERR "rtcan: cannot allocate rtcan device\n");
	return NULL;
    }

    memset(dev, 0, alloc_size);

    rtdm_lock_init(&dev->device_lock);

    /* Init TX Semaphore, will be destroyed forthwith
     * when setting stop mode */
    rtdm_sem_init(&dev->tx_sem, 0);

    if (sizeof_priv)
	dev->priv = (void *)((unsigned long)dev + sizeof(*dev));

    return dev;

}

static inline bool flexcan_rx_le(struct flexcan_priv *priv, unsigned int a, unsigned int b)
{
	if (priv->mb_first < priv->mb_last)
		return a <= b;

	return a >= b;
}

static inline unsigned int flexcan_rx_inc(struct flexcan_priv *priv, unsigned int *val)
{
	if (priv->mb_first < priv->mb_last)
		return (*val)++;

	return (*val)--;
}


static inline u32 flexcan_read(void __iomem *addr)
{
	return readl(addr);
}

static inline void flexcan_write(u32 val, void __iomem *addr)
{
	writel(val, addr);
}

static inline u64 flexcan_read64_mask(struct flexcan_priv *priv, void __iomem *addr, u64 mask)
{
	u64 reg = 0;

	if (upper_32_bits(mask))
		reg = (u64)priv->read(addr - 4) << 32;
	if (lower_32_bits(mask))
		reg |= priv->read(addr);

	return reg & mask;
}

static inline void flexcan_write64(struct flexcan_priv *priv, u64 val, void __iomem *addr)
{
	if (upper_32_bits(val))
		priv->write(upper_32_bits(val), addr - 4);
	if (lower_32_bits(val))
		priv->write(lower_32_bits(val), addr);
}

static inline u64 flexcan_read_reg_iflag_rx(struct flexcan_priv *priv)
{
	return flexcan_read64_mask(priv, &priv->regs->iflag1, priv->rx_mask);
}

static inline u64 flexcan_read_reg_iflag_tx(struct flexcan_priv *priv)
{
	return flexcan_read64_mask(priv, &priv->regs->iflag1, priv->tx_mask);
}

static struct flexcan_mb __iomem *flexcan_get_mb(const struct flexcan_priv *priv,
						 u8 mb_index)
{
	u8 bank_size;
	bool bank;

	if (WARN_ON(mb_index >= priv->mb_count))
		return NULL;

	bank_size = sizeof(priv->regs->mb[0]) / priv->mb_size;

	bank = mb_index >= bank_size;
	if (bank)
		mb_index -= bank_size;

	return (struct flexcan_mb __iomem *)
		(&priv->regs->mb[bank][priv->mb_size * mb_index]);
}

static int flexcan_low_power_enter_ack(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;

	while (timeout-- && !(priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (!(priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_low_power_exit_ack(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;

	while (timeout-- && (priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK))
		udelay(10);

	if (priv->read(&regs->mcr) & FLEXCAN_MCR_LPM_ACK)
		return -ETIMEDOUT;

	return 0;
}

static inline int flexcan_transceiver_enable(const struct flexcan_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_enable(priv->reg_xceiver);
}

static inline int flexcan_transceiver_disable(const struct flexcan_priv *priv)
{
	if (!priv->reg_xceiver)
		return 0;

	return regulator_disable(priv->reg_xceiver);
}

static int flexcan_chip_enable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg &= ~FLEXCAN_MCR_MDIS;
	priv->write(reg, &regs->mcr);

	return flexcan_low_power_exit_ack(priv);
}

static int flexcan_chip_disable(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS;
	priv->write(reg, &regs->mcr);

	return flexcan_low_power_enter_ack(priv);
}

static int flexcan_chip_softreset(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;

	priv->write(FLEXCAN_MCR_SOFTRST, &regs->mcr);
	while (timeout-- && (priv->read(&regs->mcr) & FLEXCAN_MCR_SOFTRST))
		udelay(10);
	
	if (priv->read(&regs->mcr) & FLEXCAN_MCR_SOFTRST)
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_freeze(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = 1000 * 1000 * 10 / priv->can.bittiming.bitrate;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg |= FLEXCAN_MCR_HALT;
	priv->write(reg, &regs->mcr);

	while (timeout-- && !(priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		udelay(100);

	if (!(priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		return -ETIMEDOUT;

	return 0;
}

static int flexcan_chip_unfreeze(struct flexcan_priv *priv)
{
	struct flexcan_regs __iomem *regs = priv->regs;
	unsigned int timeout = FLEXCAN_TIMEOUT_US / 10;
	u32 reg;

	reg = priv->read(&regs->mcr);
	reg &= ~FLEXCAN_MCR_HALT;
	priv->write(reg, &regs->mcr);

	while (timeout-- && (priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK))
		udelay(10);

	if (priv->read(&regs->mcr) & FLEXCAN_MCR_FRZ_ACK)
		return -ETIMEDOUT;

	return 0;
}

static void flexcan_init_ram(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_ctrl2;
	int i, size;

	/* CTRL2[WRMFRZ] grants write access to all memory positions that
	 * require initialization. MCR[RFEN] must not be set during FlexCAN
	 * memory initialization.
	 */
	reg_ctrl2 = priv->read(&regs->ctrl2);
	reg_ctrl2 |= FLEXCAN_CTRL2_WRMFRZ;
	priv->write(reg_ctrl2, &regs->ctrl2);

	/* initialize MBs RAM */
	size = sizeof(regs->mb) / sizeof(u32);
	for (i = 0; i < size; i++)
		priv->write(0, &regs->mb[0][0] + sizeof(u32) * i);

	/* initialize RXIMRs RAM */
	size = sizeof(regs->rximr) / sizeof(u32);
	for (i = 0; i < size; i++)
		priv->write(0, &regs->rximr[i]);

	/* initialize RXFIRs RAM */
	size = sizeof(regs->_rxfir) / sizeof(u32);
	for (i = 0; i < size; i++)
		priv->write(0, &regs->_rxfir[i]);

	/* initialize RXMGMASK, RXFGMASK, RX14MASK, RX15MASK RAM */
	priv->write(0, &regs->_rxmgmask);
	priv->write(0, &regs->_rxfgmask);
	priv->write(0, &regs->_rx14mask);
	priv->write(0, &regs->_rx15mask);

	/* initialize TX_SMB RAM */
	size = sizeof(regs->tx_smb) / sizeof(u32);
	for (i = 0; i < size; i++)
		priv->write(0, &regs->tx_smb[i]);

	/* initialize RX_SMB0 RAM */
	size = sizeof(regs->rx_smb0) / sizeof(u32);
	for (i = 0; i < size; i++)
		priv->write(0, &regs->rx_smb0[i]);

	/* initialize RX_SMB1 RAM */
	size = sizeof(regs->rx_smb1) / sizeof(u32);
	for (i = 0; i < size; i++)
		priv->write(0, &regs->rx_smb1[i]);

	if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
		/* initialize TX_SMB_FD RAM */
		size = sizeof(regs->tx_smb_fd) / sizeof(u32);
		for (i = 0; i < size; i++)
			priv->write(0, &regs->tx_smb_fd[i]);

		/* initialize RX_SMB0_FD RAM */
		size = sizeof(regs->rx_smb0_fd) / sizeof(u32);
		for (i = 0; i < size; i++)
			priv->write(0, &regs->rx_smb0_fd[i]);

		/* initialize RX_SMB1_FD RAM */
		size = sizeof(regs->rx_smb1_fd) / sizeof(u32);
		for (i = 0; i < size; i++)
			priv->write(0, &regs->rx_smb0_fd[i]);
	}

	reg_ctrl2 &= ~FLEXCAN_CTRL2_WRMFRZ;
	priv->write(reg_ctrl2, &regs->ctrl2);
}

static void flexcan_set_bittiming(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct can_bittiming *dbt = &priv->can.data_bittiming;
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg, reg_cbt, reg_fdcbt, reg_fdctrl;

	reg = priv->read(&regs->ctrl);
	reg &= ~(FLEXCAN_CTRL_LPB | FLEXCAN_CTRL_SMP | FLEXCAN_CTRL_LOM);
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		reg |= FLEXCAN_CTRL_LPB;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		reg |= FLEXCAN_CTRL_LOM;
	if (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES)
		reg |= FLEXCAN_CTRL_SMP;

	printk("writing ctrl=0x%08x\n", reg);
	priv->write(reg, &regs->ctrl);

	if (priv->can.ctrlmode_supported & CAN_CTRLMODE_FD) {
		reg_cbt = priv->read(&regs->cbt);
		reg_cbt &= ~(FLEXCAN_CBT_EPRESDIV(0x3ff) |
			     FLEXCAN_CBT_EPSEG1(0x1f) |
			     FLEXCAN_CBT_EPSEG2(0x1f) |
			     FLEXCAN_CBT_ERJW(0x1f) |
			     FLEXCAN_CBT_EPROPSEG(0x3f) |
			     FLEXCAN_CBT_BTF);

		/* CBT[EPSEG1] is 5 bit long and CBT[EPROPSEG] is 6 bit long.
		 * The can_calc_bittiming tries to divide the tseg1 equally
		 * between phase_seg1 and prop_seg, which may not fit in CBT
		 * register. Therefore, if phase_seg1 is more than possible
		 * value, increase prop_seg and decrease phase_seg1
		 */
		if (bt->phase_seg1 > 0x20) {
			bt->prop_seg += (bt->phase_seg1 - 0x20);
			bt->phase_seg1 = 0x20;
		}

		reg_cbt = FLEXCAN_CBT_EPRESDIV(bt->brp - 1) |
				FLEXCAN_CBT_EPSEG1(bt->phase_seg1 - 1) |
				FLEXCAN_CBT_EPSEG2(bt->phase_seg2 - 1) |
				FLEXCAN_CBT_ERJW(bt->sjw - 1) |
				FLEXCAN_CBT_EPROPSEG(bt->prop_seg - 1) |
				FLEXCAN_CBT_BTF;
		priv->write(reg_cbt, &regs->cbt);

		printk("bt: prediv %d seg1 %d seg2 %d rjw %d propseg %d\n",
			   bt->brp - 1, bt->phase_seg1 - 1, bt->phase_seg2 - 1,
			   bt->sjw - 1, bt->prop_seg - 1);

		if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
			reg_fdcbt = priv->read(&regs->fdcbt);
			reg_fdcbt &= ~(FLEXCAN_FDCBT_FPRESDIV(0x3ff) |
				       FLEXCAN_FDCBT_FPSEG1(0x07) |
				       FLEXCAN_FDCBT_FPSEG2(0x07) |
				       FLEXCAN_FDCBT_FRJW(0x07) |
				       FLEXCAN_FDCBT_FPROPSEG(0x1f));

			/* FDCBT[FPSEG1] is 3 bit long and FDCBT[FPROPSEG] is 5 bit long.
			 * The can_calc_bittiming tries to divide the tseg1 equally
			 * between phase_seg1 and prop_seg, which may not fit in FDCBT
			 * register. Therefore, if phase_seg1 is more than possible
			 * value, increase prop_seg and decrease phase_seg1
			 */
			if (dbt->phase_seg1 > 0x8) {
				dbt->prop_seg += (dbt->phase_seg1 - 0x8);
				dbt->phase_seg1 = 0x8;
			}

			reg_fdcbt = FLEXCAN_FDCBT_FPRESDIV(dbt->brp - 1) |
					FLEXCAN_FDCBT_FPSEG1(dbt->phase_seg1 - 1) |
					FLEXCAN_FDCBT_FPSEG2(dbt->phase_seg2 - 1) |
					FLEXCAN_FDCBT_FRJW(dbt->sjw - 1) |
					FLEXCAN_FDCBT_FPROPSEG(dbt->prop_seg);
			priv->write(reg_fdcbt, &regs->fdcbt);

			/* enable transceiver delay compensation(TDC) for fd frame.
			 * TDC must be disabled when Loop Back mode is enabled.
			 */
			reg_fdctrl = priv->read(&regs->fdctrl);
			if (!(reg & FLEXCAN_CTRL_LPB)) {
				reg_fdctrl |= FLEXCAN_FDCTRL_TDCEN;
				reg_fdctrl &= ~FLEXCAN_FDCTRL_TDCOFF(0x1f);
				/* for the TDC to work reliably, the offset has to use optimal settings */
				reg_fdctrl |= FLEXCAN_FDCTRL_TDCOFF(((dbt->phase_seg1 - 1) + dbt->prop_seg + 2) *
								    ((dbt->brp -1) + 1));
			} else {
				reg_fdctrl &= ~FLEXCAN_FDCTRL_TDCEN;
			}
			priv->write(reg_fdctrl, &regs->fdctrl);

			if (bt->brp != dbt->brp)
				printk("Warning!! data brp = %d and brp = %d don't match.\n"
					    "flexcan may not work. consider using different bitrate or data bitrate\n",
					    dbt->brp, bt->brp);

			printk("fdbt: prediv %d seg1 %d seg2 %d rjw %d propseg %d\n",
				   dbt->brp - 1, dbt->phase_seg1 - 1, dbt->phase_seg2 - 1,
				   dbt->sjw - 1, dbt->prop_seg);

			printk("%s: mcr=0x%08x ctrl=0x%08x cbt=0x%08x fdcbt=0x%08x\n",
				   __func__, priv->read(&regs->mcr),
				   priv->read(&regs->ctrl),
				   priv->read(&regs->cbt),
				   priv->read(&regs->fdcbt));
		}
	} else {
		reg = priv->read(&regs->ctrl);
		reg &= ~(FLEXCAN_CTRL_PRESDIV(0xff) |
			 FLEXCAN_CTRL_RJW(0x3) |
			 FLEXCAN_CTRL_PSEG1(0x7) |
			 FLEXCAN_CTRL_PSEG2(0x7) |
			 FLEXCAN_CTRL_PROPSEG(0x7));

		reg |= FLEXCAN_CTRL_PRESDIV(bt->brp - 1) |
			FLEXCAN_CTRL_PSEG1(bt->phase_seg1 - 1) |
			FLEXCAN_CTRL_PSEG2(bt->phase_seg2 - 1) |
			FLEXCAN_CTRL_RJW(bt->sjw - 1) |
			FLEXCAN_CTRL_PROPSEG(bt->prop_seg - 1);
		priv->write(reg, &regs->ctrl);

		printk("bt: prediv %d seg1 %d seg2 %d rjw %d propseg %d\n",
			   bt->brp - 1, bt->phase_seg1 - 1, bt->phase_seg2 - 1,
			   bt->sjw - 1, bt->prop_seg - 1);

		/* print chip status */
		printk("%s: mcr=0x%08x ctrl=0x%08x\n", __func__,
			   priv->read(&regs->mcr), priv->read(&regs->ctrl));
	}
}

static int flexcan_irq(rtdm_irq_t *irq_handle)
{	
	struct rtcan_device *dev = rtdm_irq_get_arg(irq_handle, void);
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_esr;
	u64 reg_iflag_tx;
	int handled = 0;

	rtdm_lock_get(&dev->device_lock);

	reg_iflag_tx = flexcan_read_reg_iflag_tx(priv);

	/* transmission complete interrupt */
	if (reg_iflag_tx & priv->tx_mask) {
		//u32 reg_ctrl = priv->read(&priv->tx_mb->can_ctrl);
		/* after sending a RTR frame MB is in RX mode */
		priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
			    &priv->tx_mb->can_ctrl);
		flexcan_write64(priv, priv->tx_mask, &regs->iflag1);
		rtdm_sem_up(&dev->tx_sem);
		handled = RTDM_IRQ_HANDLED;
	}

	/* reception interrupt */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		u64 reg_iflag_rx;
		//int ret, i;
		while ((reg_iflag_rx = flexcan_read_reg_iflag_rx(priv))) {
			handled = IRQ_HANDLED;
			priv->iflags_rx |= reg_iflag_rx;
			flexcan_write64(priv, reg_iflag_rx, &regs->iflag1);
		}
	} else {
		u32 reg_iflag1;

		reg_iflag1 = priv->read(&regs->iflag1);
		if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE) {
			handled = IRQ_HANDLED;
			//can_rx_offload_irq_offload_fifo(&priv->offload);
			flexcan_write(FLEXCAN_IFLAG_RX_FIFO_AVAILABLE, &regs->iflag1);
		}

		/* FIFO overflow interrupt */
		if (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW) {
			handled = IRQ_HANDLED;
			priv->write(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW,
				    &regs->iflag1);
			//dev->stats.rx_over_errors++;
			//dev->stats.rx_errors++;
		}
	}

	reg_esr = flexcan_read(&regs->esr);

	/* ACK all bus error and state change IRQ sources */
	if (reg_esr & FLEXCAN_ESR_ALL_INT) {
		flexcan_write(reg_esr & FLEXCAN_ESR_ALL_INT, &regs->esr);
		handled = RTDM_IRQ_HANDLED;
	}

	/* state change interrupt or broken error state quirk fix is enabled */
	if (reg_esr & FLEXCAN_ESR_ERR_STATE)
		handled = RTDM_IRQ_HANDLED;
	else if (priv->devtype_data->quirks & (FLEXCAN_QUIRK_BROKEN_WERR_STATE |
					       FLEXCAN_QUIRK_BROKEN_PERR_STATE))
		goto esr_err;
	
	if (reg_esr & FLEXCAN_ESR_ERR_STATE) {
	esr_err:
		;//if (flexcan_irq_state(dev, reg_esr, &skb)) {
			//rtcan_rcv(dev, &skb);
		//}
	}

	/* bus error IRQ - report unconditionally */
	if (reg_esr & FLEXCAN_ESR_ERR_BUS) {
		//flexcan_irq_bus_err(dev, reg_esr, &skb);
		//rtcan_rcv(dev, &skb);
		handled = RTDM_IRQ_HANDLED;
	}

	rtdm_lock_put(&dev->device_lock);

	return handled;
}

static int flexcan_chip_start(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg_mcr, reg_ctrl, reg_ctrl2, reg_mecr, reg_fdctrl;
	u64 reg_imask;
	int err, i;
	struct flexcan_mb __iomem *mb;


	/* enable module */
	err = flexcan_chip_enable(priv);
	if (err)
		return err;


	/* soft reset */
	err = flexcan_chip_softreset(priv);
	if (err)
		goto out_chip_disable;

	flexcan_set_bittiming(dev);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_MECR)
		flexcan_init_ram(dev);

	/* MCR
	 *
	 * enable freeze
	 * halt now
	 * only supervisor access
	 * enable warning int
	 * enable individual RX masking
	 * choose format C
	 * set max mailbox number
	 */
	reg_mcr = priv->read(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_MAXMB(0xff);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV |
		FLEXCAN_MCR_WRN_EN | FLEXCAN_MCR_IRMQ | FLEXCAN_MCR_IDAM_C |
		FLEXCAN_MCR_MAXMB(priv->tx_mb_idx);

	/* MCR
	 *
	 * FIFO:
	 * - disable for timestamp mode
	 * - enable for FIFO mode
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP)
		reg_mcr &= ~FLEXCAN_MCR_FEN;
	else
		reg_mcr |= FLEXCAN_MCR_FEN;

	/* MCR
	 *
	 * NOTE: In loopback mode, the CAN_MCR[SRXDIS] cannot be
	 *       asserted because this will impede the self reception
	 *       of a transmitted message. This is not documented in
	 *       earlier versions of flexcan block guide.
	 *
	 * Self Reception:
	 * - enable Self Reception for loopback mode
	 *   (by clearing "Self Reception Disable" bit)
	 * - disable for normal operation
	 */
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		reg_mcr &= ~FLEXCAN_MCR_SRX_DIS;
	else
		reg_mcr |= FLEXCAN_MCR_SRX_DIS;

	printk("%s: writing mcr=0x%08x", __func__, reg_mcr);
	priv->write(reg_mcr, &regs->mcr);

	/* CTRL
	 *
	 * disable timer sync feature
	 *
	 * disable auto busoff recovery
	 * transmit lowest buffer first
	 *
	 * enable tx and rx warning interrupt
	 * enable bus off interrupt
	 * (== FLEXCAN_CTRL_ERR_STATE)
	 */
	reg_ctrl = priv->read(&regs->ctrl);
	reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
	reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF |
		FLEXCAN_CTRL_ERR_STATE;

	/* enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK),
	 * on most Flexcan cores, too. Otherwise we don't get
	 * any error warning or passive interrupts.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_BROKEN_WERR_STATE ||
	    priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
		reg_ctrl |= FLEXCAN_CTRL_ERR_MSK;
	else
		reg_ctrl &= ~FLEXCAN_CTRL_ERR_MSK;

	/* save for later use */
	priv->reg_ctrl_default = reg_ctrl;
	/* leave interrupts disabled for now */
	reg_ctrl &= ~FLEXCAN_CTRL_ERR_ALL;
	printk("%s: writing ctrl=0x%08x", __func__, reg_ctrl);
	priv->write(reg_ctrl, &regs->ctrl);

	/* FDCTRL */
	if (priv->can.ctrlmode_supported & CAN_CTRLMODE_FD) {
		reg_fdctrl = priv->read(&regs->fdctrl) & ~FLEXCAN_FDCTRL_FDRATE;
		reg_fdctrl &= ~(FLEXCAN_FDCTRL_MBDSR1(0x3) | FLEXCAN_FDCTRL_MBDSR0(0x3));
		reg_mcr = priv->read(&regs->mcr) & ~FLEXCAN_MCR_FDEN;
		reg_ctrl2 = priv->read(&regs->ctrl2) & ~FLEXCAN_CTRL2_ISOCANFDEN;

		/* support BRS when set CAN FD mode
		 * 64 bytes payload per MB and 7 MBs per RAM block by default
		 * enable CAN FD mode
		 */
		if (priv->can.ctrlmode & CAN_CTRLMODE_FD) {
			reg_fdctrl |= FLEXCAN_FDCTRL_FDRATE;
			reg_fdctrl |= FLEXCAN_FDCTRL_MBDSR1(0x3) | FLEXCAN_FDCTRL_MBDSR0(0x3);
			reg_mcr |= FLEXCAN_MCR_FDEN;

			if (!(priv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO))
				reg_ctrl2 |= FLEXCAN_CTRL2_ISOCANFDEN;
		}

		priv->write(reg_fdctrl, &regs->fdctrl);
		priv->write(reg_mcr, &regs->mcr);
		priv->write(reg_ctrl2, &regs->ctrl2);
		printk("%s: writing reg_fdctrl=0x%08x", __func__, regs->fdctrl);
	}

	if ((priv->devtype_data->quirks & FLEXCAN_QUIRK_ENABLE_EACEN_RRS)) {
		reg_ctrl2 = priv->read(&regs->ctrl2);
		reg_ctrl2 |= FLEXCAN_CTRL2_EACEN | FLEXCAN_CTRL2_RRS;
		priv->write(reg_ctrl2, &regs->ctrl2);
	}

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		for (i = priv->mb_first; i <= priv->mb_last; i++) {
			mb = flexcan_get_mb(priv, i);
			priv->write(FLEXCAN_MB_CODE_RX_EMPTY,
				    &mb->can_ctrl);
		}
	} else {
		/* clear and invalidate unused mailboxes first */
		for (i = FLEXCAN_TX_MB_RESERVED_OFF_FIFO; i < priv->mb_count; i++) {
			mb = flexcan_get_mb(priv, i);
			priv->write(FLEXCAN_MB_CODE_RX_INACTIVE,
				    &mb->can_ctrl);
		}
	}

	/* Errata ERR005829: mark first TX mailbox as INACTIVE */
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		    &priv->tx_mb_reserved->can_ctrl);

	/* mark TX mailbox as INACTIVE */
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		    &priv->tx_mb->can_ctrl);

	/* acceptance mask/acceptance code (accept everything) */
	priv->write(0x0, &regs->rxgmask);
	priv->write(0x0, &regs->rx14mask);
	priv->write(0x0, &regs->rx15mask);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_RXFG)
		priv->write(0x0, &regs->rxfgmask);

	/* clear acceptance filters */
	for (i = 0; i < priv->mb_count; i++)
		priv->write(0, &regs->rximr[i]);

	/* On Vybrid, disable non-correctable errors interrupt and freeze
	 * mode. It still can correct the correctable errors when HW supports ECC.
	 * This also works around errata e5295 which generates
	 * false positive memory errors and put the device in
	 * freeze mode.
	 */
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_DISABLE_MECR) {
		/* Follow the protocol as described in "Detection
		 * and Correction of Memory Errors" to write to
		 * MECR register (step 1 - 5)
		 * 1. By default, CTRL2[ECRWRE] = 0, MECR[ECRWRDIS] = 1
		 * 2. set CTRL2[ECRWRE]
		 */
		reg_ctrl2 = priv->read(&regs->ctrl2);
		reg_ctrl2 |= FLEXCAN_CTRL2_ECRWRE;
		priv->write(reg_ctrl2, &regs->ctrl2);

		/* 3. clear MECR[ECRWRDIS] */
		reg_mecr = priv->read(&regs->mecr);
		reg_mecr &= ~FLEXCAN_MECR_ECRWRDIS;
		priv->write(reg_mecr, &regs->mecr);

		/* 4. all writes to MECR must keep MECR[ECRWRDIS] cleared */
		reg_mecr &= ~(FLEXCAN_MECR_NCEFAFRZ | FLEXCAN_MECR_HANCEI_MSK |
			      FLEXCAN_MECR_FANCEI_MSK);
		priv->write(reg_mecr, &regs->mecr);

		/* 5. after configuration done, lock MECR by either setting
		 * MECR[ECRWRDIS] or clearing CTRL2[ECRWRE]
		 */
		reg_mecr |= FLEXCAN_MECR_ECRWRDIS;
		priv->write(reg_mecr, &regs->mecr);
		reg_ctrl2 &= ~FLEXCAN_CTRL2_ECRWRE;
		priv->write(reg_ctrl2, &regs->ctrl2);

	}

	err = flexcan_transceiver_enable(priv);
	if (err)
		goto out_chip_disable;

	/* synchronize with the can bus */
	err = flexcan_chip_unfreeze(priv);
	if (err)
		goto out_transceiver_disable;

	//priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* enable interrupts atomically */
	rtdm_irq_disable(&dev->irq_handle);
	priv->write(priv->reg_ctrl_default, &regs->ctrl);
	reg_imask = priv->rx_mask | priv->tx_mask;
	priv->write(upper_32_bits(reg_imask), &regs->imask2);
	priv->write(lower_32_bits(reg_imask), &regs->imask1);
	rtdm_irq_enable(&dev->irq_handle);

	/* print chip status */
	printk("%s: reading mcr=0x%08x ctrl=0x%08x\n", __func__,
		   priv->read(&regs->mcr), priv->read(&regs->ctrl));

	return 0;

 out_transceiver_disable:
	flexcan_transceiver_disable(priv);
 out_chip_disable:
	flexcan_chip_disable(priv);

	return err;
}

static void flexcan_chip_stop(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;

	/* freeze + disable module */
	flexcan_chip_freeze(priv);
	flexcan_chip_disable(priv);

	/* Disable all interrupts */
	priv->write(0, &regs->imask2);
	priv->write(0, &regs->imask1);
	priv->write(priv->reg_ctrl_default & ~FLEXCAN_CTRL_ERR_ALL,
		    &regs->ctrl);

	flexcan_transceiver_disable(priv);
	//priv->can.state = CAN_STATE_STOPPED;
}

static int udd_open(struct rtdm_fd *fd, int oflags)
{
	struct udd_device *udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	struct rtcan_device *dev = container_of(udd, struct rtcan_device, udd_dev);
	struct flexcan_priv *priv = rtcan_priv(dev);
	int err;
	struct can_bittiming bt, dbt;
	rtdm_lockctx_t lock_ctx;

	if ((priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES) &&
	    (priv->can.ctrlmode & CAN_CTRLMODE_FD)) {
		printk("three samples mode and fd mode can't be used together\n");
		return -EINVAL;
	}

	/**baudrate calculation start**/

	if (!priv->can.bittiming_const)
		goto out_runtime_put;
	memset(&bt, 0, sizeof(bt));
	bt.bitrate=1000000;
	err = can_calc_bittiming(dev, &bt, priv->can.bittiming_const);
	if (err)
		goto out_runtime_put;
	memcpy(&priv->can.bittiming, &bt, sizeof(bt));
	printk("qmy: calc bittiming: bitrate=%d, prop_seg=%d, phase_seg1=%d, phase_seg2=%d, sjw=%d, brp=%d\n"
		,priv->can.bittiming.bitrate, priv->can.bittiming.prop_seg, priv->can.bittiming.phase_seg1, priv->can.bittiming.phase_seg2, priv->can.bittiming.sjw, priv->can.bittiming.brp);

	if(priv->can.ctrlmode == CAN_CTRLMODE_FD){
		if (!priv->can.data_bittiming_const)
			goto out_runtime_put;
		memset(&dbt, 0, sizeof(dbt));
		dbt.bitrate=5000000;
		err = can_calc_bittiming(dev, &dbt, priv->can.data_bittiming_const);
		if (err)
			goto out_runtime_put;
		memcpy(&priv->can.data_bittiming, &dbt, sizeof(dbt));
		printk("qmy: calc databittiming: bitrate=%d, prop_seg=%d, phase_seg1=%d, phase_seg2=%d, sjw=%d, brp=%d\n"
		,priv->can.data_bittiming.bitrate, priv->can.data_bittiming.prop_seg, priv->can.data_bittiming.phase_seg1, priv->can.data_bittiming.phase_seg2, priv->can.data_bittiming.sjw, priv->can.data_bittiming.brp);
	}

	/**baudrate calculation end**/

	if (!priv->can.bittiming.bitrate) {
		printk("bit-timing not yet defined\n");
		err=-EINVAL;
	}
	if (err)
		goto out_runtime_put;


	/* For CAN FD the data bitrate has to be >= the arbitration bitrate */
	if ((priv->can.ctrlmode & CAN_CTRLMODE_FD) &&
	    (!priv->can.data_bittiming.bitrate ||
	     priv->can.data_bittiming.bitrate < priv->can.bittiming.bitrate)) {
		printk("incorrect/missing data bit-timing\n");
		return -EINVAL;
	}
	if (err)
		goto out_runtime_put;

	//err = request_irq(dev->irq, flexcan_irq, IRQF_SHARED, dev->name, dev);
	//if (err)
	//	goto out_close;


	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		priv->mb_size = sizeof(struct flexcan_mb) + CANFD_MAX_DLEN;
	else
		priv->mb_size = sizeof(struct flexcan_mb) + CAN_MAX_DLEN;
	priv->mb_count = (sizeof(priv->regs->mb[0]) / priv->mb_size) +
			 (sizeof(priv->regs->mb[1]) / priv->mb_size);

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP)
		priv->tx_mb_reserved =
			flexcan_get_mb(priv, FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP);
	else
		priv->tx_mb_reserved =
			flexcan_get_mb(priv, FLEXCAN_TX_MB_RESERVED_OFF_FIFO);
	priv->tx_mb_idx = priv->mb_count - 1;
	priv->tx_mb = flexcan_get_mb(priv, priv->tx_mb_idx);
	priv->tx_mask = FLEXCAN_IFLAG_MB(priv->tx_mb_idx);

	//priv->offload.mailbox_read = flexcan_mailbox_read;

	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
		priv->mb_first = FLEXCAN_RX_MB_OFF_TIMESTAMP_FIRST;
		priv->mb_last = priv->mb_count - 2;

		priv->rx_mask = GENMASK_ULL(priv->mb_last,
					    priv->mb_first);
		//err = can_rx_offload_add_timestamp(dev, &priv->offload);
	} else {
		priv->rx_mask = FLEXCAN_IFLAG_RX_FIFO_OVERFLOW |
			FLEXCAN_IFLAG_RX_FIFO_AVAILABLE;
		//err = can_rx_offload_add_fifo(dev, &priv->offload, FLEXCAN_NAPI_WEIGHT);
	}

	printk("qmy: mb_count=%d, tx_mb_reserved=%d, tx_mb_idx=%d, mb_first=%d, mb_last=%d\n"
		,priv->mb_count,FLEXCAN_TX_MB_RESERVED_OFF_TIMESTAMP, priv->tx_mb_idx, priv->mb_first, priv->mb_last);
	
	err = rtdm_irq_request(&dev->irq_handle, priv->irq, flexcan_irq, 0, DRV_NAME, dev);
	if (err) {
		printk("couldn't request irq %d\n", priv->irq);
		goto out_close;
	}
	/* Set up sender "mutex" */
	rtdm_sem_init(&dev->tx_sem, 1);

	/* start chip and queuing */
	err = flexcan_chip_start(dev);
	if (err){
		printk("couldn't start chip err=%d\n", err);
		rtdm_irq_free(&dev->irq_handle);
		rtdm_sem_destroy(&dev->tx_sem);
		goto out_offload_del;
	}

	return 0;

 out_offload_del:
	//can_rx_offload_del(&priv->offload);
 out_free_irq:
	//free_irq(dev->irq, dev);
 out_close:
	//close_candev(dev);
 out_runtime_put:
	//pm_runtime_put(priv->dev);
	
	flexcan_chip_disable(priv);

	return err;
}

static void udd_close(struct rtdm_fd *fd)
{
	struct udd_device *udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	struct rtcan_device *dev = container_of(udd, struct rtcan_device, udd_dev);
	printk("Closing udd can fd...");
	flexcan_chip_stop(dev);
	rtdm_irq_free(&dev->irq_handle);
	rtdm_sem_destroy(&dev->tx_sem);
}

static int udd_ioctl_rt(struct rtdm_fd *fd,
			unsigned int request, void __user *arg)
{
	return 0;
}

static ssize_t udd_read_rt(struct rtdm_fd *fd,
			   void __user *buf, size_t len)
{
	struct udd_device *udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	struct rtcan_device *dev = container_of(udd, struct rtcan_device, udd_dev);
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_mb __iomem *mb;
	struct flexcan_regs __iomem *regs = priv->regs;
	rtdm_lockctx_t lock_ctx;
	u32 reg_ctrl, reg_id, reg_iflag1;
	int i=0, j=0, n=0, count=0, index=0;
	u32 cfd_id=0, cfd_timestamp=0;
	u8 cfd_data[64], cfd_flags=0, cfd_len=0;
	u8 cfd_buff[864];

	rtdm_lock_get_irqsave(&dev->device_lock, lock_ctx);

	for (n = priv->mb_first; flexcan_rx_le(priv, n, priv->mb_last); flexcan_rx_inc(priv, &n)) {

		if (!(priv->iflags_rx & BIT_ULL(n)))
			continue;

		mb = flexcan_get_mb(priv, n);
		if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
			u32 code;

			do {
				reg_ctrl = priv->read(&mb->can_ctrl);
			} while (reg_ctrl & FLEXCAN_MB_CODE_RX_BUSY_BIT);
			
			//printk("qmy: n=%d, CODE=%x\n", n, (reg_ctrl&FLEXCAN_MB_CODE_MASK)>>24);		
	
			/* is this MB empty? */
			code = reg_ctrl & FLEXCAN_MB_CODE_MASK;
			if ((code != FLEXCAN_MB_CODE_RX_FULL) &&
		    		(code != FLEXCAN_MB_CODE_RX_OVERRUN))
				continue;

			if (code == FLEXCAN_MB_CODE_RX_OVERRUN) {
				printk("This MB was overrun, we lost data! n=%d\n",n);
				//offload->dev->stats.rx_over_errors++;
				//offload->dev->stats.rx_errors++;
			}	
		} else {
			reg_iflag1 = priv->read(&regs->iflag1);
			if (!(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE))
				continue;
			reg_ctrl = priv->read(&mb->can_ctrl);
		}

		if (reg_ctrl & FLEXCAN_MB_CNT_EDL)
			cfd_flags=0x80;
		else
			cfd_flags=0x00;
		
		/* increase timstamp to full 32 bit */
		cfd_timestamp = reg_ctrl << 16;
	
		reg_id = priv->read(&mb->can_id);
		if (reg_ctrl & FLEXCAN_MB_CNT_IDE)
			cfd_id = ((reg_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
		else
			cfd_id = (reg_id >> 18) & CAN_SFF_MASK;

		if (reg_ctrl & FLEXCAN_MB_CNT_EDL) {
			cfd_len = dlc2len[(reg_ctrl >> 16) & 0xf];

		if (reg_ctrl & FLEXCAN_MB_CNT_BRS)
			cfd_flags |= CANFD_BRS;
		} else {
			cfd_len = (reg_ctrl >> 16) & 0xf;

		if (reg_ctrl & FLEXCAN_MB_CNT_RTR)
			cfd_id |= CAN_RTR_FLAG;
		}

		if (reg_ctrl & FLEXCAN_MB_CNT_ESI)
			cfd_flags |= CANFD_ESI;
		
		for (i = 0; i < cfd_len; i += sizeof(u32)) {
			__be32 data = cpu_to_be32(priv->read(&mb->data[i / sizeof(u32)]));
			*(__be32 *)(cfd_data + i) = data;
		}

		priv->iflags_rx &= ~FLEXCAN_IFLAG_MB(n);

		cfd_buff[index++] = '!';
		cfd_buff[index++] = cfd_id>>8;
		cfd_buff[index++] = cfd_id&0xff;
		cfd_buff[index++] = '@';
		cfd_buff[index++] = cfd_flags;
		cfd_buff[index++] = '#';
		cfd_buff[index++] = cfd_len;
		cfd_buff[index++] = '$';
		for(j=0; j<cfd_len; j++)
		{
			cfd_buff[index++] = cfd_data[j];	
		}

		count++;

		//printk("qmy: n=%d, can_id=%x, can_len=%d, can_data[0]=%x, index=%d\n", n, cfd_id, cfd_len,cfd_data[0], index);
	}

	/* Read the Free Running Timer. It is optional but recommended
	 * to unlock Mailbox as soon as possible and make it available
	 * for reception.
	 */
	priv->read(&regs->timer);

	rtdm_lock_put_irqrestore(&dev->device_lock, lock_ctx);

 	rtdm_safe_copy_to_user(fd, buf, cfd_buff, index);

	return index;
}

static ssize_t udd_write_rt(struct rtdm_fd *fd,
			    const void __user *buf, size_t len)
{
	struct udd_device *udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	struct rtcan_device *dev = container_of(udd, struct rtcan_device, udd_dev);
	struct flexcan_priv *priv = rtcan_priv(dev);
	rtdm_lockctx_t lock_ctx;
	nanosecs_rel_t timeout = (400*1000);
	u32 ctrl;
	u32 data;
	int i;
	int ret = 0;
	u32 can_id;
	u8 cfd_head[8], cfd_flags=0, cfd_data[64];
	u16 cfd_id=0, cfd_len=0;

	rtdm_safe_copy_from_user(fd, cfd_head, buf, 8);

	if(cfd_head[0]!='!') return -1;
	cfd_id = ((cfd_id|cfd_head[1])<<8 | cfd_head[2]);
	if(cfd_head[3]!='@') return -2;
	cfd_flags = cfd_head[4];
	if(cfd_head[5]!='#') return -3;
	cfd_len = cfd_head[6];
	if(cfd_len>CANFD_MAX_DLEN) cfd_len = CANFD_MAX_DLEN;
	if(cfd_head[7]!='$') return -4;

	rtdm_safe_copy_from_user(fd, cfd_data, buf+8, cfd_len);

	if(priv->can.ctrlmode != CAN_CTRLMODE_FD)
	{
		cfd_flags = 0;
		if(cfd_len>CAN_MAX_DLEN) 
			cfd_len = CAN_MAX_DLEN;	
	}

	/* Try to pass the guard in order to access the controller */
    	ret = rtdm_sem_timeddown(&dev->tx_sem, timeout, NULL);
	if(ret)
	{
		return ret;	
	}
	
	rtdm_lock_get_irqsave(&dev->device_lock, lock_ctx);

	ctrl = FLEXCAN_MB_CODE_TX_DATA | ((len2dlc[cfd_len]) << 16);

	if(cfd_flags)
	{
		ctrl |= FLEXCAN_MB_CNT_EDL;
		ctrl |= FLEXCAN_MB_CNT_BRS;
	}

	can_id = (cfd_id & CAN_SFF_MASK) << 18;

	for (i = 0; i < cfd_len; i += sizeof(u32)) {
		data = be32_to_cpup((__be32 *)(cfd_data+i));
		priv->write(data, &priv->tx_mb->data[i / sizeof(u32)]);
	}

	priv->write(can_id, &priv->tx_mb->can_id);
	priv->write(ctrl, &priv->tx_mb->can_ctrl);

	/* Errata ERR005829 step8:
	 * Write twice INACTIVE(0x8) code to first MB.
	 */
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		    &priv->tx_mb_reserved->can_ctrl);
	priv->write(FLEXCAN_MB_CODE_TX_INACTIVE,
		    &priv->tx_mb_reserved->can_ctrl);

	rtdm_lock_put_irqrestore(&dev->device_lock, lock_ctx);

	return cfd_len;
}

static int udd_select(struct rtdm_fd *fd, struct xnselector *selector,
		      unsigned int type, unsigned int index)
{
	return 0;
}
static int udd_irq_handler(rtdm_irq_t *irqh)
{

	return RTDM_IRQ_NONE;
}




static int flexcan_clks_enable(const struct flexcan_priv *priv)
{
	int err;

	err = clk_prepare_enable(priv->clk_ipg);
	if (err)
		return err;

	err = clk_prepare_enable(priv->clk_per);
	if (err)
		clk_disable_unprepare(priv->clk_ipg);

	return err;
}

static void flexcan_clks_disable(const struct flexcan_priv *priv)
{
	clk_disable_unprepare(priv->clk_per);
	clk_disable_unprepare(priv->clk_ipg);
}

static int flexcan_init_device(struct rtcan_device *dev)
{
	struct flexcan_priv *priv = rtcan_priv(dev);
	struct flexcan_regs __iomem *regs = priv->regs;
	u32 reg, err;

	err = flexcan_clks_enable(priv);
	if (err)
		return err;

	/* select "bus clock", chip must be disabled */
	err = flexcan_chip_disable(priv);
	if (err)
		goto out_clks_disable;

	reg = priv->read(&regs->ctrl);
	if (priv->clk_src)
		reg |= FLEXCAN_CTRL_CLK_SRC;
	else
		reg &= ~FLEXCAN_CTRL_CLK_SRC;
	priv->write(reg, &regs->ctrl);

	err = flexcan_chip_enable(priv);
	if (err)
		goto out_chip_disable;

	/* set freeze, halt and activate FIFO, restrict register access */
	reg = priv->read(&regs->mcr);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT |
		FLEXCAN_MCR_FEN | FLEXCAN_MCR_SUPV;
	priv->write(reg, &regs->mcr);

	/* Currently we only support newer versions of this core
	 * featuring a RX hardware FIFO (although this driver doesn't
	 * make use of it on some cores). Older cores, found on some
	 * Coldfire derivates are not tested.
	 */
	reg = priv->read(&regs->mcr);
	if (!(reg & FLEXCAN_MCR_FEN)) {
		printk("Could not enable RX FIFO, unsupported core\n");
		err = -ENODEV;
		goto out_chip_disable;
	}

	/* Disable core and let pm_runtime_put() disable the clocks.
	 * If CONFIG_PM is not enabled, the clocks will stay powered.
	 */
	flexcan_chip_disable(priv);
	//pm_runtime_put(priv->dev);

	return 0;

 out_chip_disable:
	flexcan_chip_disable(priv);
 out_clks_disable:
	flexcan_clks_disable(priv);
	return err;
}

/**
 * @brief Register a UDD device
 *
 * This routine registers a mini-driver at the UDD core.
 *
 * @param udd @ref udd_device "UDD device descriptor" which should
 * describe the new device properties.
 *
 * @return Zero is returned upon success, otherwise a negative error
 * code is received, from the set of error codes defined by
 * rtdm_dev_register(). In addition, the following error codes can be
 * returned:
 *
 * - -EINVAL, some of the memory regions declared in the
 *   udd_device.mem_regions[] array have invalid properties, i.e. bad
 *   type, NULL name, zero length or address. Any undeclared region
 *   entry from the array must bear the UDD_MEM_NONE type.
 *
 * - -EINVAL, if udd_device.irq is different from UDD_IRQ_CUSTOM and
 * UDD_IRQ_NONE but invalid, causing rtdm_irq_request() to fail.
 *
 * - -EINVAL, if udd_device.device_flags contains invalid flags.
 *
 * - -ENOSYS, if this service is called while the real-time core is disabled.
 *
 * @coretags{secondary-only}
 */
int udd_register_device(struct udd_device *udd)
{
	struct rtdm_device *dev = &udd->__reserved.device;
	struct udd_reserved *ur = &udd->__reserved;
	struct rtdm_driver *drv = &ur->driver;
	//struct udd_memregion *rn;
	int ret;
	
	if (udd->device_flags & RTDM_PROTOCOL_DEVICE)
		return -EINVAL;

	if (udd->irq != UDD_IRQ_NONE && udd->irq != UDD_IRQ_CUSTOM &&
	    udd->ops.interrupt == NULL)
		return -EINVAL;


	drv->profile_info = (struct rtdm_profile_info)
		RTDM_PROFILE_INFO(udd->device_name, RTDM_CLASS_UDD,
				  udd->device_subclass, 0);
	drv->device_flags = RTDM_NAMED_DEVICE|udd->device_flags;
	drv->device_count = 2;
	drv->ops = (struct rtdm_fd_ops){
		.open = udd_open,
		.ioctl_rt = udd_ioctl_rt,
		.read_rt = udd_read_rt,
		.write_rt = udd_write_rt,
		.close = udd_close,
		.select = udd_select,
	};

	dev->driver = drv;
	dev->label = udd->device_name;

	ret = rtdm_dev_register(dev);
	if (ret)
		return ret;

	if (udd->irq != UDD_IRQ_NONE && udd->irq != UDD_IRQ_CUSTOM) {
		ret = rtdm_irq_request(&ur->irqh, udd->irq,
				       udd_irq_handler, 0,
				       dev->name, udd);
		if (ret)
			goto fail_irq_request;
	}

	return 0;

fail_irq_request:
	rtdm_dev_unregister(dev);


	return ret;
}
EXPORT_SYMBOL_GPL(udd_register_device);

/**
 * @brief Unregister a UDD device
 *
 * This routine unregisters a mini-driver from the UDD core. This
 * routine waits until all connections to @a udd have been closed
 * prior to unregistering.
 *
 * @param udd UDD device descriptor
 *
 * @return Zero is returned upon success, otherwise -ENXIO is received
 * if this service is called while the Cobalt kernel is disabled.
 *
 * @coretags{secondary-only}
 */
int udd_unregister_device(struct udd_device *udd)
{
	struct udd_reserved *ur = &udd->__reserved;

	rtdm_dev_unregister(&ur->device);

	return 0;
}
EXPORT_SYMBOL_GPL(udd_unregister_device);

static const struct of_device_id rt_canfd_of_match[] = {
	{ .compatible = "fsl,rt-imx8mp-flexcan", .data = &fsl_imx8mp_devtype_data, },
	{ .compatible = "fsl,rt-imx8mp-flexcan-fd", .data = &fsl_imx8mp_devtype_data_fd, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rt_canfd_of_match);

static const struct platform_device_id rt_canfd_id_table[] = {
	{ .name = "rt-imx8mp-flexcan", .driver_data = (kernel_ulong_t)&fsl_imx8mp_devtype_data, },
	{ .name = "rt-imx8mp-flexcan-fd", .driver_data = (kernel_ulong_t)&fsl_imx8mp_devtype_data_fd, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(platform, rt_canfd_id_table);


static int flexcan_fd_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	const struct flexcan_devtype_data *devtype_data;
	struct rtcan_device *dev;
	struct flexcan_priv *priv;
	struct regulator *reg_xceiver;
	struct clk *clk_ipg = NULL, *clk_per = NULL;
	struct resource *mem;
	struct flexcan_regs __iomem *regs;
	int err, irq;
	u8 clk_src = 1;
	u32 clock_freq = 0;

	reg_xceiver = devm_regulator_get(&pdev->dev, "xceiver");
	if (PTR_ERR(reg_xceiver) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	else if (IS_ERR(reg_xceiver))
		reg_xceiver = NULL;

	if (pdev->dev.of_node) {
		of_property_read_u32(pdev->dev.of_node,
				     "clock-frequency", &clock_freq);
		of_property_read_u8(pdev->dev.of_node,
				    "fsl,clk-source", &clk_src);
	}

	if (!clock_freq) {
		clk_ipg = devm_clk_get(&pdev->dev, "ipg");
		if (IS_ERR(clk_ipg)) {
			dev_err(&pdev->dev, "no ipg clock defined\n");
			return PTR_ERR(clk_ipg);
		}

		clk_per = devm_clk_get(&pdev->dev, "per");
		if (IS_ERR(clk_per)) {
			dev_err(&pdev->dev, "no per clock defined\n");
			return PTR_ERR(clk_per);
		}
		clock_freq = clk_get_rate(clk_per);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENODEV;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, mem);

	if (IS_ERR(regs))
		return PTR_ERR(regs);

	of_id = of_match_device(rt_canfd_of_match, &pdev->dev);
	if (of_id) {
		devtype_data = of_id->data;
	} else if (platform_get_device_id(pdev)->driver_data) {
		devtype_data = (struct flexcan_devtype_data *)
			platform_get_device_id(pdev)->driver_data;
	} else {
		return -ENODEV;
	}

	dev = canfd_dev_alloc(sizeof(struct flexcan_priv), 0);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	
	priv = rtcan_priv(dev);
	priv->read = flexcan_read;
	priv->write = flexcan_write;
	priv->dev = &pdev->dev;
	priv->can.clock.freq = clock_freq;
	priv->can.bittiming_const = &flexcan_bittiming_const;
	//priv->can.do_set_mode = flexcan_set_mode;
	//priv->can.do_get_berr_counter = flexcan_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_LISTENONLY	| CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_BERR_REPORTING;
	priv->regs = regs;
	priv->irq = irq;
	priv->clk_ipg = clk_ipg;
	priv->clk_per = clk_per;
	priv->clk_src = clk_src;
	priv->devtype_data = devtype_data;
	priv->reg_xceiver = reg_xceiver;
	priv->iflags_rx = 0;
	
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_TIMESTAMP_SUPPORT_FD) 
	{
		priv->can.ctrlmode = CAN_CTRLMODE_FD;
		if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_OFF_TIMESTAMP) {
			priv->can.ctrlmode_supported |= CAN_CTRLMODE_FD | CAN_CTRLMODE_FD_NON_ISO;
			priv->can.bittiming_const = &flexcan_fd_bittiming_const;
			priv->can.data_bittiming_const = &flexcan_fd_data_bittiming_const;
		} else {
			dev_err(&pdev->dev, "can fd mode can't work on fifo mode\n");
			err = -EINVAL;
			goto failed_register;
		}
	}
	else
	{
		priv->can.ctrlmode = 0;	
	}


	err = flexcan_init_device(dev);
	if(err){
		dev_err(&pdev->dev, "initialize netdev failed\n");
		goto failed_register;	
	}

	if(mem->start==0x308c0000)
		dev->udd_dev.device_name = DEVICE_NAME_0;
	else if(mem->start==0x308d0000)
		dev->udd_dev.device_name = DEVICE_NAME_1;
	else;

	dev->udd_dev.device_flags = RTDM_NAMED_DEVICE;
	dev->udd_dev.device_subclass = RTDM_SUBCLASS_UDD_CAN;
	dev->udd_dev.irq = UDD_IRQ_CUSTOM;

	err = udd_register_device (&dev->udd_dev);
	if (err) {
		dev_err(&pdev->dev, "registering netdev failed, err-%d\n", err);
		goto failed_register;
	}
	
	dev_info(&pdev->dev, "device registered (reg_base=%p, irq=%d, clk_freq=%d)\n",
		 priv->regs, irq, priv->can.clock.freq);

/*
	if (priv->devtype_data->quirks & FLEXCAN_QUIRK_SETUP_STOP_MODE) {
		if (priv->devtype_data->quirks & FLEXCAN_QUIRK_USE_SCFW)
			err = flexcan_setup_stop_mode_scfw(pdev);
		else
			err = flexcan_setup_stop_mode(pdev);

		if (err) {
			dev_dbg(&pdev->dev, "failed to setup stop-mode\n");
		} else {
			device_set_wakeup_capable(&pdev->dev, true);

			if (of_property_read_bool(pdev->dev.of_node, "wakeup-source"))
				device_set_wakeup_enable(&pdev->dev, true);
		}
	}
*/

	return 0;

 failed_register:
	if(dev!=NULL){
		kfree(dev);
		rtdm_sem_destroy(&dev->tx_sem);
	}
	return err;
}


static int flexcan_fd_remove(struct platform_device *pdev)
{

	struct rtcan_device *dev = platform_get_drvdata(pdev);

	udd_unregister_device(&dev->udd_dev);
	kfree(dev);

	return 0;
}

static struct platform_driver udd_canfd_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = rt_canfd_of_match,
	},
	.probe = flexcan_fd_probe,
	.remove = flexcan_fd_remove,
	.id_table = rt_canfd_id_table,
	.prevent_deferred_probe = true,
};

static int __init udd_can_init(void)
{
	int ret;

	if (!rtdm_available())
		return -ENODEV;

	ret = platform_driver_register(&udd_canfd_driver);
	if (ret) {
		pr_err("%s; Could not register driver (err=%d)\n",
			__func__, ret);
	}

	return ret;	

}
module_init(udd_can_init);


static void __exit udd_can_exit(void)
{
	platform_driver_unregister(&udd_canfd_driver);
}
module_exit(udd_can_exit);



MODULE_AUTHOR("MY");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAN-FD driver");
