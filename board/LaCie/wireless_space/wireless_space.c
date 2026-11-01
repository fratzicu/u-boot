// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2011 Simon Guinot <sguinot@lacie.com>
 * Copyright (C) 2012 Albert ARIBAUD <albert.u.boot@aribaud.net>
 *
 * Wireless Space support, removed from U-Boot in 2015 (commit b352182a00)
 * and ported back to driver model / device tree on top of netspace_v2.
 *
 * Based on Kirkwood support:
 * (C) Copyright 2009
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Prafulla Wadaskar <prafulla@marvell.com>
 */

#include <command.h>
#include <env.h>
#include <init.h>
#include <miiphy.h>
#include <net.h>
#include <asm/global_data.h>
#include <asm/mach-types.h>
#include <asm/arch/cpu.h>
#include <asm/arch/soc.h>
#include <asm/arch/mpp.h>
#include <asm/arch/gpio.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <mapmem.h>
#include <nand.h>

#include "../common/common.h"

DECLARE_GLOBAL_DATA_PTR;

#define WIRELESS_SPACE_OE_LOW		0xFF006808
#define WIRELESS_SPACE_OE_HIGH		0x0000F989
#define WIRELESS_SPACE_OE_VAL_LOW	0x00010080
#define WIRELESS_SPACE_OE_VAL_HIGH	0x00000240

#define WIRELESS_SPACE_REAR_BUTTON	13
#define WIRELESS_SPACE_FRONT_BUTTON	43

int board_early_init_f(void)
{
	/* Gpio configuration */
	mvebu_config_gpio(WIRELESS_SPACE_OE_VAL_LOW, WIRELESS_SPACE_OE_VAL_HIGH,
			  WIRELESS_SPACE_OE_LOW, WIRELESS_SPACE_OE_HIGH);

	/* Multi-Purpose Pins Functionality configuration */
	static const u32 kwmpp_config[] = {
		MPP0_NF_IO2,
		MPP1_NF_IO3,
		MPP2_NF_IO4,
		MPP3_NF_IO5,
		MPP4_NF_IO6,
		MPP5_NF_IO7,
		MPP6_SYSRST_OUTn,
		MPP7_GPO,		/* Fan speed (bit 1) */
		MPP8_TW_SDA,
		MPP9_TW_SCK,
		MPP10_UART0_TXD,
		MPP11_UART0_RXD,
		MPP13_GPIO,		/* Rear button (0 = Pushed) */
		MPP14_GPIO,		/* USB fuse */
		MPP15_SATA0_ACTn,
		MPP16_GPIO,		/* SATA 0 power */
		MPP17_GPIO,		/* SATA 1 power */
		MPP18_NF_IO0,
		MPP19_NF_IO1,
		MPP20_GE1_0,		/* Gigabit Ethernet 1 */
		MPP21_GE1_1,
		MPP22_GE1_2,
		MPP23_GE1_3,
		MPP24_GE1_4,
		MPP25_GE1_5,
		MPP26_GE1_6,
		MPP27_GE1_7,
		MPP28_GE1_8,
		MPP29_GE1_9,
		MPP30_GE1_10,
		MPP31_GE1_11,
		MPP32_GE1_12,
		MPP33_GE1_13,
		MPP34_GE1_14,
		MPP35_GE1_15,
		MPP36_GPIO,		/* Fan speed (bit 2) */
		MPP37_GPIO,		/* Fan speed (bit 0) */
		MPP38_GPIO,		/* Fan power */
		MPP39_GPIO,		/* Fan rotation fail */
		MPP40_GPIO,		/* Ethernet switch link */
		MPP41_GPIO,		/* USB enable host vbus */
		MPP42_GPIO,		/* LED clock control */
		MPP43_GPIO,		/* WPS button (0 = Pushed, 1 = Released) */
		MPP44_GPIO,		/* Red LED on/off */
		MPP45_GPIO,		/* Red LED timer blink (on=off=100ms) */
		MPP46_GPIO,		/* Green LED on/off */
		MPP47_GPIO,		/* LED (blue, green) SATA activity blink */
		MPP48_GPIO,		/* Blue LED on/off */
		0
	};
	kirkwood_mpp_conf(kwmpp_config, NULL);

	return 0;
}

int board_init(void)
{
	/* Machine number */
	gd->bd->bi_arch_number = MACH_TYPE_WIRELESS_SPACE;

	/* Boot parameters address */
	gd->bd->bi_boot_params = mvebu_sdram_bar(0) + 0x100;

	/*
	 * Set up the mbus windows before NAND is probed: when chainloaded
	 * from the LaCie U-Boot, its window layout has no NAND at 0xD8000000
	 */
	kirkwood_mbus_init();

	return 0;
}

#if defined(CONFIG_MISC_INIT_R)
int misc_init_r(void)
{
#if defined(CONFIG_CMD_I2C) && defined(CONFIG_SYS_I2C_EEPROM_ADDR)
	if (!env_get("ethaddr")) {
		uchar mac[6];
		if (lacie_read_mac_address(mac) == 0)
			eth_env_set_enetaddr("ethaddr", mac);
	}
#endif
	return 0;
}
#endif

#if defined(CONFIG_CMD_NET) && defined(CONFIG_RESET_PHY_R)
/*
 * The 88E6161 switch on GE0 is strapped in multi-chip addressing mode and
 * answers on SMI address 0xa only: every switch register is reached through
 * the SMI command/data indirection registers.
 */
#define WS_ETH_NAME		"ethernet-controller@72000"
#define WS_SWITCH_SMI_ADDR	0xa
#define WS_SWITCH_CPU_PORT	5
#define WS_SWITCH_NUM_PORTS	6

#define SMI_CMD_REG		0x0
#define SMI_DATA_REG		0x1
#define SMI_BUSY		BIT(15)
#define SMI_MODE_22		BIT(12)
#define SMI_OP_READ		BIT(11)
#define SMI_OP_WRITE		BIT(10)

#define SW_PORT(p)		(0x10 + (p))
#define SW_PORT_PCS_CTRL	0x1
#define SW_PORT_CTRL		0x4
#define SW_PORT_CTRL_FORWARDING	0x3
#define SW_PORT_RGMII_TIMING	0x1a

/* The port PHYs are reached through the Global 2 SMI PHY registers */
#define SW_GLOBAL2		0x1c
#define SW_G2_SMI_PHY_CMD	0x18
#define SW_G2_SMI_PHY_DATA	0x19

static int ws_smi_wait(void)
{
	int timeout = 1000;
	u16 reg;

	do {
		if (miiphy_read(WS_ETH_NAME, WS_SWITCH_SMI_ADDR, SMI_CMD_REG,
				&reg))
			return -EIO;
		if (!(reg & SMI_BUSY))
			return 0;
		udelay(10);
	} while (--timeout);

	return -ETIMEDOUT;
}

static int ws_sw_read(u8 dev, u8 reg, u16 *val)
{
	if (ws_smi_wait())
		return -EIO;
	miiphy_write(WS_ETH_NAME, WS_SWITCH_SMI_ADDR, SMI_CMD_REG,
		     SMI_BUSY | SMI_MODE_22 | SMI_OP_READ | (dev << 5) | reg);
	if (ws_smi_wait())
		return -EIO;
	return miiphy_read(WS_ETH_NAME, WS_SWITCH_SMI_ADDR, SMI_DATA_REG, val);
}

static int ws_sw_write(u8 dev, u8 reg, u16 val)
{
	if (ws_smi_wait())
		return -EIO;
	miiphy_write(WS_ETH_NAME, WS_SWITCH_SMI_ADDR, SMI_DATA_REG, val);
	miiphy_write(WS_ETH_NAME, WS_SWITCH_SMI_ADDR, SMI_CMD_REG,
		     SMI_BUSY | SMI_MODE_22 | SMI_OP_WRITE | (dev << 5) | reg);
	return ws_smi_wait();
}

static int ws_g2_phy_wait(void)
{
	int timeout = 1000;
	u16 reg;

	do {
		if (ws_sw_read(SW_GLOBAL2, SW_G2_SMI_PHY_CMD, &reg))
			return -EIO;
		if (!(reg & SMI_BUSY))
			return 0;
		udelay(10);
	} while (--timeout);

	return -ETIMEDOUT;
}

static int ws_phy_read(u8 phy, u8 reg, u16 *val)
{
	if (ws_g2_phy_wait())
		return -EIO;
	ws_sw_write(SW_GLOBAL2, SW_G2_SMI_PHY_CMD,
		    SMI_BUSY | SMI_MODE_22 | SMI_OP_READ | (phy << 5) | reg);
	if (ws_g2_phy_wait())
		return -EIO;
	return ws_sw_read(SW_GLOBAL2, SW_G2_SMI_PHY_DATA, val);
}

static int ws_phy_write(u8 phy, u8 reg, u16 val)
{
	if (ws_g2_phy_wait())
		return -EIO;
	ws_sw_write(SW_GLOBAL2, SW_G2_SMI_PHY_DATA, val);
	ws_sw_write(SW_GLOBAL2, SW_G2_SMI_PHY_CMD,
		    SMI_BUSY | SMI_MODE_22 | SMI_OP_WRITE | (phy << 5) | reg);
	return ws_g2_phy_wait();
}

/* Same sequence as the former mv88e61xx_switch_initialize() for this board */
void reset_phy(void)
{
	u16 id, reg;
	int port;

	if (miiphy_set_current_dev(WS_ETH_NAME))
		return;

	if (ws_sw_read(SW_PORT(0), 0x3, &id)) {
		printf("WS: switch not found at SMI address 0x%x\n",
		       WS_SWITCH_SMI_ADDR);
		return;
	}

	/* Enable RGMII delay on Tx and Rx for the CPU port */
	ws_sw_write(SW_PORT(WS_SWITCH_CPU_PORT), SW_PORT_PCS_CTRL, 0x10);
	ws_sw_write(SW_PORT(4), SW_PORT_RGMII_TIMING, 0x81e7);
	ws_sw_read(SW_PORT(WS_SWITCH_CPU_PORT), SW_PORT_RGMII_TIMING, &reg);
	ws_sw_write(SW_PORT(WS_SWITCH_CPU_PORT), SW_PORT_RGMII_TIMING,
		    reg | 0x18);
	ws_sw_write(SW_PORT(4), SW_PORT_RGMII_TIMING, 0xc1e7);

	/* Force the CPU port to RGMII 1000 full duplex, then link up */
	ws_sw_write(SW_PORT(WS_SWITCH_CPU_PORT), SW_PORT_PCS_CTRL, 0x1e);
	ws_sw_write(SW_PORT(WS_SWITCH_CPU_PORT), SW_PORT_PCS_CTRL, 0x3e);

	for (port = 0; port < WS_SWITCH_NUM_PORTS; port++) {
		if (port != WS_SWITCH_CPU_PORT) {
			/* Power up the port PHY (port 4 has PHY 6) */
			u8 phy = (port == 4) ? 6 : port;

			if (!ws_phy_read(phy, MII_BMCR, &reg))
				ws_phy_write(phy, MII_BMCR, reg & ~BMCR_PDOWN);
		}

		/* All ports forwarding: a plain switch while in U-Boot */
		if (!ws_sw_read(SW_PORT(port), SW_PORT_CTRL, &reg))
			ws_sw_write(SW_PORT(port), SW_PORT_CTRL,
				    reg | SW_PORT_CTRL_FORWARDING);
	}

	printf("88E6%03x switch initialized on %s\n", id >> 4, WS_ETH_NAME);

	/* Port status (bit 11: link up), to see which sockets are in use */
	printf("Switch port status:");
	for (port = 0; port < WS_SWITCH_NUM_PORTS; port++)
		if (!ws_sw_read(SW_PORT(port), 0x0, &reg))
			printf(" %d:%04x", port, reg);
	printf("\n");
}
#endif

#if defined(CONFIG_CMD_NAND)
/*
 * Copy the whole NAND to RAM block by block. Unlike "nand read", which skips
 * bad blocks and so shifts everything after them, bad blocks are replaced by
 * 0xff: every offset of the backup matches the flash.
 */
static int
do_nandbackup(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct mtd_info *mtd = get_nand_dev_by_index(0);
	int bad = 0, ecc = 0;
	ulong addr;
	loff_t off;

	if (argc < 2)
		return CMD_RET_USAGE;
	if (!mtd) {
		puts("No NAND device\n");
		return CMD_RET_FAILURE;
	}

	addr = hextoul(argv[1], NULL);
	for (off = 0; off < mtd->size; off += mtd->erasesize) {
		u8 *buf = map_sysmem(addr + off, mtd->erasesize);
		size_t len = mtd->erasesize;

		if (nand_block_isbad(mtd, off)) {
			memset(buf, 0xff, mtd->erasesize);
			bad++;
		} else if (nand_read(mtd, off, &len, buf) == -EBADMSG) {
			/* keep the data as read, but report it */
			ecc++;
		}
		unmap_sysmem(buf);

		if (!(off & (SZ_4M - 1)))
			printf("\rnandbackup: %llu MiB", off >> 20);
	}

	env_set_hex("filesize", mtd->size);
	printf("\rnandbackup: %llu MiB, %d bad block(s) filled with 0xff, %d block(s) with ECC errors\n",
	       mtd->size >> 20, bad, ecc);
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(nandbackup, 2, 0, do_nandbackup,
	   "copy the whole NAND to RAM, bad blocks as 0xff",
	   "addr - sets $filesize"
);
#endif

#if defined(CONFIG_KIRKWOOD_GPIO)
/* Return GPIO button status */
static int
do_ws(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc < 3 || strcmp(argv[1], "button") != 0)
		return CMD_RET_USAGE;

	if (strcmp(argv[2], "rear") == 0)
		/* invert GPIO result for intuitive while/until use */
		return !kw_gpio_get_value(WIRELESS_SPACE_REAR_BUTTON);
	else if (strcmp(argv[2], "front") == 0)
		return kw_gpio_get_value(WIRELESS_SPACE_FRONT_BUTTON);

	return CMD_RET_USAGE;
}

U_BOOT_CMD(ws, 3, 0, do_ws,
	   "Return GPIO button status 0=off 1=on",
	   "button rear|front - test buttons' states"
);
#endif
