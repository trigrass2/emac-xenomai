// SPDX-License-Identifier: GPL-2.0
/*
 * Microchip KSZ8863 switch driver
 *
 * Copyright (C) 2019 Microchip Technology Inc.
 *	Tristram Ha <Tristram.Ha@microchip.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iopoll.h>
#include <linux/platform_data/microchip-ksz.h>
#include <linux/phy.h>
#include <linux/if_bridge.h>
#include <net/dsa.h>
#include <net/switchdev.h>

#include "ksz_priv.h"
#include "ksz8863_reg.h"
#include "ksz_common.h"

static const struct {
	char string[ETH_GSTRING_LEN];
} ksz8863_mib_names[TOTAL_SWITCH_COUNTER_NUM] = {
	{ "rx" },
	{ "rx_hi" },
	{ "rx_undersize" },
	{ "rx_fragments" },
	{ "rx_oversize" },
	{ "rx_jabbers" },
	{ "rx_symbol_err" },
	{ "rx_crc_err" },
	{ "rx_align_err" },
	{ "rx_mac_ctrl" },
	{ "rx_pause" },
	{ "rx_bcast" },
	{ "rx_mcast" },
	{ "rx_ucast" },
	{ "rx_64_or_less" },
	{ "rx_65_127" },
	{ "rx_128_255" },
	{ "rx_256_511" },
	{ "rx_512_1023" },
	{ "rx_1024_1522" },
	{ "tx" },
	{ "tx_hi" },
	{ "tx_late_col" },
	{ "tx_pause" },
	{ "tx_bcast" },
	{ "tx_mcast" },
	{ "tx_ucast" },
	{ "tx_deferred" },
	{ "tx_total_col" },
	{ "tx_exc_col" },
	{ "tx_single_col" },
	{ "tx_mult_col" },
	{ "rx_discards" },
	{ "tx_discards" },
};

static int ksz8863_reset_switch(struct ksz_device *dev)
{
	u8 data;

	/* reset switch */
	ksz_read8(dev, REG_SW_RESET, &data);
	ksz_write8(dev, REG_SW_RESET, data | GLOBAL_SOFTWARE_RESET);
	udelay(1);
	ksz_write8(dev, REG_SW_RESET, data);

	return 0;
}

static void ksz8863_set_prio_queue(struct ksz_device *dev, int port, int queue)
{
	/* Number of queues can only be 1, 2, or 4. */
	switch (queue) {
	case 4:
		ksz_port_cfg(dev, port, P_2_QUEUE_CTRL, PORT_2_QUEUES_ENABLE,
			     false);
		ksz_port_cfg(dev, port, P_4_QUEUE_CTRL, PORT_4_QUEUES_ENABLE,
			     true);
		break;
	case 2:
		ksz_port_cfg(dev, port, P_2_QUEUE_CTRL, PORT_2_QUEUES_ENABLE,
			     true);
		ksz_port_cfg(dev, port, P_4_QUEUE_CTRL, PORT_4_QUEUES_ENABLE,
			     false);
		break;
	default:
		ksz_port_cfg(dev, port, P_2_QUEUE_CTRL, PORT_2_QUEUES_ENABLE,
			     false);
		ksz_port_cfg(dev, port, P_4_QUEUE_CTRL, PORT_4_QUEUES_ENABLE,
			     false);
	}
}

#define HW_DELAY(reg)	\
do { \
	u16 dummy; \
	ksz_read16(dev, reg, &dummy); \
} while (0);

static void ksz8863_r_mib_cnt(struct ksz_device *dev, int port, u16 addr,
			      u64 *cnt)
{
	u32 data;
	u16 ctrl_addr;
	int loop;

	ctrl_addr = addr + SWITCH_COUNTER_NUM * port;
	ctrl_addr |= IND_ACC_TABLE(TABLE_MIB | TABLE_READ);

	mutex_lock(&dev->alu_mutex);
	ksz_write16(dev, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(REG_IND_CTRL_0);

	/* It is almost guaranteed to always read the valid bit because of
	 * slow SPI speed.
	 */
	for (loop = 2; loop > 0; loop--) {
		ksz_read32(dev, REG_IND_DATA_LO, &data);

		if (data & MIB_COUNTER_VALID) {
			if (data & MIB_COUNTER_OVERFLOW)
				*cnt += MIB_COUNTER_VALUE + 1;
			*cnt += data & MIB_COUNTER_VALUE;
			break;
		}
	}
	mutex_unlock(&dev->alu_mutex);
}

static void ksz8863_r_mib_pkt(struct ksz_device *dev, int port, u16 addr,
			      u64 *dropped, u64 *cnt)
{
	u32 cur;
	u32 data;
	u16 ctrl_addr;
	u32 *last = (u32 *)dropped;

	addr -= SWITCH_COUNTER_NUM;
	ctrl_addr = addr ? KS_MIB_PACKET_DROPPED_TX_0 :
			   KS_MIB_PACKET_DROPPED_RX_0;
	ctrl_addr += port;
	ctrl_addr |= IND_ACC_TABLE(TABLE_MIB | TABLE_READ);

	mutex_lock(&dev->alu_mutex);
	ksz_write16(dev, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(REG_IND_CTRL_0);
	ksz_read32(dev, REG_IND_DATA_LO, &data);
	mutex_unlock(&dev->alu_mutex);

	data &= MIB_PACKET_DROPPED;
	cur = last[addr];
	if (data != cur) {
		last[addr] = data;
		if (data < cur)
			data += MIB_PACKET_DROPPED + 1;
		data -= cur;
		*cnt += data;
	}
}

static void ksz8863_port_init_cnt(struct ksz_device *dev, int port)
{
	struct ksz_port_mib *mib = &dev->ports[port].mib;
	u64 *dropped;

	mib->cnt_ptr = 0;

	/* Some ports may not have MIB counters before SWITCH_COUNTER_NUM. */
	while (mib->cnt_ptr < dev->reg_mib_cnt) {
		dev->dev_ops->r_mib_cnt(dev, port, mib->cnt_ptr,
					&mib->counters[mib->cnt_ptr]);
		++mib->cnt_ptr;
	}

	/* last one in storage */
	dropped = &mib->counters[dev->mib_cnt];

	/* Some ports may not have MIB counters after SWITCH_COUNTER_NUM. */
	while (mib->cnt_ptr < dev->mib_cnt) {
		dev->dev_ops->r_mib_pkt(dev, port, mib->cnt_ptr,
					dropped, &mib->counters[mib->cnt_ptr]);
		++mib->cnt_ptr;
	}
	mib->cnt_ptr = 0;
	memset(mib->counters, 0, dev->mib_cnt * sizeof(u64));
}

static void ksz8863_r_table(struct ksz_device *dev, int table, u16 addr,
			    u32 *data_lo, u32 *data_hi)
{
	u16 ctrl_addr;

	ctrl_addr = IND_ACC_TABLE(table | TABLE_READ) | addr;

	mutex_lock(&dev->alu_mutex);
	ksz_write16(dev, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(REG_IND_CTRL_0);
	if (data_hi)
		ksz_read32(dev, REG_IND_DATA_HI, data_hi);
	ksz_read32(dev, REG_IND_DATA_LO, data_lo);
	mutex_unlock(&dev->alu_mutex);
}

static void ksz8863_w_table(struct ksz_device *dev, int table, u16 addr,
			    u32* data_lo, u32 *data_hi)
{
	u16 ctrl_addr;

	ctrl_addr = IND_ACC_TABLE(table) | addr;

	mutex_lock(&dev->alu_mutex);
	if (data_hi)
		ksz_write32(dev, REG_IND_DATA_HI, *data_hi);
	ksz_write32(dev, REG_IND_DATA_LO, *data_lo);
	ksz_write16(dev, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(REG_IND_CTRL_0);
	mutex_unlock(&dev->alu_mutex);
}

#define read8_op(addr)	\
({ \
	u8 data8; \
	ksz_read8(dev, addr, &data8); \
	data8; \
})

static int ksz8863_valid_dyn_entry(struct ksz_device *dev, u8 *data)
{
	readx_poll_timeout(read8_op, REG_IND_DATA_CHECK, *data,
			   !(*data & DYNAMIC_MAC_TABLE_NOT_READY), 0, 100);

	/* Entry is not ready for accessing. */
	if (*data & DYNAMIC_MAC_TABLE_NOT_READY) {
		return -EAGAIN;
	/* Entry is ready for accessing. */
	} else {
		/* There is no valid entry in the table. */
		if (*data & DYNAMIC_MAC_TABLE_MAC_EMPTY)
			return -ENXIO;
	}
	return 0;
}

static int ksz8863_r_dyn_mac_table(struct ksz_device *dev, u16 addr,
				   u8 *mac_addr, u8 *fid, u8 *src_port,
				   u8 *timestamp, u16 *entries)
{
	u16 ctrl_addr;
	int rc;
	u8 data;

	ctrl_addr = IND_ACC_TABLE(TABLE_DYNAMIC_MAC | TABLE_READ) | addr;

	mutex_lock(&dev->alu_mutex);
	ksz_write16(dev, REG_IND_CTRL_0, ctrl_addr);
	HW_DELAY(REG_IND_CTRL_0);

	rc = ksz8863_valid_dyn_entry(dev, &data);
	if (rc == -EAGAIN) {
		if (addr == 0)
			*entries = 0;
	} else if (rc == -ENXIO) {
		*entries = 0;
	/* At least one valid entry in the table. */
	} else {
		u32 data_hi;
		u32 data_lo;
		int cnt;

		ksz_read32(dev, REG_IND_DATA_HI, &data_hi);
		ksz_read32(dev, REG_IND_DATA_LO, &data_lo);

		/* Check out how many valid entry in the table. */
		cnt = data & DYNAMIC_MAC_TABLE_ENTRIES_H;
		cnt <<= DYNAMIC_MAC_ENTRIES_H_S;
		cnt |= (data_hi & DYNAMIC_MAC_TABLE_ENTRIES) >>
			DYNAMIC_MAC_ENTRIES_S;
		*entries = cnt + 1;

		*fid = (u8)(data_hi & DYNAMIC_MAC_TABLE_FID) >>
			DYNAMIC_MAC_FID_S;
		*src_port = (u8)(data_hi & DYNAMIC_MAC_TABLE_SRC_PORT) >>
			DYNAMIC_MAC_SRC_PORT_S;
		*timestamp = (u8)(data_hi & DYNAMIC_MAC_TABLE_TIMESTAMP) >>
			DYNAMIC_MAC_TIMESTAMP_S;

		mac_addr[5] = (u8)data_lo;
		mac_addr[4] = (u8)(data_lo >> 8);
		mac_addr[3] = (u8)(data_lo >> 16);
		mac_addr[2] = (u8)(data_lo >> 24);

		mac_addr[1] = (u8)data_hi;
		mac_addr[0] = (u8)(data_hi >> 8);
		rc = 0;
	}
	mutex_unlock(&dev->alu_mutex);

	return rc;
}

static int ksz8863_r_sta_mac_table(struct ksz_device *dev, u16 addr,
				   struct alu_struct *alu)
{
	u32 data_hi;
	u32 data_lo;

	ksz8863_r_table(dev, TABLE_STATIC_MAC, addr, &data_lo, &data_hi);
	if (data_hi & (STATIC_MAC_TABLE_VALID | STATIC_MAC_TABLE_OVERRIDE)) {
		alu->mac[5] = (u8)data_lo;
		alu->mac[4] = (u8)(data_lo >> 8);
		alu->mac[3] = (u8)(data_lo >> 16);
		alu->mac[2] = (u8)(data_lo >> 24);
		alu->mac[1] = (u8)data_hi;
		alu->mac[0] = (u8)(data_hi >> 8);
		alu->port_forward = (data_hi & STATIC_MAC_TABLE_FWD_PORTS) >>
			STATIC_MAC_FWD_PORTS_S;
		alu->is_override =
			(data_hi & STATIC_MAC_TABLE_OVERRIDE) ? 1 : 0;
		alu->is_use_fid = (data_hi & STATIC_MAC_TABLE_USE_FID) ? 1 : 0;
		alu->fid = (data_hi & STATIC_MAC_TABLE_FID) >>
			STATIC_MAC_FID_S;
		return 0;
	}
	return -ENXIO;
}

static void ksz8863_w_sta_mac_table(struct ksz_device *dev, u16 addr,
				    struct alu_struct *alu)
{
	u32 data_hi;
	u32 data_lo;

	data_lo = ((u32)alu->mac[2] << 24) |
		((u32)alu->mac[3] << 16) |
		((u32)alu->mac[4] << 8) | alu->mac[5];
	data_hi = ((u32)alu->mac[0] << 8) | alu->mac[1];
	data_hi |= (u32)alu->port_forward << STATIC_MAC_FWD_PORTS_S;

	if (alu->is_override)
		data_hi |= STATIC_MAC_TABLE_OVERRIDE;
	if (alu->is_use_fid) {
		data_hi |= STATIC_MAC_TABLE_USE_FID;
		data_hi |= (u32)alu->fid << STATIC_MAC_FID_S;
	}
	if (alu->is_static)
		data_hi |= STATIC_MAC_TABLE_VALID;
	else
		data_hi &= ~STATIC_MAC_TABLE_OVERRIDE;

	ksz8863_w_table(dev, TABLE_STATIC_MAC, addr, &data_lo, &data_hi);
}

static void ksz8863_from_vlan(u32 vlan, u16 *vid, u8 *fid, u8 *member,
				     u8 *valid)
{
	*vid = vlan & VLAN_TABLE_VID;
	*fid = (vlan & VLAN_TABLE_FID) >> VLAN_TABLE_FID_S;
	*member = (vlan & VLAN_TABLE_MEMBERSHIP) >> VLAN_TABLE_MEMBERSHIP_S;
	*valid = !!(vlan & VLAN_TABLE_VALID);
}

static void ksz8863_to_vlan(u16 vid, u8 fid, u8 member, u8 valid,
				   u32 *vlan)
{
	*vlan = vid;
	*vlan |= (u32)fid << VLAN_TABLE_FID_S;
	*vlan |= (u32)member << VLAN_TABLE_MEMBERSHIP_S;
	if (valid)
		*vlan |= VLAN_TABLE_VALID;
}

static int ksz8863_get_vlan(struct ksz_device *dev, u16 vlan)
{
	u16 vid;
	u8 fid;
	u8 member;
	u8 valid;
	int addr;

	for (addr = 1; addr < dev->num_vlans; addr++) {
		ksz8863_from_vlan(dev->vlan_cache[addr].table[0], &vid, &fid,
				  &member, &valid);
		if (vlan == vid)
			return addr;
	}
	for (addr = 1; addr < dev->num_vlans; addr++) {
		ksz8863_from_vlan(dev->vlan_cache[addr].table[0], &vid, &fid,
				  &member, &valid);
		if (!valid)
			return addr;
	}
	return -1;
}

static void ksz8863_r_vlan_entries(struct ksz_device *dev, u16 addr)
{
	u32 data;

	ksz8863_r_table(dev, TABLE_VLAN, addr, &data, NULL);
	dev->vlan_cache[addr].table[0] = data;
}

static void ksz8863_r_vlan_table(struct ksz_device *dev, u16 addr, u32 *vlan)
{
	ksz8863_r_table(dev, TABLE_VLAN, addr, vlan, NULL);
}

static void ksz8863_w_vlan_table(struct ksz_device *dev, u16 addr, u32 vlan)
{
	dev->vlan_cache[addr].table[0] = vlan;
	ksz8863_w_table(dev, TABLE_VLAN, addr, &vlan, NULL);
}

#define KSZ8863_SW_ID		0x8863
#define PHY_ID_KSZ8863_SW	((KSZ8863_ID_HI << 16) | KSZ8863_SW_ID)

static void ksz8863_r_phy(struct ksz_device *dev, u16 phy, u16 reg, u16 *val)
{
	struct ksz_port *port;
	u8 ctrl;
	u8 restart;
	u8 link;
	u8 speed;
	u8 p = phy;
	u16 data = 0;
	int processed = true;

	port = &dev->ports[p];
	switch (reg) {
	case PHY_REG_CTRL:
		ksz_pread8(dev, p, P_PHY_CTRL, &ctrl);
		ksz_pread8(dev, p, P_NEG_RESTART_CTRL, &restart);
		ksz_pread8(dev, p, P_SPEED_STATUS, &speed);
		if (restart & PORT_LOOPBACK)
			data |= PHY_LOOPBACK;
		if (ctrl & PORT_FORCE_100_MBIT)
			data |= PHY_SPEED_100MBIT;
		if (ctrl & PORT_AUTO_NEG_ENABLE)
			data |= PHY_AUTO_NEG_ENABLE;
		if (restart & PORT_POWER_DOWN)
			data |= PHY_POWER_DOWN;
		if (restart & PORT_AUTO_NEG_RESTART)
			data |= PHY_AUTO_NEG_RESTART;
		if (ctrl & PORT_FORCE_FULL_DUPLEX)
			data |= PHY_FULL_DUPLEX;
		if (speed & PORT_HP_MDIX)
			data |= PHY_HP_MDIX;
		if (restart & PORT_FORCE_MDIX)
			data |= PHY_FORCE_MDIX;
		if (restart & PORT_AUTO_MDIX_DISABLE)
			data |= PHY_AUTO_MDIX_DISABLE;
		if (restart & PORT_TX_DISABLE)
			data |= PHY_TRANSMIT_DISABLE;
		if (restart & PORT_LED_OFF)
			data |= PHY_LED_DISABLE;
		break;
	case PHY_REG_STATUS:
		ksz_pread8(dev, p, P_LINK_STATUS, &link);
		ksz_pread8(dev, p, P_SPEED_STATUS, &speed);
		data = PHY_100BTX_FD_CAPABLE |
		       PHY_100BTX_CAPABLE |
		       PHY_10BT_FD_CAPABLE |
		       PHY_10BT_CAPABLE |
		       PHY_AUTO_NEG_CAPABLE;
		if (link & PORT_AUTO_NEG_COMPLETE)
			data |= PHY_AUTO_NEG_ACKNOWLEDGE;
		if (link & PORT_STAT_LINK_GOOD)
			data |= PHY_LINK_STATUS;
		if (speed & PORT_REMOTE_FAULT)
			data |= PHY_REMOTE_FAULT;
		break;
	case PHY_REG_ID_1:
		data = KSZ8863_ID_HI;
		break;
	case PHY_REG_ID_2:
		data = KSZ8863_ID_LO;
		data = KSZ8863_SW_ID;
		break;
	case PHY_REG_AUTO_NEGOTIATION:
		ksz_pread8(dev, p, P_PHY_CTRL, &ctrl);
		data = PHY_AUTO_NEG_802_3;
		if (ctrl & PORT_AUTO_NEG_SYM_PAUSE)
			data |= PHY_AUTO_NEG_SYM_PAUSE;
		if (ctrl & PORT_AUTO_NEG_100BTX_FD)
			data |= PHY_AUTO_NEG_100BTX_FD;
		if (ctrl & PORT_AUTO_NEG_100BTX)
			data |= PHY_AUTO_NEG_100BTX;
		if (ctrl & PORT_AUTO_NEG_10BT_FD)
			data |= PHY_AUTO_NEG_10BT_FD;
		if (ctrl & PORT_AUTO_NEG_10BT)
			data |= PHY_AUTO_NEG_10BT;
		break;
	case PHY_REG_REMOTE_CAPABILITY:
		ksz_pread8(dev, p, P_LINK_STATUS, &link);
		data = PHY_AUTO_NEG_802_3;
		if (link & PORT_REMOTE_SYM_PAUSE)
			data |= PHY_AUTO_NEG_SYM_PAUSE;
		if (link & PORT_REMOTE_100BTX_FD)
			data |= PHY_AUTO_NEG_100BTX_FD;
		if (link & PORT_REMOTE_100BTX)
			data |= PHY_AUTO_NEG_100BTX;
		if (link & PORT_REMOTE_10BT_FD)
			data |= PHY_AUTO_NEG_10BT_FD;
		if (link & PORT_REMOTE_10BT)
			data |= PHY_AUTO_NEG_10BT;
		break;
	default:
		processed = false;
		break;
	}
	if (processed)
		*val = data;
}

static void ksz8863_w_phy(struct ksz_device *dev, u16 phy, u16 reg, u16 val)
{
	u8 ctrl;
	u8 restart;
	u8 speed;
	u8 data;
	u8 p = phy;

	switch (reg) {
	case PHY_REG_CTRL:

		/* Do not support PHY reset function. */
		if (val & PHY_RESET_NOT)
			break;
		ksz_pread8(dev, p, P_SPEED_STATUS, &speed);
		data = speed;
		if (val & PHY_HP_MDIX)
			data |= PORT_HP_MDIX;
		else
			data &= ~PORT_HP_MDIX;
		if (data != speed)
			ksz_pwrite8(dev, p, P_SPEED_STATUS, data);
		ksz_pread8(dev, p, P_PHY_CTRL, &ctrl);
		data = ctrl;
		if (val & PHY_AUTO_NEG_ENABLE)
			data |= PORT_AUTO_NEG_ENABLE;
		else
			data &= ~PORT_AUTO_NEG_ENABLE;

		/* Fiber port does not support auto-negotiation. */
		if (dev->ports[p].fiber)
			data &= ~PORT_AUTO_NEG_ENABLE;
		if (val & PHY_SPEED_100MBIT)
			data |= PORT_FORCE_100_MBIT;
		else
			data &= ~PORT_FORCE_100_MBIT;
		if (val & PHY_FULL_DUPLEX)
			data |= PORT_FORCE_FULL_DUPLEX;
		else
			data &= ~PORT_FORCE_FULL_DUPLEX;
		if (data != ctrl)
			ksz_pwrite8(dev, p, P_PHY_CTRL, data);
		ksz_pread8(dev, p, P_NEG_RESTART_CTRL, &restart);
		data = restart;
		if (val & PHY_LED_DISABLE)
			data |= PORT_LED_OFF;
		else
			data &= ~PORT_LED_OFF;
		if (val & PHY_TRANSMIT_DISABLE)
			data |= PORT_TX_DISABLE;
		else
			data &= ~PORT_TX_DISABLE;
		if (val & PHY_AUTO_NEG_RESTART)
			data |= PORT_AUTO_NEG_RESTART;
		else
			data &= ~(PORT_AUTO_NEG_RESTART);
		if (val & PHY_POWER_DOWN)
			data |= PORT_POWER_DOWN;
		else
			data &= ~PORT_POWER_DOWN;
		if (val & PHY_AUTO_MDIX_DISABLE)
			data |= PORT_AUTO_MDIX_DISABLE;
		else
			data &= ~PORT_AUTO_MDIX_DISABLE;
		if (val & PHY_FORCE_MDIX)
			data |= PORT_FORCE_MDIX;
		else
			data &= ~PORT_FORCE_MDIX;
		if (val & PHY_LOOPBACK)
			data |= PORT_LOOPBACK;
		else
			data &= ~PORT_LOOPBACK;
		if (data != restart)
			ksz_pwrite8(dev, p, P_NEG_RESTART_CTRL, data);
		break;
	case PHY_REG_AUTO_NEGOTIATION:
		ksz_pread8(dev, p, P_PHY_CTRL, &ctrl);
		data = ctrl;
		data &= ~(PORT_AUTO_NEG_SYM_PAUSE |
			  PORT_AUTO_NEG_100BTX_FD |
			  PORT_AUTO_NEG_100BTX |
			  PORT_AUTO_NEG_10BT_FD |
			  PORT_AUTO_NEG_10BT);
		if (val & PHY_AUTO_NEG_SYM_PAUSE)
			data |= PORT_AUTO_NEG_SYM_PAUSE;
		if (val & PHY_AUTO_NEG_100BTX_FD)
			data |= PORT_AUTO_NEG_100BTX_FD;
		if (val & PHY_AUTO_NEG_100BTX)
			data |= PORT_AUTO_NEG_100BTX;
		if (val & PHY_AUTO_NEG_10BT_FD)
			data |= PORT_AUTO_NEG_10BT_FD;
		if (val & PHY_AUTO_NEG_10BT)
			data |= PORT_AUTO_NEG_10BT;
		if (data != ctrl)
			ksz_pwrite8(dev, p, P_PHY_CTRL, data);
		break;
	default:
		break;
	}
}

static enum dsa_tag_protocol ksz8863_get_tag_protocol(struct dsa_switch *ds)
{
	return DSA_TAG_PROTO_KSZ;
}

static void ksz8863_get_strings(struct dsa_switch *ds, int port, uint8_t *buf)
{
	int i;

	for (i = 0; i < TOTAL_SWITCH_COUNTER_NUM; i++) {
		memcpy(buf + i * ETH_GSTRING_LEN, ksz8863_mib_names[i].string,
		       ETH_GSTRING_LEN);
	}
}

static const u8 stp_multicast_addr[] = {
	0x01, 0x80, 0xC2, 0x00, 0x00, 0x00
};

static void ksz8863_cfg_port_member(struct ksz_device *dev, int port,
				    u8 member)
{
	u8 data;

	ksz_pread8(dev, port, P_MIRROR_CTRL, &data);
	data &= ~PORT_VLAN_MEMBERSHIP;
	data |= (member & dev->port_mask);
	ksz_pwrite8(dev, port, P_MIRROR_CTRL, data);
	dev->ports[port].member = member;
}

static void ksz8863_port_stp_state_set(struct dsa_switch *ds, int port,
				       u8 state)
{
	struct ksz_device *dev = ds->priv;
	struct ksz_port *p = &dev->ports[port];
	u8 data;
	int member = -1;
	int forward = dev->member;

	ksz_pread8(dev, port, P_STP_CTRL, &data);
	data &= ~(PORT_TX_ENABLE | PORT_RX_ENABLE | PORT_LEARN_DISABLE);

	switch (state) {
	case BR_STATE_DISABLED:
		data |= PORT_LEARN_DISABLE;
		if (port < SWITCH_PORT_NUM)
			member = 0;
		break;
	case BR_STATE_LISTENING:
		data |= (PORT_RX_ENABLE | PORT_LEARN_DISABLE);
		if (port < SWITCH_PORT_NUM &&
		    p->stp_state == BR_STATE_DISABLED)
			member = dev->host_mask | p->vid_member;
		break;
	case BR_STATE_LEARNING:
		data |= PORT_RX_ENABLE;
		break;
	case BR_STATE_FORWARDING:
		data |= (PORT_TX_ENABLE | PORT_RX_ENABLE);

		/* This function is also used internally. */
		if (port == dev->cpu_port)
			break;

		member = dev->host_mask | p->vid_member;

		/* Port is a member of a bridge. */
		if (dev->br_member & (1 << port)) {
			dev->member |= (1 << port);
			member = dev->member;
		}
		break;
	case BR_STATE_BLOCKING:
		data |= PORT_LEARN_DISABLE;
		if (port < SWITCH_PORT_NUM &&
		    p->stp_state == BR_STATE_DISABLED)
			member = dev->host_mask | p->vid_member;
		break;
	default:
		dev_err(ds->dev, "invalid STP state: %d\n", state);
		return;
	}

	ksz_pwrite8(dev, port, P_STP_CTRL, data);
	p->stp_state = state;
	if (data & PORT_RX_ENABLE)
		dev->rx_ports |= (1 << port);
	else
		dev->rx_ports &= ~(1 << port);
	if (data & PORT_TX_ENABLE)
		dev->tx_ports |= (1 << port);
	else
		dev->tx_ports &= ~(1 << port);

	/* Port membership may share register with STP state. */
	if (member >= 0 && member != p->member)
		dev->dev_ops->cfg_port_member(dev, port, (u8)member);

	/* Check if forwarding needs to be updated. */
	if (state != BR_STATE_FORWARDING) {
		if (dev->br_member & (1 << port))
			dev->member &= ~(1 << port);
	}

	/* When topology has changed the function ksz_update_port_member
	 * should be called to modify port forwarding behavior.
	 */
	if (forward != dev->member)
		ksz_update_port_member(dev, port);
}

static void ksz8863_flush_dyn_mac_table(struct ksz_device *dev, int port)
{
	int cnt;
	int first;
	int index;
	u8 learn[TOTAL_PORT_NUM];

	if ((uint)port < TOTAL_PORT_NUM) {
		first = port;
		cnt = port + 1;
	} else {
		/* Flush all ports. */
		first = 0;
		cnt = dev->mib_port_cnt;
	}
	for (index = first; index < cnt; index++) {
		ksz_pread8(dev, index, P_STP_CTRL, &learn[index]);
		if (!(learn[index] & PORT_LEARN_DISABLE))
			ksz_pwrite8(dev, index, P_STP_CTRL,
				    learn[index] | PORT_LEARN_DISABLE);
	}
	ksz_cfg(dev, S_FLUSH_TABLE_CTRL, SW_FLUSH_DYN_MAC_TABLE, true);
	for (index = first; index < cnt; index++) {
		if (!(learn[index] & PORT_LEARN_DISABLE))
			ksz_pwrite8(dev, index, P_STP_CTRL, learn[index]);
	}
}

static int ksz8863_port_vlan_filtering(struct dsa_switch *ds, int port,
				       bool flag)
{
	struct ksz_device *dev = ds->priv;
	u16 vlan_ports = dev->vlan_ports;

	if (flag)
		dev->vlan_ports |= (1 << port);
	else
		dev->vlan_ports &= ~(1 << port);
	if ((flag && !vlan_ports) ||
	    (!flag && !dev->vlan_ports && dev->vlan_up)) {
		ksz_cfg(dev, S_MIRROR_CTRL, SW_VLAN_ENABLE, flag);
		ksz_port_cfg(dev, dev->cpu_port, P_TAG_CTRL, PORT_INSERT_TAG,
			     false);
		dev->vlan_up = flag;
	}

	return 0;
}

static void ksz8863_port_vlan_add(struct dsa_switch *ds, int port,
				  const struct switchdev_obj_port_vlan *vlan,
				  struct switchdev_trans *trans)
{
	struct ksz_device *dev = ds->priv;
	u32 data;
	int addr;
	u16 vid;
	u8 fid;
	u8 member;
	u8 valid;
	u16 tmp;
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	u16 new_pvid = 1;

	if (!dev->vlan_up)
		return;

	ksz_port_cfg(dev, port, P_TAG_CTRL, PORT_REMOVE_TAG, untagged);

	for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {

		/* VID 1 is reserved. */
		if (vid == 1)
			continue;

		/* change PVID */
		if (pvid)
			new_pvid = vid;
		addr = ksz8863_get_vlan(dev, vid);
		if (addr < 0)
			return;
		ksz8863_r_vlan_table(dev, addr, &data);
		ksz8863_from_vlan(data, &tmp, &fid, &member, &valid);

		/* First time to setup the VLAN entry. */
		if (!valid) {
			/* Need to find a way to map VID to FID. */
			fid = (u8)addr;
			valid = 1;
		}
		member |= BIT(port);
		member |= dev->host_mask;

		ksz8863_to_vlan(vid, fid, member, valid, &data);
		ksz8863_w_vlan_table(dev, addr, data);
	}

	ksz_pread16(dev, port, REG_PORT_CTRL_VID, &vid);
	if (new_pvid != (vid & 0xfff)) {
		vid &= ~0xfff;
		vid |= new_pvid;
		ksz_pwrite16(dev, port, REG_PORT_CTRL_VID, vid);

		/* Switch may use lookup to forward unicast frame. */
		dev->dev_ops->flush_dyn_mac_table(dev, port);
		if (!dev->vid_ports)
			ksz_port_cfg(dev, dev->cpu_port, P_TAG_CTRL,
				     PORT_INSERT_TAG, true);
		dev->vid_ports |= (1 << port);
	}
}

static int ksz8863_port_vlan_del(struct dsa_switch *ds, int port,
				 const struct switchdev_obj_port_vlan *vlan)
{
	struct ksz_device *dev = ds->priv;
	u32 data;
	int addr;
	u16 vid;
	u16 pvid;
	u8 fid;
	u8 member;
	u8 valid;
	u16 tmp;
	u16 new_pvid = 0;

	if (!dev->vlan_up)
		return 0;

	ksz_pread16(dev, port, REG_PORT_CTRL_VID, &pvid);
	pvid = pvid & 0xFFF;

	for (vid = vlan->vid_begin; vid <= vlan->vid_end; vid++) {

		/* VID 1 is reserved. */
		if (vid == 1)
			continue;
		addr = ksz8863_get_vlan(dev, vid);
		if (addr < 0)
			return 0;
		ksz8863_r_vlan_table(dev, addr, &data);
		ksz8863_from_vlan(data, &tmp, &fid, &member, &valid);

		member &= ~BIT(port);

		/* Invalidate the entry if no more member. */
		if (!(member & ~dev->host_mask)) {
			fid = 0;
			valid = 0;
		}

		if (pvid == vid)
			new_pvid = 1;

		ksz8863_to_vlan(vid, fid, member, valid, &data);
		ksz8863_w_vlan_table(dev, addr, data);
	}

	if (new_pvid && new_pvid != pvid) {
		ksz_pwrite16(dev, port, REG_PORT_CTRL_VID, new_pvid);

		/* Switch may use lookup to forward unicast frame. */
		dev->dev_ops->flush_dyn_mac_table(dev, port);
		dev->vid_ports &= ~(1 << port);
		if (!dev->vid_ports)
			ksz_port_cfg(dev, dev->cpu_port, P_TAG_CTRL,
				     PORT_INSERT_TAG, false);
	}

	return 0;
}

#if 0
static int ksz8863_port_mirror_add(struct dsa_switch *ds, int port,
				   struct dsa_mall_mirror_tc_entry *mirror,
				   bool ingress)
{
	struct ksz_device *dev = ds->priv;

	if (ingress) {
		ksz_port_cfg(dev, port, P_MIRROR_CTRL, PORT_MIRROR_RX, true);
		dev->mirror_rx |= (1 << port);
	} else {
		ksz_port_cfg(dev, port, P_MIRROR_CTRL, PORT_MIRROR_TX, true);
		dev->mirror_tx |= (1 << port);
	}

	ksz_port_cfg(dev, port, P_MIRROR_CTRL, PORT_MIRROR_SNIFFER, false);

	/* configure mirror port */
	if (dev->mirror_rx || dev->mirror_tx)
		ksz_port_cfg(dev, mirror->to_local_port, P_MIRROR_CTRL,
			     PORT_MIRROR_SNIFFER, true);

	return 0;
}

static void ksz8863_port_mirror_del(struct dsa_switch *ds, int port,
				    struct dsa_mall_mirror_tc_entry *mirror)
{
	struct ksz_device *dev = ds->priv;
	u8 data;

	if (mirror->ingress) {
		ksz_port_cfg(dev, port, P_MIRROR_CTRL, PORT_MIRROR_RX, false);
		dev->mirror_rx &= ~(1 << port);
	} else {
		ksz_port_cfg(dev, port, P_MIRROR_CTRL, PORT_MIRROR_TX, false);
		dev->mirror_tx &= ~(1 << port);
	}

	ksz_pread8(dev, port, P_MIRROR_CTRL, &data);

	if (!dev->mirror_rx && !dev->mirror_tx)
		ksz_port_cfg(dev, mirror->to_local_port, P_MIRROR_CTRL,
			     PORT_MIRROR_SNIFFER, false);
}
#endif

static void ksz8863_phy_setup(struct ksz_device *dev, int port,
			      struct phy_device *phy)
{
	/* SUPPORTED_Pause can be removed to disable flow control when
	 * rate limiting is used.
	 */
	phy->supported &= ~SUPPORTED_Asym_Pause;
	phy->supported |= SUPPORTED_Pause;
	phy->advertising = phy->supported;
}

static void ksz8863_port_setup(struct ksz_device *dev, int port, bool cpu_port)
{
	u8 member;
	struct ksz_port *p = &dev->ports[port];

	/* enable broadcast storm limit */
	ksz_port_cfg(dev, port, P_BCAST_STORM_CTRL, PORT_BROADCAST_STORM, true);

	ksz8863_set_prio_queue(dev, port, 4);

	/* disable DiffServ priority */
	ksz_port_cfg(dev, port, P_PRIO_CTRL, PORT_DIFFSERV_ENABLE, false);

	/* replace priority */
	ksz_port_cfg(dev, port, P_MIRROR_CTRL, PORT_802_1P_REMAPPING, false);

	/* enable 802.1p priority */
	ksz_port_cfg(dev, port, P_PRIO_CTRL, PORT_802_1P_ENABLE, true);

	if (cpu_port) {
		member = dev->port_mask;
		dev->on_ports = dev->host_mask;
		dev->live_ports = dev->host_mask;
	} else {
		member = dev->host_mask | p->vid_member;
		dev->on_ports |= (1 << port);

		/* Link was detected before port is enabled. */
		if (p->phydev.link)
			dev->live_ports |= (1 << port);
	}
	dev->dev_ops->cfg_port_member(dev, port, member);
}

static void ksz8863_config_cpu_port(struct dsa_switch *ds)
{
	struct ksz_device *dev = ds->priv;
	struct ksz_port *p;
	int i;
	u8 copper;

	ksz_cfg(dev, S_TAIL_TAG_CTRL, SW_TAIL_TAG_ENABLE, true);

	p = &dev->ports[dev->cpu_port];
	p->vid_member = dev->port_mask;
	p->on = 1;

	ksz8863_port_setup(dev, dev->cpu_port, true);
	dev->member = dev->host_mask;

	for (i = 0; i < SWITCH_PORT_NUM; i++) {
		p = &dev->ports[i];

		/* Initialize to non-zero so that ksz_cfg_port_member() will
		 * be called.
		 */
		p->vid_member = (1 << i);
		p->member = dev->port_mask;
		ksz8863_port_stp_state_set(ds, i, BR_STATE_DISABLED);
		p->on = 1;
		p->phy = 1;
	}
	ksz_read8(dev, REG_MODE_INDICATOR, &copper);
	if (!(copper & PORT_1_COPPER))
		dev->ports[0].fiber = 1;
	if (!(copper & PORT_2_COPPER))
		dev->ports[1].fiber = 1;
	for (i = 0; i < dev->phy_port_cnt; i++) {
		p = &dev->ports[i];
		if (!p->on)
			continue;
		if (p->fiber)
			ksz_port_cfg(dev, i, P_STP_CTRL, PORT_FORCE_FLOW_CTRL,
				     true);
		else
			ksz_port_cfg(dev, i, P_STP_CTRL, PORT_FORCE_FLOW_CTRL,
				     false);
	}
}

static int ksz8863_setup(struct dsa_switch *ds)
{
	u8 data8;
	u16 data16;
	u32 value;
	int i;
	struct alu_struct alu;
	struct ksz_device *dev = ds->priv;
	int ret = 0;

	dev->vlan_cache = devm_kcalloc(dev->dev, sizeof(struct vlan_table),
				       dev->num_vlans, GFP_KERNEL);
	if (!dev->vlan_cache)
		return -ENOMEM;

	ret = ksz8863_reset_switch(dev);
	if (ret) {
		dev_err(ds->dev, "failed to reset switch\n");
		return ret;
	}

	ksz_cfg(dev, S_REPLACE_VID_CTRL, SW_FLOW_CTRL, true);

	/* Enable automatic fast aging when link changed detected. */
	ksz_cfg(dev, S_LINK_AGING_CTRL, SW_LINK_AUTO_AGING, true);

	ksz_read8(dev, REG_SW_CTRL_1, &data8);

	/* Enable aggressive back off algorithm in half duplex mode. */
	data8 |= SW_AGGR_BACKOFF;
	ksz_write8(dev, REG_SW_CTRL_1, data8);

	ksz_read8(dev, REG_SW_CTRL_2, &data8);

	/* Make sure unicast VLAN boundary is set as default. */
	data8 |= UNICAST_VLAN_BOUNDARY;

	/* Enable no excessive collision drop. */
	data8 |= NO_EXC_COLLISION_DROP;
	ksz_write8(dev, REG_SW_CTRL_2, data8);

	ksz8863_config_cpu_port(ds);

	ksz_cfg(dev, REG_SW_CTRL_2, MULTICAST_STORM_DISABLE, true);

	ksz_cfg(dev, S_REPLACE_VID_CTRL, SW_REPLACE_VID, false);

	ksz_cfg(dev, S_MIRROR_CTRL, SW_MIRROR_RX_TX, false);

	/* set broadcast storm protection 10% rate */
	data8 = BROADCAST_STORM_PROT_RATE;
	value = ((u32)BROADCAST_STORM_VALUE * data8) / 100;
	if (value > BROADCAST_STORM_RATE)
		value = BROADCAST_STORM_RATE;
	ksz_read16(dev, S_REPLACE_VID_CTRL, &data16);
	data16 &= ~BROADCAST_STORM_RATE;
	data16 |= value;
	ksz_write16(dev, S_REPLACE_VID_CTRL, data16);

	for (i = 0; i < dev->num_vlans; i++)
		ksz8863_r_vlan_entries(dev, i);

	/* All VLAN entries are set to use VID 1. */
	for (i = 1; i < dev->num_vlans; i++) {
		dev->vlan_cache[i].table[0] = 0;
		ksz8863_w_vlan_table(dev, i, 0);
	}

	/* Setup STP address for STP operation. */
	memset(&alu, 0, sizeof(alu));
	memcpy(alu.mac, stp_multicast_addr, ETH_ALEN);
	alu.is_static = true;
	alu.is_override = true;
	alu.port_forward = dev->host_mask;

	ksz8863_w_sta_mac_table(dev, 0, &alu);

	ksz_init_mib_timer(dev);

	return 0;
}

static struct dsa_switch_ops ksz8863_switch_ops = {
	.get_tag_protocol	= ksz8863_get_tag_protocol,
	.setup			= ksz8863_setup,
	.phy_read		= ksz_phy_read16,
	.phy_write		= ksz_phy_write16,
	.adjust_link		= ksz_adjust_link,
	.port_enable		= ksz_enable_port,
	.port_disable		= ksz_disable_port,
	.get_strings		= ksz8863_get_strings,
	.get_ethtool_stats	= ksz_get_ethtool_stats,
	.get_sset_count		= ksz_sset_count,
	.port_bridge_join	= ksz_port_bridge_join,
	.port_bridge_leave	= ksz_port_bridge_leave,
	.port_stp_state_set	= ksz8863_port_stp_state_set,
	.port_fast_age		= ksz_port_fast_age,
	.port_vlan_filtering	= ksz8863_port_vlan_filtering,
	.port_vlan_prepare	= ksz_port_vlan_prepare,
	.port_vlan_add		= ksz8863_port_vlan_add,
	.port_vlan_del		= ksz8863_port_vlan_del,
	.port_fdb_dump		= ksz_port_fdb_dump,
	.port_mdb_prepare       = ksz_port_mdb_prepare,
	.port_mdb_add           = ksz_port_mdb_add,
	.port_mdb_del           = ksz_port_mdb_del,
#if 0
	.port_mirror_add	= ksz8863_port_mirror_add,
	.port_mirror_del	= ksz8863_port_mirror_del,
#endif
};

#define KSZ8863_REGS_SIZE		0x100

static struct bin_attribute ksz8863_registers_attr = {
	.attr = {
		.name	= "registers",
		.mode	= 00600,
	},
	.size	= KSZ8863_REGS_SIZE,
	.read	= ksz_registers_read,
	.write	= ksz_registers_write,
};

#define KSZ_CHIP_NAME_SIZE		18

static const char *ksz8863_chip_names[KSZ_CHIP_NAME_SIZE] = {
	"Microchip KSZ8863",
	"Microchip KSZ8873",
};

enum {
	KSZ8863_SW_CHIP,
	KSZ8873_SW_CHIP,
};

static int kszphy_config_init(struct phy_device *phydev)
{
	return 0;
}

static struct phy_driver ksz8863_phy_driver[] = {
{
	.phy_id		= PHY_ID_KSZ8863_SW,
	.phy_id_mask	= 0x00ffffff,
	.name		= "Microchip KSZ8863",
	.features	= PHY_BASIC_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_init	= kszphy_config_init,
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
}};

static int ksz8863_switch_detect(struct ksz_device *dev)
{
	u16 id16;
	u8 id1;
	u8 id2;
	int ret;
	int chip = -1;

	/* read chip id */
	ret = ksz_read16(dev, REG_CHIP_ID0, &id16);
	if (ret)
		return ret;

	id1 = id16 >> 8;
	id2 = id16 & SW_CHIP_ID_M;
	if (id1 != FAMILY_ID ||
	    (id2 != CHIP_ID_63))
		return -ENODEV;

	dev->mib_port_cnt = TOTAL_PORT_NUM;
	dev->phy_port_cnt = SWITCH_PORT_NUM;
	dev->port_cnt = SWITCH_PORT_NUM;

	chip = KSZ8863_SW_CHIP;
	ret = ksz_read8(dev, REG_MODE_INDICATOR, &id1);
	if (!(id1 & (PORT_1_COPPER | PORT_2_COPPER)) ||
	    !(id1 & MODE_2_PHY))
		chip = KSZ8873_SW_CHIP;
	if (chip >= 0) {
		dev->name = ksz8863_chip_names[chip];
		strlcpy(ksz8863_phy_driver[0].name, ksz8863_chip_names[chip],
			KSZ_CHIP_NAME_SIZE);
	}
	id2 = 0x63;

	id16 = 0x8800;
	id16 |= id2;
	dev->chip_id = id16;

	dev->cpu_port = dev->mib_port_cnt - 1;
	dev->host_mask = (1 << dev->cpu_port);

	return 0;
}

struct ksz_chip_data {
	u16 chip_id;
	const char *dev_name;
	int num_vlans;
	int num_alus;
	int num_statics;
	int cpu_ports;
	int port_cnt;
};

static const struct ksz_chip_data ksz8863_switch_chips[] = {
	{
		.chip_id = 0x8863,
		.dev_name = "KSZ8863",
		.num_vlans = 16,
		.num_alus = 0,
		.num_statics = 8,
		.cpu_ports = 0x4,	/* can be configured as cpu port */
		.port_cnt = 2,		/* total physical port count */
	},
};

static int ksz8863_switch_init(struct ksz_device *dev)
{
	int i;

	dev->ds->ops = &ksz8863_switch_ops;

	for (i = 0; i < ARRAY_SIZE(ksz8863_switch_chips); i++) {
		const struct ksz_chip_data *chip = &ksz8863_switch_chips[i];

		if (dev->chip_id == chip->chip_id) {
			if (!dev->name)
				dev->name = chip->dev_name;
			dev->num_vlans = chip->num_vlans;
			dev->num_alus = chip->num_alus;
			dev->num_statics = chip->num_statics;
			dev->port_cnt = chip->port_cnt;
			dev->cpu_ports = chip->cpu_ports;

			break;
		}
	}

	/* no switch found */
	if (!dev->cpu_ports)
		return -ENODEV;

	dev->port_mask = (1 << dev->port_cnt) - 1;
	dev->port_mask |= dev->host_mask;

	dev->reg_mib_cnt = SWITCH_COUNTER_NUM;
	dev->mib_cnt = TOTAL_SWITCH_COUNTER_NUM;

	i = dev->mib_port_cnt;
	dev->ports = devm_kzalloc(dev->dev, sizeof(struct ksz_port) * i,
				  GFP_KERNEL);
	if (!dev->ports)
		return -ENOMEM;
	for (i = 0; i < dev->mib_port_cnt; i++) {
		mutex_init(&dev->ports[i].mib.cnt_mutex);
		dev->ports[i].mib.counters =
			devm_kzalloc(dev->dev,
				     sizeof(u64) *
				     (TOTAL_SWITCH_COUNTER_NUM + 1),
				     GFP_KERNEL);
		if (!dev->ports[i].mib.counters)
			return -ENOMEM;
	}
	i = phy_drivers_register(ksz8863_phy_driver,
				 ARRAY_SIZE(ksz8863_phy_driver), THIS_MODULE);

	dev->regs_size = KSZ8863_REGS_SIZE;
	i = sysfs_create_bin_file(&dev->dev->kobj,
				  &ksz8863_registers_attr);

	return 0;
}

static void ksz8863_switch_exit(struct ksz_device *dev)
{
	sysfs_remove_bin_file(&dev->dev->kobj, &ksz8863_registers_attr);
	phy_drivers_unregister(ksz8863_phy_driver,
			       ARRAY_SIZE(ksz8863_phy_driver));
	ksz8863_reset_switch(dev);
}

static const struct ksz_dev_ops ksz8863_dev_ops = {
	.cfg_port_member = ksz8863_cfg_port_member,
	.flush_dyn_mac_table = ksz8863_flush_dyn_mac_table,
	.phy_setup = ksz8863_phy_setup,
	.port_setup = ksz8863_port_setup,
	.r_phy = ksz8863_r_phy,
	.w_phy = ksz8863_w_phy,
	.r_dyn_mac_table = ksz8863_r_dyn_mac_table,
	.r_sta_mac_table = ksz8863_r_sta_mac_table,
	.w_sta_mac_table = ksz8863_w_sta_mac_table,
	.r_mib_cnt = ksz8863_r_mib_cnt,
	.r_mib_pkt = ksz8863_r_mib_pkt,
	.port_init_cnt = ksz8863_port_init_cnt,
	.shutdown = ksz8863_reset_switch,
	.detect = ksz8863_switch_detect,
	.init = ksz8863_switch_init,
	.exit = ksz8863_switch_exit,
};

static void ksz8863_set_tag(struct ksz_device *dev, void *ptr, u8 *addr, int p)
{
	u8 *tag = (u8 *)ptr;

	*tag = 1 << p;
}

static const struct ksz_tag_ops ksz8863_tag_ops = {
	.set_tag = ksz8863_set_tag,
};

int ksz8863_switch_register(struct ksz_device *dev)
{
	return ksz_switch_register(dev, &ksz8863_dev_ops, &ksz8863_tag_ops);
}
EXPORT_SYMBOL(ksz8863_switch_register);

MODULE_AUTHOR("Tristram Ha <Tristram.Ha@microchip.com>");
MODULE_DESCRIPTION("Microchip KSZ8863 Series Switch DSA Driver");
MODULE_LICENSE("GPL v2");
