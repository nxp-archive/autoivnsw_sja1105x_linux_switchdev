/*
 * Copyright 2017 - 2018 NXP
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * \file nxp_sja1105p_switchdev.c
 *
 * \author Marco Hartmann
 *
 * \date 2017-08-02
 *
 */

#include <linux/module.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/rtnetlink.h>
#include <net/switchdev.h>
#include <net/netlink.h>
#include <linux/of_mdio.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 1)
#include <linux/sched.h>
#else
#include <linux/sched/signal.h>
#endif

#include "NXP_SJA1105P_addressResolutionTable.h"
#include "NXP_SJA1105P_diagnostics.h"
#include "NXP_SJA1105P_vlan.h"
#include "NXP_SJA1105P_config.h"
#include "NXP_SJA1105P_portConfig.h"
#include "sja1105p_init.h"

#include "sja1105p_switchdev.h"

/*******************************************************************************
 * Macros and Datatypes
 ******************************************************************************/
#define PRODUCT_NAME "SJA1105P"
#define PORT_NAME_LEN 22U
#define ARL_TABLE_SIZE 1024U
#define DTS_NAME_LEN 8U

/* Linux version dependent macros */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
#define PHYDEV_DEV dev
#define PHYDEV_ADDR addr
#else
#define PHYDEV_DEV mdio.dev
#define PHYDEV_ADDR mdio.addr
#endif

/* added with Linux commit d643a75ac2bcc559994405d29c50ed086aeae434 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 13, 1)
#define SET_SWDEV_OPS(ndev, ops) ndev->switchdev_ops = &ops
#else
#define SET_SWDEV_OPS(ndev, ops) SWITCHDEV_SET_OPS(ndev, &ops)
#endif

enum { UP, DOWN } linkstatus_t;

/* struct declarations */
typedef struct {
	uint64_t rx_packets;
	uint64_t tx_packets;
	uint64_t rx_bytes;
	uint64_t tx_bytes;
	uint64_t rx_errors;
	uint64_t tx_errors;
	uint32_t rx_crc_errors;
	uint32_t rx_length_errors;
	uint32_t polerr;
	uint32_t vlanerr;
	uint32_t n664err;
	uint32_t not_reach;
	uint32_t egr_disabled;
	uint32_t part_drop;
	uint32_t qfull;
	uint32_t addr_not_learned_drop;
	uint32_t empty_route_drop;
	uint32_t illegal_double_drop;
	uint32_t double_tagged_drop;
	uint32_t single_outer_drop;
	uint32_t single_inner_drop;
	uint32_t untagged_drop;
	SJA1105P_macLevelErrors_t p_macLevelErrors;
} nxp_port_stats_t;

typedef struct {
	unsigned long ppid;
	int port_num;
	unsigned char *base_mac;
	int link_state;
	int link_speed;
	struct port_desc *physical_port;
	int physical_port_num;
	nxp_port_stats_t stats;
	struct delayed_work stats_work;
} nxp_port_data_t;

/* forward declarations */
void copy_mac(const unsigned char *from, unsigned char *to);

/*******************************************************************************
 * Callback Implementations
 ******************************************************************************/

/****************************netdev ops****************************************/

/* Adds an FDB entry to dev for addr */
static int nxp_port_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
			    struct net_device *netdev,
			    const unsigned char *addr, const u16 vid, u16 nlm_flags)
{
	int err;
	nxp_port_data_t *nxp_port;
	SJA1105P_addressResolutionTableEntry_t read_entry = {0}, new_entry = {0};
	SJA1105P_extendedAddressResolutionTableEntry_t read_entry_ext = {0}, new_entry_ext = {0};

	nxp_port = netdev_priv(netdev);

	if (verbosity > 1)
		netdev_alert(
			netdev,
			"%s called [%d]! Add [%pM] in vlan [%x] to device [%s], flags [%x]\n",
			__func__, nxp_port->port_num, addr, vid, netdev->name,
			nlm_flags);

	/* TCAM is searched by MAC and VID, so fill out those fields */
	copy_mac(addr, (char*)&read_entry.dstMacAddress);
	read_entry_ext.dstMacAddressMask = 0xFFFFFFFFFFFF;
	read_entry_ext.vlanIdMask        = (vid == 0) ? 0x000 : 0xFFF; /* if VLAN ID is unset, adjust TCAM Search mask to ignore VLAN ID field */
	read_entry_ext.innerOuterVlan    = 1;
	read_entry.p_extension           = &read_entry_ext;
	read_entry.vlanId                = vid;

	/* in case there already exists an entry corresponding to the MAC or VID,
	 * retrieve it to add current port to the port mask
	 */
	err = SJA1105P_readArlTableEntryByAddress(&read_entry);
	if (err)
		goto sja1105p_read_error;

	/* if a matching entry was found: it needs to be deleted,
	 * and an updated entry needs to be added
	 */
	if (read_entry.index < SJA1105P_N_ARL_ENTRIES) {
		/* Entry sanity check:
		 * if the previous entry had VLAN configured (i.e. mask is 0xFFF),
		 * but new entry has a dont care value (vid is unset)
		 * or vice versa: abort the entry insertion
		 */
		if ((read_entry.p_extension->vlanIdMask != 0 && vid == 0) || (read_entry.p_extension->vlanIdMask == 0 && vid != 0)) {
			netdev_err(netdev, "Could not add table entry: There already exists a conflicting entry!");
			return -EINVAL;
		}

		if (verbosity > 1)
			netdev_alert(netdev, "Matching FDB entry found, replacing\n");

		err = SJA1105P_removeArlTableEntryByIndex(&read_entry);
		if (err)
			goto sja1105p_delete_error;
	} else {
		if (verbosity > 1)
			netdev_alert(netdev, "No matching FDB entry found, creating new entry\n");
	}

	/* update table entry */
	copy_mac(addr, (char*)&new_entry.dstMacAddress);
	new_entry_ext.dstMacAddressMask = 0xFFFFFFFFFFFF;
	new_entry_ext.vlanIdMask        = (vid == 0) ? 0x000 : 0xFFF; /* if VLAN ID is unset, adjust TCAM Search mask to ignore VLAN ID field */
	new_entry_ext.innerOuterVlan    = 1;
	new_entry.p_extension           = &new_entry_ext;
	new_entry.vlanId                = vid;
	new_entry.ports                 = read_entry.ports | (1 << nxp_port->port_num);
	new_entry.enabled               = 1;
	new_entry.enforcePorts          = 0;
	new_entry.index                 = 0; /* Will be set on return */

	/* add new or updated entry to arl table */
	err = SJA1105P_addArlTableEntry(&new_entry);
	if (err)
		goto sja1105p_write_error;

	return 0;

sja1105p_read_error:
	netdev_err(netdev, "Could not read entry from arl table of sja1105p!");
	return err;

sja1105p_delete_error:
	netdev_err(netdev, "Could not delete entry from arl table of sja1105p!");
	return err;

sja1105p_write_error:
	netdev_err(netdev, "Could not add entry to arl table of sja1105p!");
	return err;
}

/* Deletes the FDB entry from dev coresponding to addr */
static int nxp_port_fdb_del(struct ndmsg *ndm, struct nlattr *tb[],
			    struct net_device *netdev,
			    const unsigned char *addr, u16 vid)
{
	int err;
	nxp_port_data_t *nxp_port;
	SJA1105P_addressResolutionTableEntry_t read_entry = {0}, new_entry = {0};
	SJA1105P_extendedAddressResolutionTableEntry_t read_entry_ext = {0}, new_entry_ext = {0};

	nxp_port = netdev_priv(netdev);

	if (verbosity > 1)
		netdev_alert(
			netdev,
			"%s called [%d]! Del [%pM] in vlan [%x] from device [%s]\n",
			__func__, nxp_port->port_num, addr, vid, netdev->name);

	copy_mac(addr, (char*)&read_entry.dstMacAddress);
	read_entry_ext.dstMacAddressMask = 0xFFFFFFFFFFFF;
	read_entry_ext.vlanIdMask        = (vid == 0) ? 0x000 : 0xFFF; /* if VLAN ID is unset, adjust TCAM Search mask to ignore VLAN ID field */
	read_entry_ext.innerOuterVlan    = 1;
	read_entry.p_extension           = &read_entry_ext;
	read_entry.vlanId                = vid;

	err = SJA1105P_readArlTableEntryByAddress(&read_entry);
	if (err)
		goto sja1105p_entry_not_found;

	/* delete outdated entry */
	err = SJA1105P_removeArlTableEntryByIndex(&read_entry);
	if (err)
		goto sja1105p_delete_error;

	/* disable current port */
	read_entry.ports &= ~(1 << nxp_port->port_num);
	
	/* if some  other port is still active:
	 * re-add the modified entry to the table
	 */
	if (read_entry.ports) {
		if (verbosity > 1)
			netdev_alert(
				netdev,
				"deactivated port, upload modified entry\n");

		copy_mac(addr, (char*)&new_entry.dstMacAddress);
		new_entry_ext.dstMacAddressMask = 0xFFFFFFFFFFFF;
		new_entry_ext.vlanIdMask        = (vid == 0) ? 0x000 : 0xFFF; /* if VLAN ID is unset, adjust TCAM Search mask to ignore VLAN ID field */
		new_entry_ext.innerOuterVlan    = 1;
		new_entry.p_extension           = &new_entry_ext;
		new_entry.vlanId                = vid;
		new_entry.ports                 = read_entry.ports;
		new_entry.enabled               = 1;
		new_entry.enforcePorts          = 0;
		new_entry.index                 = 0; /* Will be set on return */

		err = SJA1105P_addArlTableEntry(&new_entry);
		if (err)
			goto sja1105p_write_error;
	}

	return 0;

sja1105p_entry_not_found:
	netdev_err(netdev, "Entry does not exist!");
	return err;

sja1105p_write_error:
	netdev_err(netdev, "Could not add entry to arl table of sja1105p!");
	return err;

sja1105p_delete_error:
	netdev_err(netdev, "Could not delete entry from arl table of sja1105p!");
	return err;
}

static int nxp_send_netlink_msg(struct net_device *netdev, struct sk_buff *skb,
				struct netlink_callback *cb, int vid,
				char *mac_addr)
{
	int portid, seq, type, flags;
	struct nlmsghdr *nlmsg_header;
	struct ndmsg *ndm;

	/* numerical identifier that is assigned by Netlink,
	 * different port-ID values are used to identify several socket
	 * channels opened by the same user-space process
	 */
	portid = NETLINK_CB(cb->skb).portid;

	/* the message sequence number */
	seq = cb->nlh->nlmsg_seq;

	/* NEWNEIGH := New or updated neighbour entry */
	type = RTM_NEWNEIGH;

	/* NLM_F_MULTI := multipart message */
	flags = NLM_F_MULTI;

	/* Add a new netlink message to the skb */
	nlmsg_header = nlmsg_put(skb, portid, seq, type, sizeof(*ndm), flags);
	if (!nlmsg_header)
		return -EMSGSIZE;

	/* get the head of the message payload */
	ndm = nlmsg_data(nlmsg_header);

	/* fill out ndmsg structure (RTM_NEWNEIGH indicates presence of ndm) */
	ndm->ndm_family = AF_BRIDGE; /* Multiprotocol bridge */
	ndm->ndm_pad1 = 0;
	ndm->ndm_pad2 = 0;
	ndm->ndm_flags = NTF_SELF;
	ndm->ndm_type = 0;
	ndm->ndm_ifindex = netdev->ifindex; /* interface index */
	ndm->ndm_state = NUD_REACHABLE; /* Network Unreachability Detection */

	/* add addr and vid as additional attributes */
	if (nla_put(skb, NDA_LLADDR, ETH_ALEN, mac_addr))
		goto nla_put_failure;

	if (vid && nla_put_u16(skb, NDA_VLAN, vid))
		goto nla_put_failure;

	nlmsg_end(skb, nlmsg_header);
	return 0;

nla_put_failure:
	nlmsg_cancel(skb, nlmsg_header);
	return -EMSGSIZE;
}

/* signature changed in Linux commit d297653dd6f07afbe7e6c702a4bcd7615680002e */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 1)
#define FDB_IDX (idx)
#define FDB_RET idx
#else
#define FDB_IDX (*idx)
#define FDB_RET 0
#endif
/* Used to add FDB entries to dump requests. Implementers should add entries
 * to skb and update idx with the number of entries
 */
static int nxp_port_fdb_dump(struct sk_buff *skb, struct netlink_callback *cb,
			     struct net_device *netdev,
			     struct net_device *filter_dev, int FDB_IDX)
{
	int vid;
	unsigned char mac_addr[6];

	int index, err;
	nxp_port_data_t *nxp_port;

	nxp_port = netdev_priv(netdev);

	if (verbosity > 1)
		netdev_alert(netdev,
			     "%s called (%d)! idx [%d], arg is [%ld]%s\n",
			     __func__, nxp_port->port_num, FDB_IDX, cb->args[0],
			     ((FDB_IDX < cb->args[0]) ? " (skipping)" : ""));

	for (index = 0; index < ARL_TABLE_SIZE; index++) {
		SJA1105P_addressResolutionTableEntry_t entry = {0};

		/* fdb dump may take a long time,
		 * so handle user interrupts gracefully
		 */
		if (fatal_signal_pending(current))
			goto signal_interrupt;

		entry.index = index;

		/* get table entry at position index from arl table */
		err = SJA1105P_readArlTableEntryByIndex(&entry);
		if (err)
			goto sja1105p_read_error;

		/* skip without incrementing idx if entry is not valid (empty)
		 * or belongs to a different port
		 */
		if (!entry.enabled
		    || !(entry.ports & BIT(nxp_port->port_num))) {
			continue;
		}

		/* On the first pass, cb->args[0] is always 0, idx is
		 * incremented with every valid table entry.
		 * Skip the netlink send on second pass,
		 * cb->args[0] now holds the total number
		 * of entries for all devices,
		 * idx must still be incremented per valid table entry.
		 */
		if (FDB_IDX < cb->args[0]) {
			FDB_IDX++;
			continue;
		}

		vid = entry.vlanId;
		copy_mac((unsigned char*)&entry.dstMacAddress, mac_addr);

		if (verbosity > 1)
			netdev_alert(
				netdev,
				"discovered [%pM] in vlan [%x] on device [%s]\n",
				mac_addr, vid, netdev->name);

		/* send data via netlink message */
		err = nxp_send_netlink_msg(netdev, skb, cb, vid, mac_addr);
		if (err)
			goto send_error;

		FDB_IDX++;
	}

	return FDB_RET;

sja1105p_read_error:
	netdev_err(netdev, "Could not read table entry from sja1105p!\n");
	return err;

send_error:
	netdev_err(netdev, "nla put faillure, could not send netlink msg\n");
	return err;

signal_interrupt:
	netdev_err(netdev, "FDB dump interrupted by user signal\n");
	return FDB_RET;
}

#undef FDB_IDX
#undef FDB_RET

static int nxp_port_get_phys_port_name(struct net_device *netdev, char *buf,
				       size_t len)
{
	nxp_port_data_t *nxp_port;

	nxp_port = netdev_priv(netdev);

	if (snprintf(buf, len, netdev->name) >= len)
		return -EINVAL;

	if (verbosity > 5) {
		netdev_alert(netdev, "%s called for [%d], name is [%s]\n",
			     __func__, nxp_port->port_num, buf);
	}

	return 0;
}

static void set_port_linkstatus(struct net_device *netdev, int status)
{
	int needlock, flags;

	flags = dev_get_flags(netdev);
	if (status == UP)
		flags |= IFF_UP;
	else
		flags &= ~IFF_UP;

	/* dev_change_flags requires rtnl mutex to be locked when called.
	 * register_netdev calls nxp_get_stats with rtnl locked already,
	 * syscalls however (for example by ifconfig)
	 * do not lock rtnl before calling nxp_get_stats.
	 * Thus we must check if the mutex is locked,
	 * and only lock it if it is not (which defeats its purpose)
	 */
	needlock = !rtnl_is_locked();
	if (needlock)
		rtnl_lock();

	netdev->operstate = (status == UP) ? IF_OPER_UP : IF_OPER_DOWN;
	dev_change_flags(netdev, flags);

	if (needlock)
		rtnl_unlock();
}

static void nxp_update_stats(struct work_struct *work)
{
	int err;
	struct net_device *netdev;
	struct delayed_work *dwork = to_delayed_work(work);

	nxp_port_data_t *nxp_port =
		container_of(dwork, nxp_port_data_t, stats_work);

	netdev = nxp_port->physical_port->netdev;

	/* get stats from switch using the sja1105p drv functions */
	err = SJA1105P_get64bitEtherStatCounter(
		SJA1105P_e_etherStat64_N_OCTETS, &nxp_port->stats.tx_bytes, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_EGRESS);

	err += SJA1105P_get64bitEtherStatCounter(
		SJA1105P_e_etherStat64_N_PKTS, &nxp_port->stats.tx_packets, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_EGRESS);

	err += SJA1105P_get64bitEtherStatCounter(
		SJA1105P_e_etherStat64_N_OCTETS, &nxp_port->stats.rx_bytes, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_INGRESS);

	err += SJA1105P_get64bitEtherStatCounter(
		SJA1105P_e_etherStat64_N_PKTS, &nxp_port->stats.rx_packets, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_INGRESS);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_CRCERR, &nxp_port->stats.rx_crc_errors,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_SIZEERR, &nxp_port->stats.rx_length_errors,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_NOT_REACH, &nxp_port->stats.not_reach,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_EGR_DISABLED, &nxp_port->stats.egr_disabled,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_PART_DROP, &nxp_port->stats.part_drop,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_QFULL, &nxp_port->stats.qfull, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_POLERR, &nxp_port->stats.polerr, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_VLANERR, &nxp_port->stats.vlanerr, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_N664ERR, &nxp_port->stats.n664err, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_ADDR_NOT_LEARNED_DROP,
		&nxp_port->stats.addr_not_learned_drop, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_EMPTY_ROUTE_DROP, &nxp_port->stats.empty_route_drop,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_ILLEGAL_DOUBLE_DROP,
		&nxp_port->stats.illegal_double_drop, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_DOUBLE_TAGGED_DROP,
		&nxp_port->stats.double_tagged_drop, nxp_port->port_num,
		SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_SINGLE_OUTER_DROP, &nxp_port->stats.single_outer_drop,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_SINGLE_INNER_DROP, &nxp_port->stats.single_inner_drop,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_get32bitEtherStatCounter(
		SJA1105P_e_etherStat32_N_UNTAGGED_DROP, &nxp_port->stats.untagged_drop,
		nxp_port->port_num, SJA1105P_e_etherStatDirection_BOTH);

	err += SJA1105P_getMacErrors(&nxp_port->stats.p_macLevelErrors, nxp_port->port_num);

	schedule_delayed_work(&nxp_port->stats_work, HZ);

	if (err)
		netdev_err(netdev, "Could not read stats from sja1105p!\n");


}

/* is a void function since Linux commit bc1f44709cf27fb2a5766cadafe7e2ad5e9cb221 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 1)
static struct rtnl_link_stats64 *nxp_get_stats(struct net_device *netdev,
					       struct rtnl_link_stats64 *storage)
#else
static void nxp_get_stats(struct net_device *netdev, struct rtnl_link_stats64 *storage)
#endif
{
	nxp_port_data_t *nxp_port = netdev_priv(netdev);

	/* indicate the linkstate of the port, show host port as always up */
	if (nxp_port->link_state || nxp_port->physical_port->is_host) {
		if (verbosity > 2)
			netdev_alert(netdev,
				     "Change link status of [%s] to: up\n",
				     netdev->name);

		netdev->flags |= IFF_UP;

		/* if flag change is not visible via dev_get_flags,
		 * use dev_change_flags to commit the change
		 */
		if (!(dev_get_flags(netdev) & IFF_UP))
			set_port_linkstatus(netdev, UP);
	} else {
		if (verbosity > 0)
			netdev_alert(netdev,
				     "Change link status of [%s] to: down\n",
				     netdev->name);

		netdev->flags &= ~IFF_UP;

		if (dev_get_flags(netdev) & IFF_UP)
			set_port_linkstatus(netdev, DOWN);
	}

	/* fill out the provided struct with cached values */
	storage->tx_bytes = nxp_port->stats.tx_bytes;
	storage->tx_packets = nxp_port->stats.tx_packets;
	storage->rx_bytes = nxp_port->stats.rx_bytes;
	storage->rx_packets = nxp_port->stats.rx_packets;

	storage->rx_crc_errors = nxp_port->stats.rx_crc_errors;
	storage->rx_length_errors = nxp_port->stats.rx_length_errors;
	storage->rx_frame_errors = nxp_port->stats.p_macLevelErrors.nAlignerr;

	storage->tx_errors = nxp_port->stats.not_reach + nxp_port->stats.egr_disabled;
	storage->rx_errors = nxp_port->stats.rx_crc_errors + nxp_port->stats.rx_length_errors + nxp_port->stats.part_drop
			   + nxp_port->stats.p_macLevelErrors.nSoferr
			   + nxp_port->stats.p_macLevelErrors.nMiierr
			   + nxp_port->stats.p_macLevelErrors.nAlignerr;

	storage->tx_dropped = nxp_port->stats.qfull;
	storage->rx_dropped = nxp_port->stats.part_drop + nxp_port->stats.polerr + nxp_port->stats.vlanerr + nxp_port->stats.n664err;
	storage->rx_dropped += nxp_port->stats.addr_not_learned_drop + nxp_port->stats.empty_route_drop
			     + nxp_port->stats.illegal_double_drop + nxp_port->stats.double_tagged_drop
			     + nxp_port->stats.single_outer_drop + nxp_port->stats.single_inner_drop
			     + nxp_port->stats.untagged_drop;

	if (verbosity > 3) {
		netdev_alert(
			netdev,
			"%s called for [%d]: txb [%llu], txp [%llu],"
			"rxb [%llu], rxp [%llu], rx_crc_errors[%u], rx_length_errors[%u],"
			"not_reach[%u], egr_disabled[%u], part_drop[%u], qfull[%u],"
			"polerr[%u], vlanerr[%u], n664err[%u]\n",
			__func__, nxp_port->port_num, nxp_port->stats.tx_bytes, nxp_port->stats.tx_packets,
			nxp_port->stats.rx_bytes, nxp_port->stats.rx_packets, nxp_port->stats.rx_crc_errors, nxp_port->stats.rx_length_errors,
			nxp_port->stats.not_reach, nxp_port->stats.egr_disabled, nxp_port->stats.part_drop, nxp_port->stats.qfull, nxp_port->stats.polerr,
			nxp_port->stats.vlanerr, nxp_port->stats.n664err);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 11, 1)
	return storage;
#else
	return;
#endif

}

/* this function is called when a VLAN id is registered */
static int nxp_port_vlan_rx_add_vid(struct net_device *netdev, __be16 proto,
				    u16 vid)
{
	int err, i;
	uint16_t ports_enable;
	nxp_port_data_t *nxp_port;
	SJA1105P_vlanForwarding_t vlanFwd[SJA1105P_N_LOGICAL_PORTS] = {0};

	nxp_port = netdev_priv(netdev);

	if (verbosity > 1)
		netdev_alert(netdev,
			     "%s called for [%d], vid is [%d], proto is [%d]\n",
			     __func__, nxp_port->port_num, vid, proto);

	/* retrieve current configuration from switch */
	err = SJA1105P_readVlanConfig(vid, vlanFwd, &ports_enable);

	if (err == 1) {
		/* configuration for the specified vid was not yet present,
		 * so create an initial configuration
		 */
		ports_enable = 0;
		for (i = 0; i < SJA1105P_N_LOGICAL_PORTS; i++) {
			vlanFwd[i] = SJA1105P_e_vlanForwarding_NOT;
		}
	}

	/* change forwarding status of current port */
	vlanFwd[nxp_port->port_num] = SJA1105P_e_vlanForwarding_TAGGED;

	/* enable output from the current port */
	ports_enable |= (1 << nxp_port->port_num);

	/* write modified cfg to switch */
	err = SJA1105P_writeVlanConfig(vid, vlanFwd, ports_enable);
	if (err)
		goto sja1105p_write_error;

	return 0;

sja1105p_write_error:
	netdev_err(netdev, "Could not write vlan cfg to sja1105p!\n");
	return err;
}

/* this function is called when a VLAN id is unregistered */
static int nxp_port_vlan_rx_kill_vid(struct net_device *netdev, __be16 proto,
				     u16 vid)
{
	int err;
	uint16_t ports_enable;
	nxp_port_data_t *nxp_port;
	SJA1105P_vlanForwarding_t vlanFwd[SJA1105P_N_LOGICAL_PORTS] = {0};

	nxp_port = netdev_priv(netdev);

	if (verbosity > 1)
		netdev_alert(netdev,
			     "%s called for [%d], vid is [%d], proto is [%d]\n",
			     __func__, nxp_port->port_num, vid, proto);

	/* retrieve current configuration from switch */
	err = SJA1105P_readVlanConfig(vid, vlanFwd, &ports_enable);

	/* only modify cfg if a cfg for the specified vid exists already */
	if (err == 0) {
		/* change forwarding status of current port */
		vlanFwd[nxp_port->port_num] = SJA1105P_e_vlanForwarding_NOT;

		/* disable output from the current port */
		ports_enable &= ~(1 << nxp_port->port_num);

		/* write modified cfg to switch */
		err = SJA1105P_writeVlanConfig(vid, vlanFwd, ports_enable);
		if (err)
			goto sja1105p_write_error;
	}

	return 0;

sja1105p_write_error:
	netdev_err(netdev, "Could not write vlan cfg to sja1105p!\n");
	return err;
}

/* Transmit a packet (called by the kernel)  */
static int nxp_port_tx(struct sk_buff *skb, struct net_device *dev)
{
	/* Not implemented */
	return -EOPNOTSUPP;
}

/*************************swdev ops********************************************/

/* led to get an ID of the switch chip this port is part of.
 * If driver implements this, it indicates that it represents a port of
 * a switch chip.
 */
static int nxp_port_swdev_parent_id_get(struct net_device *netdev,
					struct netdev_phys_item_id *psid)
{
	nxp_port_data_t *nxp_port = netdev_priv(netdev);
	u32 n = nxp_port->ppid;

	psid->id[0] = (n >> 24) & 0xFF;
	psid->id[1] = (n >> 16) & 0xFF;
	psid->id[2] = (n >>  8) & 0xFF;
	psid->id[3] = (n >>  0) & 0xFF;
	psid->id_len = 4;

	if (verbosity > 5)
		netdev_alert(netdev, "%s called for [%d], ppid is [%lu]\n",
			     __func__, nxp_port->port_num, nxp_port->ppid);

	return 0;
}

/* Called to notify switch device port of bridge port STP state change */
static int nxp_port_swdev_port_stp_update(struct net_device *netdev, u8 state)
{
	char *s;
	nxp_port_data_t *nxp_port;

	nxp_port = netdev_priv(netdev);

	/* implementation not needed, as sja1105p does not support STP */

	switch (state) {
	case BR_STATE_DISABLED:
		s = "DISABLED";
		break;
	case BR_STATE_LISTENING:
		s = "LISTENING";
		break;
	case BR_STATE_LEARNING:
		s = "LEARNING";
		break;
	case BR_STATE_FORWARDING:
		s = "FORWARDING";
		break;
	case BR_STATE_BLOCKING:
		s = "BLOCKING";
		break;
	default:
		s = "unknown_state";
	}
	if (verbosity > 3)
		netdev_alert(netdev, "%s called [%d], change state to [%s]!\n",
			     __func__, nxp_port->port_num, s);

	return 0;
}

/* attr based ops introduced with Linux commit 3094333d9089d43e8b8f0418676fa6ae06c27b51 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 2, 1)

static int nxp_port_obj_add(struct net_device *netdev,
			    const struct switchdev_obj *obj,
			    struct switchdev_trans *trans)
{
	/* Not implemented */
	return -EOPNOTSUPP;
}

static int nxp_port_obj_del(struct net_device *netdev,
			    const struct switchdev_obj *obj)
{
	/* Not implemented */
	return -EOPNOTSUPP;
}

static int nxp_port_attr_get(struct net_device *netdev,
			     struct switchdev_attr *attr)
{
	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_PARENT_ID:
		return nxp_port_swdev_parent_id_get(netdev, &attr->u.ppid);
	default:
		return -EOPNOTSUPP;
	}
}

static int nxp_port_attr_set(struct net_device *netdev,
			     const struct switchdev_attr *attr,
			     struct switchdev_trans *trans)
{
	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
		return nxp_port_swdev_port_stp_update(netdev,
						      attr->u.stp_state);
	default:
		return -EOPNOTSUPP;
	}
}

#endif

/********************************ethtool_ops***********************************/
static int nxp_port_get_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	int ret = -ENODEV;

	if (netdev->phydev) {
		struct phy_device *phydev = netdev->phydev;

		ecmd->speed = phydev->speed;
		ecmd->duplex = phydev->duplex;
		ecmd->autoneg = phydev->autoneg;
		ecmd->supported = phydev->supported;
		ecmd->advertising = phydev->advertising;
		ecmd->phy_address = phydev->PHYDEV_ADDR;

		ret = 0;
	}

	return ret;
}

/************** helper functions for link/speed transitions *******************/

/* update speed configuration of a given switch port */
static void nxp_update_speed(nxp_port_data_t *nxp_port,
			     struct phy_device *phydev)
{
	int speed_cfg, ret;

	switch (phydev->speed) {
	case 1000:
		speed_cfg = SJA1105P_e_speed_1_GBPS;
		break;
	case 100:
		speed_cfg = SJA1105P_e_speed_100_MBPS;
		break;
	case 10:
		speed_cfg = SJA1105P_e_speed_10_MBPS;
		break;
	default:
		pr_err("Unsupported speed %d\n", phydev->speed);
		return;
	}

	/* update speed settings */
	ret = SJA1105P_setSpeed(nxp_port->physical_port_num, nxp_port->ppid,
				speed_cfg);
	if (ret) {
		dev_err(&phydev->attached_dev->dev,
			"Error: Could not update Speed setting in MAC cfg Table\n");
		return;
	}

	/* update CGU settings */
	ret = SJA1105P_reconfigPort(nxp_port->physical_port_num, nxp_port->ppid,
				    speed_cfg);
	if (ret) {
		dev_err(&phydev->attached_dev->dev,
			"Error: Could not reconfigure the port\n");
	}
}

/* Called when link of port came up */
static void nxp_port_linkup(nxp_port_data_t *nxp_port, struct phy_device *phydev)
{
	SJA1105P_portStatusMiixArgument_t portStatus = {0};
	SJA1105P_getPortStatusMiix(&portStatus, nxp_port->physical_port_num,
				   nxp_port->ppid);

	/* check if link speed changed, and update switch accordingly */
	if (nxp_port->link_speed != phydev->speed) {
		if (verbosity > 0)
			dev_info(&phydev->PHYDEV_DEV,
				 "Link speed changed from %d to %d\n",
				 nxp_port->link_speed, phydev->speed);

		if (portStatus.xmiiMode == SJA1105P_e_xmiiMode_RGMII)
			nxp_update_speed(nxp_port, phydev);

		nxp_port->link_speed = phydev->speed;
	}

	/* RGMII ports require a reset of delay lines */
	if (portStatus.xmiiMode == SJA1105P_e_xmiiMode_RGMII) {
		/* only reset clock if there is a delay configured */
		if (nxp_port->physical_port->rx_delay > 0)
			SJA1105P_resetClockDelay(nxp_port->physical_port_num,
						 nxp_port->ppid,
						 SJA1105P_e_direction_RX);
		if (nxp_port->physical_port->tx_delay > 0)
			SJA1105P_resetClockDelay(nxp_port->physical_port_num,
						 nxp_port->ppid,
						 SJA1105P_e_direction_TX);
	}
}

/***************************link_state callback********************************/

/* called on phydev state machine changes */
static void nxp_adjust_link(struct net_device *netdev)
{
	nxp_port_data_t *nxp_port;
	struct phy_device *phydev;

	phydev = netdev->phydev;
	if (!phydev)
		return;

	if (verbosity > 3)
		netdev_alert(netdev,
			"%s called for %s: phy [%x]: state: [%x], link [%x]\n",
			__func__, netdev->name, phydev->PHYDEV_ADDR,
			phydev->state, phydev->link);

	nxp_port = netdev_priv(netdev);
	if (phydev->link && !nxp_port->link_state) {
		/* If we just came up */
		nxp_port_linkup(nxp_port, phydev);
	}

	/* update port link state */
	nxp_port->link_state = !!phydev->link;
}

/**********************************nw_ops**************************************/
static const struct net_device_ops nxp_port_netdev_ops = {
	.ndo_fdb_add		= nxp_port_fdb_add,
	.ndo_fdb_del		= nxp_port_fdb_del,
	.ndo_fdb_dump		= nxp_port_fdb_dump,
	.ndo_get_stats64	= nxp_get_stats,
	.ndo_vlan_rx_add_vid	= nxp_port_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= nxp_port_vlan_rx_kill_vid,
	.ndo_get_phys_port_name	= nxp_port_get_phys_port_name,
	.ndo_start_xmit		= nxp_port_tx, /* required, cannot be NULL */
};

/**********************************sw_ops**************************************/
#ifndef CONFIG_NET_SWITCHDEV
#error CONFIG_NET_SWITCHDEV is not configured!
#endif

/* Terminology changed in Linux commit 9d47c0a2d958e06322c88245749278633d333cca */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 2, 1)
static const struct swdev_ops nxp_port_swdev_ops = {
	.swdev_parent_id_get 	= nxp_port_swdev_parent_id_get,
	.swdev_port_stp_update 	= nxp_port_swdev_port_stp_update,
};
#else
static const struct switchdev_ops nxp_port_swdev_ops = {
	.switchdev_port_attr_get	= nxp_port_attr_get,
	.switchdev_port_attr_set	= nxp_port_attr_set,
	.switchdev_port_obj_add		= nxp_port_obj_add,
	.switchdev_port_obj_del		= nxp_port_obj_del,
};
#endif

/********************************ethtool_ops***********************************/
static const struct ethtool_ops nxp_port_ethtool_ops = {
	.get_settings		= nxp_port_get_settings,
};

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * SJA1105P HAL retrieves a big endian, 64bit data type.
 * So that the MAC is contained in bits [47 .. 0].
 * Copy to byte array, taking into consideration the CPU endianness.
 */
void copy_mac(const unsigned char *from, unsigned char *to)
{
#if defined(__BIG_ENDIAN)
	to[0] = from[0]; to[1] = from[1]; to[2] = from[2];
	to[3] = from[3]; to[4] = from[4]; to[5] = from[5];
#elif defined(__LITTLE_ENDIAN)
	to[0] = from[5]; to[1] = from[4]; to[2] = from[3];
	to[3] = from[2]; to[4] = from[1]; to[5] = from[0];
#else
#error No Endianness configured
#endif
}

/**
 * Get pointer of switch Context given a pointer to the pport_num'th array
 * element in sja1105p_platform_data member of sja1105p_context_data
 */
static struct sja1105p_context_data *get_ctx_of_port(struct port_desc *p_desc,
						     int pport_num)
{
	struct port_desc *firstport;
	struct sja1105p_platform_data *pdata;

	firstport = p_desc - pport_num; /* ptr to first array element */
	pdata = container_of(firstport, struct sja1105p_platform_data, ports[0]);

	return container_of(pdata, struct sja1105p_context_data, pdata);
}

/* get the device tree node corresponding to a given logical port,
 * returns NULL if not found
 * Returns node with incremented refcount, needs to decremented again when done
 */
static struct device_node *get_dt_node_for_port(struct net_device *netdev, int lport)
{
	struct sja1105p_context_data *ctx;
	struct device_node *switch_dt_node = NULL;
	struct device_node *port_dt_node;
	nxp_port_data_t *nxp_port;
	char phy_dts_name[DTS_NAME_LEN];

	nxp_port = netdev_priv(netdev);

	/* get the DT node of the SJA1105P; it uses the physical port nums */
	ctx = get_ctx_of_port(nxp_port->physical_port,
			      nxp_port->physical_port_num);
	switch_dt_node = ctx->of_node;
	if (!switch_dt_node)
		goto node_not_found;

	/* get the DT node of specified port, node is child of switch_dt_node */
	scnprintf(phy_dts_name, DTS_NAME_LEN, "port-%d",
		  nxp_port->physical_port_num);
	port_dt_node = of_get_child_by_name(switch_dt_node, phy_dts_name);

	/* decrement refcount, was incremented by of_find_node_by_name */
	of_node_put(switch_dt_node);

	return port_dt_node;

node_not_found:
	dev_err(&netdev->dev, "Could not find dt node for port %d\n", lport);
	return NULL;
}

/* find phydev corresponding to port number, return NULL if not found */
static struct phy_device *find_phydev(struct net_device *netdev, int lport)
{
	struct device_node *port_dt_node;
	struct device_node *phy_dt_node;
	struct phy_device *phydev;

	port_dt_node = get_dt_node_for_port(netdev, lport);
	if (!port_dt_node)
		return NULL;

	/* the switch port node contains a reference (phandle) called "phy-ref"
	 * to the ethernet phy it is connected to. Retrieve the phy node
	 */
	phy_dt_node = of_parse_phandle(port_dt_node, "phy-ref", 0);

	/* decrement refcount, was incremented by get_dt_node_for_port */
	of_node_put(port_dt_node);

	if (!phy_dt_node)
		goto phandle_error;

	/* retrieve the phydev associated to the DT node */
	phydev = of_phy_find_device(phy_dt_node);

	/* decrement refcount, was incremented by of_parse_phandle */
	of_node_put(phy_dt_node);

	return phydev;

phandle_error:
	if (verbosity > 0)
		netdev_alert(
			netdev,
			"Port %d does not have a (valid) phandle to an ethernet phy\n",
			lport);
	return NULL;
}

/* connect corresponding phydev to the port-netdev */
static void attach_phydev(struct net_device *netdev)
{
	int err;
	nxp_port_data_t *nxp_port;
	struct phy_device *phydev;

	nxp_port = netdev_priv(netdev);
	phydev = find_phydev(netdev, nxp_port->port_num);
	if (!phydev)
		goto phydev_not_found;

	/* this sets the phy state to PHY_READY and starts the phy state machine
	 * If no Phy driver is loaded until now, a generic one is assumed.
	 * Later loading of phy driver will have no effect, as phy already bound
	 */
	err = phy_connect_direct(netdev, phydev, &nxp_adjust_link,
				 PHY_INTERFACE_MODE_RGMII);
	if (err)
		goto phydev_attach_error;

	/* force one iteration of the phy state machine
	 * This ensures adjust_link() is called at least once
	 */
	mutex_lock(&phydev->lock);
	phydev->state = PHY_CHANGELINK;
	mutex_unlock(&phydev->lock);

	if (verbosity > 0)
		netdev_alert(
			netdev,
			"connected phy [%x] (state: [%x], link [%x]) to port [%d], attached_dev [%s]\n",
			phydev->PHYDEV_ADDR, phydev->state, phydev->link,
			nxp_port->port_num,
			(phydev->attached_dev) ? phydev->attached_dev->name
					       : "none");

	return;

phydev_not_found:
	if (verbosity > 0)
		netdev_alert(netdev, "No phydev found for %s\n", netdev->name);
	return;

phydev_attach_error:
	netdev_err(netdev, "Error: could not attach phydev to %s [err=%d]\n",
		   netdev->name, err);
	return;
}


/*******************************************************************************
 * power management
 ******************************************************************************/

/**
 * Since Linux commit d5c3d84657db57bd23ecd58b97f1c99dd42a7b80,
 * the state machine only reschedules itself in case polling is used
 * (i.e. phydev->irq == PHY_POLL). Otherwise, we need to manually trigger a
 * state machine iteration in case phydev->state was changed not on account of
 * an interrupt but manually (as is done by phy_stop()).
 * In older versions, this is not strictly necessary, but has the effect that
 * the state machine is run as soon as possible.
 */
void phydev_state_machine_trigger(struct phy_device *phydev)
{
	if (phydev->irq == PHY_POLL)
		return;

	cancel_delayed_work_sync(&phydev->state_queue);
	queue_delayed_work(system_power_efficient_wq, &phydev->state_queue, 0);
}

static void manual_clock_reset(int lport, int rx_delay, int tx_delay)
{
	SJA1105P_port_t pport_info = {0};

	SJA1105P_getPhysicalPort(lport, &pport_info);

	if (rx_delay > 0)
		SJA1105P_resetClockDelay(pport_info.physicalPort,
					 pport_info.switchId,
					 SJA1105P_e_direction_RX);
	if (tx_delay > 0)
		SJA1105P_resetClockDelay(pport_info.physicalPort,
					 pport_info.switchId,
					 SJA1105P_e_direction_TX);
}

int do_pm_request(struct port_desc *p_desc, int unused, int pm_req)
{
	int ret = 0;
	nxp_port_data_t *nxp_port;
	struct phy_device *phydev;

	/* non-phy ports need a special treatment on resume:
	 * since there is no phy, a link change will not be detected
	 * automatically (which would result in a reset of the clock delays) and
	 * the clock delays need to be reset manually here
	 */
	if (pm_req == PM_DO_RESUME && p_desc->phy_not_mac) {
		manual_clock_reset(p_desc->logical_port_num,
				   p_desc->rx_delay, p_desc->tx_delay);
		goto out;
	}

	if (!p_desc->netdev || !p_desc->netdev->phydev)
		goto out;

	nxp_port = netdev_priv(p_desc->netdev);
	phydev = p_desc->netdev->phydev;

	if (verbosity > 0)
		netdev_alert(p_desc->netdev, "PM: %s phy 0x%x\n",
		            (pm_req==PM_DO_RESUME)?"resuming":"suspending",
			    phydev->PHYDEV_ADDR);

	switch (pm_req) {
	case PM_DO_RESUME:
		/* update state, resume phy, enable interrupts
		 * (and trigger state machine since Linux v4.9)
		 */
		phy_start(phydev);
		/* force reconfig of speed settings (may not be necessary) */
		nxp_port->link_speed = -1;
		nxp_port_linkup(nxp_port, phydev);
		break;
	case PM_DO_SUSPEND:
		/* set state to PHY_HALTED and disable interrupts */
		phy_stop(phydev);
		/* if link=1: set link=0 and suspend */
		phydev_state_machine_trigger(phydev);
		/* suspend again in case link was 0 */
		phy_suspend(phydev);
		break;
	default:
		netdev_err(p_desc->netdev, "Invalid pm request\n");
		break;
	}

out:
	return ret;
}

/*******************************************************************************
 * Init / Deinit
 ******************************************************************************/

/* set up net devices and private memory */
int register_port(struct port_desc *p_desc, int port, int unused)
{
	int err;
	struct spi_device *spidev;
	struct sja1105p_context_data *ctx;

	char *port_name;
	struct net_device *netdev;
	nxp_port_data_t *nxp_port;
	SJA1105P_port_t physicalPortInfo = {0};

	netdev = alloc_etherdev(sizeof(*nxp_port));
	if (!netdev) {
		err = -ENOMEM;
		goto netdev_allocation_error;
	}

	/* get private memory from netdev,
	 * which is located behind the netdev struct
	 */
	nxp_port = netdev_priv(netdev);

	/* get info about the physical port */
	err = SJA1105P_getPhysicalPort(port, &physicalPortInfo);
	if (err) {
		pr_err("register_ports failed: could not read physical port data from sja1105p for port %d!\n",
		       port);
		goto sja1105p_read_error;
	}

	ctx = get_ctx_of_port(p_desc, physicalPortInfo.physicalPort);
	spidev = ctx->spi_dev;

	/* populate nxp_port */
	nxp_port->port_num = port;
	nxp_port->ppid = physicalPortInfo.switchId;
	nxp_port->physical_port = p_desc;
	nxp_port->physical_port_num = physicalPortInfo.physicalPort;

	/*reset cached stats*/
	memset(&nxp_port->stats, 0, sizeof(nxp_port_stats_t));

	/* save netdev so it can be unregistered later */
	nxp_port->physical_port->netdev = netdev;

	/* give dev a meaningful name */
	port_name = kzalloc(sizeof(char) * PORT_NAME_LEN, GFP_KERNEL);
	if (!port_name) {
		err = -ENOMEM;
		goto port_name_allocation_error;
	}
	scnprintf(port_name, PORT_NAME_LEN, "%s_p%d%s", PRODUCT_NAME, port,
		  nxp_port->physical_port->is_host ? "*" : "");
	err = dev_alloc_name(netdev, port_name);
	if (err) {
		err = -ENOMEM;
		goto devname_allocation_error;
	}

	/* populate netdev */
	netdev->netdev_ops = &nxp_port_netdev_ops;
	netdev->ethtool_ops = &nxp_port_ethtool_ops;
	SET_SWDEV_OPS(netdev, nxp_port_swdev_ops);


	/* Flags:
	 *	- Does not change network namespaces. Device is network
	 *	  namespace local. These nw devices are not allowed
	 *	  to move between nw namespaces
	 */
	/* TODO taken from rocker: recheck, possibly expand */
	netdev->features |= (NETIF_F_NETNS_LOCAL | NETIF_F_VLAN_FEATURES);

	/* set parent dev of netdev */
	SET_NETDEV_DEV(netdev, &spidev->dev);

	/* bind a phydev to the netdev */
	attach_phydev(netdev);

	if (verbosity > 4) {
		dev_info(&spidev->dev, "lPort [%d], pPort [%d], netdev [%s]\n",
			 port, physicalPortInfo.physicalPort, port_name);
	}

	err = register_netdev(netdev);
	if (err) {
		pr_err("register_netdev failed for port [%d]\n", port);
		goto netdev_registration_error;
	}

	/*Init and schedule the stats work*/
	INIT_DELAYED_WORK(&nxp_port->stats_work, nxp_update_stats);
	schedule_delayed_work(&nxp_port->stats_work, HZ);

	if (verbosity > 0)
		netdev_info(netdev, "registered netdevice: [%s]\n",
			    netdev->name);
	return 0;

netdev_registration_error:
	cancel_delayed_work_sync(&nxp_port->stats_work);
devname_allocation_error:
	kfree(port_name);
port_name_allocation_error:
sja1105p_read_error:
	free_netdev(netdev);
netdev_allocation_error:
	return err;
}

int unregister_port(struct port_desc *physical_port, int unused, int unused2)
{
	nxp_port_data_t *nxp_port;
	struct net_device *netdev = physical_port->netdev;

	nxp_port = netdev_priv(netdev);

	if (netdev) {
		if (netdev->phydev) {
			struct phy_device *phydev = netdev->phydev;

			if (verbosity > 0)
				netdev_alert(netdev, "disconnecting phy from [%s]\n",
					     netdev->name);

			/* this stops the phy state machine */
			phy_disconnect(phydev);

			/* decrement the refcount,
			 * that was incremented by of_phy_find_device
			 */
			put_device(&phydev->PHYDEV_DEV);
		}

		if (verbosity > 0)
			netdev_alert(netdev, "unregistering: [%s]\n", netdev->name);

		cancel_delayed_work_sync(&nxp_port->stats_work);
		unregister_netdev(netdev);
		free_netdev(netdev);
	}

	return 0;
}
