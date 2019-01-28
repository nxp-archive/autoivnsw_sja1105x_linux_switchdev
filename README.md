# SJA1105PQRS Linux Switch Driver Release v0.2 - User Notes

## Changelog
v0.1: (Feb 2018)
- Initial release

v0.2: (Dez 2018)
- Add support for PHY power management
- Add support for multiple [Linux versions](#supported-linux-versions)
- Fix SPI communication for S32V234 platform
- Add RGMII clock delay configration option in the device tree
- Correctly handle resetting of RGMII delay lines
- Coldreset switch before initialization
- Add support for dynamic link speed changes
- Add support for Linux userspace tool 'ethtool'
- Miscellaneous improvements and bug fixes

---

## Table of Contents
1. [Overview](#overview)
2. [SJA1105PQRS loading](#sja1105pqrs-loading)
3. [Switchdev](#switchdev)
4. [Power Management](#power-management)
5. [DTS Information](#dts-information)
6. [Compile time configuration](#compile-time-configuration)
7. [Supported Linux versions](#supported-linux-versions)
8. [Limitations and known issues](#limitations-and-known-issues)

---

## Overview
The **SJA1105PQRS** Linux Switch Driver contains several components:
- *platform independent* HAL for the switch hardware (supports **SJA1105P**, **SJA1105Q**, **SJA1105R**, **SJA1105S**)
- *platform dependent* sublayer of the HAL, enables SPI communication with the hardware
- *switchdev* component exposes some functionality of the switch to linux userspace

## SJA1105PQRS loading
- Load the kernel module: eg. `insmod <MODULE_NAME>`
- Available module parameters are:
	- `max_hz`: SPI bus speed may be limited for the remote **SJA1105PQRS** application board, 25 MHz is the maximum
	- `ifname`: Network interface name for **SJA1105PQRS** Host port: default to `eth0`
	- `verbosity`: Trace level
	- `enable_switchdev`: Enable the *switchdev* driver component

## Switchdev
The *switchdev* component exposes some functionality of the **SJA1105PQRS** switch to linux userspace:
- General information and statistics  
**NOTE**: Since the switch ports do not have a MAC address, the `HWaddr` field will display `00:00:00:00:00:00`.
> `ifconfig`
- Information about a certain PHY
> `ethtool <PORT_NETDEVICE>`
- Manipulation of the switching table (**FDB**)
	- Add an **FDB** entry for a given address (and optional VLAN id) to the table of a given switchport
	> `bridge fdb add <MAC> dev <PORT_NETDEVICE> (vlan <VLAN_ID>)`

	**NOTE**: If a VLAN id is specified when adding a new entry (or adding a new port to an existing entry), the `MASK_VLANID` field of the L2 Address Lookup table is set to `0xFFF`.
	If no VLAN id is specified, the `MASK_VLANID` field is set to `0x000` (i.e. don't care). In case there already exists an entry with no VLAN id, it is not possible to add a more specified entry for the same MAC address that has a VLAN id.
	- Deletes the **FDB** entry corresponding to a given address (and optional VLAN id) from the table of a given switchport
	> `bridge fdb del <MAC> dev <PORT_NETDEVICE> (vlan <VLAN_ID>)`
	- Dump all **FDB** entries
	> `bridge fdb show`
- Manipulation of **VLAN** configuration
	- Register a **VLAN** id
	> `vconfig add <PORT_NETDEVICE> <VID>`
	- Unregister a **VLAN** id
	> `vconfig rem <PORT_NETDEVICE> <VID>`

**NOTE:** For using the *switchdev* component, the `CONFIG_NET_SWITCHDEV` kernel configuration is required.

## Power Management
The **SJA1105PQRS** driver handles power management for all PHYs that are connected to the switch.
If the host system is suspended or hibernated, the attached PHYs will also be suspended. Correspondingly, once the host system is resumed, the attached PHYs will also be resumed.  
**NOTE:** For using the power management functionality, the kernel needs to be configured accordingly (e.g. `CONFIG_SUSPEND` and `CONFIG_PM_SLEEP` should at least be enabled).
Power management can be tested by using the kernel integrated test facility (requires `CONFIG_PM_DEBUG` to be enabled): A suspend to memory can for example be simulated by
> `echo devices > /sys/power/pm_test`  
> `echo mem > /sys/power/state`

which should suspend, and after 5 seconds resume all PHYs attached to the switch.

## DTS Information
Please refer to `doc/README`

## Compile time configuration
Some platform dependent parameters need to be configured at compile time. These parameters can be found in the Makefile:
- `NUMBER_SWITCHES`: The number of switches attached to the system
- `SPI_FREQ`: Frequency at which the SPI Bus operates
- `SPI_SWAP`: If given a nonzero value, the upper 16-bit of a 32-bit word are swapped with the lower 16-bit
- `SPI_BPW`: *bits_per_word* setting of the SPI Controller that is used
- `SPI_BPW_MSG`: *bits_per_word* setting for an individual message, in general this should be equal to SPI_BPW
- `NR_CFG_BLOCKS`: number of words that are sent at once in a single SPI transmission

## Supported Linux versions
The driver was tested with **Linux v4.1.26** and **Linux v4.14.34**.  
However, it should work for every Linux version that is **v4.0** or newer.

## Limitations and known issues
- Incompatibility with **spi_imx** driver.  
The **chipselect** (**CS**) pin is de-asserted in between 32-bit words when using *bits_per_word* setting of 32. The switch aborts transaction as soon as **CS** is de-asserted, which makes communication impossible.
The `spi_transfer.cs_change` option is ignored by **spi_imx** driver. As a temporary workaround, a low level patch (in `core/spi.c`) can be used that allows arbitrarily large *bits_per_word* settings so there will be no de-assertion in between words.
This is a known problem, refer for example to:
	- https://community.nxp.com/thread/387852
	- https://community.nxp.com/thread/309866

- If the **SJA1105PQRS** driver is used in combination with a PHY driver (like the **TJA110x** driver), a load order has to be respected.  
	Load the PHY driver (e.g. **TJA110x** driver) FIRST, THEN load the **SJA1105PQRS** driver.  
	**Reason**: During *switchdev* initialization, `phy_attach_direct()` is called. This function checks, if there is a driver loaded for the PHY to be attached. If not, it assumes the generic `genphy` driver and binds it. As a result, the **TJA110x** driver will not be bound once it is loaded.
