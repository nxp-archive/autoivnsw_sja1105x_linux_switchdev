# SJA1105PQRS Linux Switch Driver: Device Tree Information

---

## Table of Contents
- [Ethernet PHY node](#Ethernet-PHY-node)
- [Switch node](#Switch-node)
	- [Properties of top level](#Properties-of-top-level)
	- [Properties of the port-X child node](#Properties-of-the-port-X-child-node)
---

## Ethernet PHY node
Device tree node `ethernet-phy` represents an ethernet PHY, there should be one node for each PHY connected to the system.  
The ethernet PHY nodes need to have a _label_, so they can be referenced from the `sja1105p` node (see below).  
For example: `phy0: ethernet-phy@8` indicates that PHY at address 8 can be referenced by its label `phy0`
(see `doc/example_DT/ethernet_phys.dts`)

## Switch node
Device tree node `sja1105p` represents one switch device, there should be one node for each switch connected to the system.

### Properties of top level
- `compatible`: String that is matched against the of_device_id table `sja1105p_dt_ids` in `sja1105p_init.c`. Used to determine the switch type.
- `firmware_name`: Name of the firmware (in `/lib/firmware`) to be loaded for that switch  
	(_optional, if property is not present then the default firmware name is used_)
- `spi-max-frequency`: the max SPI frequency
- `spi-cpha`: SPI configuration to be used
- `reg`: address of chip select

### Properties of the port-X child node
- `is-host`: Determines if the port is a host port or not
- `null-phy`: Determines if the port has a PHY connected to it or not
- `phy-ref`: _phandle_ to the connected ethernet PHY  
	**NOTE**: Must be `0x00` in case there is no PHY connected to port-X (for example if port-X is a host port or a cascaded port)
- `logical-port-num`: logical port number, used for the port mapping  
	**NOTE**: Must be `0xff` in case port-X is a cascaded port (see `doc/example_DT/sja1105p_node.dts` for an example)
