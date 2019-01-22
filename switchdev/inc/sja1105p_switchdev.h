/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * \file nxp_sja1105p_switchdev.h
 *
 * \author Marco Hartmann
 *
 * \date 2017-08-02
 *
 */

#ifndef _NXP_SJA1105P_H
#define _NXP_SJA1105P_H

#include <linux/spi/spi.h>

#include "sja1105p_cfg_file.h"

int register_port(struct port_desc*, int, int);
int unregister_port(struct port_desc*, int, int);
int do_pm_request(struct port_desc *, int, int);

#endif /* _NXP_SJA1105P_H */
