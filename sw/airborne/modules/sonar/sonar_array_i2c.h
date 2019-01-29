/*
 * Copyright (C) 2014  Sebastian Mai, Andreas Pfohl
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file sonar_array_i2c.h
 *  @brief driver for the 5 sonar sensors of the ovgu-fink
 */

#ifndef SONAR_ARRAY_I2C_H
#define SONAR_ARRAY_I2C_H

#include "std.h"
#include "mcu_periph/i2c.h"
#include "subsystems/datalink/transport.h"
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/telemetry.h"

enum Sonars {
	SONAR_START = 0, SONAR_FRONT = 0, SONAR_RIGHT, SONAR_BACK, SONAR_LEFT, SONAR_END
};

struct sonar_values_s {
	uint16_t front;
	uint16_t right;
	uint16_t back;
	uint16_t left;
};

extern struct sonar_values_s sonar_values;

extern void sonar_array_i2c_init(void);
extern void sonar_array_i2c_periodic(void);
extern void send_sonar_array_telemetry(struct transport_tx *trans, struct link_device *link);

#endif
