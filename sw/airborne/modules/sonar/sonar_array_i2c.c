/*
 * Copyright (C) 2014 Sebastian Mai, Andreas Pfohl
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

#include "modules/sonar/sonar_array_i2c.h"
#include "generated/airframe.h"
#include "mcu_periph/i2c.h"
#include "state.h"

#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/radio_control.h"

/** Sonar offset.
 *  Offset value in m (float)
 *  distance mesured by the i2c sensor
 */
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 0.
#endif

/** Sonar scale.
 *  Scaling factor to compute real distances(float)
 */
#ifndef SONAR_SCALE
#define SONAR_SCALE 1.0000
#endif

#ifndef SONAR_I2C_DEV
#define SONAR_I2C_DEV i2c2
#endif

/** SONAR_ADDR_FRONT
 * 	adress for the front Sensor
 * 	same as RIGHT, LEFT, BACK ...
 */
#ifndef SONAR_ADDR_START
#define SONAR_ADDR_START 0x71
#endif

struct sonar_values_s sonar_values;

enum SonarState{
	READY,
	RANGING,
	FETCHING
};

static void setSonarValue(enum Sonars sonar, uint16_t value) {
	switch (sonar) {
	case (SONAR_FRONT):
		sonar_values.front = value;
		break;
	case (SONAR_RIGHT):
		sonar_values.right = value;
		break;
	case (SONAR_BACK):
		sonar_values.back = value;
		break;
	case (SONAR_LEFT):
		sonar_values.left = value;
		break;
	default:
		break;
	}
}

static enum Sonars current_sonar;
static enum SonarState sonarState[SONAR_END];
static struct i2c_transaction sonar_i2c_read_trans[SONAR_END];
static struct i2c_transaction sonar_i2c_write_trans[SONAR_END];

void sonar_array_i2c_init(void)
{
	current_sonar = SONAR_START;

	for (unsigned int i = SONAR_START; i < SONAR_END; i++) {
		sonar_i2c_read_trans[i].buf[0] = 0;
		sonar_i2c_read_trans[i].buf[1] = 0;
		sonar_i2c_read_trans[i].slave_addr = (SONAR_ADDR_START + i) << 1 | 1;
		sonar_i2c_read_trans[i].len_r = 2;
		sonar_i2c_read_trans[i].status = I2CTransDone;
		
		sonar_i2c_write_trans[i].buf[0] = 81;
		sonar_i2c_write_trans[i].slave_addr = (SONAR_ADDR_START + i) << 1;
		sonar_i2c_write_trans[i].len_w = 1;
		sonar_i2c_write_trans[i].status = I2CTransDone;

		setSonarValue(i, FINKEN_SONAR_MAX_DIST);

		sonarState[i] = READY;

	}

	// register telemetry
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_SONAR_ARRAY,
			send_sonar_array_telemetry);
}

/** sonar_send_command
 *	send take_range_reading command (0x51) to the sonar sensors to trigger the range readin
 */
static void sonar_start_ranging(enum Sonars sonar)
{
	if(sonarState[sonar] == READY){
		sonar_i2c_write_trans[sonar].buf[0] = 81;
		i2c_transmit(&SONAR_I2C_DEV, 
							 &sonar_i2c_write_trans[sonar],
							 (SONAR_ADDR_START + sonar) << 1,
							 1); 
		sonarState[sonar]= RANGING;
	}
}

static void sonar_start_read(enum Sonars sonar)
{
	if(sonarState[sonar] == RANGING) {
		sonar_i2c_read_trans[sonar].buf[0] = 0;
		sonar_i2c_read_trans[sonar].buf[1] = 0;
		i2c_receive(&SONAR_I2C_DEV, 
							  &sonar_i2c_read_trans[sonar], 
				        (SONAR_ADDR_START + sonar)<<1 | 1, 
				        2);
		sonarState[sonar] = FETCHING;
	}
}

static uint16_t sonar_read(enum Sonars sonar)
{
	if(sonarState[sonar] == FETCHING){
		uint16_t value = sonar_i2c_read_trans[sonar].buf[0];
		value<<=8;
		value |= sonar_i2c_read_trans[sonar].buf[1];
		sonarState[sonar] = READY;
		return value;
	} else
	return 0;
}


/** Read I2C value to update sonar measurement and request new value
 */
void sonar_array_i2c_periodic(void) {
#ifndef DISABLE_SONAR_IN_RC_PASSTHROUGH
#define DISABLE_SONAR_IN_RC_PASSTHROUGH 0
#endif

#if DISABLE_SONAR_IN_RC_PASSTHROUGH
	if(radio_control.values[ WALL_AVOID_RC_SWITCH ] < 0) {
		for (unsigned int i = SONAR_START; i < SONAR_END; i++) {
			setSonarValue(i, FINKEN_SONAR_MAX_DIST);
		}
		return;
	}

#endif
	enum Sonars read_sonar = current_sonar;
	enum Sonars fetch_sonar = (current_sonar+1)%SONAR_END;
	enum Sonars range_sonar = (current_sonar+3)%SONAR_END;
	
	uint16_t value = sonar_read(read_sonar);
	setSonarValue(read_sonar, value);
	sonar_start_read(fetch_sonar);
	sonar_start_ranging(range_sonar);
	current_sonar = fetch_sonar;
}

void send_sonar_array_telemetry(struct transport_tx *trans, struct link_device *link)
{
	trans=trans;
	link=link;
	DOWNLINK_SEND_SONAR_ARRAY(DefaultChannel, DefaultDevice,
			&sonar_values.front, &sonar_values.right, &sonar_values.back,
			&sonar_values.left);
}
