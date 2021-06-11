/*
 * Copyright (C) 2014 Freek van Tienen
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
 *
 */

/** @file follow.c
 *  @brief Follow a certain AC ID.
 * Only for rotorcraft firmware.
 */

#include "swarm/swarm.h"
#include "swarm/swarm_info.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "math.h"

#include "subsystems/navigation/waypoints.h"
#include "state.h"

/* FOLLOW_OFFSET_ X Y and Z are all in ENU frame */
#ifndef FOLLOW_OFFSET_X
#define FOLLOW_OFFSET_X 0.0
#endif

#ifndef FOLLOW_OFFSET_Y
#define FOLLOW_OFFSET_Y 0.0
#endif

#ifndef FOLLOW_OFFSET_Z
#define FOLLOW_OFFSET_Z 42.0
#endif

#ifndef GRAVITY
#define GRAVITY 198
#endif

#ifndef PERLIMITER
#define PERLIMITER 1.5
#endif

#ifndef DRONE_REPULSION_MULTIPLIER
#define DRONE_REPULSION_MULTIPLIER 3
#endif

#ifndef FOLLOW_AC_ID
#error "Please define FOLLOW_AC_ID"
#endif

#ifndef SWARM_WAYPOINT_ID
#error "Please define FOLLOW_WAYPOINT_ID"
#endif

struct Message_f {
   struct EnuCoor_f own_pos;
   struct EnuCoor_f target_pos;
   uint8_t target_ac_id;

   struct EnuCoor_f attraction_force;
   float attraction_d;
   float attraction_strength;
   bool attraction;

   struct EnuCoor_f repulsion_force;
   float repulsion_d;
   float repulsion_strength;
   bool repulsion;
};

static struct EnuCoor_f acc = {0.0f, 0.0f, 0.0f};
static struct Message_f msg = {{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0,{0.0f,0.0f,0.0f},0.0f,0.0f,false,{0.0f,0.0f,0.0f},0.0f,0.0f,false};

/** Get position in local ENU coordinates (float).
 * @param[in] ac_id aircraft id of aircraft info to get
 */
static struct EnuCoor_f *getPositionEnu_f(uint8_t ac_id)
{
  return (ti_acs[ti_acs_id[ac_id]].ac_id != ac_id)? NULL: acInfoGetPositionEnu_f(ac_id);
}

static void send_acc_info(struct transport_tx *trans, struct link_device *dev) {
	pprz_msg_send_ACC(trans, dev, AC_ID, &acc.x, &acc.y, &acc.z);
}

static void send_attract_and_repulse_info(struct transport_tx *trans, struct link_device *dev) {
        pprz_msg_send_ATTREP(trans, dev, AC_ID, &msg.own_pos.x, &msg.own_pos.y, &msg.own_pos.z, &msg.target_pos.x, &msg.target_pos.y, &msg.target_pos.z, &msg.target_ac_id, &msg.attraction_force.x, &msg.attraction_force.y, &msg.attraction_force.z, &msg.attraction_d, &msg.attraction_strength, (uint8_t*)&msg.attraction, &msg.repulsion_force.x, &msg.repulsion_force.y, &msg.repulsion_force.z, &msg.repulsion_d, &msg.repulsion_strength, (uint8_t*)&msg.repulsion);
}


void swarm_init(void) {
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ACC, send_acc_info);
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_ATTREP, send_attract_and_repulse_info);
}

static void attract(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float perlimiter)
{
  struct EnuCoor_f force = {
  	pos_ac->x - own_pos->x,
  	pos_ac->y - own_pos->y,
  	pos_ac->z - own_pos->z
  };
  msg.attraction_force = force;
  float d = sqrtf(force.x*force.x + force.y*force.y + force.z*force.z);
  msg.attraction_d = d;
  d = fmin(fmax(1, d),25);
  msg.attraction_d = d;
  if(d>perlimiter)
  {
    msg.attraction = true;
    //printf("attract %i %f %f %f %f\n", AC_ID, d, force.x, force.y, force.z);
    float strength = GRAVITY / (d * d);
    msg.attraction_strength = strength;
    force.x = force.x * (strength/d);
    force.y = force.y * (strength/d);
    force.z = force.z * (strength/d);
    msg.attraction_force = force;

    acc->x += force.x;
    acc->y += force.y;
    acc->z += force.z;
  }
  else msg.attraction = false;
}

static void repulse(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float perlimiter, uint8_t multiplier)
{
  struct EnuCoor_f force = {
  	pos_ac->x - own_pos->x,
  	pos_ac->y - own_pos->y,
  	pos_ac->z - own_pos->z
  };
  msg.repulsion_force = force;
  float d = sqrtf(force.x*force.x + force.y*force.y + force.z*force.z);
  msg.repulsion_d = d;
  d = fmin(fmax(1, d),25);
  msg.repulsion_d = d;
  if(d<perlimiter)
  {
    msg.repulsion = true;
    //printf("repulse %i\n", AC_ID);
    float strength = (GRAVITY * multiplier) / (d * d);
    msg.repulsion_strength = strength;
    force.x = force.x * (strength/d);
    force.y = force.y * (strength/d);
    force.z = force.z * (strength/d);
    msg.repulsion_force = force;

    acc->x -= force.x;
    acc->y -= force.y;
    acc->z -= force.z;
  }
  else msg.repulsion = false;
}

/*
 * swarm_follow_wp(void)
 * updates the FOLLOW_WAYPOINT_ID to a fixed offset from the last received location
 * of other aircraft with id FOLLOW_AC_ID
 */
void swarm_follow_wp(void)
{
  acc.x = 0.0f;
  acc.y = 0.0f;
  acc.z = 0.0f;

  struct EnuCoor_f *own_pos = stateGetPositionEnu_f();
  msg.own_pos = *own_pos;

  for(uint8_t ac_id=30; ac_id<41; ++ac_id)
  {
    struct EnuCoor_f *ac_pos = getPositionEnu_f(ac_id);
    if(ac_pos != NULL && ac_id != AC_ID)
    {
      msg.target_pos = *ac_pos;
      msg.target_ac_id = ac_id;
      attract(own_pos,ac_pos,&acc,PERLIMITER);
      repulse(own_pos,ac_pos,&acc,PERLIMITER,DRONE_REPULSION_MULTIPLIER);
    }
    else
    {
      msg.target_ac_id = 0;
      msg.attraction = false;
      msg.repulsion = false;
    }
  }

  struct EnuCoor_f* vel = acInfoGetVelocityEnu_f(AC_ID);
  vel->x += acc.x;
  vel->y += acc.y;
  vel->z += acc.z;
  acInfoSetVelocityEnu_f(AC_ID,vel);

  struct EnuCoor_i enu = *stateGetPositionEnu_i();
  enu.x += POS_BFP_OF_REAL(vel->x)+POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  enu.y += POS_BFP_OF_REAL(vel->y)+POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
  enu.z += POS_BFP_OF_REAL(vel->z)+POS_BFP_OF_REAL(FOLLOW_OFFSET_Z);

  // Move the waypoint
  waypoint_set_enu_i(SWARM_WAYPOINT_ID, &enu);
}

