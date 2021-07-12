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
#define FOLLOW_OFFSET_X 0.0f
#endif

#ifndef FOLLOW_OFFSET_Y
#define FOLLOW_OFFSET_Y 0.0f
#endif

#ifndef FLIGHT_HEIGHT
#define FLIGHT_HEIGHT 47.0f
#endif

// g parameter -> ka = g; kb = g * m
#ifndef GRAVITY
#define GRAVITY 1.985f
#endif

// d parameter
#ifndef COMFY_DIST
#define COMFY_DIST 5.25f
#endif

// r parameter
#ifndef REGION_SIZE
#define REGION_SIZE 2.5f
#endif

// m parameter -> ka = g * m
#ifndef ATTRECTION_MULTIPLIER
#define ATTRECTION_MULTIPLIER 5.0f
#endif

// m parameter -> kb = g * m
#ifndef REPULSION_MULTIPLIER
#define REPULSION_MULTIPLIER 20.0f
#endif

// m parameter -> ka = g * m
#ifndef DRONE_ATTRECTION_MULTIPLIER
#define DRONE_ATTRECTION_MULTIPLIER 1.0f
#endif

// m parameter -> kb = g * m
#ifndef DRONE_REPULSION_MULTIPLIER
#define DRONE_REPULSION_MULTIPLIER 2.5f
#endif

// velocity limit parameter
#ifndef VELOCITY_LIMIT
#define VELOCITY_LIMIT 5.0f
#endif

#ifndef SWARM_WAYPOINT_ID
#error "Please define SWARM_WAYPOINT_ID with the ID of FOLLOW wp"
#endif

#ifndef ATTRACTION_POINT_ID
#error "Please define the ATTRACTION_POINT_ID"
#endif

#ifndef REPELL_POINT_ID
#error "Please define the REPELL_POINT_ID"
#endif

#ifndef FIRST_SWARM_MEMBER_ID
#error "Please define the FIRST_SWARM_MEMBER_ID"
#endif

#ifndef LAST_SWARM_MEMBER_ID
#error "Please define the LAST_SWARM_MEMBER_ID"
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

static uint8_t tick = 0;
static bool init = false;
struct LlaCoor_f offset;  //= {lat,lon,alt};  ->  lat, lon and alt coordinates are specified in radients(like specified in the paparazzi documentation)
static struct EnuCoor_f acc = {0.0f, 0.0f, 0.0f};

// lat and lon coordinates are specified in degrees in contrast to the paparazzi documentation
// alt coordinate is not used but should be specified as FLIGHT_HEIGHT
static struct LlaCoor_f att_points[] = { {52.13878f, 11.64539f, FLIGHT_HEIGHT}, {52.13894f, 11.64589f, FLIGHT_HEIGHT}, {52.13899f, 11.64465f, FLIGHT_HEIGHT}, {52.13843f, 11.64448f, FLIGHT_HEIGHT}, {52.13829f, 11.64588f, FLIGHT_HEIGHT} };
static struct LlaCoor_f rep_points[] = { {52.139193227158565f, 11.645442243819168f, FLIGHT_HEIGHT}, {52.13873185661875f, 11.64604126781022f, FLIGHT_HEIGHT}, {52.13983556877823f, 11.647903288820826f, FLIGHT_HEIGHT}, {52.13927000303959f, 11.646538979628147f, FLIGHT_HEIGHT}, {52.13972261306308f, 11.644770246818533f, FLIGHT_HEIGHT} };
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


//attraction_force = ka*distance
static void attract(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float multiplier)
{
  struct EnuCoor_f force = {
  	pos_ac->x - own_pos->x,
  	pos_ac->y - own_pos->y,
  	0.0f
  };
  msg.attraction_force = force;

  float d = sqrtf(force.x*force.x + force.y*force.y);
  msg.attraction_d = d;
  msg.attraction = true;

  float strength = GRAVITY * multiplier;
  msg.attraction_strength = strength;

  force.x = force.x * strength;
  force.y = force.y * strength;
  msg.attraction_force = force;

  acc->x += force.x;
  acc->y += force.y;
}

//repulsion_force = (kb*exp(-||distance||²/2r²))*distance
static void repulse(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float perlimiter, float multiplier)
{
  struct EnuCoor_f force = {
  	pos_ac->x - own_pos->x,
  	pos_ac->y - own_pos->y,
  	0.0f
  };
  msg.repulsion_force = force;

  float d = sqrtf(force.x*force.x + force.y*force.y);
  msg.repulsion_d = d;
  msg.repulsion = true;

  float strength = (GRAVITY * multiplier) * expf(-(d*d)/(2 * perlimiter * perlimiter));
  msg.repulsion_strength = strength;

  force.x = force.x * strength;
  force.y = force.y * strength;
  msg.repulsion_force = force;

  acc->x -= force.x;
  acc->y -= force.y;
}

//total_force = (ka*(||distance||-d)/max(||distance||,0.01) - kb*exp(-||distance||²/c))*distance
static void attRep(struct EnuCoor_f *own_pos, struct EnuCoor_f* pos_ac, struct EnuCoor_f* acc, float reg_size, float comfy_dist, float att_multiplier, float rep_multiplier)
{
    struct EnuCoor_f force = {
            pos_ac->x - own_pos->x,
            pos_ac->y - own_pos->y,
            0.0f
    };
    msg.repulsion_force = force;
    msg.attraction_force = force;

    float d = sqrtf(force.x*force.x + force.y*force.y);
    float c = (reg_size * reg_size)/logf(rep_multiplier/att_multiplier);
    msg.repulsion_d = d;
    msg.attraction_d = d;
    msg.repulsion = true;
    msg.attraction = true;

    float strength_att = (GRAVITY * att_multiplier) * ((d - comfy_dist) / fmax(d, 0.01f));
    float strength_rep = (GRAVITY * rep_multiplier) * expf(-(d*d)/c);
    msg.attraction_strength = strength_att;
    msg.repulsion_strength = strength_rep;

    force.x = force.x * (strength_att - strength_rep);
    force.y = force.y * (strength_att - strength_rep);
    msg.attraction_force = force;
    msg.repulsion_force = force;

    acc->x += force.x;
    acc->y += force.y;
}

static void updateAttRepPoints(struct LlaCoor_i* own_pos)
{
    if (abs(own_pos->lon - (int)(att_points[tick].lon * 1e7))<=abs((int)(offset.lon*1e7*180/M_PI) - own_pos->lon)
     && abs(own_pos->lat - (int)(att_points[tick].lat * 1e7))<=abs((int)(offset.lat*1e7*180/M_PI) - own_pos->lat))
    {
        tick = (tick+1) % (sizeof(att_points)/sizeof(struct EnuCoor_f));
        struct LlaCoor_i attPoint = { 0, 0, 0 };
        struct LlaCoor_i repPoint = { 0, 0, 0 };

        attPoint.lon = (int)(att_points[tick].lon*1e7);
        attPoint.lat = (int)(att_points[tick].lat*1e7);

        repPoint.lon = (int)(rep_points[tick].lon*1e7);
        repPoint.lat = (int)(rep_points[tick].lat*1e7);
        
        waypoint_set_latlon(ATTRACTION_POINT_ID, &attPoint);
        waypoint_set_latlon(REPELL_POINT_ID, &repPoint);
    }
    else if (!init) 
    {
        struct LlaCoor_i attPoint = { 0, 0, 0 };
        struct LlaCoor_i repPoint = { 0, 0, 0 };

        attPoint.lon = (int)(att_points[tick].lon * 1e7);
        attPoint.lat = (int)(att_points[tick].lat * 1e7);

        repPoint.lon = (int)(rep_points[tick].lon * 1e7);
        repPoint.lat = (int)(rep_points[tick].lat * 1e7);

        waypoint_set_latlon(ATTRACTION_POINT_ID, &attPoint);
        waypoint_set_latlon(REPELL_POINT_ID, &repPoint);

        init = true;
    }
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

  for(uint8_t ac_id = FIRST_SWARM_MEMBER_ID; ac_id <= LAST_SWARM_MEMBER_ID; ++ac_id)
  {
    struct EnuCoor_f *ac_pos = getPositionEnu_f(ac_id);
    if(ac_pos != NULL && ac_id != AC_ID) {
        msg.target_pos = *ac_pos;
        msg.target_ac_id = ac_id;        
        attRep(own_pos, ac_pos, &acc, REGION_SIZE, COMFY_DIST, DRONE_ATTRECTION_MULTIPLIER, DRONE_REPULSION_MULTIPLIER);
    }
    else
    {
      msg.target_ac_id = 0;
      msg.attraction = false;
      msg.repulsion = false;
    }
  }

  struct EnuCoor_f att_point = { 0.0f, 0.0f, 0.0f };
  att_point.x = waypoint_get_x(ATTRACTION_POINT_ID);
  att_point.y = waypoint_get_y(ATTRACTION_POINT_ID);
  attract(own_pos,&att_point,&acc, ATTRECTION_MULTIPLIER);

  struct EnuCoor_f rep_point = { 0.0f, 0.0f, 0.0f };
  rep_point.x = waypoint_get_x(REPELL_POINT_ID);
  rep_point.y = waypoint_get_y(REPELL_POINT_ID);
  repulse(own_pos,&rep_point,&acc, REGION_SIZE, REPULSION_MULTIPLIER);

  struct EnuCoor_f* vel = acInfoGetVelocityEnu_f(AC_ID);
  vel->x = VELOCITY_LIMIT * tanhf(vel->x+acc.x);
  vel->y = VELOCITY_LIMIT * tanhf(vel->y+acc.y);
  acInfoSetVelocityEnu_f(AC_ID,vel);

  struct EnuCoor_i enu = *stateGetPositionEnu_i();
  enu.x += POS_BFP_OF_REAL(vel->x)+POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  enu.y += POS_BFP_OF_REAL(vel->y)+POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
  enu.z = POS_BFP_OF_REAL(FLIGHT_HEIGHT);

  // Move the waypoint
  waypoint_set_enu_i(SWARM_WAYPOINT_ID, &enu);
  struct LlaCoor_i* pos = stateGetPositionLla_i();
  struct EcefCoor_f* ecef_pos = stateGetPositionEcef_f();
  ecef_pos->x += 1.5f;
  ecef_pos->y += 1.5f;
  lla_of_ecef_f(&offset,ecef_pos);
  updateAttRepPoints(pos);
}
