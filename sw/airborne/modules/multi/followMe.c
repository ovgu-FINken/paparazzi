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

#include "multi/follow.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/datalink/telemetry.h"

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
#define FOLLOW_OFFSET_Z 0.0
#endif

#ifndef FOLLOW_AC_ID
#error "Please define FOLLOW_AC_ID"
#endif

#ifndef FOLLOW_WAYPOINT_ID
#error "Please define FOLLOW_WAYPOINT_ID"
#endif

#ifndef CONTEXTMAP_LENGTH
#define CONTEXTMAP_LENGTH 8
#endif
//ContextMaps
struct EnuCoor_f _directionMap[CONTEXTMAP_LENGTH];
float _interestMap[CONTEXTMAP_LENGTH]={0};
//Array with stored AircraftIDX
int *_objectIdx;
int _maxObjectIdxLength=4;
int _objectIdxLength=0;
//angle of directionMap-Vectors
float _angle;
//debug_-stuff
struct EnuCoor_f *_debugVector_f={0,0,0};
struct EnuCoor_i *_debugVector_i={0,0,0};
int *HighestInterestId=0;
int choosenIndex=-1;

float data[3];

/*
* Creating a new Vector who gets rotated by an angle(cw: +, ccw:-)
*/
static struct EnuCoor_f rotate_2D_vector(struct EnuCoor_f vector, double angle){
	
	double x= vector.x * (float) cos(angle) - vector.y*(float)sin(angle);
	double y= vector.x * (float) sin(angle) + vector.y*(float)cos(angle);
	struct EnuCoor_f Result= {x,y,vector.z};
	return Result;
}


static void send_leader_info(struct transport_tx *trans, struct link_device *dev) {
  //struct EnuCoor_i *ac = acInfoGetPositionEnu_i(FOLLOW_AC_ID);
  int index = 2;
  int number = index * _angle;
  float k= cosf(M_PI_2);
  float x, y, z;
  x = POS_FLOAT_OF_BFP(_debugVector_i->x);
  y = POS_FLOAT_OF_BFP(_debugVector_i->y);
  z = POS_FLOAT_OF_BFP(_debugVector_i->z);
  struct EnuCoor_f v={1,0,0};
  struct EnuCoor_f enu=rotate_2D_vector(v,(double)number);	
  float dataF[] = {enu.x, enu.y, enu.z};
  char dataC[] = "test";
  int32_t dataI[] = {42};
  //pprz_msg_send_LEADER(trans, dev, AC_ID, &_debugVector_f->x, &_debugVector_f->y, &_debugVector_f->z, choosenIndex);
  pprz_msg_send_LEADER(trans, dev, AC_ID, sizeof(dataF)/sizeof(float), dataF, sizeof(dataI)/4, dataI, sizeof(dataC)-1, dataC);
  //pprz_msg_send_LEADER(trans, dev, AC_ID, &_directionMap[2].x, &_directionMap[2].y, &_diretionMap[2].z, &choosenIndex);
}

void follow_init(void) {
  _angle= 360/CONTEXTMAP_LENGTH;
  create_direction_map();
  create_ObjectIdx_Array();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LEADER, send_leader_info);
}


void create_direction_map(void) {
	struct EnuCoor_f defaultDirectionVector={1,0,0};
	for(int i=0; i<CONTEXTMAP_LENGTH; i++){
		_directionMap[i]=rotate_2D_vector(defaultDirectionVector,(double)_angle*i);
	}
	
}

void create_ObjectIdx_Array(void) {
	_objectIdx=malloc(ti_acs_idx*sizeof(int));
	for(int i=0; i<ti_acs_idx;i++){
		_objectIdx[i]= FOLLOW_AC_ID+i;	
		_objectIdxLength++;
	}
}
void update_ObjectIdx_Array(void) {
	if(ti_acs_idx>_objectIdxLength){
		if(_maxObjectIdxLength==_objectIdxLength){
			_objectIdx=realloc(_objectIdx,2*_maxObjectIdxLength*sizeof(int));
			_maxObjectIdxLength=2*_maxObjectIdxLength;
		}
		for(int i=_objectIdxLength; i<ti_acs_idx;i++){
			_objectIdx[i]= FOLLOW_AC_ID+i;	
			_objectIdxLength++;
		}
	}
}


static float calc_Vector_Magnitude(struct EnuCoor_f v){
	return sqrt(pow((double)v.x,2)+pow((double)v.y,2));
}
static float calc_Angle_2D(struct EnuCoor_f v1, struct EnuCoor_f v2){
	double dotProduct=(double)(v1.x * v2.x + v1.y * v2.y);
	double vecMagnitudeProduct= calc_Vector_Magnitude(v1)*calc_Vector_Magnitude(v2);
	return acos(dotProduct/vecMagnitudeProduct);
}	

void calc_Interest_Map(void){
	struct EnuCoor_f *myPos= stateGetPositionEnu_f();
	struct EnuCoor_f objectPos;
	float threshold= 45;
	float vMagnitude;
	for(int i=0; i<ti_acs_idx;i++){
		struct EnuCoor_f *position =acInfoGetPositionEnu_i(_objectIdx[i]);
		objectPos.x = position->x - myPos->x;
		objectPos.y = position->y - myPos->y;
		objectPos.z = position->z - myPos->z;	
		for(int j=0; j<CONTEXTMAP_LENGTH;j++){
			_debugVector_f=&_directionMap[j];
			float angleBetweenObjVecAndDirectionVec= calc_Angle_2D(objectPos,_directionMap[j]);
			if(angleBetweenObjVecAndDirectionVec<threshold){
				vMagnitude=calc_Vector_Magnitude(objectPos);
				_interestMap[j]+=(threshold-angleBetweenObjVecAndDirectionVec)/angleBetweenObjVecAndDirectionVec * vMagnitude;	
			}
		}
	}
}
int chooseDirection(void){
	int value=0; 
	int valueIndex;
	for(int i=0;i<CONTEXTMAP_LENGTH;i++){
		if(value<_interestMap[i]){
			value=_interestMap[i];
			valueIndex=i;
			HighestInterestId=_objectIdx[i];
			//_debugVector_f=acInfoGetPositionEnu_f(_objectIdx[i]);
		}	
	}
	return valueIndex;
}
/*
 * follow_wp(void)
 * updates the FOLLOW_WAYPOINT_ID to a fixed offset from the last received location
 * of other aircraft with id FOLLOW_AC_ID
 */
void follow_wp(void)
{
  //struct EnuCoor_i *ac = acInfoGetPositionEnu_i(FOLLOW_AC_ID);
  calc_Interest_Map();
  int direction= chooseDirection();
  choosenIndex=direction;
  struct EnuCoor_i *followPoint= acInfoGetPositionEnu_i(&HighestInterestId);
  struct EnuCoor_i enu;
  enu.x = _directionMap[direction].x * followPoint->x; //+ POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  enu.y = _directionMap[direction].y * followPoint->y; //+ POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
  enu.z = _directionMap[direction].z * followPoint->z; //+ POS_BFP_OF_REAL(FOLLOW_OFFSET_Z);
  _debugVector_i= &enu;	
  // Move the waypoint
  waypoint_set_enu_i(FOLLOW_WAYPOINT_ID, &enu);
}

