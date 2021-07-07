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
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "multi/followMe.h"
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
int _objectIdx[50]={0};
int _objectIdxLength=0;
//angle of directionMap-Vectors
float _angleInDegree;
//debug_-stuff
struct EnuCoor_f _debugVector_f={0};
struct EnuCoor_f *_debugVector_f1={0};
struct EnuCoor_f *_debugVector_f2={0};
struct EnuCoor_i *_debugVector_i={0};
float debugFloat=0;
float debugArray_f[8]={-1,-1,-1,-1,-1,-1,-1,-1,};
int *HighestInterestId=0;
int choosenIndex=-1;

float data[3];

/*
* Creating a new Vector who gets rotated by an angle(cw: -, ccw:+)
*/
static struct EnuCoor_f rotate_2D_vector(struct EnuCoor_f vector, float angle){
	angle= angle * M_PI/180;
	float x= vector.x * cosf(angle) - vector.y * sinf(angle);
	float y= vector.x * sinf(angle) + vector.y * cosf(angle);
	struct EnuCoor_f Result= {x,y,vector.z};
	return Result;
}
static float calc_Vector_Magnitude(struct EnuCoor_f v){
	return sqrtf(powf(v.x,2)+powf(v.y,2));
}
static float calc_Angle_2DOLD(struct EnuCoor_f v1, struct EnuCoor_f v2){
	float dotProduct=(v1.x * v2.x + v1.y * v2.y);
	float vecMagnitudeProduct= calc_Vector_Magnitude(v1)*calc_Vector_Magnitude(v2);
	float resultInRadiant= acosf(dotProduct/vecMagnitudeProduct);
	return resultInRadiant*180/M_PI;
}
static float calc_Angle_2D(struct EnuCoor_f v1, struct EnuCoor_f v2){
	float dotProduct=(v1.x * v2.x + v1.y * v2.y);
	float determinant= (v1.x * v2.y - v1.y * v2.x);
	float resultInRadiant= atan2f(determinant,dotProduct);
	return resultInRadiant*180/M_PI;
}	

static void send_leader_info(struct transport_tx *trans, struct link_device *dev) {
  //struct EnuCoor_i *ac = acInfoGetPositionEnu_i(FOLLOW_AC_ID);
  int index = 3;
  int number = index * _angleInDegree;
  float k= cosf(M_PI_2);
  //x = POS_FLOAT_OF_BFP(_debugVector_i->x);
  struct EnuCoor_f v= _directionMap[index];
  struct EnuCoor_f d= _directionMap[0];
  struct EnuCoor_f enu=rotate_2D_vector(v,number);
  float debugAngle1= calc_Vector_Magnitude(v);
  float debugAngle2= calc_Vector_Magnitude(d);
  float debugAngle3= calc_Vector_Magnitude(_debugVector_f);			
  float dataF[] = {_debugVector_f.x,_debugVector_f.y, _debugVector_f.z,8,8, debugAngle1, debugAngle2, debugFloat  };
  float vectorArray[] = {_debugVector_f.x,_debugVector_f.y, _debugVector_f.z,_debugVector_f1->x,_debugVector_f1->y, _debugVector_f1->z,_debugVector_f2->x,_debugVector_f2->y, _debugVector_f2->z};	
  char dataC[] = "test";
  int32_t dataI[] = {choosenIndex, _debugVector_i->x,_debugVector_i->y,_debugVector_i->z};
  	
  pprz_msg_send_LEADER(trans, dev, AC_ID, sizeof(_interestMap)/sizeof(float), _interestMap, sizeof(dataI)/4, dataI, sizeof(dataC)-1, dataC);
  //pprz_msg_send_LEADER(trans, dev, AC_ID, sizeof(dataF)/sizeof(float), dataF, sizeof(_objectIdx)/4, _objectIdx, sizeof(dataC)-1, dataC);
  //pprz_msg_send_LEADER(trans, dev, AC_ID, sizeof(_interestMap)/sizeof(float), _interestMap, sizeof(_objectIdx)/4, _objectIdx, sizeof(dataC)-1, dataC);
  //pprz_msg_send_LEADER(trans, dev, AC_ID, sizeof(debugArray_f)/sizeof(float), debugArray_f, sizeof(_objectIdx)/4, _objectIdx, sizeof(dataC)-1, dataC);
  
}

void follow_init(void) {
  _angleInDegree= 360/CONTEXTMAP_LENGTH;
  create_direction_map();
  create_ObjectIdx_Array();
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_LEADER, send_leader_info);
}


void create_direction_map(void) {
	//rotate
	struct EnuCoor_f defaultDirectionVector={1,0,0};
	for(int i=0; i<CONTEXTMAP_LENGTH; i++){
		_directionMap[i]=rotate_2D_vector(defaultDirectionVector,_angleInDegree*i);
	}
	
}

void create_ObjectIdx_Array(void) {
	for(int i=0; i<50; i++){
		_objectIdx[i]=-1;
	}
	for(int i=0; i<ti_acs_idx-1;i++){
		_objectIdx[i]= ti_acs[i+1].ac_id ;	
		_objectIdxLength++;
	}
}
void update_ObjectIdx_Array(void) {
	if(ti_acs_idx>_objectIdxLength){
		_objectIdxLength=0;
		for(int i=0; i<50; i++){
			_objectIdx[i]=-1;
		}
		for(int i=0; i<ti_acs_idx-1;i++){
			_objectIdx[i]=ti_acs[i+1].ac_id;	
			_objectIdxLength++;
		}
	}
}



void calc_Interest_Map(void){
	for(int i=0;i<CONTEXTMAP_LENGTH;i++){	
		_interestMap[i]=0;
	}
	struct EnuCoor_f *myPos= stateGetPositionEnu_f();
	struct EnuCoor_f objectPos;
	float threshold= 45;
	float vMagnitude;
	for(int i=0; i<ti_acs_idx-1;i++){
		struct EnuCoor_f *position =acInfoGetPositionEnu_f(_objectIdx[i]);
		_debugVector_f.x = position->x - myPos->x;
		_debugVector_f.y = position->y - myPos->y;
		_debugVector_f.z = position->z - myPos->z;
		_debugVector_f1=position;
		_debugVector_f2=myPos;
		for(int j=0; j<CONTEXTMAP_LENGTH;j++){
			//_debugVector_f=&_directionMap[j];
			float angle= calc_Angle_2D(_debugVector_f,_directionMap[j]);
			debugArray_f[j]=angle;
			//debugFloat= angleBetweenObjVecAndDirectionVec;
			if(angle<threshold && angle>-threshold){
				vMagnitude=calc_Vector_Magnitude(_debugVector_f);
				//debugFloat=vMagnitude;
				if(angle>0){
					_interestMap[j]+=(threshold-angle)/angle * vMagnitude;	
				}
				else {
					_interestMap[j]+=(-threshold+angle)/angle * vMagnitude;
				}
			}
		}
	}	
}
static void calc_Interest_MapDebug(void){
	for(int i=0;i<CONTEXTMAP_LENGTH;i++){	
		_interestMap[i]=0;
	}
	struct EnuCoor_f *myPos= stateGetPositionEnu_f();
	struct EnuCoor_f objectPos;
	float threshold= 45;
	float vMagnitude;
	
	struct EnuCoor_f *position =acInfoGetPositionEnu_f(_objectIdx[1]);
	_debugVector_f.x = position->x - myPos->x;
	_debugVector_f.y = position->y - myPos->y;
	_debugVector_f.z = position->z - myPos->z;
	//struct EnuCoor_f *obj = &objectPos;
	//struct EnuCoor_f *obj ={x=objectPos.x, y=objectPos.y, z=objectPos.z};
	//_debugVector_f=objectPos;
	_debugVector_f1=position;
	_debugVector_f2=myPos;
	for(int j=0; j<CONTEXTMAP_LENGTH;j++){
		//_debugVector_f=&_directionMap[j];
		float angle= calc_Angle_2D(_debugVector_f,_directionMap[j]);
		debugArray_f[j]=angle;
		//debugFloat= angleBetweenObjVecAndDirectionVec;
		if(angle<threshold && angle>-threshold){
			vMagnitude=calc_Vector_Magnitude(_debugVector_f);
			//debugFloat=vMagnitude;
			if(angle>0){
				_interestMap[j]+=(threshold-angle)/angle * vMagnitude;	
			}
			else {
				_interestMap[j]+=(-threshold+angle)/angle * vMagnitude;
			}
		}
	}
	
}
int chooseDirection(void){
	int value=0; 
	int valueIndex=0;
	for(int i=0;i<CONTEXTMAP_LENGTH;i++){
		if(value<_interestMap[i]){
			value=_interestMap[i];
			valueIndex=i;
			//HighestInterestId=&_objectIdx[i];
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
  update_ObjectIdx_Array();
  calc_Interest_Map();
  int direction= chooseDirection();
  choosenIndex=direction;
  struct EnuCoor_i *myPos= stateGetPositionEnu_i();
  struct EnuCoor_i enu;
  enu.x = myPos->x + (int) (_directionMap[direction].x * _interestMap[direction]); //+ POS_BFP_OF_REAL(FOLLOW_OFFSET_X);
  enu.y = myPos->y + (int) (_directionMap[direction].y * _interestMap[direction]); //+ POS_BFP_OF_REAL(FOLLOW_OFFSET_Y);
  enu.z = myPos->z + (int) (_directionMap[direction].z * _interestMap[direction]); //+ POS_BFP_OF_REAL(FOLLOW_OFFSET_Z);
  _debugVector_i= &enu;	
  // Move the waypoint
  waypoint_set_enu_i(FOLLOW_WAYPOINT_ID, &enu);
}

