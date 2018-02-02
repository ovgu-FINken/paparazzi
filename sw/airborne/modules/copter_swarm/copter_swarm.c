#include <modules/copter_swarm/copter_swarm.h>

#include <std.h>
#include <subsystems/datalink/downlink.h>
#include <subsystems/datalink/telemetry.h>
#include <pprzlink/messages.h>
#include <pprzlink/dl_protocol.h>
#include <math.h>
#include <state.h>
#include <subsystems/ins/ins_int.h>
#include <subsystems/gps.h>
#include <firmwares/rotorcraft/navigation.h>
#include <firmwares/rotorcraft/guidance.h>
#include <autopilot.h>
#include "subsystems/navigation/waypoints.h"

static gps_node_t swarm[SWARM_SIZE];
bool is_running;
float point;
struct EnuCoor_f Target_waypoint;

//Reminder: ecef is in cm from earth's centre
const int wall_top_x = 384205118;
const int wall_bottom_x = 384205341; //bigger value
const int wall_left_y = 79184823;
const int wall_right_y = 79185130; //bigger value
const int safe_dist = 50;

void copter_swarm_init( void ) {
	int i = 0;
	for( ; i < SWARM_SIZE; i++){
		swarm[i].ac_id=-1;
	}	
	is_running = false;
}

void copter_gps_action(void) { // if this does not get called, put the COPTER_GPS message into the datalink section of messages.xml

	//To get the current copter id
	int copter_ac_id = DL_COPTER_GPS_ac_id(dl_buffer);
	// we already know own position so skip this.
	if(DL_COPTER_GPS_copter_id(dl_buffer) == AC_ID)
		return;
	

	// gather all recieved GPS values and store them in the list
	int ac_id = DL_COPTER_GPS_copter_id(dl_buffer);
	// To use call by reference manage memory
	fill_gps_node(ac_id, swarm);

}


void fill_gps_node(int ac_id, gps_node_t* copter_gps)
{
	// return for invalid copters
	int i=0;
	if(copter_gps == NULL){
		return;
	}

	// run through current swarm and look for the ac_id of the current GPS-data
	while (copter_gps->ac_id != -1 && i < SWARM_SIZE){
		if(ac_id == copter_gps->ac_id)
		{
			// stop if the ac_id is already inside the swarm
			break;
		}
		// or search for an empty place in the swarm when the ac_id is not inside the swarm
		i++;
		copter_gps++;
	}

	
	// fill the current position of the swarm with the given data
	// either it already existed and gets new data
	// or a new copter is added to the swarm 
	if(i < SWARM_SIZE)
	{
		copter_gps->ac_id = ac_id;
		copter_gps->ecef_x = DL_COPTER_GPS_ecef_x(dl_buffer);
		copter_gps->ecef_y = DL_COPTER_GPS_ecef_y(dl_buffer);
		copter_gps->ecef_z = DL_COPTER_GPS_ecef_z(dl_buffer);

		copter_gps->ecef_xd = DL_COPTER_GPS_ecef_xd(dl_buffer);
		copter_gps->ecef_yd = DL_COPTER_GPS_ecef_yd(dl_buffer);
		copter_gps->ecef_zd = DL_COPTER_GPS_ecef_zd(dl_buffer);
		
	}

	
	return;	
}

void copter_swarm_periodic(void)
{
    if(is_running){
	gps_node_t* copter = swarm;
	int i = 0;	
	double fx_sum = 0;
	double fy_sum = 0;
	double fx = 0;
	double fy = 0;
	for ( ; i < SWARM_SIZE; i++){
		if (copter->ac_id == -1)
			break;		
		fx = 0;
		fy = 0;
		calcForce(copter, &fx, &fy);
		fx_sum += fx;
		fy_sum += fy;
		copter++;
	}

	wall_avoid(&fx_sum, &fy_sum);
	
	struct EnuCoor_f myPos = *(stateGetPositionEnu_f());

	autopilot_set_mode(AP_MODE_NAV);
	
	Target_waypoint.x = (myPos.x + fy_sum);
	Target_waypoint.y = (myPos.y + (-fx_sum));
	Target_waypoint.z = 0.5;
	//Target_waypoint.x = -1.5;
	//Target_waypoint.y = 0;
	//Target_waypoint.z = 0.4;

	waypoint_set_enu(1, &Target_waypoint);
    }
	
}

//function for Wall Avoidance
void wall_avoid(double* fx_out, double* fy_out){
	float constant = 2.0;
	float d = 0.0;
	if (gps.ecef_pos.x < (wall_top_x + safe_dist) ){
		d = (wall_top_x+safe_dist) - gps.ecef_pos.x;
		*fx_out = *fx_out + (constant * d/100);
	}
	else if (gps.ecef_pos.x > (wall_bottom_x - safe_dist) ){
		d = gps.ecef_pos.x - (wall_bottom_x - safe_dist) ;
		*fx_out = *fx_out - (constant * d/100);
	}

	if (gps.ecef_pos.y < (wall_left_y + safe_dist) ){
		d = (wall_left_y+safe_dist) - gps.ecef_pos.y;
		*fy_out = *fy_out + (constant * d/100);
	}
	else if (gps.ecef_pos.y > (wall_right_y - safe_dist) ){
		d = gps.ecef_pos.y - (wall_right_y - safe_dist) ;
		*fy_out = *fy_out - (constant * d/100);
	}



}

//function to calculate the force
void calcForce(gps_node_t* copter0, double* fx_out, double* fy_out){
	gps_node_t copter1;
	copter1.ac_id = AC_ID;
	copter1.ecef_x = gps.ecef_pos.x;
	copter1.ecef_y = gps.ecef_pos.y;
	copter1.ecef_z = gps.ecef_pos.z;

	float cons = 0.3;
	float d = 0.5; //in metres
	float diff_x;
	float diff_y;
	double dist;

	diff_x = (copter1.ecef_x - copter0->ecef_x) /100.0; //in metres (as ecef gets us cm)
	diff_y = (copter1.ecef_y - copter0->ecef_y) /100.0;  
		dist = sqrt((diff_x * diff_x) + (diff_y * diff_y));	

	// Repel when the copters are close
	if(dist < d){
		cons = 1;
	}

    	*fx_out = - cons*(dist-d) * diff_x;
    	*fy_out = - cons*(dist-d) * diff_y;

	// To return fx and fy
}

void enableCopterSwarm(){
	is_running = true;
}


void disableCopterSwarm(void){
	is_running = false;
}


