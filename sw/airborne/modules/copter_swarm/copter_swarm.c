#include <modules/copter_swarm/copter_swarm.h>

#include <std.h>
#include <subsystems/datalink/downlink.h>
#include <subsystems/datalink/telemetry.h>
#include <messages.h>
#include <dl_protocol.h>
#include <math.h>
#include <subsystems/ins/ins_int.h>
#include <firmwares/rotorcraft/navigation.h>
#include <firmwares/rotorcraft/guidance.h>
#include <firmwares/rotorcraft/autopilot.h>

const float insToMeter = 0.0039063;

static ins_node_t swarm[SWARM_SIZE];

void copter_swarm_init( void ) {
	int i = 0;
	for( ; i < SWARM_SIZE; i++){
		swarm[i].ac_id=-1;
	}	
}

void copter_ins_action(void) {

	//To get the current copter id
	int copter_ac_id = DL_COPTER_INS_ac_id(dl_buffer);
	// we already know own position so skip this.
	if(DL_COPTER_INS_copter_id(dl_buffer) == AC_ID)
		return;
	

	// gather all recieved INS values and store them in the list
	int ac_id = DL_COPTER_INS_copter_id(dl_buffer);
	// To use call by reference manage memory
	fill_ins_node(ac_id, swarm);

}


void fill_ins_node(int ac_id, ins_node_t* copter_ins)
{
	int i=0;
	if(copter_ins == NULL){
		return;
	}

	while (copter_ins->ac_id != -1 && i < SWARM_SIZE){
		if(ac_id == copter_ins->ac_id)
		{
			break;
		}
		i++;
		copter_ins++;
	}

	

	
	if(i < SWARM_SIZE)
	{
		copter_ins->ac_id = ac_id;
		copter_ins->ins_x = DL_COPTER_INS_ins_x(dl_buffer);
		copter_ins->ins_y = DL_COPTER_INS_ins_y(dl_buffer);
		copter_ins->ins_z = DL_COPTER_INS_ins_z(dl_buffer);

		copter_ins->ins_xd = DL_COPTER_INS_ins_xd(dl_buffer);
		copter_ins->ins_yd = DL_COPTER_INS_ins_yd(dl_buffer);
		copter_ins->ins_zd = DL_COPTER_INS_ins_zd(dl_buffer);

		copter_ins->ins_xdd = DL_COPTER_INS_ins_xdd(dl_buffer);
		copter_ins->ins_ydd = DL_COPTER_INS_ins_ydd(dl_buffer);
		copter_ins->ins_zdd = DL_COPTER_INS_ins_zdd(dl_buffer);
		
	}

	
	return;	
}

void copter_swarm_periodic(void)
{
	ins_node_t* copter = swarm;
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
	// TODO limit speed

	// TODO transform forces to speed command
	struct EnuCoor_i myPos = *(stateGetPositionEnu_i());
	
	//guidance_h_mode_changed(GUIDANCE_H_MODE_NAV);
	autopilot_set_mode(AP_MODE_NAV);

	//navigation_carrot.x = myPos.x + fx_sum;
	//navigation_carrot.y = myPos.y + fy_sum;

	guidance_h_pos_sp.x = myPos.x + fx_sum;
	guidance_h_pos_sp.y = myPos.y + fy_sum;

	// speed command?
	//guidance_h_set_guided_vel(float vx, float vy)
	// bool guidance_h_set_guided_vel 	( 	float  	vx,		float  	vy 	) 	
	
	
}

//function to calculate the force
void calcForce(ins_node_t* copter0, double* fx_out, double* fy_out){
	ins_node_t copter1;
	copter1.ac_id = AC_ID;
	copter1.ins_x = ins_int.ltp_pos.x;
	copter1.ins_y = ins_int.ltp_pos.y;
	copter1.ins_z = ins_int.ltp_pos.z;

	float cons = 10.0;
	float d = 0.5;
	float diff_x;
	float diff_y;
	double dist;

	diff_x = copter1.ins_x - copter0->ins_x;
	diff_y = copter1.ins_y - copter0->ins_y; 
	diff_x = diff_x * insToMeter;
	diff_y = diff_y * insToMeter;
	dist = sqrt((diff_x*diff_x) + (diff_y*diff_y));	

	if(dist < d){
		cons = 0.6;
	}

    	*fx_out = - cons*(dist-d) * diff_x;
    	*fy_out = - cons*(dist-d) * diff_y;

	// To return fx and fy
}


//void copter_swarm_deinit( void ) {}

