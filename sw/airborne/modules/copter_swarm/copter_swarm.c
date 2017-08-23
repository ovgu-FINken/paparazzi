#include <modules/copter_swarm/copter_swarm.h>

#include <std.h>
#include <subsystems/datalink/downlink.h>
#include <subsystems/datalink/telemetry.h>
#include <messages.h>
#include <dl_protocol.h>
#include <math.h>


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
	
}

//function to calculate the force

void calcForce(ins_node_t* copter0, ins_node_t* copter1){
	int cons = 0.5;
	int d = 0.9;
	int diff_x;
	int diff_y;
	double dist;
	double fx;
	double fy;

	diff_x = copter0->ins_x - copter1->ins_x;
	diff_y = copter0->ins_y - copter1->ins_y; 
	dist = sqrt((diff_x*diff_x) + (diff_y*diff_y));	

	if(dist < d){
		cons = 3.0;
	}

    	fx = - cons*(dist-d) * diff_x;
    	fy = - cons*(dist-d) * diff_y;

	// To return fx and fy
}


//void copter_swarm_deinit( void ) {}

