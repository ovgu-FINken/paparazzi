#include <modules/copter_swarm/copter_swarm.h>

#include <std.h>
#include <subsystems/datalink/downlink.h>
#include <messages.h>
#include <dl_protocol.h>
#include <math.h>


ins_node_t* list;

void copter_swarm_init( void ) {
    list = NULL;
}


void copter_ins_action(void) {
	

	struct ins_node *ptr = NULL;	
	ptr = (struct ins_node *) malloc (sizeof (struct ins_node));
	if(list == NULL){
		//ins_node_t *test = sbrk(0);
	}
	list->ac_id = DL_COPTER_INS_ac_id(dl_buffer);

	//To get the current copter id
	int copter_ac_id = DL_COPTER_INS_ac_id(dl_buffer);
	// we already know own position so skip this.
	if(DL_COPTER_INS_ac_id(dl_buffer) == AC_ID)
		return;


	// gather all recieved INS values and store them in the list
	int ac_id = DL_COPTER_INS_ac_id(dl_buffer);
	//To use call by reference manage memory
	ins_node_t* copter_ins = find_ins_node(ac_id, list);
/*
	
	copter_ins->ins_x = DL_COPTER_INS_ins_x(dl_buffer);
	copter_ins->ins_y = DL_COPTER_INS_ins_y(dl_buffer);
	copter_ins->ins_z = DL_COPTER_INS_ins_z(dl_buffer);

	copter_ins->ins_xd = DL_COPTER_INS_ins_xd(dl_buffer);
	copter_ins->ins_yd = DL_COPTER_INS_ins_yd(dl_buffer);
	copter_ins->ins_zd = DL_COPTER_INS_ins_zd(dl_buffer);

	copter_ins->ins_xdd = DL_COPTER_INS_ins_xdd(dl_buffer);
	copter_ins->ins_ydd = DL_COPTER_INS_ins_ydd(dl_buffer);
	copter_ins->ins_zdd = DL_COPTER_INS_ins_zdd(dl_buffer);
*/}

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

ins_node_t* find_ins_node(int ac_id, ins_node_t* copter_ins)
{

	//ins_node_t* ins = &copter_ins;
	/*
	//Value of the address 
	while (((ins)->ac_id) != ac_id && &(ins)->next != NULL)
		ins =&(ins)->next;		

	//Check if the copter_ins value is NULL

	
	copter_ins->ins_x = DL_COPTER_INS_ins_x(dl_buffer);
	copter_ins->ins_y = DL_COPTER_INS_ins_y(dl_buffer);
	copter_ins->ins_z = DL_COPTER_INS_ins_z(dl_buffer);

	copter_ins->ins_xd = DL_COPTER_INS_ins_xd(dl_buffer);
	copter_ins->ins_yd = DL_COPTER_INS_ins_yd(dl_buffer);
	copter_ins->ins_zd = DL_COPTER_INS_ins_zd(dl_buffer);

	copter_ins->ins_xdd = DL_COPTER_INS_ins_xdd(dl_buffer);
	copter_ins->ins_ydd = DL_COPTER_INS_ins_ydd(dl_buffer);
	copter_ins->ins_zdd = DL_COPTER_INS_ins_zdd(dl_buffer);

			
	return copter_ins;
*/
	
	
	if(copter_ins == NULL){
		// TODO ask christoph where to free this memory if needed
		//first = malloc(sizeof(ins_node_t));
		copter_ins->ac_id = ac_id;
	}

	ins_node_t* node = copter_ins;
	while (copter_ins->next != NULL){
		if(ac_id == copter_ins->ac_id)
		{

			copter_ins->ins_x = DL_COPTER_INS_ins_x(dl_buffer);
			copter_ins->ins_y = DL_COPTER_INS_ins_y(dl_buffer);
			copter_ins->ins_z = DL_COPTER_INS_ins_z(dl_buffer);

			copter_ins->ins_xd = DL_COPTER_INS_ins_xd(dl_buffer);
			copter_ins->ins_yd = DL_COPTER_INS_ins_yd(dl_buffer);
			copter_ins->ins_zd = DL_COPTER_INS_ins_zd(dl_buffer);

			copter_ins->ins_xdd = DL_COPTER_INS_ins_xdd(dl_buffer);
			copter_ins->ins_ydd = DL_COPTER_INS_ins_ydd(dl_buffer);
			copter_ins->ins_zdd = DL_COPTER_INS_ins_zdd(dl_buffer);
			
			return copter_ins;
		}
		copter_ins = copter_ins->next;
		}

	ins_node_t *new_node; //= (ins_node_t *) malloc(sizeof(ins_node_t));
	new_node->ac_id = ac_id;
	node->next = new_node;
	return new_node;

}


//void copter_swarm_deinit( void ) {}

