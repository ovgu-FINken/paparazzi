#include <modules/copter_swarm/copter_swarm.h>

#include <std.h>
#include <subsystems/datalink/downlink.h>
#include <messages.h>
#include <dl_protocol.h>


//ins_node_t* list;

void copter_swarm_init( void ) {
    //list = NULL;
}


void copter_ins_action(void) {
	// we already know own position so skip this.
	if(DL_COPTER_INS_ac_id(dl_buffer) == AC_ID)
		return;
/*
	// gather all recieved INS values and store them in the list
	int ac_id = DL_COPTER_INS_ac_id(dl_buffer);
	ins_node_t* copter_ins = find_ins_node(ac_id, list);

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
/*
ins_node_t* find_ins_node(int ac_id, ins_node_t* first)
{
	if(first == NULL){
		// TODO ask christoph where to free this memory if needed
		first = malloc(sizeof(ins_node_t));
		first->ac_id = ac_id;
	}

	ins_node_t* node = first;
	while (node->next != NULL){
		if(ac_id == node->ac_id)
			return node;
		node = node->next;
	}
	ins_node_t* new_node = malloc(sizeof(ins_node_t));
	new_node->ac_id = ac_id;
	node->next = new_node;
	return new_node;	
}
*/

//void copter_swarm_deinit( void ) {}

