#include <modules/copter_swarm/copter_swarm.h>

#include <std.h>
#include <subsystems/datalink/downlink.h>
#include <messages.h>
#include <dl_protocol.h>


void copter_swarm_init( void ) {

}

void copter_ins_action(void) {
	// we already know own position so skip this.
	if(DL_COPTER_INS_ac_id(dl_buffer) == AC_ID)
		return;

	// gather all recieved INS values
	int ac_id = DL_COPTER_INS_ac_id(dl_buffer);

	int ins_x = DL_COPTER_INS_ins_x(dl_buffer);
	int ins_y = DL_COPTER_INS_ins_y(dl_buffer);
	int ins_z = DL_COPTER_INS_ins_z(dl_buffer);

	int ins_xd = DL_COPTER_INS_ins_xd(dl_buffer);
	int ins_yd = DL_COPTER_INS_ins_yd(dl_buffer);
	int ins_zd = DL_COPTER_INS_ins_zd(dl_buffer);

	int ins_xdd = DL_COPTER_INS_ins_xdd(dl_buffer);
	int ins_ydd = DL_COPTER_INS_ins_ydd(dl_buffer);
	int ins_zdd = DL_COPTER_INS_ins_zdd(dl_buffer);

	// and store them
	
	
}

void copter_swarm_periodic(void)
{
	

}
