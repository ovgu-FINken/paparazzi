//#pragma once
#ifndef COPTER_SWARM_H
#define COPTER_SWARM_H

#ifndef SWARM_SIZE
#define SWARM_SIZE 5
#endif

void copter_swarm_init(void);
void copter_ins_action(void);
void copter_swarm_periodic(void);

typedef struct ins_node {

    int ac_id;
    int ins_x;
    int ins_y;
    int ins_z;
    int ins_xd;
    int ins_yd;
    int ins_zd;
    int ins_xdd;
    int ins_ydd;
    int ins_zdd;

}ins_node_t;


// returns the node with the given ac_id or a newly created one
void fill_ins_node(int ac_id, ins_node_t* list);


//function to calculate the force
void calcForce(ins_node_t* copter0, double* fx_out, double* fy_out);


#endif
