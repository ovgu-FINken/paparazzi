#ifndef COPTER_SWARM_H
#define COPTER_SWARM_H

#ifndef SWARM_SIZE
#define SWARM_SIZE 5
#endif

void copter_swarm_init(void);
void copter_gps_action(void);
void copter_swarm_periodic(void);
void enableCopterSwarm(void);
void disableCopterSwarm(void);
void wall_avoid(double* fx_out, double* fy_out);

typedef struct gps_node {

    int ac_id;
    int ecef_x;
    int ecef_y;
    int ecef_z;
    int ecef_xd;
    int ecef_yd;
    int ecef_zd;

}gps_node_t;


// returns the node with the given ac_id or a newly created one
void fill_gps_node(int ac_id, gps_node_t* list);


//function to calculate the force
void calcForce(gps_node_t* copter0, double* fx_out, double* fy_out);


#endif
