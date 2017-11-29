#include <std.h>
#include <modules/sensors/virt_magnetometer.h>
#include <subsystems/abi.h>
#include <messages.h>
#include <subsystems/datalink/telemetry.h>
#include <subsystems/datalink/downlink.h>
#include <math/pprz_algebra_float.h>
#include <dl_protocol.h>
#include <math.h>

#ifndef GEO_MAG_SENDER_ID
#define GEO_MAG_SENDER_ID 1
#endif

float cameraHeading;
struct FloatVect3 mag;
float deg2rad = M_PI/180;

static void send_virt_mag(struct transport_tx* trans, struct link_device* dev)
{
    pprz_msg_send_VIRT_MAG(trans, dev, AC_ID,
			 &cameraHeading,
                         &mag.x,
    			 &mag.y,
    			 &mag.z);
}

//Still have to figure out...!!!
void getTheta(void)
{
    cameraHeading = DL_CAMERA_THETA_theta(dl_buffer);
}

void calculate_magXYZ()
{
    mag.x=cos(cameraHeading*deg2rad);
    mag.y=sin(cameraHeading*deg2rad);
}

void virt_magnetometer_init(void)
{
    cameraHeading = 0.0;
    mag.x = 0;
    mag.y = 0;
    mag.z = 0;

    register_periodic_telemetry(DefaultPeriodic, "VIRT_MAG", send_virt_mag);
}

void virt_magnetometer_periodic(void)
{
    // transform heading into magnetometer values
    
    calculate_magXYZ();
    float_vect3_normalize(&mag);
    AbiSendMsgGEO_MAG(GEO_MAG_SENDER_ID, &mag);
}


