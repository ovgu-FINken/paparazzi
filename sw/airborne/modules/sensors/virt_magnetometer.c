#include <std.h>
#include <modules/sensors/virt_magnetometer.h>
#include <subsystems/abi.h>
#include <pprzlink/messages.h>
#include <math/pprz_algebra_float.h>
#include "math/pprz_algebra_int.h"
#include <pprzlink/dl_protocol.h>
#include "subsystems/datalink/datalink.h"
#include <math.h>
#include "mcu_periph/sys_time.h"
#include "subsystems/imu.h"

#include "subsystems/datalink/telemetry.h"

#ifndef GEO_MAG_SENDER_ID
#define GEO_MAG_SENDER_ID 1
#endif

float cameraHeading;
struct Int32Vect3 mag;
struct Imu virt_imu;
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
    mag.x = cos(cameraHeading*deg2rad) * 2048; // cos(cameraHeading*deg2rad) << INT32_MAG_FRAC
    mag.y = sin(cameraHeading*deg2rad) * 2048; // sin(cameraHeading*deg2rad) << INT32_MAG_FRAC
}

void virt_magnetometer_init(void)
{
    cameraHeading = 0.0;
    mag.x = 0;
    mag.y = 0;
    mag.z = 0;

    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VIRT_MAG, send_virt_mag);
}

void virt_magnetometer_periodic(void)
{
    uint32_t now_ts = get_sys_time_usec();

    // transform heading into magnetometer values    
    calculate_magXYZ();

    VECT3_COPY(imu.mag, mag);
    AbiSendMsgIMU_MAG_INT32(IMU_ASPIRIN2_ID, now_ts, &imu.mag);
}


