#include "modules/sensors/virt_baro_tera_ranger_one.h"
#include "math/pprz_isa.h"
#include "subsystems/abi.h"
#include "pprzlink/messages.h"
#include "mcu_periph/i2c.h"
#include "autopilot.h"
#include "subsystems/datalink/telemetry.h"
#include "state.h"
#include "math/pprz_algebra_int.h"

#ifndef ADDRESS
  #define ADDRESS 0x30
#endif

#if TERA_I2C_DEV==I2C0
  #define I2C_DEV i2c0
#elif TERA_I2C_DEV==I2C1
  #define I2C_DEV i2c1
#elif TERA_I2C_DEV==I2C2
  #define I2C_DEV i2c2
#elif
  #error "Invalid I2C device defined for Tera Ranger"
#endif

static uint32_t intDistance;
static float    floatDistance;
static float    pressure;


enum SonarState{
	READY,
	RANGING,
	FETCHING
};

static enum SonarState i2c_state;

static struct i2c_transaction read_trans;

static void start_read(void) {
	if(i2c_state == READY) {
	  read_trans.buf[0] = 0;
		read_trans.buf[1] = 0;
		read_trans.buf[2] = 0;
		i2c_receive(&I2C_DEV, 
							  &read_trans, 
				        ADDRESS<<1 | 1, 
				        3);
		i2c_state = FETCHING;
	}
}

static uint16_t read(void)
{
	if(i2c_state == FETCHING){
		uint16_t value = read_trans.buf[0];
		value<<=8;
		value |= read_trans.buf[1];
		i2c_state = READY;
		return value;
	} else
	return 0;
}

static void send_virt_baro(struct transport_tx* trans, struct link_device* dev) {
  pprz_msg_send_VIRT_BARO(trans, dev, AC_ID,
    &intDistance,
    &floatDistance,
    &floatDistance,
    &pressure
  );
}

void virt_baro_tera_ranger_one_init(void) {
  i2c_state=READY;
  intDistance = 0;
  floatDistance = 0;
  pressure = 0;
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_VIRT_BARO, send_virt_baro);
}

void virt_baro_tera_ranger_one_periodic(void) {
  intDistance = read();
  start_read();
  struct Int32RMat* rmat = stateGetNedToBodyRMat_i();
  struct Int32Vect3 z= { 0, 0, POS_BFP_OF_REAL(1) };
  struct Int32Vect3 result;
  int32_rmat_vmult(&result, rmat, &z);
  floatDistance = POS_FLOAT_OF_BFP(intDistance * result.z)/1000;
  if (!autopilot.kill_throttle)	
	  pressure = pprz_isa_pressure_of_altitude(floatDistance);
  else 
	  pressure = pprz_isa_pressure_of_altitude(0);
  AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, pressure);
}
