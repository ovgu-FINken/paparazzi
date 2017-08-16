#include "modules/sensors/virt_baro_ir_adc.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
#include "math/pprz_isa.h"
#include "subsystems/gps.h"
#include "subsystems/abi.h"

uint16_t ir_measurement;
float ir_distance;
float ir_distance_equalized;
bool_t ir_data_available;

#ifndef EQ_RANGE
#define EQ_RANGE 3
#endif

float eq_array[EQ_RANGE];
uint8_t eq_idx = 0;

static struct adc_buf ir_adc;

void virt_baro_ir_adc_init(void)
{
    ir_measurement = 0;
    ir_data_available = FALSE;
    ir_distance = 0;
    ir_distance_equalized = 0;

    adc_buf_channel(ADC_CHANNEL_IR, &ir_adc, DEFAULT_AV_NB_SAMPLE);
}

#define IR_SAMPLE_SIZE 6
// adc-values to distance (in meters!)
static uint16_t ir_in_samples[IR_SAMPLE_SIZE] = {800, 830, 900, 1150, 1650, 2700};
static float ir_out_samples[IR_SAMPLE_SIZE] = {1.30, 1.00, .80, .60, .40, .20};

static void update_ir_distance_from_measurement(void)
{
    if(ir_measurement <= ir_in_samples[0]) {
        ir_distance = ir_out_samples[0];
        return;
    }
    if(ir_measurement >= ir_in_samples[IR_SAMPLE_SIZE - 1]) {
        ir_distance = ir_out_samples[IR_SAMPLE_SIZE - 1];
        return;
    }
    int i = 0;
    while(ir_measurement > ir_in_samples[++i]) {
        //void;
    }
    // w: something in range [0;1], for position in interval between i - 1 and i
    float w = (float)(ir_measurement - ir_in_samples[i - 1]) / (ir_in_samples[i] - ir_in_samples[i - 1]);
    ir_distance = (1.0 - w) * ir_out_samples[i - 1] + w * ir_out_samples[i];
}

static void update_ir_distance_equalized_from_ir_distance(void)
{
    float sum = 0;

    eq_array[eq_idx] = ir_distance;

    for(int i = 0; i < EQ_RANGE; i++) {
        sum += eq_array[i];
    }

    ir_distance_equalized = sum / EQ_RANGE;

    eq_idx = (eq_idx + 1) % EQ_RANGE;
}

void virt_baro_ir_adc_periodic(void)
{
    ir_measurement = ir_adc.sum / ir_adc.av_nb_sample;
    ir_data_available = TRUE;
    update_ir_distance_from_measurement();
    update_ir_distance_equalized_from_ir_distance();
    float pressure = pprz_isa_pressure_of_altitude(ir_distance_equalized);
    AbiSendMsgBARO_ABS(BARO_SIM_SENDER_ID, pressure);

}
