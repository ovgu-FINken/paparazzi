#ifndef SONAR_ADC_H
#define SONAR_ADC_H

#include "std.h"

extern float ir_distance;
extern float ir_distance_equalized;

extern void virt_baro_ir_adc_init(void);
extern void virt_baro_ir_adc_periodic(void);

#endif
