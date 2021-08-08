/**
 * @brief  Header file for sensors.c.
 *
 * @author Lukas Probst
 */

#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "adc.h"

/* Sensors of the Armuro 1 robot */
#define LINESENSOR_MIDDLE adc[0]
#define ENCODER_LEFT      adc[1]
#define LINESENSOR_RIGHT  adc[2]
#define BATTERY           adc[3]
#define ENCODER_RIGHT     adc[4]
#define LINESENSOR_LEFT   adc[5]

extern volatile uint32_t adc[6];
extern uint32_t buffer[6];

extern uint32_t encoder_left_cnt;
extern uint32_t encoder_right_cnt;

typedef enum {BLACK, WHITE} Linesensor;
extern Linesensor left_linesensor_state;
extern Linesensor middle_linesensor_state;
extern Linesensor right_linesensor_state;

void SchmittTrigger();
void detectColour();

#endif /* __SENSORS_H__ */
