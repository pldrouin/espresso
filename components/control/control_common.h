/*
 * control_common.h
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_CONTROL_COMMON_H_
#define COMPONENTS_CONTROL_CONTROL_COMMON_H_

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

extern EventGroupHandle_t eg;
extern uint32_t samp_counter;
extern float pwm_output;

extern float target_temp;

#define TEMP_UPDATE_TASK_BIT (1)
#define PWM_TASK_BIT (1<<1)
#define CONTROLLER_UPDATE_TASK_BIT (1<<2)

#define MIN_TEMP (10)
#define MAX_TEMP (150)

#endif /* COMPONENTS_CONTROL_CONTROL_COMMON_H_ */
