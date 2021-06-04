/*
 * pwm.h
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_PWM_H_
#define COMPONENTS_CONTROL_PWM_H_

#include <cstdio>
#include <cstdint>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "sdkconfig.h"

#include "sdkconfig.h"
#include "control_common.h"

extern volatile float pwm_output;

void PWMInit();
void PWMDeinit();

inline const volatile float& PWMGetOutput(){return pwm_output;}
inline void PWMSetOutput(const float& output){pwm_output=output;}

void PWMUpdate(void* parameter);

#endif /* COMPONENTS_CONTROL_PWM_H_ */
