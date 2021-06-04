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

void PWMInit();
void PWMDeinit();

inline float PWMGetOutput(){float ret; __atomic_load(&pwm_output, &ret, __ATOMIC_RELAXED); return ret;}
inline void PWMSetOutput(const float& output){__atomic_store(&pwm_output, &output, __ATOMIC_RELAXED);}

void PWMUpdate(void* parameter);

#endif /* COMPONENTS_CONTROL_PWM_H_ */
