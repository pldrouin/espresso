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
#include "driver/timer.h"

#include "sdkconfig.h"

#include "sdkconfig.h"
#include "control_common.h"

#define PWM_MAX_LEVEL UINT32_MAX
#define PWM_HALF_LEVEL (UINT32_MAX>>1)

extern uint32_t pwm_output;

void PWMSetup();
void PWMInit();
void PWMDeinit();

inline uint32_t PWMGetOutputLevel(){uint32_t ret; __atomic_load(&pwm_output, &ret, __ATOMIC_RELAXED); return ret;}
inline float PWMGetOutput(){return PWMGetOutputLevel()/(float)PWM_MAX_LEVEL;}
inline void PWMSetOutputLevel(const uint32_t& level){__atomic_store(&pwm_output, &level, __ATOMIC_RELAXED);}
inline void PWMSetOutput(const float& output){__atomic_store_n(&pwm_output, (uint32_t)(output*PWM_MAX_LEVEL), __ATOMIC_RELAXED);}

bool IRAM_ATTR PWMCallback(void *args);

#endif /* COMPONENTS_CONTROL_PWM_H_ */
