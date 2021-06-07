/*
 * controller.h
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_CONTROLLER_H_
#define COMPONENTS_CONTROL_CONTROLLER_H_

#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"
#include "driver/timer.h"

#include "esp_log.h"

#include "control_common.h"
#include "killswitch.h"
#include "temperature.h"
#include "pwm.h"

extern float target_temp;

int ControllerSetup();
int ControllerInit();
void ControllerDeinit();

int ControllerSetAlgorithm(float (*algo)(), void (*init)());

bool IRAM_ATTR ControllerCallback(void *args);

void ControllerUpdate(void* parameter);

void StartStats();
void StopStats();

inline void SetTargetTemp(const float& target){target_temp=target;}
inline const float& GetTargetTemp(){return target_temp;}

#endif /* COMPONENTS_CONTROL_CONTROLLER_H_ */
