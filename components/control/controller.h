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

#include "control_common.h"
#include "killswitch.h"
#include "temperature.h"
#include "pwm.h"

void ControllerInit();
void ControllerDeinit();

bool IRAM_ATTR ControllerCallback(void *args);

void ControllerUpdate(void* parameter);

#endif /* COMPONENTS_CONTROL_CONTROLLER_H_ */
