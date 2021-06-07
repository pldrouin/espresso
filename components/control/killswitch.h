/*
 * killswitch.h
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_KILLSWITCH_H_
#define COMPONENTS_CONTROL_KILLSWITCH_H_

#include <cstdio>
#include <cstdint>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "sdkconfig.h"

#include "sdkconfig.h"
#include "control_common.h"

extern volatile float killswitch_output;

void KillSwitchSetup();
inline void KillSwitchSetNoKill(const bool& state=true){gpio_set_level((gpio_num_t)CONFIG_KILLSWITCH_GPIO, state);}

#endif /* COMPONENTS_CONTROL_KILLSWITCH_H_ */
