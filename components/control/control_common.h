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
extern float init_output;
extern float tempnoise;

extern float target_temp;

#define TEMP_UPDATE_TASK_BIT (1)
#define PWM_TASK_BIT (1<<1)
#define CONTROLLER_UPDATE_TASK_BIT (1<<2)

#define MIN_TEMP (10)
#define MAX_TEMP (130)

inline const float& GetInitOutput(){return init_output;}
inline void SetInitOutput(const float& initoutput){init_output=initoutput;}

inline const float& GetTempNoise() {return tempnoise;}
inline void SetTempNoise(const float& noise) {tempnoise=noise;}

#endif /* COMPONENTS_CONTROL_CONTROL_COMMON_H_ */
