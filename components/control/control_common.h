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
#include "driver/timer.h"

extern EventGroupHandle_t eg;
extern float init_output;
extern float tempnoise;

extern float target_temp;

#define TIMER_DIVIDER         (10000)  //  Hardware timer clock divider
#define TIMER_N_TICKS_PER_SEC (TIMER_BASE_CLK / TIMER_DIVIDER)
#define ALARM_N_TICKS		  (TIMER_N_TICKS_PER_SEC / CONFIG_CONTROL_SAMPLING_FREQ)

#define TEMP_UPDATE_TASK_BIT (1)
#define PWM_TASK_BIT (1<<1)
#define CONTROLLER_UPDATE_TASK_BIT (1<<2)

#define MIN_TEMP (10)
#define MAX_TEMP (130)

inline float Tick2Sec(const uint32_t& tick){return tick/(float)TIMER_N_TICKS_PER_SEC;}

inline const float& GetInitOutput(){return init_output;}
inline void SetInitOutput(const float& initoutput){init_output=initoutput;}

inline const float& GetTempNoise() {return tempnoise;}
inline void SetTempNoise(const float& noise) {tempnoise=noise;}

#endif /* COMPONENTS_CONTROL_CONTROL_COMMON_H_ */
