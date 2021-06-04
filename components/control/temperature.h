/*
 * temperature.h
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_TEMPERATURE_H_
#define COMPONENTS_CONTROL_TEMPERATURE_H_

#include <cstdio>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/timer.h"

#include "sdkconfig.h"

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_ADS1X15.h"

#include "sdkconfig.h"
#include "control_common.h"

#define TEMP_T0 (100+273.15)
#define TEMP_Vr_V_fact0 (1.8425780357363906)
#define TEMP_a1 (2.177804532216248e-4)
#define TEMP_a2 (2.880722504878676e-5)
#define TEMP_a3 (-5.900153918112557e-6)

extern float radcave;
extern float tempave;
extern int tempstate;
extern uint32_t temptime;

enum {kTempUninitialized=-1, kTempOK=0, kTempInvalid=1};

void TemperatureInit();
void TemperatureDeinit();

inline const float& TempGetRelativeADCAve(){return radcave;}
inline const float& TempGetTempAve(){return tempave;}
inline const int& TempState(){return tempstate;}
inline uint32_t TempTime(){uint32_t ret; __atomic_load(&temptime, &ret, __ATOMIC_ACQUIRE); return ret;}

void TempUpdate(void* parameter);

#endif /* COMPONENTS_CONTROL_TEMPERATURE_H_ */
