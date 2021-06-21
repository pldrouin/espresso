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
#define TEMP_Vr_V_fact0 (1.940691202271725e+00)
#define TEMP_a1 (2.623205820973477e-04)
#define TEMP_a2 (-1.000778145046185e-05)
#define TEMP_a3 (2.581602059858263e-06)

extern float radcave;
extern float tempave;
extern int tempstate;
extern uint32_t temptime;

enum {kTempUninitialized=-1, kTempOK=0, kTempInvalid=1};

void TemperatureSetup();
int TemperatureInit();
void TemperatureDeinit();

inline const float& TempGetRelativeADCAve(){return radcave;}
inline const float& TempGetTempAve(){return tempave;}
inline const int& TempState(){return tempstate;}
inline uint32_t TempTime(){uint32_t ret; __atomic_load(&temptime, &ret, __ATOMIC_ACQUIRE); return ret;}

void TempUpdate(const uint32_t& sample);

#endif /* COMPONENTS_CONTROL_TEMPERATURE_H_ */
