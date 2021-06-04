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

extern volatile float radcave;
extern volatile float tempave;
extern volatile int tempstate;

enum {kTempUninitialized=-1, kTempOK=0, kTempInvalid=1};

void TemperatureInit();
void TemperatureDeinit();

inline const volatile float& TempGetRelativeADCAve(){return radcave;}
inline const volatile float& TempGetTempAve(){return tempave;}
inline const volatile int& TempState(){return tempstate;}

void TempUpdate(void* parameter);

#endif /* COMPONENTS_CONTROL_TEMPERATURE_H_ */
