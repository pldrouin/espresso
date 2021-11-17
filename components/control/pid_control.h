/*
 * pid_control.h
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_PID_CONTROL_H_
#define COMPONENTS_CONTROL_PID_CONTROL_H_

#define PID_MAX_D_AVE (100)

#include <cmath>
#include <cstring>

#include "control_common.h"
#include "temperature.h"
#include "pwm.h"
#include "controller.h"

void PIDSetParams(const float& Ki, const float& Theta0, const float& Kcfact=0.5, const float& Tifact=4, const float& Tdfact=0.5);
void PIDSetLimitParams(const float& maxintegralval=INFINITY, const float& minintegralval=0);
void PIDSetDeriveTime(const float& time=1.00);
void PIDSetRampThreshold(const float& thresh);
void PIDSetDeadband(const float& dband);
void PIDSetIntegral(const float& theintegral);
void PIDSetNIntegralEstimateCycles(const uint8_t& numcycles);

void PIDPrintParams();

void PIDControlInit();
void PIDControlDeinit();

float PIDControl();

#endif
