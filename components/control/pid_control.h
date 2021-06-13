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

void PIDSetParams(const float& kp, const float& ki, const float& kd);
void PIDSetLimitParams(const float& maxintegralval=INFINITY, const float& minintegralval=0, const float& maxmidoutputdriftup=INFINITY, const float& maxmidoutputdriftdown=INFINITY);
void PIDSetNDAve(const int& n);
void PIDSetIntegral(const float& theintegral);

void PIDPrintParams();

void PIDControlInit();

void PIDIntegralReset(const float& output);

float PIDControl();

#endif
