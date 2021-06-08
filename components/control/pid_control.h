/*
 * pid_control.h
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_PID_CONTROL_H_
#define COMPONENTS_CONTROL_PID_CONTROL_H_

#include "control_common.h"
#include "temperature.h"
#include "pwm.h"
#include "controller.h"

void PIDSetParams(const double& kp, const double& ki, const double& kd);

void PIDPrintParams();

void PIDControlInit();

float PIDControl();

#endif
