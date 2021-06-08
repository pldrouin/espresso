/*
 * pid_atune.h
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_PID_ATUNE_H_
#define COMPONENTS_CONTROL_PID_ATUNE_H_

#define AUTOTUNE_DEBUG

#include <PID_AutoTune_v0.h>

#include "control_common.h"
#include "temperature.h"
#include "pwm.h"
#include "controller.h"
#include "pid_control.h"

void PIDATuneInit();
void PIDATuneDeinit();

void PIDATuneSetNoiseLevel(const double& level);
void PIDATuneSetOutputStep(const double& step);
void PIDATuneSetNLookbackSamples(const int& nlookback);

float PIDATune();

#endif
