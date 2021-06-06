/*
 * simple_control.h
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_SIMPLE_CONTROL_H_
#define COMPONENTS_CONTROL_SIMPLE_CONTROL_H_

#include "control_common.h"
#include "temperature.h"
#include "pwm.h"
#include "controller.h"

void SimpleControlInit();

float SimpleControl();

#endif
