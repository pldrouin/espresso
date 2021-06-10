/*
 * control_common.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "control_common.h"

EventGroupHandle_t eg;
uint32_t samp_counter=0; //Sampling interrupt counter, using __ATOMIC_RELAXED
float init_output=0;
float tempnoise=0;
