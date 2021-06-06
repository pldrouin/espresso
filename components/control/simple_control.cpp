/*
 * simple_control.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "simple_control.h"

#define SC_STABLE_STATE (4)

void SimpleControlInit()
{
}

float SimpleControl()
{
	uint32_t temptime=TempTime(), newtemptime;
	float tempval;

	for(;;) {
		tempval=TempGetTempAve();
		newtemptime=TempTime();

		if(newtemptime==temptime) break;
		temptime=newtemptime;
	}
	printf("%8.3f: Temp: %6.2f C vs %6.2f C setpoint => ",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,GetTargetTemp());

	if(tempval < GetTargetTemp()) {
		printf("On\n");
		PWMSetOutput(1);

	} else {
		printf("Off\n");
		PWMSetOutput(0);
	}
	return tempval;
}
