/*
 * simple_control.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "simple_control.h"

float SimpleControl()
{
	//Same thread as temperature, so we don't need to use non-blocking algorithms
	uint32_t temptime=TempTime();
	float tempval=TempGetTempAve();

	printf("%8.3f: Temp: %6.2f C vs %6.2f C setpoint => ",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,GetTargetTemp());

	if(tempval < GetTargetTemp()-GetTempNoise()) {
		printf("On\n");
		PWMSetOutput(1);

	} else if(tempval >= GetTargetTemp()){
		printf("Off\n");
		PWMSetOutput(0);

	} else printf("Noise band\n");
	return tempval;
}
