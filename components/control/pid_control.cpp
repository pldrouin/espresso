/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

static int state;
static double outputsum;
static float lasttemp;
static float lasterror;
static uint32_t lasttemptime;

static double Kp=0, Ki=0, Kd=0;

void PIDSetParams(const double& kp, const double& ki, const double& kd)
{
	Kp=kp;
	Ki=ki;
	Kd=kd;
}

void PIDPrintParams()
{
	printf("Kp=%22.15e, Ki=%22.15e, Kd=%22.15e\n",Kp,Ki,Kd);
}

void PIDControlInit()
{
	state=0;
}

float PIDControl()
{
	uint32_t temptime=TempTime(), newtemptime;
	float tempval;

	for(;;) {
		tempval=TempGetTempAve();
		newtemptime=TempTime();

		if(newtemptime==temptime) break;
		temptime=newtemptime;
	}

	double dtime = (temptime - lasttemptime) / (double)CONFIG_CONTROL_SAMPLING_FREQ;
	float error = GetTargetTemp() - tempval;
	float dtemp = (tempval - lasttemp);

	if((state==0 && error <= 0 && lasterror > 0) || (state==1 && error >= 0 && lasterror <0)) {
		outputsum = GetInitOutput();
		++state;

	} else outputsum += Ki * error * dtime;

	if(outputsum > 1) outputsum= 1;
	else if(outputsum < 0) outputsum= 0;

	double output = Kp * error + outputsum - Kd * dtemp / dtime;

	if(output > 1) output = 1;
	else if(output < 0) output = 0;
	PWMSetOutput(output);

	printf("%8.3f: Temp: %6.2f C => %6.2f%%",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,100*output);

	/*Remember some variables for next time*/
	lasttemp = tempval;
	lasterror = error;
	lasttemptime = temptime;
	return tempval;
}
