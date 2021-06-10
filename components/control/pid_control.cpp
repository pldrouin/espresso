/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

static int state;
static float outputsum;
static float lasterror;

static float Kp=0, Ki=0, Kd=0;

static float lasttemps[PID_MAX_D_AVE];
static uint32_t lasttemptimes[PID_MAX_D_AVE];
static int ndave=1;
static int didx=0;

void PIDSetParams(const float& kp, const float& ki, const float& kd)
{
	Kp=kp;
	Ki=ki;
	Kd=kd;
}

void PIDSetNDAve(const int& n)
{
	ndave=n;
	memset(lasttemps,0,ndave*sizeof(float));
	memset(lasttemptimes,0,ndave*sizeof(uint32_t));
}

void PIDSetOutputSum(const float& sum)
{
	outputsum=sum;
}

void PIDPrintParams()
{
	printf("Kp=%22.15e, Ki=%22.15e, Kd=%22.15e\n",Kp,Ki,Kd);
}

void PIDControlInit()
{
	state=-ndave;
	memset(lasttemps,0,ndave*sizeof(float));
	memset(lasttemptimes,0,ndave*sizeof(uint32_t));
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

	float dtime = (temptime - lasttemptimes[(didx+ndave-1)%ndave]) / (float)CONFIG_CONTROL_SAMPLING_FREQ;
	float ddtime = (temptime - lasttemptimes[didx]) / (float)CONFIG_CONTROL_SAMPLING_FREQ;
	float error = GetTargetTemp() - tempval;
	float dtemp = (tempval - lasttemps[didx]);

	lasttemptimes[didx]=temptime;
	lasttemps[didx]=tempval;
	didx=(didx+1)%ndave;
    lasterror = error;

    if(state<0) {
    	++state;
    	return tempval;
    }

	if(fabsf(dtemp) < 2*GetTempNoise()) dtemp=0;

	if((state==0 && error <= 0 && lasterror > 0) || (state==1 && error >= 0 && lasterror <0)) {
		outputsum = GetInitOutput();
		++state;

	} else outputsum += Ki * error * dtime;

	if(outputsum > 1) outputsum= 1;
	else if(outputsum < 0) outputsum= 0;

	float pterm = Kp * error;
	float dterm = -Kd * dtemp / ddtime;
	float output = pterm + outputsum + dterm;

	if(output > 1) output = 1;
	else if(output < 0) output = 0;
	PWMSetOutput(output);

	printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, I=%6.2f%%, D=%6.2f%%)\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,100*output,100*pterm,100*outputsum,100*dterm);

	/*Remember some variables for next time*/
	return tempval;
}
