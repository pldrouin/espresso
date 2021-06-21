/*
 * pid_control.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

static float atunestep=0;
static int lookbacknsamples=0;
static bool tuning;
static PID_ATune* aTune=NULL;

void PIDATuneInit()
{
	if(!aTune) {
		aTune=new PID_ATune;
		aTune->SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
		aTune->SetNoiseBand(tempnoise);
		aTune->SetOutputStep(atunestep);
		aTune->SetLookbackNSamples(lookbacknsamples);
	}
}

void PIDATuneDeinit()
{
	PIDATuneStop();
	if(aTune) {
		delete aTune;
		aTune=NULL;
	}
}

void PIDATuneStart()
{
	tuning=true;
}

void PIDATuneStop()
{
	if(tuning) {

		if(aTune) aTune->Cancel();
		tuning=false;
	}
}

void PIDATuneSetOutputStep(const float& step)
{
	atunestep=step;
}

void PIDATuneSetNLookbackSamples(const int& nlookback)
{
	lookbacknsamples=nlookback;
}

float PIDATune()
{
	//Same thread as temperature, so we don't need to use non-blocking algorithms
	uint32_t temptime=TempTime();
	float tempval=TempGetTempAve();
	float output;

	if(tuning) {
		byte val = aTune->Runtime(tempval, (unsigned long)(temptime / (float)CONFIG_CONTROL_SAMPLING_FREQ * 1000),GetInitOutput(),&output);

		if (val != 0) {
			printf("Tuning ended!\n");
			tuning = false;
			output = 0;
			PWMSetOutput(0);
		}
		PWMSetOutput(output);

		if(!tuning) {
			PIDSetParams(aTune->GetKp(), aTune->GetKi(), aTune->GetKd());
			PIDPrintParams();
		}
	}

	printf("%8.3f: Temp: %6.2f C => %6.2f%%\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,100*output);

	return tempval;
}
