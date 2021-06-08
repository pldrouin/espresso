/*
 * pid_control.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

static double atunenoise=0;
static double atunestep=0;
static int lookbacknsamples=0;
static bool tuning;
static PID_ATune* aTune=NULL;

void PIDATuneInit()
{
	if(!aTune) {
		aTune=new PID_ATune;
		aTune->SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
		aTune->SetNoiseBand(atunenoise);
		aTune->SetOutputStep(atunestep);
		aTune->SetLookbackNSamples(lookbacknsamples);
		tuning=true;
	}
}

void PIDATuneDeinit()
{
	if(aTune) {
		delete aTune;
		aTune=NULL;
	}
}

void PIDATuneSetNoiseLevel(const double& level)
{
	atunenoise=level;
}

void PIDATuneSetOutputStep(const double& step)
{
	atunestep=step;
}

void PIDATuneSetNLookbackSamples(const int& nlookback)
{
	lookbacknsamples=nlookback;
}

float PIDATune()
{
	uint32_t temptime=TempTime(), newtemptime;
	float tempval;
	double output;

	for(;;) {
		tempval=TempGetTempAve();
		newtemptime=TempTime();

		if(newtemptime==temptime) break;
		temptime=newtemptime;
	}

	if(tuning) {
		byte val = aTune->Runtime(tempval, (unsigned long)(temptime / (double)CONFIG_CONTROL_SAMPLING_FREQ * 1000),GetInitOutput(),&output);

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
