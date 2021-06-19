/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

#define NFULLPOWERITERS (10)
#define HISTORYTIMELENGTH (30) //In seconds

#define HISTORYLENGTH (HISTORYTIMELENGTH*CONFIG_CONTROL_SAMPLING_FREQ/CONFIG_CONTROLLER_SAMPLING_PERIOD_N_SAMPLES)

static int dstate;
static float integral;
static float outputsum=0;
static uint32_t outputavestarttime=0;
static float avecycletemp;
static float outputhistory[HISTORYLENGTH];
static float minoutput, maxoutput, midoutput;
static float lastoutput;
static int hidx=0;
static int hlength=0;
static int minoutputidx, maxoutputidx;
static int state;
static int substate;
static bool clearednoise;

static float Kp=0, Ki=0, Kd=0;
static float maxintegralvalue=1, minintegralvalue=0;
static float maxmiddriftup=INFINITY, maxmiddriftdown=-INFINITY;

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

void PIDSetLimitParams(const float& maxintegralval, const float& minintegralval, const float& maxmidoutputdriftup, const float& maxmidoutputdriftdown)
{
	maxintegralvalue=maxintegralval;
	minintegralvalue=minintegralval;
	maxmiddriftup=maxmidoutputdriftup;
	maxmiddriftdown=maxmidoutputdriftdown;
}

void PIDSetNDAve(const int& n)
{
	ndave=n;
	memset(lasttemps,0,ndave*sizeof(float));
	memset(lasttemptimes,0,ndave*sizeof(uint32_t));
}

void PIDSetIntegral(const float& theintegral)
{
	integral=theintegral;
}

void PIDPrintParams()
{
	printf("Kp=%22.15e, Ki=%22.15e, Kd=%22.15e\n",Kp,Ki,Kd);
}

void PIDControlInit()
{
	dstate=-ndave;
	clearednoise=false;
	outputavestarttime=0;
	state=0;
	memset(lasttemps,0,ndave*sizeof(float));
	memset(lasttemptimes,0,ndave*sizeof(uint32_t));
	PIDOutputHistoryReset(true);
	PIDSetIntegral(GetInitOutput());
}

bool PIDOutputHistoryReset(bool force)
{
	if(hlength==HISTORYLENGTH || force) {
		minoutput=0;
		maxoutput=0;
		midoutput=0;
	    minoutputidx=0;
	    maxoutputidx=0;
		hlength=0;
		hidx=-1;
		printf("Resetting output history!\n");
		return true;
	}
	return false;
}

void PIDOutputHistoryUpdate(const float& output)
{
	int i;

	if(hlength < HISTORYLENGTH) {
		++hidx;
		memmove(outputhistory+hidx+1,outputhistory+hidx,(hlength-hidx)*sizeof(float));
		++hlength;
	}

	if(hlength==1) {
		midoutput=minoutput=maxoutput=output;
		printf("%i: First output history element set to %7.3f%%\n",hidx,100*output);

		//Else if the current value is a new maximum
	} else if(output>maxoutput) {
		maxoutputidx=hidx;
		maxoutput=outputhistory[hidx]=output;
		midoutput=0.5*(minoutput+maxoutput);
		printf("%i: Maximum output updated to current value of %7.3f%% (%7.3f%%, %7.3f%%, %7.3f%%)\n",hidx,100*maxoutput,100*minoutput,100*midoutput,100*maxoutput);

		//If the previous minimum gets overwritten
		if(minoutputidx==hidx) {
			minoutputidx=HISTORYLENGTH-1;
			minoutput=outputhistory[HISTORYLENGTH-1];

			for(i=HISTORYLENGTH-2; i>=0; --i)

				if(outputhistory[i] < minoutput) {
					minoutputidx=i;
					minoutput=outputhistory[i];
				}
			midoutput=0.5*(minoutput+maxoutput);
			printf("%i: Minimum output updated to %7.3f%% (%7.3f%%, %7.3f%%, %7.3f%%)\n",hidx,100*minoutput,100*minoutput,100*midoutput,100*maxoutput);
		}

		//Else if the current value is a new minimum
	} else if(output<minoutput) {
		minoutputidx=hidx;
		minoutput=outputhistory[hidx]=output;
		midoutput=0.5*(minoutput+maxoutput);
		printf("%i: Minimum output updated to current value of %7.3f%% (%7.3f%%, %7.3f%%, %7.3f%%)\n",hidx,100*minoutput,100*minoutput,100*midoutput,100*maxoutput);

		//If the previous maximum gets overwritten
		if(maxoutputidx==hidx) {
			maxoutputidx=HISTORYLENGTH-1;
			maxoutput=outputhistory[HISTORYLENGTH-1];

			for(i=HISTORYLENGTH-2; i>=0; --i)

				if(outputhistory[i] > maxoutput) {
					maxoutputidx=i;
					maxoutput=outputhistory[i];
				}
			midoutput=0.5*(minoutput+maxoutput);
			printf("%i: Maximum output updated to %7.3f%% (%7.3f%%, %7.3f%%, %7.3f%%)\n",hidx,100*maxoutput,100*minoutput,100*midoutput,100*maxoutput);
		}

		//If the previous maximum gets overwritten
	} else {
		outputhistory[hidx]=output;

		if(maxoutputidx==hidx) {
			maxoutputidx=HISTORYLENGTH-1;
			maxoutput=outputhistory[HISTORYLENGTH-1];

			for(i=HISTORYLENGTH-2; i>=0; --i)

				if(outputhistory[i] > maxoutput) {
					maxoutputidx=i;
					maxoutput=outputhistory[i];
				}
			midoutput=0.5*(minoutput+maxoutput);
			printf("%i: Maximum output updated to %7.3f%% (%7.3f%%, %7.3f%%, %7.3f%%)\n",hidx,100*maxoutput,100*minoutput,100*midoutput,100*maxoutput);

			//Else if the previous minimum gets overwritten
		} else if(minoutputidx==hidx) {
			minoutputidx=HISTORYLENGTH-1;
			minoutput=outputhistory[HISTORYLENGTH-1];

			for(i=HISTORYLENGTH-2; i>=0; --i)

				if(outputhistory[i] < minoutput) {
					minoutputidx=i;
					minoutput=outputhistory[i];
				}
			midoutput=0.5*(minoutput+maxoutput);
			printf("%i: Minimum output updated to %7.3f%% (%7.3f%%, %7.3f%%, %7.3f%%)\n",hidx,100*minoutput,100*minoutput,100*midoutput,100*maxoutput);
		}
	}
	hidx=(hidx+1)%hlength;
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

	uint32_t dtick = temptime - lasttemptimes[(didx+ndave-1)%ndave];
	float dtime = dtick / (float)CONFIG_CONTROL_SAMPLING_FREQ;
	float ddtime = (temptime - lasttemptimes[didx]) / (float)CONFIG_CONTROL_SAMPLING_FREQ;
	float error = tempval - GetTargetTemp();
	float dtemp = (tempval - lasttemps[didx]);

	lasttemptimes[didx]=temptime;
	lasttemps[didx]=tempval;
	didx=(didx+1)%ndave;

	if(dstate<0) {
		++dstate;
		return tempval;
	}

	if(fabsf(dtemp) < 2*GetTempNoise()) dtemp=0;

	//If the noise threshold has not been cleared (power off)
	if(!clearednoise) {

		//If it is now reached
		if(error<=-GetTempNoise()) {
			//Clear it and turn the power on
			clearednoise=true;
		}

		//Else if the noise threshold has been cleared and the target has just been reached (power on)
	} else if(error>=0) {
		clearednoise=false;
		float newoutputave=outputsum/(temptime-outputavestarttime);
		avecycletemp/=temptime-outputavestarttime;
		printf("Computed new average output is %7.3f%%, average temp is %7.3f C, cycle time was %8.3f\n",newoutputave*100,avecycletemp,(temptime-outputavestarttime)/(float)CONFIG_CONTROL_SAMPLING_FREQ);

		outputsum=0;
		avecycletemp=0;
		outputavestarttime=temptime;
	}

	float diterm, fastditerm;

	if(Ki!=0) {

		/*if(error < -0.25) {
			diterm=(maxintegralvalue-minintegralvalue)/HISTORYLENGTH;
			fastditerm=16 * Ki * -error * dtime;

			if(diterm<fastditerm) diterm=fastditerm;

		} else if(error > 0.25) {
			diterm=-(maxintegralvalue-minintegralvalue)/HISTORYLENGTH;
			fastditerm=16 * Ki * -error * dtime;

			if(diterm>fastditerm) diterm=fastditerm;

		} else*/ if (fabsf(error) > 0.10) diterm=16 * Ki * -error * dtime;

		else diterm=Ki * -error * dtime;

		integral+=diterm;

	} else diterm=0;

	float pterm = Kp * -error;
	float dterm = -Kd * dtemp / ddtime;

	/*
	if(error < 0 && (integral < minoutput || integral < midoutput-maxmiddriftdown)) {

		if(integral < minoutput) {
			integral=lastoutput;
			PIDOutputHistoryReset();

		} else integral=(lastoutput<midoutput?lastoutput:midoutput);
		printf("Integral clamped up to %7.3f%%\n",100*integral);

	} else if(error > 0 && (integral > maxoutput || integral > midoutput+maxmiddriftup)) {

		if(integral > maxoutput) {
			integral=lastoutput;
			PIDOutputHistoryReset();

		} else integral=0;
		printf("Integral clamped down to %7.3f%%\n",100*integral);

	} else integral += diterm;
	*/

	if(integral > maxintegralvalue) integral=maxintegralvalue;
	else if(integral < minintegralvalue) integral=minintegralvalue;

	float output = pterm + integral + dterm;

	//if(output > 1) output = 1;
	if(output > 1) {

		if(output < 2) integral-=output-1;
		output = 1;

	} else if(output < 0) output = 0;

	PWMSetOutput(output);
	PIDOutputHistoryUpdate(output);
	outputsum+=output*dtick;
	avecycletemp+=tempval*dtick;
	printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, DeltaI=%6.2f%%, D=%6.2f%%, I=%6.2f%%)\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,100*output,100*pterm,100*diterm,100*dterm,100*integral);

	lastoutput=output;

	/*Remember some variables for next time*/
	return tempval;
}
