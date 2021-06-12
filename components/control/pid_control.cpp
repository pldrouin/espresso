/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

#define NFULLPOWERITERS (2)

static int state;
static int dstate;
static float integral;
static float outputsum=0;
static float outputave;
static float outputavestarttime;
static bool clearednoise;

static float Kp=0, Ki=0, Kd=0;
static float maxaveoutputscaling=INFINITY, minaveoutputscaling=0;
static float maxintegralrvalue=INFINITY, minintegralrvalue=0;
static float maxintegralvalue, minintegralvalue;

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

void PIDSetLimitParams(const float& maxaveoutputscalingfactor, const float& minaveoutputscalingfactor, const float& maxintegralrval, const float& minintegralrval)
{
	maxaveoutputscaling=maxaveoutputscalingfactor;
	minaveoutputscaling=minaveoutputscalingfactor;
	maxintegralrvalue=maxintegralrval;
	minintegralrvalue=minintegralrval;
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
	state=0;
	clearednoise=false;
	memset(lasttemps,0,ndave*sizeof(float));
	memset(lasttemptimes,0,ndave*sizeof(uint32_t));
	//PIDSetIntegral(GetInitOutput());
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

    //- state is even when going up and odd when going down
    //- state 0->1 Going up until setpoint at full power
    //- state 1->2 Going down under setpoint without power
    //- state 2->3 Going up until setpoint at full power
    //- Integral initialized to average required power. Stating PID
    //- After that, capping the integral windup to a
    //  value representing the average required power + the integral
    //  correction during the overshoot that will follow
    //- The average required power is updated between each crossing,
    //  but its change should be range bound, at least on the upside.

    if(fabsf(dtemp) < 2*GetTempNoise()) dtemp=0;

    if(state>2+NFULLPOWERITERS) {

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
    		printf("Computed new average output is %7.3f%%, cycle time was %8.3f\n",newoutputave*100,(temptime-outputavestarttime)/(float)CONFIG_CONTROL_SAMPLING_FREQ);

    		if(newoutputave > outputave*maxaveoutputscaling) {
    			outputave*=maxaveoutputscaling;
    			printf("New average output clamped down to %7.3f%%\n",100*outputave);

    		} else if(newoutputave < outputave*minaveoutputscaling) {
    			outputave*=minaveoutputscaling;
    			printf("New average output clamped up to %7.3f%%"\n,100*outputave);

    		} else outputave=newoutputave;
    		maxintegralvalue=outputave*maxintegralrvalue;

    		if(maxintegralvalue>1) maxintegralvalue=1;
    		minintegralvalue=outputave*minintegralrvalue;

    		if(minintegralvalue<0) minintegralvalue=0;
    		printf("New allowed range for integral value is [%7.3f%%, %7.3f%%]\n",100*minintegralvalue,100*maxintegralvalue);
    		outputsum=0;
    		outputavestarttime=temptime;
    	}

    	float diterm=Ki * -error * dtime;

    	integral += diterm;

    	if(integral > maxintegralvalue) integral=maxintegralvalue;
    	else if(integral < minintegralvalue) integral=minintegralvalue;

    	float pterm = Kp * -error;
    	float dterm = -Kd * dtemp / ddtime;
    	float output = pterm + integral + dterm;

    	if(output > 1) output = 1;
    	else if(output < 0) output = 0;
    	PWMSetOutput(output);
    	outputsum+=output*dtick;
    	printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, DeltaI=%6.2f%%, D=%6.2f%%, I=%6.2f%%)\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,100*output,100*pterm,100*diterm,100*dterm,100*integral);

    } else {

    	//If the noise threshold has not been cleared (power off)
    	if(!clearednoise) {
    	    printf("%8.3f: Temp: %6.2f C =>   0.00%%\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval);

    		//If it is now reached
    		if(error<=-GetTempNoise()) {
    			//Clear it and turn the power on
    			clearednoise=true;
    			PWMSetOutput(1);
    			outputsum+=dtick;
    		}

    	//Else if the noise threshold has been cleared and the target has just been reached (power on)
    	} else if(error>=0) {
    	    printf("%8.3f: Temp: %6.2f C => 100.00%%\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval);
    		++state;
    		clearednoise=false;
    		PWMSetOutput(0);

    		if(state>1) {
    			outputave=outputsum/(temptime-outputavestarttime);

    			if(state==3+NFULLPOWERITERS) integral=outputave;
    			if(state>1) printf("Computed average output is %7.3f%%, cycle time was %8.3f\n",outputave*100,(temptime-outputavestarttime)/(float)CONFIG_CONTROL_SAMPLING_FREQ);
    			maxintegralvalue=outputave*maxintegralrvalue;

    			if(maxintegralvalue>1) maxintegralvalue=1;
    			minintegralvalue=outputave*minintegralrvalue;

    			if(minintegralvalue<0) minintegralvalue=0;
    		}
    		outputsum=0;
    		outputavestarttime=temptime;

    	//Else if the noise threshold has been cleared but the target has not been reached yet
    	} else {
    	    printf("%8.3f: Temp: %6.2f C => 100.00%%\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval);
    		outputsum+=dtick;
    	}
    }

	/*Remember some variables for next time*/
	return tempval;
}
