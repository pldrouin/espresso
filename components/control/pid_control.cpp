/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

enum ramp_status_type {kRampDisabled, kRampInitialising, kRampActive, kRampWait};

static float integral;
static float dterm=0;
static float lasttemp;
static uint32_t lasttemptick;
static float outputsum=0;
static uint32_t outputavestarttick=0;
static float avecycletemp;
static bool clearednoise;
static uint8_t rampstatus;
static float* rampvalues;
static uint32_t ridx;

static float theta0=0;
static uint32_t theta0ticks;
static float Pgain=0, Igain=0, Dgain=0, Ti=0, Td=0, Tf=0;
static float maxintegralvalue=1, minintegralvalue=0;
static float rampthresh=INFINITY;
static uint32_t rampwaitticks;
static float deadband=0;

void PIDSetParams(const float& Ki, const float& Theta0, const float& Kcfact, const float& Tifact, const float& Tdfact)
{
	theta0=Theta0;
	Pgain=Kcfact/(Ki*Theta0);
	Ti=Tifact*Theta0;
	Igain=Pgain/Ti;
	Td=Tdfact*Theta0;
	Dgain=Pgain*Td;
	printf("Pgain (Kc)=%10.6f, Igain=%10.6f/s, Dgain=%10.6fs, Ti=%7.4fs, Td=%7.4fs\n",Pgain,Igain,Dgain,Ti,Td);
}

void PIDSetLimitParams(const float& maxintegralval, const float& minintegralval)
{
	maxintegralvalue=maxintegralval;
	minintegralvalue=minintegralval;
}

void PIDSetDFilter(const float& Tdfilterfact)
{
	Tf=Tdfilterfact*Td;
}

void PIDSetRampThreshold(const float& thresh)
{
	rampthresh=thresh;
}

void PIDSetDeadband(const float& dband)
{
	deadband=dband;
}

void PIDSetIntegral(const float& theintegral)
{
	integral=theintegral;
}

void PIDPrintParams()
{
	printf("Pgain (Kc)=%10.6f, Igain=%10.6f/s, Dgain=%10.6fs, Ti=%7.4fs, Td=%7.4fs, Tf=%7.4fs, deadband=%5.3f C\n",Pgain,Igain,Dgain,Ti,Td,Tf,deadband);
}

void PIDControlInit()
{
	clearednoise=false;
	outputavestarttick=0;
	PIDSetIntegral(GetInitOutput());
	dterm=0;
	lasttemptick=TempTick();
	lasttemp=TempGetTempAve();
	rampstatus=kRampDisabled;
	theta0ticks=ceil(TIMER_N_TICKS_PER_SEC*theta0);
	rampvalues=(float*)malloc((theta0ticks+1)*sizeof(float));
}

void PIDControlDeinit()
{
	free(rampvalues);
}

float PIDControl()
{
	//Same thread as temperature, so we don't need to use non-blocking algorithms
	uint32_t temptick=TempTick();
	float tempval=TempGetTempAve();

	uint32_t dtick = temptick - lasttemptick;
	float dtime = Tick2Sec(dtick);
	float error = tempval - GetTargetTemp();

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
		float newoutputave=outputsum/(temptick-outputavestarttick);
		avecycletemp/=temptick-outputavestarttick;
		printf("Computed new average output is %7.3f%%, average temp is %7.3f C, cycle time was %8.3f\n",newoutputave*100,avecycletemp,Tick2Sec(temptick-outputavestarttick));

		outputsum=0;
		avecycletemp=0;
		outputavestarttick=temptick;
	}

	float output;

	if(rampstatus==kRampDisabled && error < -rampthresh) {
		output=1;
		rampstatus=kRampInitialising;
		ridx=0;
		rampvalues[ridx]=error;
		printf("%8.3f: Temp: %6.2f C => %6.2f%% (initialising ramp)\n",Tick2Sec(temptick),tempval,100*output);

	} else if(rampstatus==kRampInitialising) {
		++ridx;
		rampvalues[ridx]=error;

		if(error>=0) {
			output=0;
			rampstatus=kRampWait;
			rampwaitticks=theta0ticks;
		    printf("%8.3f: Temp: %6.2f C => %6.2f%% (waiting for ramp effect: %u)\n",Tick2Sec(temptick),tempval,100*output,rampwaitticks);

		} else{
			output=1;

			if(ridx==theta0ticks) rampstatus=kRampActive;
		    printf("%8.3f: Temp: %6.2f C => %6.2f%% (activating ramp)\n",Tick2Sec(temptick),tempval,100*output);
		}

	} else if(rampstatus==kRampActive) {
		ridx=(ridx+1)%(theta0ticks+1);
		rampvalues[ridx]=error;

		//If reach setpoint within the next deadtime
		if(2*error-rampvalues[(ridx+theta0ticks)%(theta0ticks+1)] >=0) {
			output=0;
			rampstatus=kRampWait;
			rampwaitticks=theta0ticks;
		    printf("%8.3f: Temp: %6.2f C => %6.2f%% (waiting for ramp effect: %u)\n",Tick2Sec(temptick),tempval,100*output,rampwaitticks);

		} else {
			output=1;
		    printf("%8.3f: Temp: %6.2f C => %6.2f%% (ramp active)\n",Tick2Sec(temptick),tempval,100*output);
		}

	} else if(rampstatus==kRampWait) {
		output=0;
		--rampwaitticks;

		if(rampwaitticks==1) rampstatus=kRampDisabled;
		printf("%8.3f: Temp: %6.2f C => %6.2f%% (waiting for ramp effect: %u)\n",Tick2Sec(temptick),tempval,100*output,rampwaitticks);

	} else {
		dterm = (Tf * dterm - Dgain * (tempval - lasttemp)) / (Tf + dtime);

		//If outside the deadband, the integral value needs to be updated
		if(fabsf(error) > deadband) {
			const float defpterm = Pgain * -error;
			float pterm = defpterm;
			float diterm = Igain * dtime * -error;
			float errorterm = defpterm + diterm;

			float errorscale=1;

			output = errorterm + integral + dterm;

			//If maxing out the output
			if(output > 1) {
				errorscale = (1 - output + errorterm) / errorterm;
				output=1;
				diterm *= errorscale;
				integral += diterm;

				//If reaching the integral upper bound
				if(integral > maxintegralvalue) {
					diterm = maxintegralvalue - integral;
					integral = maxintegralvalue;
					pterm = 1 - (integral + dterm);

					if(pterm > defpterm) {
						pterm = defpterm;
						output = pterm + integral + dterm;
					}

				} else pterm = 1 - (integral + dterm);

				//If zeroing the output
			} else if(output < 0) {
				errorscale = (errorterm - output) / errorterm;
				output = 0;
				diterm *= errorscale;
				integral += diterm;

				//If reaching the integral lower bound
				if(integral < minintegralvalue) {
					diterm = integral - minintegralvalue;
					integral = minintegralvalue;
					pterm = -(integral + dterm);

					if(pterm < defpterm) {
						pterm = defpterm;
						output = pterm + integral + dterm;
					}

				} else pterm = -(integral + dterm);

				//Else if the errorterm is not saturating the output
			} else {
				integral += diterm;

				if(integral > maxintegralvalue) {
					diterm = maxintegralvalue - integral;
					integral = maxintegralvalue;
					output = pterm + integral + dterm;

				} else if(integral < minintegralvalue) {
					diterm = integral - minintegralvalue;
					integral = minintegralvalue;
					output = pterm + integral + dterm;
				}
			}
			printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, DeltaI=%6.2f%%, D=%6.2f%%, I=%6.2f%%, Escale=%6.2f%%)\n",Tick2Sec(temptick),tempval,100*output,100*pterm,100*diterm,100*dterm,100*integral,100*errorscale);

		} else {
			float pterm = Pgain * -error;
			output = pterm + integral + dterm;

			if(output > 1) {
				output=1;

			} else if(output < 0) {
				output=0;
			}
			printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, DeltaI=>0.00%%<, D=%6.2f%%, I=%6.2f%%)\n",Tick2Sec(temptick),tempval,100*output,100*pterm,100*dterm,100*integral);
		}
	}

	PWMSetOutput(output);
	outputsum+=output*dtick;
	avecycletemp+=tempval*dtick;

	lasttemp = tempval;
	lasttemptick = temptick;

	return tempval;
}
