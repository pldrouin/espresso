/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

enum ramp_status_type {kRampDisabled, kRampActive, kRampWait};

static const float callspersec=(float)TIMER_N_TICKS_PER_SEC/(ALARM_N_TICKS*CONFIG_CONTROLLER_SAMPLING_PERIOD_N_SAMPLES);

static float integral; //PID integral term
static float dterm=0;  //PID derivative term
static float lasttemp; //Temperature read in the previous PID cycle
static uint32_t lasttemptick; //Time tick of the previous PID cycle
static float outputsum=0; //Used to compute average output for reporting purpose
static uint32_t outputavestarttick=0; //Used to keep track of time when computing average output for reporting purpose
static float avecycletemp; //Used to compute average temperature for reporting purpose
static bool clearednoise; //Used to prevent triggering on noise when computing statistics for reporting purpose
static uint8_t rampstatus; //State machine status for ramp mode
static uint32_t ridx; //Index used to compute the temperature derivative using a rolling time window
static float* errorvalues; //Array used to compute the temperature derivative using a rolling time window
static float* derivvalues; //Array used to compute the temperature derivative using a rolling time window

static float theta0=0; //Total deadtime (in seconds)
static float roundedtheta0; //Total deadtime rounded up to the next multiple of PID cycle time
static uint32_t derivencalls; //Number of PID cycles for the temperature derivative using a rolling time window
static float derivetime=1; //Time interval used to compute the temperature derivative
static float Pgain=0, Igain=0, Dgain=0, Ti=0, Td=0; //PID parameters
static float maxintegralvalue=1, minintegralvalue=0; //PID integral term limits
static float rampthresh=INFINITY; //Temperature error used to bump up the integral term to its maximum
static float deadband=0; //Temperature deadband where the integral should be fixed

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

void PIDSetDeriveTime(const float& time)
{
	derivetime=time;
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
	printf("Pgain (Kc)=%10.6f, Igain=%10.6f/s, Dgain=%10.6fs, Ti=%7.4fs, Td=%7.4fs, deadband=%5.3f C, derivetime=%7.4f\n",Pgain,Igain,Dgain,Ti,Td,deadband,derivetime);
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
	derivencalls=ceil(derivetime*callspersec);
	roundedtheta0=ceil(theta0*callspersec)/callspersec;
	errorvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	derivvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	memset(errorvalues,0,(derivencalls+1)*sizeof(float));
	memset(derivvalues,0,(derivencalls+1)*sizeof(float));
	ridx=0;
}

void PIDControlDeinit()
{
	free(errorvalues);
	free(derivvalues);
}

float PIDControl()
{
	//Same thread as temperature, so we don't need to use non-blocking algorithms
	uint32_t temptick=TempTick(); //Current time tick
	float tempval=TempGetTempAve(); //Current temperature

	uint32_t dtick = temptick - lasttemptick; //Time increment in ticks since last cycle
	float dtime = Tick2Sec(dtick); //Time increment in seconds since last cycle
	float error = tempval - GetTargetTemp(); //Temperature error

	//Compute statistics
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

	float output=0;

	//dterm = (Tf * dterm - Dgain * (tempval - lasttemp)) / (Tf + dtime);
	//Compute PID derivative term
	const float tderiv = (errorvalues[ridx]-errorvalues[(ridx+1)%(derivencalls+1)])/derivetime;
	const float sectderiv = (derivvalues[ridx]-derivvalues[(ridx+1)%(derivencalls+1)])/derivetime;
	dterm = tderiv * -Dgain;
	//Increment temperature derivative rolling time window index and update the temperature in the array
	ridx = (ridx+1)%(derivencalls+1);
	errorvalues[ridx]=error;
	derivvalues[ridx]=tderiv;
	printf("Derivative: %f C/s\tSecond Derivative: %f C/s^2\n",tderiv,sectderiv);

	//If the temperature drifted down below the ramp threshold
	if(rampstatus==kRampDisabled && error < -rampthresh) {
		output=1;
		rampstatus=kRampActive;
		printf("%8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*output);

    //If the ramp mode is already active
	} else if(rampstatus==kRampActive) {

		if(error >=0 || error + roundedtheta0 * (tderiv + roundedtheta0 * 0.5 * sectderiv) >= 0) {
			output=0;
			rampstatus=kRampWait;
			printf("%8.3f: Temp: %6.2f C => %6.2f%% (waiting for ramp effect)\n",Tick2Sec(temptick),tempval,100*output);

		} else {
			output=1;
			printf("%8.3f: Temp: %6.2f C => %6.2f%% (ramp active)\n",Tick2Sec(temptick),tempval,100*output);
		}

	//If waiting for the temperature to stop increasing at the end of the ramp mode
	} else if(rampstatus==kRampWait) {
		output=0;

		if(tderiv + sectderiv * roundedtheta0 <= 0) {
			rampstatus=kRampDisabled;
			integral=maxintegralvalue;
		}
		printf("%8.3f: Temp: %6.2f C => %6.2f%% (waiting for ramp effect)\n",Tick2Sec(temptick),tempval,100*output);
	}

	if(rampstatus==kRampDisabled) {

		//If outside the deadband, the integral value needs to be updated
		if(fabsf(error) > deadband) {

			//Set the integral to its maximum value if exceeding the threshold to ramp up the temperature
			if(error < -rampthresh) integral=maxintegralvalue;

			const float defpterm = Pgain * -error;
			//Default PID P term value
			float pterm = defpterm;
			//Initial PID I term increment
			float diterm = Igain * dtime * -error;
			//Initial PID P+I terms
			float errorterm = defpterm + diterm;

			//Initial scaling factor value
			float errorscale=1;

			//Initial PID output value
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

			//Else if within the PID deadband
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
