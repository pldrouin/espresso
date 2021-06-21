/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 */

#include "pid_atune.h"

static float integral;
static float dterm=0;
static float lasttemp;
static uint32_t lasttemptime;
static float outputsum=0;
static uint32_t outputavestarttime=0;
static float avecycletemp;
static bool clearednoise;

static float Pgain=0, Igain=0, Dgain=0, Ti=0, Td=0, Tf=0;
static float maxintegralvalue=1, minintegralvalue=0;
static float deadband=0;
static float minerrorlevel=0.05;
static float curerrorlevel;
static uint32_t maxintmultiplicator=64;
static uint32_t curintmultiplicator;

void PIDSetParams(const float& Ki, const float& Theta0, const float& Kcfact, const float& Tifact, const float& Tdfact)
{
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
	outputavestarttime=0;
	PIDSetIntegral(GetInitOutput());
	dterm=0;
	curintmultiplicator=1;
	curerrorlevel=minerrorlevel;
	lasttemptime=TempTime();
	lasttemp=TempGetTempAve();
}

float PIDControl()
{
	//Same thread as temperature, so we don't need to use non-blocking algorithms
	uint32_t temptime=TempTime();
	float tempval=TempGetTempAve();

	uint32_t dtick = temptime - lasttemptime;
	float dtime = dtick / (float)CONFIG_CONTROL_SAMPLING_FREQ;
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
		float newoutputave=outputsum/(temptime-outputavestarttime);
		avecycletemp/=temptime-outputavestarttime;
		printf("Computed new average output is %7.3f%%, average temp is %7.3f C, cycle time was %8.3f\n",newoutputave*100,avecycletemp,(temptime-outputavestarttime)/(float)CONFIG_CONTROL_SAMPLING_FREQ);

		outputsum=0;
		avecycletemp=0;
		outputavestarttime=temptime;
	}

	float pterm = Pgain * -error;
	dterm = (Tf * dterm - Dgain * (tempval - lasttemp)) / (Tf + dtime);

	//Start by computing the new output using the previous integral value
	float output = pterm + integral + dterm;
	const float abserror=fabsf(error);

	//If outside the deadband, the integral value needs to be updated
	if(abserror > deadband) {

		if(abserror > curerrorlevel && curintmultiplicator < maxintmultiplicator) {
			curerrorlevel*=2;
			curintmultiplicator<<=1;

		} else if(abserror < 0.5*curerrorlevel && curintmultiplicator > 1) {
			curerrorlevel*=0.5;
			curintmultiplicator>>=1;
		}
		float diterm=Igain * -error * curintmultiplicator * dtime; //Default integral increment

		//Ensure the integral value remains within the defined limits
		if(integral+diterm > maxintegralvalue) diterm=maxintegralvalue-integral;
		else if(integral+diterm < minintegralvalue) diterm=minintegralvalue-integral;
		//Attempt of a new output value using the updated integral value
		float outputp = output + diterm;

		//Dynamic reset limit
		//If the diterm can be included without saturating, add it to the integral
		if(outputp <= 1 && outputp >= 0) {
			integral += diterm;
			output = outputp;

		//Else if reaching saturation somewhere along the way between the previous integral value and the new integral value,
		//only go up to saturation and stop the integral there
		} else if(output < 1 && output > 0) {

			if(outputp > 1) {
				integral += 1-output;
				output = 1;

			} else { //outputp < 0
				integral -= output;
				output = 0;
			}

		} else if(output > 1) output=1;

		//Else if output < 0
		else output=0;
	    printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, DeltaI=%6.2f%% (x%2u), D=%6.2f%%, I=%6.2f%%)\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,100*output,100*pterm,100*diterm,curintmultiplicator,100*dterm,100*integral);

	} else {

		if(output > 1) output=1;

		else if(output < 0) output=0;
		printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, DeltaI=>0.00%%< (x 0), D=%6.2f%%, I=%6.2f%%)\n",temptime/(float)CONFIG_CONTROL_SAMPLING_FREQ,tempval,100*output,100*pterm,100*dterm,100*integral);
	}

	PWMSetOutput(output);
	outputsum+=output*dtick;
	avecycletemp+=tempval*dtick;

	lasttemp = tempval;
	lasttemptime = temptime;

	return tempval;
}
