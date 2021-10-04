/*
 * pid_atune.cpp
 *
 *  Created on: Jun 4, 2021
 *      Author: pldrouin
 *      Description: Optimised PID algorithm with integral windup prevention, integral deadband, automatic derivative disabling
 *                   and switching to a Taylor series based control algorithm when a large regime change is detected. This allows
 *                   to greatly improve response and minimise overshooting and undershooting compared to a PID-based only algorithm.
 */

#include "pid_atune.h"

//kRampActive: Temperature ramp is active, output is 100%
//kRampWait: Waiting for the temperature forecast to peak
//kRampPostMin, kRampPostMax: PID algorithm back on with derivative term disabled while waiting for the temperature to peak
//kRampDisabled: Regular PID algorithm
enum ramp_state_type {kRampActive=0, kRampWait=1, kRampDisabled=2, kRampPostMin=3, kRampPostMax=4};

static const float callspersec=(float)TIMER_N_TICKS_PER_SEC/(ALARM_N_TICKS*CONFIG_CONTROLLER_SAMPLING_PERIOD_N_SAMPLES);

static float integral; //PID integral term
static float dterm=0;  //PID derivative term
static float lasttemp; //Temperature read in the previous PID cycle
static float lasttderiv;
static uint32_t lasttemptick; //Time tick of the previous PID cycle
static float output;
#ifdef PID_ALGO_STATS
static float outputsum; //Used to compute average output for reporting purpose
static uint32_t outputavestarttick; //Used to keep track of time when computing average output for reporting purpose
static float avecycletemp; //Used to compute average temperature for reporting purpose
static bool clearednoise; //Used to prevent triggering on noise when computing statistics for reporting purpose
#endif
static uint8_t rampstate; //State machine state for ramp mode
static uint16_t ridx; //Index used to compute the temperature derivative using a rolling time window
static int16_t maxidx; //Index used for the maxoutputsums array
static float* errorvalues; //Array used to compute the temperature derivative using a rolling time window
static float* derivvalues; //Array used to compute the second order temperature derivative using a rolling time window
static float* maxoutputsums; //Used to store the sums of output*dtick for all the maxima, with enough cells for a whole deadtime
static uint32_t* maxoutputdticks; //Used to store the dticks for all the maxima, with enough cells for a while deadtime
static bool integralforcedupdate;

static float theta0=0; //Total deadtime (in seconds)
static uint32_t theta0nticks; //Total deadtime (in ticks);
static float roundedtheta0; //Total deadtime rounded up to the next multiple of PID cycle time
static uint16_t derivencalls; //Number of PID cycles for the temperature derivative using a rolling time window
static uint16_t moalength; //Number of cells in the maxoutputsums array (=ceil(0.5*ceil(theta0)/(secs/cycle))+1)
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
	output=0;
#ifdef PID_ALGO_STATS
	clearednoise=false;
	outputsum=0;
	outputavestarttick=0;
#endif
	PIDSetIntegral(GetInitOutput());
	dterm=0;
	lasttemptick=TempTick();
	lasttemp=TempGetTempAve();
	lasttderiv=0;
	rampstate=kRampDisabled;
	theta0nticks=ceil(theta0*TIMER_N_TICKS_PER_SEC);
	derivencalls=ceil(derivetime*callspersec);
	roundedtheta0=ceil(theta0*callspersec)/callspersec;
	moalength=ceil(0.5*ceil(theta0*callspersec))+1;
	errorvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	derivvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	memset(errorvalues,0,(derivencalls+1)*sizeof(float));
	memset(derivvalues,0,(derivencalls+1)*sizeof(float));
	maxoutputsums=(float*)malloc(moalength*sizeof(float));
	maxoutputdticks=(uint32_t*)malloc(moalength*sizeof(uint32_t));
	memset(maxoutputsums,0,moalength*sizeof(float));
	memset(maxoutputdticks,0,moalength*sizeof(uint32_t));
	maxoutputdticks[0]=lasttemptick;
	maxoutputdticks[moalength-1]=1;
	ridx=0;
	maxidx=0;
	integralforcedupdate=true;
}

void PIDControlDeinit()
{
	free(errorvalues);
	free(derivvalues);
	free(maxoutputsums);
	free(maxoutputdticks);
}

float PIDControlGetPreviousAxeMaxOutput()
{
	int16_t idx=(maxidx-1)%moalength;
	uint32_t nticks=maxoutputdticks[idx];
	float sum=maxoutputsums[idx];

	if(nticks<theta0nticks) {
		int16_t i;
		idx=(idx-1)%moalength;

		if(idx<maxidx) {

			for(i=idx; i>=0; --i) {

				if(nticks+maxoutputdticks[i] >= theta0nticks) {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;

					goto found_prev_amo;

				} else {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;
				}
			}

			for(i=moalength-1; i>maxidx; --i) {

				if(nticks+maxoutputdticks[i] >= theta0nticks) {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;

					goto found_prev_amo;

				} else {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;
				}
			}

		} else {

			for(i=idx; i>maxidx; --i) {

				if(nticks+maxoutputdticks[i] >= theta0nticks) {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;

					goto found_prev_amo;

				} else {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;
				}
			}
		}
	}
	found_prev_amo:
	sum/=nticks;

	printf("Previous max-based average output is %7.3f%%, cycle time was %8.3f\n",sum*100,Tick2Sec(nticks));

	if(sum < minintegralvalue) sum=minintegralvalue;
	else if(sum > maxintegralvalue) sum=maxintegralvalue;
	return sum;
}

float PIDControlGetNextAxeMaxOutput(const uint32_t& nowticks, const float& dtime)
{
	int16_t idx;
	uint32_t nticks=nowticks-maxoutputdticks[maxidx];
	float sum=maxoutputsums[maxidx];

	if(nticks<theta0nticks) {
		int16_t i;
		idx=(maxidx-1)%moalength;

		if(idx<maxidx) {

			for(i=idx; i>=0; --i) {

				if(nticks+maxoutputdticks[i] >= theta0nticks) {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;

					goto found_next_amo;

				} else {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;
				}
			}

			for(i=moalength-1; i>maxidx; --i) {

				if(nticks+maxoutputdticks[i] >= theta0nticks) {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;

					goto found_next_amo;

				} else {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;
				}
			}

		} else {

			for(i=idx; i>maxidx; --i) {

				if(nticks+maxoutputdticks[i] >= theta0nticks) {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;

					goto found_next_amo;

				} else {
					sum+=maxoutputsums[i]*(theta0nticks-nticks)/(float)maxoutputdticks[i];
					nticks=theta0nticks;
				}
			}
		}
	}
	found_next_amo:
	sum/=nticks+dtime*TIMER_N_TICKS_PER_SEC;
	printf("Next max-based average output is %7.3f%%, cycle time was %8.3f\n",sum*100,Tick2Sec(nticks));

	if(sum < minintegralvalue) sum=minintegralvalue;
	else if(sum > maxintegralvalue) sum=maxintegralvalue;
	return sum;
}

float PIDControl()
{
	//Same thread as temperature, so we don't need to use non-blocking algorithms
	uint32_t temptick=TempTick(); //Current time tick
	float tempval=TempGetTempAve(); //Current temperature

	uint32_t dtick = temptick - lasttemptick; //Time increment in ticks since last cycle
	float dtime = Tick2Sec(dtick); //Time increment in seconds since last cycle
	float error = tempval - GetTargetTemp(); //Temperature error
	float newintegral;

#ifdef PID_ALGO_STATS
	outputsum+=output*dtick;
	avecycletemp+=lasttemp*dtick;

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
#endif

	maxoutputsums[maxidx]+=output*dtick;

	//dterm = (Tf * dterm - Dgain * (tempval - lasttemp)) / (Tf + dtime);
	//Compute PID derivative term
	errorvalues[ridx]=error;
	const float tderiv = (errorvalues[ridx]-errorvalues[(ridx+1)%(derivencalls+1)])/derivetime;
	derivvalues[ridx]=tderiv;
	const float sectderiv = (derivvalues[ridx]-derivvalues[(ridx+1)%(derivencalls+1)])/derivetime;
	//Increment temperature derivative rolling time window index and update the temperature in the array
	ridx = (ridx+1)%(derivencalls+1);

	//Forecasted values are computed one deadtime into the future
	const float terrorforecast = error + roundedtheta0 * (tderiv + roundedtheta0 * 0.5 * sectderiv);
	//const float tderivforecast = tderiv + sectderiv * roundedtheta0;
	//printf("Derivative: %9.6f C/s, Second Derivative: %9.6f C/s^2, Error forecast: %6.2f C, Derivative forecast: %9.6f C/s\n",tderiv,sectderiv,terrorforecast,tderivforecast);
	printf("Derivative: %9.6f C/s, Second Derivative: %9.6f C/s^2, Error forecast: %6.2f C\n",tderiv,sectderiv,terrorforecast);

	if(tderiv<=0 && lasttderiv>0 && sectderiv<0) {
		maxoutputdticks[maxidx]=temptick-maxoutputdticks[maxidx];
		printf("Computed new max-based average output is %7.3f%%, cycle time was %8.3f\n",maxoutputsums[maxidx]/maxoutputdticks[maxidx]*100,Tick2Sec(maxoutputdticks[maxidx]));
		maxidx=(maxidx+1)%moalength;
		maxoutputsums[maxidx]=0;
		maxoutputdticks[maxidx]=temptick;

		if(integralforcedupdate && terrorforecast < -rampthresh) {

			if(terrorforecast < -rampthresh || terrorforecast > rampthresh) integral=PIDControlGetPreviousAxeMaxOutput();
			integralforcedupdate=false;
		}
	}

	//Note: A full rampthreshold discrepancy is required to exit PID or to reactivate the ramp, but 1/2 discrepancy is required to reenter.

	//If the PID algorithm is active
	if(rampstate>=kRampDisabled) {

		//If the temperature drifted down below the ramp threshold
		if(terrorforecast < -rampthresh) {
			output=1;
			rampstate=kRampActive;
			printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*output);

		//Else if the temperature is above the ramp threshold
		} else {

			//If post ramp state, can switch into regular PID mode
			//Note: The PID derivative term does not affect the integral directly, so it is better to deactivate it when turning
			//on the PID algorithm before reaching a maximum or a minimum in order to reach it as fast as possible to avoid affecting the integral
			//term more than necessary through the error.
			if((rampstate==kRampPostMin && (error<=0 || sectderiv<=0 || tderiv>=0)) || (rampstate==kRampPostMax && (error>=0 || sectderiv>=0 || tderiv<=0))) rampstate=kRampDisabled;

			if(terrorforecast>rampthresh) {
				output=0;
				rampstate=kRampWait;
				printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (now waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*output);
			}
		}

		//Else if the PID algorithm was not active
	} else {

		//If the ramp mode is already active
		if(rampstate==kRampActive) {

			if(terrorforecast < -0.5*rampthresh) {
				output=1;
				printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (ramp active)\n",Tick2Sec(temptick),tempval,100*output);

			} else {

				if(terrorforecast <= 0.5*rampthresh) {
					integralforcedupdate=true;

					if(error<0 && tderiv>0) rampstate=kRampPostMax;
					else rampstate=kRampDisabled;
				    printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (starting PID)\n",Tick2Sec(temptick),tempval,100*output);

				} else {
					output=0;
					rampstate=kRampWait;
					printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (now waiting for ramp effect)\n",Tick2Sec(temptick),tempval,100*output);
				}
			}

			//If waiting for the temperature to stop increasing at the end of the ramp mode
		} else if(rampstate==kRampWait) {

			if(terrorforecast < -rampthresh) {
				output=1;
				rampstate=kRampActive;
				printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*output);

				//We want to wait if the temperature forecast is too high or if it is accelerating
			} else if(terrorforecast > 0.5*rampthresh) {
				output=0;
				printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (waiting for ramp effect)\n",Tick2Sec(temptick),tempval,100*output);

			} else {
				//It is possible to greatly simplify the code below if ramp post states are no longer needed and the current
				//integral updating algorithm turn out to be optimal.

				if(tderiv >= 0) {

					if(sectderiv < 0) {
						//Use an estimate of the average power since the last temperature maximum to the forecasted time of
						//the next temperature maximum as an estimate for the integral term
						float newintegral=PIDControlGetNextAxeMaxOutput(temptick, tderiv / -sectderiv);

						//Since the power is turned off to prevent the temperature from overshooting, the smallest value
						//between the projected estimate and the current value is used, but only if we are still expecting to overshoot.
						if(error > (tderiv*tderiv)/(2*sectderiv) && integral>newintegral) integral=newintegral;
						integralforcedupdate=true;

						if(error<0) rampstate=kRampPostMax;
						else rampstate=kRampDisabled;

					} else {
						//Temperature is rising and accelerating, so an estimate for the next maximum cannot be computed.
						newintegral=PIDControlGetPreviousAxeMaxOutput();

						//Since the power is turned off to prevent the temperature from overshooting, the smallest value
						//between the last value and the current value is used.
						if(integral>newintegral) integral=newintegral;
						integralforcedupdate=true;
						rampstate=kRampDisabled;
					}

				} else { //tderiv < 0
					//Not sure this algorithm here is the best

					if(sectderiv < 0) {
						//Assertion: The temperature peaked very recently so we should know the best integral estimate
						newintegral=PIDControlGetPreviousAxeMaxOutput();

						//Since the power is turned off to prevent the temperature from overshooting, the smallest value
						//between the last value and the current value is used.
						if(integral>newintegral) integral=newintegral;
						integralforcedupdate=false;
						rampstate=kRampDisabled;

					} else {
						//Temperature is dropping but might reach a minimum soon
						newintegral=PIDControlGetPreviousAxeMaxOutput();

						//Since the power is turned off to prevent the temperature from overshooting, the smallest value
						//between the last value and the current value is used.
						if(integral>newintegral) integral=newintegral;
						integralforcedupdate=true;

						if(error>0 && sectderiv>0) rampstate=kRampPostMin;
						else rampstate=kRampDisabled;
					}
				}
				printf("TSC %8.3f: Temp: %6.2f C => %6.2f%% (starting PID)\n",Tick2Sec(temptick),tempval,100*output);
			}
		}
	}

	if(rampstate>=kRampDisabled) {

		//No D term in ramp post mode because the derivative forecast is actually <= 0, while the current derivative is greater than 0.
		//The forecasted value can be quite noisy though.
		/*
	    if(rampstate>kRampDisabled) dterm = 0;

	    else dterm = tderiv * -Dgain;
	    */
		if((tderiv > 0 && sectderiv < 0 && error < (tderiv*tderiv)/(2*sectderiv)) ||
		(tderiv < 0 && sectderiv > 0 && error > (tderiv*tderiv)/(2*sectderiv))) {
			dterm=0;
			printf("PI  ");

		} else {
			dterm = tderiv * -Dgain;
			printf("PID ");
		}

		//If outside the deadband, the integral value needs to be updated
		if(fabsf(error) > deadband) {

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

	lasttemp = tempval;
	lasttderiv = tderiv;
	lasttemptick = temptick;

	return tempval;
}
