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
//kRampDisabled: Regular PID algorithm
enum ramp_state_type {kRampActive=0, kRampWait=1, kRampDisabled=2};

enum ramp_reason_type {kTempTooLow=0, kTempTooHigh=1};

static const uint32_t ticksbetweencalls=ALARM_N_TICKS*CONFIG_CONTROLLER_SAMPLING_PERIOD_N_SAMPLES;
static const float secsbetweencalls=(float)ticksbetweencalls/TIMER_N_TICKS_PER_SEC;
static const float callspersec=(float)TIMER_N_TICKS_PER_SEC/ticksbetweencalls;

static float integral; //PID integral term
static float dterm=0;  //PID derivative term
static float lasttemp; //Temperature read in the previous PID cycle
static float lasttderiv;
static uint32_t lasttemptick; //Time tick of the previous PID cycle
static float output;
static uint8_t rampstate; //State machine state for ramp mode
static bool first_ramp;
static bool aggressive_ramp;
static uint16_t ridx; //Index used to compute the temperature derivative using a rolling time window
static int16_t doidx; //Index used for the delayedouputs array
static int16_t minidx; //Index used for the minoutputsums array
static int16_t maxidx; //Index used for the maxoutputsums array
static float* errorvalues; //Array used to compute the temperature derivative using a rolling time window
static float* derivvalues; //Array used to compute the second order temperature derivative using a rolling time window
static float* delayedoutputs; //Used to store outputs over one deadtime
static float* minoutputsums; //Used to store the sums of output*dtick for all the minima, with enough cells for a whole deadtime
static float* maxoutputsums; //Used to store the sums of output*dtick for all the maxima, with enough cells for a whole deadtime
static uint32_t* minoutputdticks; //Used to store the dticks for all the minima, with enough cells for a while deadtime
static uint32_t* maxoutputdticks; //Used to store the dticks for all the maxima, with enough cells for a while deadtime
static float lastextoutputave; //Output average leading to the previous extremum
static bool eoaready; //Output average at extremum ready
static bool lasteoamax; //Last extremum output average was a max
static uint8_t integralforcedupdate; //Number of remaining forced update iterations
static uint8_t nifucycles=4; //Number of integral forced update cycles
static bool lastifuup;  //Last integral forced update was up (error>0)

static float theta0=0; //Total deadtime (in seconds)
static uint8_t naeocycles=1; //Number of cycles used to compute the average output between temperature extrema
static uint32_t theta0nticks; //Total deadtime (in ticks);
static float roundedtheta0; //Total deadtime rounded up to the next multiple of PID cycle time
static uint16_t derivencalls; //Number of PID cycles for the temperature derivative using a rolling time window
static uint16_t doalength; //Number of cells in the delayedoutputs array (=ceil(ceil(theta0)/(secs/cycle))+1)
static uint16_t eoalength; //Number of cells in the minoutputsums and maxoutputsums arrays (=ceil(0.5*ceil(theta0)/(secs/cycle))+1)
static float derivetime=1; //Time interval used to compute the temperature derivative
static float Pgain=0, Igain=0, Dgain=0, Ti=0, Td=0; //PID parameters
static float maxintegralvalue=1, minintegralvalue=0; //PID integral term limits
static float rampthresh=INFINITY; //Temperature error used to bump up the integral term to its maximum. Should not get triggered when idling
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

void PIDSetNIntegralEstimateCycles(const uint8_t& numcycles)
{
	naeocycles=numcycles;
}

void PIDPrintParams()
{
	printf("Pgain (Kc)=%10.6f, Igain=%10.6f/s, Dgain=%10.6fs, Ti=%7.4fs, Td=%7.4fs, deadband=%5.3f C, derivetime=%7.4f\n",Pgain,Igain,Dgain,Ti,Td,deadband,derivetime);
}

void PIDControlInit()
{
	output=0;
	PIDSetIntegral(GetInitOutput());
	dterm=0;
	lasttemptick=TempTick();
	lasttemp=TempGetTempAve();
	lasttderiv=0;
	rampstate=kRampDisabled;
	theta0nticks=ceil(theta0*TIMER_N_TICKS_PER_SEC);
	derivencalls=ceil(derivetime*callspersec);
	roundedtheta0=ceil(theta0*callspersec)/callspersec;
	doalength=ceil(theta0*callspersec)+1;
	eoalength=ceil(0.5*ceil(theta0*callspersec))+1;
	errorvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	derivvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	const float error= lasttemp - GetTargetTemp();

	for(int i=derivencalls; i>=0; --i)  errorvalues[i]=error;
	memset(derivvalues,0,(derivencalls+1)*sizeof(float));
	delayedoutputs=(float*)malloc(doalength*sizeof(float));
	minoutputsums=(float*)malloc(eoalength*sizeof(float));
	maxoutputsums=(float*)malloc(eoalength*sizeof(float));
	minoutputdticks=(uint32_t*)malloc(eoalength*sizeof(uint32_t));
	maxoutputdticks=(uint32_t*)malloc(eoalength*sizeof(uint32_t));
	memset(delayedoutputs,0,doalength*sizeof(float));
	memset(minoutputsums,0,eoalength*sizeof(float));
	memset(maxoutputsums,0,eoalength*sizeof(float));
	memset(minoutputdticks+1,0,(eoalength-1-naeocycles)*sizeof(uint32_t));
	memset(maxoutputdticks+1,0,(eoalength-1-naeocycles)*sizeof(uint32_t));
	minoutputdticks[0]=maxoutputdticks[0]=lasttemptick;

	for(int i=1; i<=naeocycles; ++i) minoutputdticks[eoalength-i]=maxoutputdticks[eoalength-i]=theta0nticks;
	ridx=0;
	doidx=0;
	maxidx=0;
	integralforcedupdate=0;
	lastextoutputave=0;
	lastifuup=true;
	eoaready=false;
	lasteoamax=true;
	integral=0;
}

void PIDControlDeinit()
{
	free(errorvalues);
	free(derivvalues);
	free(delayedoutputs);
	free(minoutputsums);
	free(maxoutputsums);
	free(minoutputdticks);
	free(maxoutputdticks);
}

float PIDControlGetPreviousAveExtOutput(float const* const& extoutputsums, uint32_t const* const& extoutputdticks, int16_t const& extidx)
{
	int16_t idx=(extidx-1)%eoalength;
	uint32_t nticks=extoutputdticks[idx];
	float sum=extoutputsums[idx];

	if(nticks<theta0nticks) {
		int16_t i,j=1;
		idx=(idx-1)%eoalength;

		if(idx<extidx) {

			for(i=idx; i>=0; --i) {
				sum+=extoutputsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= theta0nticks) goto found_prev_amo;
				++j;
			}

			for(i=eoalength-1; i>extidx; --i) {
				sum+=extoutputsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= theta0nticks) goto found_prev_amo;
				++j;
			}

		} else {

			for(i=idx; i>extidx; --i) {
				sum+=extoutputsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= theta0nticks) goto found_prev_amo;
				++j;
			}
		}
	}
	found_prev_amo:
	sum/=nticks;

	printf("Previous average output is %7.3f%%, cycle time was %8.3f\n",sum*100,Tick2Sec(nticks));

	if(sum < minintegralvalue) sum=minintegralvalue;
	else if(sum > maxintegralvalue) sum=maxintegralvalue;
	return sum;
}

float PIDControlGetNextAveMaxOutput(float const* const& extoutputsums, uint32_t const* const& extoutputdticks, int16_t const& extidx, const uint32_t& nowticks, const float& dtime)
{
	int16_t idx;
	uint32_t nticks=nowticks-extoutputdticks[extidx];
	float sum=extoutputsums[extidx];

	if(naeocycles>1 || nticks<theta0nticks) {
		int16_t i,j=1;
		idx=(extidx-1)%eoalength;

		if(idx<extidx) {

			for(i=idx; i>=0; --i) {
				sum+=extoutputsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= theta0nticks) goto found_next_amo;
				++j;
			}

			for(i=eoalength-1; i>extidx; --i) {
				sum+=extoutputsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= theta0nticks) goto found_next_amo;
				++j;
			}

		} else {

			for(i=idx; i>extidx; --i) {
				sum+=extoutputsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= theta0nticks) goto found_next_amo;
				++j;
			}
		}
	}
	found_next_amo:

	int16_t doidxp=(doidx+1)%doalength;
	float remtime=dtime;

	while(remtime>secsbetweencalls) {
		sum+=delayedoutputs[doidxp]*ticksbetweencalls;
		doidxp=(doidxp+1)%doalength;
		remtime-=secsbetweencalls;
	}

	if(remtime>0) sum+=delayedoutputs[doidxp]*remtime*TIMER_N_TICKS_PER_SEC;

	sum/=nticks+dtime*TIMER_N_TICKS_PER_SEC;
	printf("Next average output is %7.3f%%, cycle time was %8.3f\n",sum*100,Tick2Sec(nticks));

	if(sum < minintegralvalue) sum=minintegralvalue;
	else if(sum > maxintegralvalue) sum=maxintegralvalue;
	return sum;
}

float PIDControl()
{
	//Same thread as temperature, so we don't need to use non-blocking algorithms
	uint32_t temptick=TempTick(); //Current time tick
	const float tempval=TempGetTempAve(); //Current temperature

	const uint32_t dtick = temptick - lasttemptick; //Time increment in ticks since last cycle
	const float dtime = Tick2Sec(dtick); //Time increment in seconds since last cycle
	const float error = tempval - GetTargetTemp(); //Temperature error

	const float dos=delayedoutputs[doidx]*dtick;
	minoutputsums[minidx]+=dos;
	maxoutputsums[maxidx]+=dos;

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
	printf("Derivative: %9.6f C/s, Second Derivative: %9.6f C/s^2, Error forecast: %6.2f C\n",tderiv,sectderiv,terrorforecast);

	const bool isnotdeadband=(error>deadband && terrorforecast>deadband) || (error<-deadband && terrorforecast<-deadband);

	if(tderiv>=0 && lasttderiv<0 && sectderiv>0) {
		minoutputdticks[minidx]=temptick-minoutputdticks[minidx];
		printf("Computed new min-based average output is %7.3f%%, cycle time was %8.3f\n",minoutputsums[minidx]/minoutputdticks[minidx]*100,Tick2Sec(minoutputdticks[minidx]));
		minidx=(minidx+1)%eoalength;
		lastextoutputave=PIDControlGetPreviousAveExtOutput(minoutputsums, minoutputdticks, minidx);
		minoutputsums[minidx]=0;
		minoutputdticks[minidx]=temptick;
		eoaready=true;
		lasteoamax=false;

	} else if(tderiv<=0 && lasttderiv>0 && sectderiv<0) {
		maxoutputdticks[maxidx]=temptick-maxoutputdticks[maxidx];
		printf("Computed new max-based average output is %7.3f%%, cycle time was %8.3f\n",maxoutputsums[maxidx]/maxoutputdticks[maxidx]*100,Tick2Sec(maxoutputdticks[maxidx]));
		maxidx=(maxidx+1)%eoalength;
		lastextoutputave=PIDControlGetPreviousAveExtOutput(maxoutputsums, maxoutputdticks, maxidx);
		maxoutputsums[maxidx]=0;
		maxoutputdticks[maxidx]=temptick;
		eoaready=true;
		lasteoamax=true;
	}

	//Note: A full rampthreshold discrepancy is required to exit PID or to reactivate the ramp, but 1/2 discrepancy is required to reenter.

	//If the PID algorithm is active
	if(rampstate>=kRampDisabled) {

		if(eoaready) {
			eoaready=false;

			if(integralforcedupdate) {

				if(lastifuup^(error>0)) {
					--integralforcedupdate;
					lastifuup=!lastifuup;
				}
			}
		}

		if(fabs(error) > deadband) {

			//If the temperature drifted down below the ramp threshold
			if(terrorforecast < -rampthresh) {
				//output=1;
				rampstate=kRampActive;
				aggressive_ramp=false;

				if(integralforcedupdate) {
					first_ramp=false;
					integral=(1+0.5*integralforcedupdate/nifucycles)*lastextoutputave;

				} else {
					first_ramp=true;
					integral=maxintegralvalue;
				}
				printf("Integral value set to %7.3f%%\n",integral*100);

				if(terrorforecast < rampthresh) integralforcedupdate=nifucycles;
				printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*integral);

				//Else if the temperature is above the ramp threshold
			} else if(terrorforecast > rampthresh) {
				//output=0;
				rampstate=kRampWait;
				aggressive_ramp=false;

				if(integralforcedupdate) {
					first_ramp=false;
					integral=(1-0.5*integralforcedupdate/nifucycles)*lastextoutputave;

				} else {
					first_ramp=true;
					integral=minintegralvalue;
				}
				printf("Integral value set to %7.3f%%\n",integral*100);

				if(terrorforecast > rampthresh) integralforcedupdate=nifucycles;
				printf("OFF: %8.3f: Temp: %6.2f C => %6.2f%% (now waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*integral);
			}
		}

		//Else if the PID algorithm was not active
	} else {

		//If the ramp mode is already active
		if(rampstate==kRampActive) {

			if(terrorforecast < -0.5*rampthresh && error < -0.5*rampthresh) {

				if(!aggressive_ramp) {
					aggressive_ramp=true;
					integral=maxintegralvalue;
					printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting aggressive ramp)\n",Tick2Sec(temptick),tempval,100*integral);
					integralforcedupdate=nifucycles;
				}

			} else {

				if(eoaready) {

					if(integralforcedupdate && (lastifuup^(error>0))) {
						--integralforcedupdate;
						lastifuup=!lastifuup;
					}
				}

				if(terrorforecast < 0) {

					if(aggressive_ramp || (!first_ramp && eoaready)) {
					    integral=(1+0.5*integralforcedupdate/nifucycles)*lastextoutputave;

						if(integral>maxintegralvalue) integral=maxintegralvalue;
						aggressive_ramp=false;
						printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*integral);
					}

				} else {
					rampstate=kRampDisabled;
					integral=lastextoutputave;

					if(tderiv <= 0 && sectderiv > 0) {
						float zderivdt=-tderiv/sectderiv;
						integral=PIDControlGetNextAveMaxOutput(minoutputsums, minoutputdticks, minidx, temptick, (zderivdt<theta0 ? zderivdt : theta0));
					}
					printf("Integral value set to %7.3f%%\n",integral*100);
					printf("TSC: %8.3f: Temp: %6.2f C => %6.2f%% (starting regular PID)\n",Tick2Sec(temptick),tempval,100*integral);
				}
			}

			//If waiting for the temperature to stop increasing at the end of the ramp mode
		} else if(rampstate==kRampWait) {

			if(terrorforecast > 0.5*rampthresh && error > 0.5*rampthresh) {

				if(!aggressive_ramp) {
					aggressive_ramp=true;
					integral=minintegralvalue;
					printf("OFF: %8.3f: Temp: %6.2f C => %6.2f%% (now aggressively waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*integral);
					integralforcedupdate=nifucycles;
				}

			} else {

				if(eoaready) {

					if(integralforcedupdate && (lastifuup^(error>0))) {
						--integralforcedupdate;
						lastifuup=!lastifuup;
					}
				}

				if(terrorforecast > 0) {

					if(aggressive_ramp || (!first_ramp && eoaready)) {
					    integral=(1-0.5*integralforcedupdate/nifucycles)*lastextoutputave;

						if(integral<minintegralvalue) integral=minintegralvalue;
						aggressive_ramp=false;
						printf("OFF: %8.3f: Temp: %6.2f C => %6.2f%% (now waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*integral);
					}

				} else {
					rampstate=kRampDisabled;
					integral=lastextoutputave;

					if(tderiv >= 0 && sectderiv < 0) {
						float zderivdt=-tderiv/sectderiv;
						integral=PIDControlGetNextAveMaxOutput(maxoutputsums, maxoutputdticks, maxidx, temptick, (zderivdt<theta0 ? zderivdt : theta0));
					}

					printf("Integral value set to %7.3f%%\n",integral*100);
					printf("TSC: %8.3f: Temp: %6.2f C => %6.2f%% (starting regular PID)\n",Tick2Sec(temptick),tempval,100*integral);
				}
			}
		}
		eoaready=false;
	}

	//If outside the deadband, the integral value needs to be updated
	if(isnotdeadband) {

		if((tderiv > 0 && sectderiv < 0 && error < (tderiv*tderiv)/(2*sectderiv)) ||
				(tderiv < 0 && sectderiv > 0 && error > (tderiv*tderiv)/(2*sectderiv))) {
			dterm=0;
			printf("PI : ");

		} else {
			dterm = tderiv * -Dgain;
			printf("PID: ");
		}

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

		if((tderiv > 0 && sectderiv < 0 && error < (tderiv*tderiv)/(2*sectderiv)) ||
				(tderiv < 0 && sectderiv > 0 && error > (tderiv*tderiv)/(2*sectderiv))) {
			dterm=0;
			printf("P  : ");

		} else {
			dterm = tderiv * -Dgain;
			printf("P D: ");
		}
		float pterm = Pgain * -error;
		output = pterm + integral + dterm;

		if(output > 1) {
			output=1;

		} else if(output < 0) {
			output=0;
		}
		printf("%8.3f: Temp: %6.2f C => %6.2f%% (P=%6.2f%%, DeltaI=  0.00%%, D=%6.2f%%, I=%6.2f%%)\n",Tick2Sec(temptick),tempval,100*output,100*pterm,100*dterm,100*integral);
	}

	delayedoutputs[doidx]=output;
	doidx=(doidx+1)%doalength;

	PWMSetOutput(output);

	lasttemp = tempval;
	lasttderiv = tderiv;
	lasttemptick = temptick;

	return tempval;
}
