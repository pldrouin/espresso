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
static uint8_t aggressive_ramp;
static uint16_t ridx; //Index used to compute the temperature derivative using a rolling time window
static int16_t minidx; //Index used for the minoutputsums array
static int16_t maxidx; //Index used for the maxoutputsums array
static float* errorvalues; //Array used to compute the temperature derivative using a rolling time window
static float* derivvalues; //Array used to compute the second order temperature derivative using a rolling time window
static float* minoutputsums; //Used to store the sums of output*dtick for all the minima, with enough cells for a whole deadtime
static float* minerrorsums; //Used to store the sums of error*dtick for all the minima, with enough cells for a whole deadtime
static float* maxoutputsums; //Used to store the sums of output*dtick for all the maxima, with enough cells for a whole deadtime
static float* maxerrorsums; //Used to store the sums of error*dtick for all the maxima, with enough cells for a whole deadtime
static uint32_t* minoutputdticks; //Used to store the dticks for all the minima, with enough cells for a while deadtime
static uint32_t* maxoutputdticks; //Used to store the dticks for all the maxima, with enough cells for a while deadtime
static float lastextoutputave; //Last output average at extremum
static float lastexterrave; //Last error average at extremum
static float integralest; //Best integral estimate
static float prevminerror; //Error for the previous minimum
static float prevmaxerror; //Error for the previous maximum
static float integralscaling; //Scaling factor for the integral term when in ramp mode
static bool eoaready; //Output average at extremum ready
static uint8_t integralforcedupdate; //Number of remaining forced update iterations
static uint8_t nifucycles=4; //Number of integral forced update cycles
static bool lastextup;  //Last extremum was up (error>0)
static bool lastifuup;  //Last integral forced update was up (error>0)

static float theta0=0; //Total deadtime (in seconds)
static uint8_t naeocycles=1; //Number of cycles used to compute the average output between temperature extrema
static uint32_t theta0nticks; //Total deadtime (in ticks);
static uint32_t twotheta0nticks; //2 total deadtimes (in ticks);
static float roundedtheta0; //Total deadtime rounded up to the next multiple of PID cycle time
static uint16_t derivencalls; //Number of PID cycles for the temperature derivative using a rolling time window
static uint16_t eoalength; //Number of cells in the minoutput*sums and maxoutpu*tsums arrays (=ceil(0.5*ceil(theta0)/(secs/cycle))+1)
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
	twotheta0nticks=2*theta0nticks;
	derivencalls=ceil(derivetime*callspersec);
	roundedtheta0=ceil(theta0*callspersec)/callspersec;
	eoalength=ceil(0.5*ceil(theta0*callspersec))+1;
	errorvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	derivvalues=(float*)malloc((derivencalls+1)*sizeof(float));
	const float error= lasttemp - GetTargetTemp();

	for(int i=derivencalls; i>=0; --i)  errorvalues[i]=error;
	memset(derivvalues,0,(derivencalls+1)*sizeof(float));
	minoutputsums=(float*)malloc(eoalength*sizeof(float));
	minerrorsums=(float*)malloc(eoalength*sizeof(float));
	maxoutputsums=(float*)malloc(eoalength*sizeof(float));
	maxerrorsums=(float*)malloc(eoalength*sizeof(float));
	minoutputdticks=(uint32_t*)malloc(eoalength*sizeof(uint32_t));
	maxoutputdticks=(uint32_t*)malloc(eoalength*sizeof(uint32_t));
	memset(minoutputsums,0,eoalength*sizeof(float));
	memset(minerrorsums,0,eoalength*sizeof(float));
	memset(maxoutputsums,0,eoalength*sizeof(float));
	memset(maxerrorsums,0,eoalength*sizeof(float));
	memset(minoutputdticks+1,0,(eoalength-1-naeocycles)*sizeof(uint32_t));
	memset(maxoutputdticks+1,0,(eoalength-1-naeocycles)*sizeof(uint32_t));
	minoutputdticks[0]=maxoutputdticks[0]=lasttemptick;

	for(int i=1; i<=naeocycles; ++i) minoutputdticks[eoalength-i]=maxoutputdticks[eoalength-i]=theta0nticks;
	ridx=0;
	maxidx=0;
	integralforcedupdate=0;
	integralscaling=0;
	lastextoutputave=integralest=0;
	prevminerror=prevmaxerror=0;
	lastextup=false;
	lastifuup=true;
	lastexterrave=error;
	eoaready=false;
	integral=0;
}

void PIDControlDeinit()
{
	free(errorvalues);
	free(derivvalues);
	free(minoutputsums);
	free(minerrorsums);
	free(maxoutputsums);
	free(maxerrorsums);
	free(minoutputdticks);
	free(maxoutputdticks);
}

float PIDControlGetPreviousAveExtOutput(float const* const& extoutputsums, float const* const& exterrorsums, uint32_t const* const& extoutputdticks, uint32_t const& extlastdtick, int16_t const& extidx, float* errorsum)
{
	int16_t idx=extidx%eoalength;
	uint32_t nticks=extlastdtick;
	float sum=extoutputsums[idx];
	*errorsum=exterrorsums[idx];

	if(nticks<twotheta0nticks) {
		int16_t i,j=1;
		idx=(idx-1)%eoalength;

		if(idx<extidx) {

			for(i=idx; i>=0; --i) {
				sum+=extoutputsums[i];
				*errorsum+=exterrorsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= twotheta0nticks) goto found_prev_amo;
				++j;
			}

			for(i=eoalength-1; i>extidx; --i) {
				sum+=extoutputsums[i];
				*errorsum+=exterrorsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= twotheta0nticks) goto found_prev_amo;
				++j;
			}

		} else {

			for(i=idx; i>extidx; --i) {
				sum+=extoutputsums[i];
				*errorsum+=exterrorsums[i];
				nticks+=extoutputdticks[i];

				if(j>=naeocycles && nticks >= twotheta0nticks) goto found_prev_amo;
				++j;
			}
		}
	}
	found_prev_amo:
	sum/=nticks;
	*errorsum/=nticks;

	printf("Previous average output is %7.3f%% with average temperature error %6.2f C, cycle time was %8.3f\n",sum*100,*errorsum,Tick2Sec(nticks));
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

	const float dos=output*dtick;
	const float es=error*dtick;
	minoutputsums[minidx]+=dos;
	minerrorsums[minidx]+=es;
	maxoutputsums[maxidx]+=dos;
	maxerrorsums[maxidx]+=es;

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

	uint32_t extlastdtick;

	if(tderiv>=0 && lasttderiv<0 && sectderiv>0 && ((((extlastdtick=temptick-minoutputdticks[minidx])>twotheta0nticks || error < prevminerror) && temptick-maxoutputdticks[maxidx]>theta0nticks) || (lastextup^(error>0)))) {
		float newlastexterrave;
		float newlastextoutputave=PIDControlGetPreviousAveExtOutput(minoutputsums, minerrorsums, minoutputdticks, extlastdtick, minidx, &newlastexterrave);
		printf("Computed new min-based average output is %7.3f%% with average temperature error %6.2f C, cycle time was %8.3f\n",minoutputsums[minidx]/extlastdtick*100,minerrorsums[minidx]/extlastdtick,Tick2Sec(extlastdtick));
		minoutputdticks[minidx]=extlastdtick;
		minidx=(minidx+1)%eoalength;

		lastextoutputave=newlastextoutputave;
		lastexterrave=newlastexterrave;
		lastextup=(error>0);
		minoutputsums[minidx]=0;
		minerrorsums[minidx]=0;
		minoutputdticks[minidx]=temptick;
		prevminerror=error;

		if((error<0 && newlastextoutputave>integralest) || (error>0 && newlastextoutputave<integralest)) {
			/*
		if(fabs(newlastexterrave) < fabs(lastexterrave) && ((newlastexterrave>0)^(lastexterrave>0)))
			integralest=(lastextoutputave*newlastexterrave - newlastextoutputave*lastexterrave) / (newlastexterrave - lastexterrave);

		else integralest=newlastextoutputave;
			 */

			integralest=newlastextoutputave;
			eoaready=true;
			printf("Computed best integral estimate is %7.3f%%\n",integralest*100);
		}

	} else if(tderiv<=0 && lasttderiv>0 && sectderiv<0 && ((((extlastdtick=temptick-maxoutputdticks[maxidx])>twotheta0nticks || error > prevmaxerror) && temptick-minoutputdticks[minidx]>theta0nticks) || (lastextup^(error>0)))) {
		float newlastexterrave;
		float newlastextoutputave=PIDControlGetPreviousAveExtOutput(maxoutputsums, maxerrorsums, maxoutputdticks, extlastdtick, maxidx, &newlastexterrave);
		printf("Computed new max-based average output is %7.3f%% with average temperature error %6.2f C, cycle time was %8.3f\n",maxoutputsums[maxidx]/extlastdtick*100,maxerrorsums[maxidx]/extlastdtick,Tick2Sec(extlastdtick));
		maxoutputdticks[maxidx]=extlastdtick;
		maxidx=(maxidx+1)%eoalength;

		lastextoutputave=newlastextoutputave;
		lastexterrave=newlastexterrave;
		lastextup=(error>0);
		maxoutputsums[maxidx]=0;
		maxerrorsums[maxidx]=0;
		maxoutputdticks[maxidx]=temptick;
		prevmaxerror=error;

		if((error<0 && newlastextoutputave>integralest) || (error>0 && newlastextoutputave<integralest)) {
			/*
		if(fabs(newlastexterrave) < fabs(lastexterrave) && ((newlastexterrave>0)^(lastexterrave>0)))
			integralest=(lastextoutputave*newlastexterrave - newlastextoutputave*lastexterrave) / (newlastexterrave - lastexterrave);

		else integralest=newlastextoutputave;
			 */

			integralest=newlastextoutputave;
			eoaready=true;
			printf("Computed best integral estimate is %7.3f%%\n",integralest*100);
		}
	}

	//Note: A full rampthreshold discrepancy is required to exit PID or to reactivate the ramp, but 1/2 discrepancy is required to reenter.

	//If the PID algorithm is active
	if(rampstate>=kRampDisabled) {

		if(eoaready) {
			eoaready=false;

			if(integralforcedupdate) {

				if(lastifuup^(error>0)) {
					--integralforcedupdate;
					integralscaling=(integralforcedupdate?0.5*integralscaling:0);
					lastifuup=!lastifuup;
				}

				if((error>deadband && integralest<integral) || (error<-deadband && integralest>integral)) integral=integralest;

			}
		}

		if(fabs(error) > deadband) {

			//If the temperature forecast is down below the ramp threshold
			if(terrorforecast < -rampthresh || (integralforcedupdate && terrorforecast < -0.5*rampthresh)) {
				rampstate=kRampActive;
				aggressive_ramp=0;

				if(integralforcedupdate) {
					first_ramp=false;
					integral=(1+integralscaling)*integralest;

				    if(integral>maxintegralvalue) integral=maxintegralvalue;

					if(terrorforecast < -rampthresh) {
						integralforcedupdate=nifucycles;
						integralscaling=1;
						lastifuup=true;
					}

				} else {
					first_ramp=true;
					integral=maxintegralvalue;
					aggressive_ramp=2;
					integralforcedupdate=nifucycles;
					integralscaling=1;
					lastifuup=true;
				}
				printf("Integral value set to %7.3f%%\n",integral*100);
				printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*integral);

				//Else if the temperature forecast is above the ramp threshold
			} else if(terrorforecast > rampthresh || (integralforcedupdate && terrorforecast > 0.5*rampthresh)) {
				rampstate=kRampWait;
				aggressive_ramp=0;

				if(integralforcedupdate) {
					first_ramp=false;
					integral=(1-integralscaling)*integralest;

				    if(integral<minintegralvalue) integral=minintegralvalue;

					if(terrorforecast > rampthresh) {
						integralforcedupdate=nifucycles;
						integralscaling=1;
						lastifuup=false;
					}

				} else {
					first_ramp=true;
					integral=minintegralvalue;
					aggressive_ramp=1;
					integralforcedupdate=nifucycles;
					integralscaling=1;
					lastifuup=false;
				}
				printf("Integral value set to %7.3f%%\n",integral*100);
				printf("OFF: %8.3f: Temp: %6.2f C => %6.2f%% (now waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*integral);

			} else if(integralforcedupdate && ((terrorforecast < -0.5*rampthresh && integralest > integral) || (terrorforecast > 0.5*rampthresh && integralest < integral))) {
				integral=integralest;
				printf("Integral value set to %7.3f%%\n",integral*100);
			}
		}

		//Else if the PID algorithm was not active
	} else {
		bool updateallowed=false;

		//If the ramp mode is already active
		if(rampstate==kRampActive) {

			if(first_ramp && (tderiv < 0 || terrorforecast < -0.5*rampthresh) && error < 0) {
				updateallowed=true;

				//if(first_ramp || error < -0.5*rampthresh) {

					if(aggressive_ramp!=2) {
						aggressive_ramp=2;
						integral=maxintegralvalue;
						printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting full power ramp)\n",Tick2Sec(temptick),tempval,100*integral);
						integralforcedupdate=nifucycles;
						integralscaling=1;
						lastifuup=true;
					}

				/*} else {

					if(aggressive_ramp!=1) {
						aggressive_ramp=1;
						integral=2*integralest;

					    if(integral>maxintegralvalue) integral=maxintegralvalue;
						printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting aggressive ramp)\n",Tick2Sec(temptick),tempval,100*integral);
						//integralforcedupdate=nifucycles;
					    integralscaling=1;
						lastifuup=true;
					}
				}
				*/

			} else {

				if(eoaready && integralforcedupdate) {

					if(lastifuup^(error>0)) {
						--integralforcedupdate;
					    integralscaling=(integralforcedupdate?0.5*integralscaling:0);
						lastifuup=!lastifuup;
					}

					if((error>0 && integralest<integral) || (error<0 && integralest>integral)) updateallowed=true;
				}

				if(terrorforecast < 0 || (error < 0 && sectderiv < 0)) {

					if(aggressive_ramp) {

					    float newintegral=(1+integralscaling)*integralest;

					    if(newintegral < integral) integral=newintegral;

					    else if(newintegral>maxintegralvalue) integral=maxintegralvalue;

					    else integral=newintegral;
						aggressive_ramp=0;
						printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*integral);

					} else if(!first_ramp && updateallowed) {
					    integral=(1+integralscaling)*integralest;

						if(integral>maxintegralvalue) integral=maxintegralvalue;
						printf(" ON: %8.3f: Temp: %6.2f C => %6.2f%% (starting ramp)\n",Tick2Sec(temptick),tempval,100*integral);
					}

				} else {
					rampstate=kRampDisabled;

					integral=integralest;
					printf("Integral value set to %7.3f%%\n",integral*100);
					printf("TSC: %8.3f: Temp: %6.2f C => %6.2f%% (starting regular PID)\n",Tick2Sec(temptick),tempval,100*integral);
				}
			}

			//If waiting for the temperature to stop increasing at the end of the ramp mode
		} else if(rampstate==kRampWait) {

			if((tderiv > 0 || terrorforecast > 0.5*rampthresh) && error > 0) {
				updateallowed=true;

				if(!aggressive_ramp) {
					aggressive_ramp=1;
					integral=minintegralvalue;
					printf("OFF: %8.3f: Temp: %6.2f C => %6.2f%% (now aggressively waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*integral);
					//integralforcedupdate=nifucycles;
					lastifuup=false;
				}

			} else {

				if(eoaready && integralforcedupdate && (lastifuup^(error>0))) {
					--integralforcedupdate;
				    integralscaling=(integralforcedupdate?0.5*integralscaling:0);
					lastifuup=!lastifuup;

					if((error>0 && integralest<integral) || (error<0 && integralest>integral)) updateallowed=true;
				}

				if(terrorforecast > 0 || (error > 0 && sectderiv > 0)) {

					if(aggressive_ramp) {
					    float newintegral=(1-integralscaling)*integralest;

					    if(newintegral > integral) integral=newintegral;

					    else if(newintegral<minintegralvalue) integral=minintegralvalue;

					    else integral=newintegral;
						aggressive_ramp=0;
						printf("OFF: %8.3f: Temp: %6.2f C => %6.2f%% (now waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*integral);

					} else if(!first_ramp && updateallowed) {
					    integral=(1-integralscaling)*integralest;

						if(integral<minintegralvalue) integral=minintegralvalue;
						printf("OFF: %8.3f: Temp: %6.2f C => %6.2f%% (now waiting for lower temperature)\n",Tick2Sec(temptick),tempval,100*integral);
					}

				} else {
					rampstate=kRampDisabled;

					integral=integralest;
					printf("Integral value set to %7.3f%%\n",integral*100);
					printf("TSC: %8.3f: Temp: %6.2f C => %6.2f%% (starting regular PID)\n",Tick2Sec(temptick),tempval,100*integral);
				}
			}
		}

		if(updateallowed) eoaready=false;
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

	PWMSetOutput(output);

	lasttemp = tempval;
	lasttderiv = tderiv;
	lasttemptick = temptick;

	return tempval;
}
