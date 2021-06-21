/*
 * temperature.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "temperature.h"

static float radcsamples[CONFIG_TEMP_AVE_N_MEAS]; //Ring buffer for ADC0 values
static float radcsum;
Adafruit_ADS1115 ads;
static int tidx; //Ring buffer index

float radcave;
float tempave=NAN;
int tempstate=kTempUninitialized;
uint32_t temptime=0; //Using __ATOMIC_ACQUIRE/__ATOMIC_RELEASE because we want consistent temperature values, but we don't need consistency of independent atomic operations between threads

inline static float CalcTemp(const float& radc)
{
	float logval=logf(radc/((1.-radc)*TEMP_Vr_V_fact0));
	return 1/(1./TEMP_T0+logval*(TEMP_a1+logval*(TEMP_a2+logval*TEMP_a3)))-273.15;
}

void TemperatureSetup()
{
	Wire.begin(CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL, CONFIG_I2C_MASTER_FREQUENCY);
	ads.setGain(GAIN_ONE);
	//ads.setGain(GAIN_TWOTHIRDS);
	ads.begin(ADS1X15_ADDRESS, &Wire);
}

int TemperatureInit()
{
	tidx=0;
	uint16_t v1;

	do {
		//This is a ugly fix for a bug with the firsd read
		v1=ads.readADC_SingleEnded(1);

	} while(v1==INT16_MAX);
	radcsamples[CONFIG_TEMP_AVE_N_MEAS-1]=v1/(float)ads.readADC_SingleEnded(0);

	for(int i=CONFIG_TEMP_AVE_N_MEAS-2; i>=0; --i) radcsamples[i]=radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];
	radcsum=CONFIG_TEMP_AVE_N_MEAS*radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];
	radcave=radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];
	//printf("radcave=%e\n",radcave);
	tempave=CalcTemp(radcave);
	//printf("tempave is %7.3f C\n",tempave);

	if(tempave >= MIN_TEMP && tempave <= MAX_TEMP) tempstate=kTempOK;
	else tempstate=kTempInvalid;
	__atomic_store_n(&temptime, 0, __ATOMIC_RELEASE);

	if(tempstate==kTempOK) {
		printf("Temperature initialisation completed\n");
		return 0;

	} else {
		ESP_LOGE(__func_, "Temperature initialization returned an invalid temperature (%6.2f C)!",tempave);
		return -1;
	}
}

void TemperatureDeinit()
{
	tempave=NAN;
	__atomic_store_n(&temptime, 0, __ATOMIC_RELEASE);
	printf("Temperature deinitialisation completed\n");
}

void TempUpdate(const uint32_t& sample)
{
	int i;
	uint16_t v1, v0;

	//if(sample!=prevtime+1 && prevtime>0) printf("%s: Error: previous measurement at %u, current at %u\n",__func__,prevtime,sample);

	radcsum-=radcsamples[tidx];
	v1=ads.readADC_SingleEnded(1);
	v0=ads.readADC_SingleEnded(0);
	radcsamples[tidx]=v1/(float)v0;

	//if(v1==INT16_MAX || v0==INT16_MAX) printf("%s: Error: ADCs are %u/%u\n",__func__,v1,v0);
	//printf("ADCs are %u/%u\n",v1,v0);
	//printf("Values are %u and %u\n",ads.readADC_SingleEnded(0),ads.readADC_SingleEnded(1));
	radcsum+=radcsamples[tidx];

	radcsum=radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];

	for(i=CONFIG_TEMP_AVE_N_MEAS-2; i>=0; --i) radcsum+=radcsamples[i];
	radcave=radcsum/CONFIG_TEMP_AVE_N_MEAS;
	tempave=CalcTemp(radcave);
	//printf("tempave is %7.3f C at %u\n",tempave,sample);

	if(tempave < MIN_TEMP || tempave > MAX_TEMP) tempstate=kTempInvalid;
	__atomic_store_n(&temptime, sample, __ATOMIC_RELEASE);
	tidx=(tidx+1)%CONFIG_TEMP_AVE_N_MEAS;
}
