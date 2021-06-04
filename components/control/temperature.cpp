/*
 * temperature.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "temperature.h"

static TaskHandle_t TemperatureHandle = NULL;

static float radcsamples[CONFIG_TEMP_AVE_N_MEAS]; //Ring buffer for ADC0 values
static float radcsum;
Adafruit_ADS1115 ads;
static int tidx; //Ring buffer index

volatile float radcave;
volatile float tempave=NAN;
volatile int tempstate=kTempUninitialized;

#define MIN_TEMP (10)
#define MAX_TEMP (127.41)

inline static float CalcTemp(const volatile float& radcave)
{
	double logf=log(radcave/((1.-radcave)*TEMP_Vr_V_fact0));
	return 1/(1./TEMP_T0+logf*(TEMP_a1+logf*(TEMP_a2+logf*TEMP_a3)))-273.15;
}

void TemperatureInit()
{
	tidx=0;

	Wire.begin(CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL, CONFIG_I2C_MASTER_FREQUENCY);
	ads.setGain(GAIN_ONE);
	//ads.setGain(GAIN_TWOTHIRDS);
	ads.begin(ADS1X15_ADDRESS, &Wire);
	radcsamples[CONFIG_TEMP_AVE_N_MEAS-1]=ads.readADC_SingleEnded(1)/(float)ads.readADC_SingleEnded(0);

	for(int i=CONFIG_TEMP_AVE_N_MEAS-2; i>=0; --i) radcsamples[i]=radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];
	radcsum=CONFIG_TEMP_AVE_N_MEAS*radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];
	radcave=radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];
	tempave=CalcTemp(radcave);

	if(tempave >= MIN_TEMP && tempave <= MAX_TEMP) tempstate=kTempOK;
	else tempstate=kTempInvalid;

	xTaskCreate(TempUpdate, "Temperature Update", 2048, NULL, configMAX_PRIORITIES-1, &TemperatureHandle);
}

void TemperatureDeinit()
{
	if(TemperatureHandle) {
		vTaskDelete(TemperatureHandle);
		TemperatureHandle=NULL;
	}
	tempave=NAN;
}

void TempUpdate(void* parameter)
{
	int i;

	for(;;) {
		xEventGroupWaitBits(eg, TEMP_UPDATE_TASK_BIT, pdTRUE, pdTRUE, portMAX_DELAY) ;

		radcsum-=radcsamples[tidx];
		radcsamples[tidx]=ads.readADC_SingleEnded(1)/(float)ads.readADC_SingleEnded(0);
		//printf("Values are %u and %u\n",ads.readADC_SingleEnded(0),ads.readADC_SingleEnded(1));
		radcsum+=radcsamples[tidx];

		radcsum=radcsamples[CONFIG_TEMP_AVE_N_MEAS-1];

		for(i=CONFIG_TEMP_AVE_N_MEAS-2; i>=0; --i) radcsum+=radcsamples[i];
		radcave=radcsum/CONFIG_TEMP_AVE_N_MEAS;
    	tempave=CalcTemp(radcave);

    	if(tempave < MIN_TEMP || tempave > MAX_TEMP) tempstate=kTempInvalid;
		tidx=(tidx+1)%CONFIG_TEMP_AVE_N_MEAS;
	}
}

