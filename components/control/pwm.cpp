/*
 * pwm.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "pwm.h"

static int keepgoing=0;
static int pwmidx;

void PWMSetup()
{
	/* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
	 */
	gpio_pad_select_gpio((gpio_num_t)CONFIG_PWM_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction((gpio_num_t)CONFIG_PWM_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode((gpio_num_t)CONFIG_PWM_GPIO, GPIO_FLOATING);
}

void PWMInit()
{
	pwmidx=0;
	keepgoing=1;
	xTaskCreate(PWMUpdate, "PWM Update", 2048, NULL, configMAX_PRIORITIES-1, NULL);
}

void PWMDeinit()
{
	if(keepgoing==1) {
		keepgoing=0;

		while(!keepgoing) vTaskDelay(1 / portTICK_PERIOD_MS);
		keepgoing=0;
	}
    gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, 0);
}

void PWMUpdate(void* parameter)
{
	float output;
	float fval;
	int ival;

	while(keepgoing==1) {
		xEventGroupWaitBits(eg, PWM_TASK_BIT, pdTRUE, pdTRUE, portMAX_DELAY) ;
		output=pwm_output;
		fval=output*(pwmidx+1);
		ival=(int)fval;

		if(ival == 0 || (int)(fval-output)==ival) gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, 0);
		else gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, 1);
		pwmidx=(1+pwmidx)%CONFIG_CONTROL_PWM_CLOCK_PERIOD_N_TICKS;
	}
	keepgoing=-1;
	vTaskDelete(NULL);
}

