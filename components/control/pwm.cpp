/*
 * pwm.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "pwm.h"

static TaskHandle_t PWMHandle = NULL;
static int pwmidx;

void PWMInit()
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

	pwmidx=0;
	xTaskCreate(PWMUpdate, "PWM Update", 2048, NULL, configMAX_PRIORITIES-1, &PWMHandle);
}

void PWMDeinit()
{
	if(PWMHandle) {
		vTaskDelete(PWMHandle);
		PWMHandle=NULL;
	}
    gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, 0);
}

void PWMUpdate(void* parameter)
{
	float output;
	float fval;
	int ival;

	for(;;) {
		xEventGroupWaitBits(eg, PWM_TASK_BIT, pdTRUE, pdTRUE, portMAX_DELAY) ;
		output=pwm_output;
		fval=output*(pwmidx+1);
		ival=(int)fval;

		if(ival == 0 || (int)(fval-output)==ival) gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, 0);
		else gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, 1);
		pwmidx=(1+pwmidx)%CONFIG_CONTROL_PWM_CLOCK_PERIOD_N_TICKS;
	}
}

