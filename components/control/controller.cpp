/*
 * controller.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "controller.h"

#define TIMER_DIVIDER         (10000)  //  Hardware timer clock divider
#define ALARM_N_TICKS		  (TIMER_BASE_CLK / TIMER_DIVIDER / CONFIG_CONTROL_SAMPLING_FREQ)

static TaskHandle_t ControllerHandle = NULL;
static void (*ControllerAlgorithm)() = NULL;

void ControllerInit()
{
	KillSwitchInit();
	TemperatureInit();

    if(TempGetTempAve() != kTempOK) return;
	KillSwitchSetNoKill(true);
	PWMInit();

	xTaskCreate(ControllerUpdate, "Controller Update", 2048, NULL, 5, &ControllerHandle);

	timer_config_t config = {
			.alarm_en = TIMER_ALARM_EN,
			.counter_en = TIMER_PAUSE,
			.counter_dir = TIMER_COUNT_UP,
			.auto_reload = (timer_autoreload_t)false,
			.divider = TIMER_DIVIDER,
	}; // default clock source is APB
	timer_init(TIMER_GROUP_0, TIMER_0, &config);

	/* Timer's counter will initially start from value below.
		   Also, if auto_reload is set, this value will be automatically reload on alarm */
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ALARM_N_TICKS);
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);

	timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, ControllerCallback, NULL, 0);

	timer_start(TIMER_GROUP_0, TIMER_0);
}

void ControllerDeinit()
{
	timer_disable_intr(TIMER_GROUP_0, TIMER_0);
	timer_deinit(TIMER_GROUP_0, TIMER_0);
	PWMDeinit();
	TemperatureDeinit();

	if(ControllerHandle) {
		vTaskDelete(ControllerHandle);
		ControllerHandle=NULL;
	}
	samp_counter=0;
	printf("Controller has ended\n");
}

void ControllerSetAlgorithm(void (*algo)())
{
	ControllerAlgorithm=algo;
}

bool IRAM_ATTR ControllerCallback(void *args)
{
	uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
    BaseType_t high_task_awoken = pdFALSE;

    __atomic_add_fetch(&samp_counter, 1, __ATOMIC_RELAXED);

    uint64_t sample=timer_counter_value/ALARM_N_TICKS;

    if(sample%(CONFIG_TEMP_N_SAMPLES) == 0) xEventGroupSetBitsFromISR(eg, TEMP_UPDATE_TASK_BIT, &high_task_awoken);

    if(sample%(CONFIG_CONTROL_PWM_CLOCK_TICK_N_SAMPLES) == 0) xEventGroupSetBitsFromISR(eg, PWM_TASK_BIT, &high_task_awoken);

    if(sample%(CONFIG_CONTROLLER_SAMPLING_PERIOD_N_SAMPLES) == 0) xEventGroupSetBitsFromISR(eg, CONTROLLER_UPDATE_TASK_BIT, &high_task_awoken);

    timer_counter_value += ALARM_N_TICKS;
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, timer_counter_value);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void ControllerUpdate(void* parameter)
{
	if(TempState() != kTempOK) {
  	    KillSwitchSetNoKill(false);
		ControllerDeinit();
		return;
	}
	ControllerAlgorithm();
}
