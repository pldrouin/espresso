/*
 * controller.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "controller.h"

static uint32_t samp_counter=0;
static uint64_t timer_tick;
static int keepgoing=0;
static float (*ControllerAlgorithm)() = NULL;
static void (*ControllerAlgorithmInit)() = NULL;
static void (*ControllerAlgorithmDeinit)() = NULL;

static float powersum;
static float tempsum;
static float nmeas;
static bool showstats=false; //Using __ATOMIC_ACQUIRE/__ATOMIC_RELEASE because we want consistent measurements, but we don't need consistency of independent atomic operations between threads

float target_temp=0;

int ControllerSetup()
{
	PWMSetup();
	KillSwitchSetup();
	TemperatureSetup();
	return 0;
}

int ControllerInit()
{
	if(keepgoing==0) {

		if(TemperatureInit()) return -1;
		KillSwitchSetNoKill(true);
		PWMInit();

		if(ControllerAlgorithmInit) ControllerAlgorithmInit();

		keepgoing=1;
		xTaskCreate(ControllerUpdate, "Controller Update", 4096, NULL, configMAX_PRIORITIES-1, NULL);

		timer_config_t config = {
				.alarm_en = TIMER_ALARM_EN,
				.counter_en = TIMER_PAUSE,
				.intr_type = (timer_intr_mode_t)(TIMER_INTR_MAX-1),
				.counter_dir = TIMER_COUNT_UP,
				.auto_reload = (timer_autoreload_t)false,
				.divider = TIMER_DIVIDER
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
 	    printf("Controller initialisation completed\n");

	} else {
     	ESP_LOGE(__func_, "Controller is already initialized. Ignoring");
	}
	return 0;
}

void ControllerDeinit()
{
	if(keepgoing==1) {
		PWMDeinit();
		PWMSetOutput(0);
		keepgoing=0;

		while(!keepgoing) vTaskDelay(1 / portTICK_PERIOD_MS);
		keepgoing=0;

		if(ControllerAlgorithmDeinit) ControllerAlgorithmDeinit();
		TemperatureDeinit();
		timer_pause(TIMER_GROUP_0, TIMER_0);
		timer_isr_callback_remove(TIMER_GROUP_0, TIMER_0);
		timer_disable_intr(TIMER_GROUP_0, TIMER_0);
		timer_deinit(TIMER_GROUP_0, TIMER_0);
		samp_counter=0;
 	    printf("Controller deinitialisation completed\n");

	} else {
     	ESP_LOGE(__func_, "Controller is not initialized. Ignoring");
	}
}

int ControllerSetAlgorithm(float (*algo)(), void (*init)(), void (*deinit)())
{
	if(keepgoing) {
     	ESP_LOGE(__func_, "Cannot change control algorithm when the controller is active. Ignoring");
		return -1;
	}
	ControllerAlgorithm=algo;
	ControllerAlgorithmInit=init;
	ControllerAlgorithmDeinit=deinit;
	return 0;
}

bool IRAM_ATTR ControllerCallback(void *args)
{
	uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
    BaseType_t high_task_awoken = pdFALSE;

    xEventGroupSetBitsFromISR(eg, CONTROLLER_UPDATE_TASK_BIT, &high_task_awoken);

    timer_counter_value += ALARM_N_TICKS;
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, timer_counter_value);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

void ControllerUpdate(void* parameter)
{
	while(keepgoing==1) {
		xEventGroupWaitBits(eg, CONTROLLER_UPDATE_TASK_BIT, pdTRUE, pdTRUE, portMAX_DELAY) ;
	    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_tick);

		if(samp_counter%CONFIG_TEMP_N_SAMPLES == 0) {
			TempUpdate(timer_tick);

			if(TempState() != kTempOK) {
				KillSwitchSetNoKill(false);
				ControllerDeinit();
				ESP_LOGE(__func_, "Controller has ended");
				return;
			}
		}

		if(ControllerAlgorithm) {

			if(samp_counter%CONFIG_CONTROLLER_SAMPLING_PERIOD_N_SAMPLES == 0) {
				float tempval=ControllerAlgorithm();
				bool stats;
				__atomic_load(&showstats, &stats, __ATOMIC_ACQUIRE);

				if(stats) {
					powersum+=PWMGetOutput();
					tempsum+=tempval;
					++nmeas;
					printf("\t(average temp %7.3f C, power %7.3f%%)\n",tempsum/nmeas,100*powersum/nmeas);
				}
			}
		}
		++samp_counter;
	}
	keepgoing=-1;
	vTaskDelete(NULL);
}

void StartStats()
{
	powersum=0;
	tempsum=0;
	nmeas=0;
	__atomic_store_n(&showstats, true, __ATOMIC_RELEASE);
}

void StopStats()
{
	__atomic_store_n(&showstats, false, __ATOMIC_RELEASE);
}
