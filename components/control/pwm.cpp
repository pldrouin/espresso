/*
 * pwm.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "pwm.h"

#define PWM_TIMER_DIVIDER         (8000)  //  Hardware timer clock divider
#define CONFIG_CONTROL_PWM_MAX_FREQUENCY (8) //In Hz
#define PWM_MIN_TICKS ((int64_t)(TIMER_BASE_CLK / (CONFIG_CONTROL_PWM_MAX_FREQUENCY * PWM_TIMER_DIVIDER)))

uint32_t pwm_output=0; //PWM output level, using __ATOMIC_RELAXED

static bool running=false;
static enum {kLowPower, kHighPower} state;
static bool ison;
static bool updatealarm;
static uint32_t lastoutput;
static uint64_t nexttrigger; //Time at which the output turns on. Updates to a future value when the output turns off.
static uint64_t nextalarm;
static int64_t period;

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
	PWMSetOutput(0);
}

void PWMInit()
{
	state=kLowPower;
	ison=false;
	lastoutput=0;
	nexttrigger=nextalarm=0;

	timer_config_t config = {
			.alarm_en = TIMER_ALARM_EN,
			.counter_en = TIMER_PAUSE,
			.intr_type = (timer_intr_mode_t)(TIMER_INTR_MAX-1),
			.counter_dir = TIMER_COUNT_UP,
			.auto_reload = (timer_autoreload_t)false,
			.divider = PWM_TIMER_DIVIDER
	}; // default clock source is APB
	timer_init(TIMER_GROUP_0, TIMER_1, &config);

	/* Timer's counter will initially start from value below.
		   Also, if auto_reload is set, this value will be automatically reload on alarm */
	timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);

	/* Configure the alarm value and the interrupt on alarm. */
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 0);
	timer_enable_intr(TIMER_GROUP_0, TIMER_1);

	timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, PWMCallback, NULL, 0);

	timer_start(TIMER_GROUP_0, TIMER_1);
	running=true;
	printf("PWM initialisation completed\n");
}

void PWMDeinit()
{
	if(running) {
		timer_pause(TIMER_GROUP_0, TIMER_1);
		timer_isr_callback_remove(TIMER_GROUP_0, TIMER_1);
		timer_disable_intr(TIMER_GROUP_0, TIMER_1);
		timer_deinit(TIMER_GROUP_0, TIMER_1);
	}
    gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, 0);
	printf("PWM deinitialisation completed\n");
}

bool IRAM_ATTR PWMCallback(void *args)
{
	for(;;) {
		const uint32_t output=PWMGetOutputLevel();
		nextalarm+=PWM_MIN_TICKS;

		//If the output has not changed
		if(output == lastoutput) {

			if(state == kLowPower) {

				if(output > 0) {

					if(!updatealarm) { //If the alarm is not for an update
						ison=!ison;
						gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);

						if(!ison) { //Just turned off
							nexttrigger+=period;

						    if(nexttrigger > nextalarm) updatealarm=true;
						}

					} else { // Else if the alarm is for an update (but the output has not changed)
						//(turned off)

						if(nextalarm > nexttrigger) {
							nextalarm=nexttrigger;
							updatealarm=false;
						}
					}
				}

			} else { //Else if high output

				if(output < PWM_MAX_LEVEL) {

					if(!updatealarm) { //If the alarm is not for an update
						ison=!ison;
						gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);

						if(!ison) { //Just turned off
							nexttrigger+=period;

						} else { //Just turned on

							if(period > 2*PWM_MIN_TICKS) updatealarm=true; //This condition is simpler than based on nexttrigger
						}

					} else { // Else if the alarm is for an update (but the output has not changed)
						//(turned on)

						if(nextalarm > nexttrigger+period-PWM_MIN_TICKS) {
							nextalarm=nexttrigger+period-PWM_MIN_TICKS;
							updatealarm=false;
						}
					}
				}
			}

		} else { //Else if output has changed
			int64_t newperiod;

			if(output <= PWM_HALF_LEVEL) { //If new output is low

				if(output > 0) { //If new output is greater than 0
					newperiod=(PWM_MIN_TICKS*PWM_MAX_LEVEL)/output;

					if(ison) {
						ison=false;
						gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);
						nexttrigger=nextalarm+newperiod-2*PWM_MIN_TICKS;

						if(nexttrigger > nextalarm) updatealarm=true;
						else updatealarm=false;

					} else if(state == kLowPower){ //If output is already turned off and it was low
						//Assertion: last output is not 100% because output is turned off

						if(lastoutput==0) nexttrigger+=newperiod; //Adjust nexttrigger because it was set for the previous cycle where the output was non-zero
						else nexttrigger+=newperiod-period; //Modify the next trigger time to reflect the new output value

						if(nexttrigger > nextalarm) updatealarm=true;

						else {

							if(nexttrigger+PWM_MIN_TICKS > nextalarm) nextalarm=nexttrigger;

							else { //If the output has been at 0 for long enough already for the new output value
								ison=true;
								gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);
								nexttrigger=nextalarm-PWM_MIN_TICKS;
							}
							updatealarm=false;
						}

					} else { //State was high power and output was off.
						//Using the last segment where the last output was on as the on segment for the new low power regime
						nexttrigger+=newperiod-2*PWM_MIN_TICKS;

						if(nexttrigger > nextalarm) updatealarm=true;

						else {
							nextalarm=nexttrigger;
							//If the state was highpower and the output was off, then updatealarm has to be false already
						}
					}
					period=newperiod;

				} else { //Else if new output is 0

					if(ison) {
						ison=false;
						gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);
						nexttrigger=nextalarm-2*PWM_MIN_TICKS;

					} else if(state == kLowPower) //Else if was already off and the state was low power
						nexttrigger-=period;

					else //Else if was already off and the state was high power, using the last segment where the last output was on as the on segment for the new zero power regime
						nexttrigger-=2*PWM_MIN_TICKS;
				}
				state=kLowPower;

			} else { //Else if new output is high

				if(output < PWM_MAX_LEVEL) {
					newperiod=(PWM_MIN_TICKS*PWM_MAX_LEVEL)/(PWM_MAX_LEVEL-output);

					if(!ison) {
						ison=true;
						gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);
						nexttrigger=nextalarm-PWM_MIN_TICKS;

						if(newperiod > 2*PWM_MIN_TICKS) updatealarm=true;
						else updatealarm=false;

					} else { //If output is still on

						if(nexttrigger + newperiod > nextalarm + PWM_MIN_TICKS) updatealarm=true; //If needs to turn off passed the next alarm

						else { //Else if the turn off time if not after the next scheduled alarm

							//Else if needs to turn off later than now
							if(nexttrigger + newperiod > nextalarm) nextalarm=nexttrigger + newperiod - PWM_MIN_TICKS;

							else { //If the output has been at 100% for long enough already for the new output value
								ison=false;
								gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);
								nexttrigger=nextalarm;
							}
							updatealarm=false;
						}
					}
					period=newperiod;

				} else { //Else if new output is 100%

					if(!ison) {
						ison=true;
						gpio_set_level((gpio_num_t)CONFIG_PWM_GPIO, ison);
						nexttrigger=nextalarm-PWM_MIN_TICKS;
					}
				}
				state=kHighPower;
			}
			lastoutput=output;
		}

		uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_1);

		if(nextalarm > timer_counter_value) {
			timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_1, nextalarm);
			break;
		}
	}
	return pdFALSE; // return whether we need to yield at the end of ISR
}
