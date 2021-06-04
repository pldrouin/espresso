/*
 * cmd_system.cpp
 *
 *  Created on: May 27, 2021
 *      Author: pldrouin
 */

#include "cmd_system.h"
#include "killswitch.h"
#include "temperature.h"
#include "pwm.h"

static struct {
	struct arg_dbl *output;
	struct arg_end *end;
} pwm_output_args;


static int cmd_kill(int argc, char **argv)
{
	KillSwitchSetNoKill(false);
	return 0;
}

static void register_kill(void)
{
    const esp_console_cmd_t cmd = {
        .command = "kill",
        .help = "Kill switch enable",
        .hint = NULL,
        .func = &cmd_kill,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int cmd_nokill(int argc, char **argv)
{
	KillSwitchSetNoKill(true);
	return 0;
}

static void register_nokill(void)
{
    const esp_console_cmd_t cmd = {
        .command = "nokill",
        .help = "Kill switch disable",
        .hint = NULL,
        .func = &cmd_nokill,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'pwm_output' command */
static int cmd_pwm_set_output(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &pwm_output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, pwm_output_args.end, argv[0]);
		return 1;
	}

	if(pwm_output_args.output->count) {

		if(pwm_output_args.output->dval[0]<0 || pwm_output_args.output->dval[0]>1) {
			ESP_LOGE("", "PWM output value must be in the inverval [0,1]");
			return 1;
		}
		PWMSetOutput(pwm_output_args.output->dval[0]);
	}
    return 0;
}

static void register_pwm_set_output(void)
{
    pwm_output_args.output = arg_dbl1(NULL, NULL, "<level>", "PWM output level");
    pwm_output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "pwm_set_output",
        .help = "Set PWM output level",
        .hint = NULL,
        .func = &cmd_pwm_set_output,
		.argtable = &pwm_output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int cmd_status(int argc, char **argv)
{
	uint64_t temptime, newtemptime=TempTime();
	float radcval, tempval;

	do {
		temptime=newtemptime;
		radcval=TempGetRelativeADCAve();
		tempval=TempGetTempAve();
		newtemptime=TempTime();

	} while(newtemptime!=temptime);

	printf("Relative ADC average is %.6f\n",radcval);
	printf("Temperature average is %.4f C\n",tempval);
	printf("Temperature measurement time is %" PRIu64 "\n",temptime);
	printf("PWM Output is %.4f\n",PWMGetOutput());
	return 0;
}

static void register_status(void)
{
    const esp_console_cmd_t cmd = {
        .command = "status",
        .help = "System status",
        .hint = NULL,
        .func = &cmd_status,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void register_system(void)
{
	register_kill();
	register_nokill();
	register_status();
	register_pwm_set_output();
}
