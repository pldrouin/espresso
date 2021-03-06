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
#include "controller.h"
#include "simple_control.h"
#include "pid_control.h"
#include "pid_atune.h"

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

static int cmd_controller_init(int argc, char **argv)
{
	ControllerInit();
	return 0;
}

static void register_controller_init(void)
{
    const esp_console_cmd_t cmd = {
        .command = "controller_init",
        .help = "Initialize controller",
        .hint = NULL,
        .func = &cmd_controller_init,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int cmd_controller_deinit(int argc, char **argv)
{
	ControllerDeinit();
	return 0;
}

static void register_controller_deinit(void)
{
    const esp_console_cmd_t cmd = {
        .command = "controller_deinit",
        .help = "Deinitialize controller",
        .hint = NULL,
        .func = &cmd_controller_deinit,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
	struct arg_dbl *value;
	struct arg_end *end;
} target_temp_args;

/* 'target_temp' command */
static int cmd_set_target_temp(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &target_temp_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, target_temp_args.end, argv[0]);
		return 1;
	}

	if(target_temp_args.value->count) {

		if(target_temp_args.value->dval[0]>=MAX_TEMP) {
			ESP_LOGE("", "Taget temperature must be smaller than %7.3f C\n",MAX_TEMP);
			return 1;
		}
		SetTargetTemp(target_temp_args.value->dval[0]);
	}
    return 0;
}

static void register_set_target_temp(void)
{
    target_temp_args.value = arg_dbl1(NULL, NULL, "<value>", "Target temperature");
    target_temp_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_target_temp",
        .help = "Set target temperature",
        .hint = NULL,
        .func = &cmd_set_target_temp,
		.argtable = &target_temp_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}
static struct {
	struct arg_lit *simple;
	struct arg_lit *pid;
	struct arg_lit *pidatune;
	struct arg_end *end;
} algo_args;

/* 'algo' command */
static int cmd_set_algo(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &algo_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, algo_args.end, argv[0]);
		return 1;
	}

	if(algo_args.simple->count) ControllerSetAlgorithm(SimpleControl, NULL);
	else if(algo_args.pid->count) ControllerSetAlgorithm(PIDControl, PIDControlInit, PIDControlDeinit);
	else if(algo_args.pidatune->count) ControllerSetAlgorithm(PIDATune, PIDATuneInit, PIDATuneDeinit);
	else ControllerSetAlgorithm();
    return 0;
}

static void register_set_algo(void)
{
    algo_args.simple = arg_lit0("s", "simple", "Simple control algorithm");
    algo_args.pid = arg_lit0("p", "pid", "PID control algorithm");
    algo_args.pidatune = arg_lit0("t", "pid_atune", "PID autotune algorithm");
    algo_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_algo",
        .help = "Set control algoritum",
        .hint = NULL,
        .func = &cmd_set_algo,
		.argtable = &algo_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
	struct arg_dbl *output;
	struct arg_end *end;
} output_args;

/* 'pwm_output' command */
static int cmd_pwm_set_output(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {

		if(output_args.output->dval[0]<0 || output_args.output->dval[0]>1) {
			ESP_LOGE("", "PWM output value must be in the inverval [0,1]");
			return 1;
		}
		PWMSetOutput(output_args.output->dval[0]);
	}
    return 0;
}

static void register_pwm_set_output(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<level>", "PWM output level");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "pwm_set_output",
        .help = "Set PWM output level",
        .hint = NULL,
        .func = &cmd_pwm_set_output,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_init_output' command */
static int cmd_set_init_output(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {

		if(output_args.output->dval[0]<0 || output_args.output->dval[0]>1) {
			ESP_LOGE("", "Output value must be in the inverval [0,1]");
			return 1;
		}
		SetInitOutput(output_args.output->dval[0]);
	}
    return 0;
}

static void register_set_init_output(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<level>", "Initial output level");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_init_output",
        .help = "Set initial output level",
        .hint = NULL,
        .func = &cmd_set_init_output,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
	struct arg_dbl *Ki;
	struct arg_dbl *Theta0;
	struct arg_dbl *Kcfact;
	struct arg_dbl *Tifact;
	struct arg_dbl *Tdfact;
	struct arg_end *end;
} pid_args;

/* 'set_pid_params' command */
static int cmd_set_pid_params(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &pid_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, pid_args.end, argv[0]);
		return 1;
	}

	if(pid_args.Ki->count && pid_args.Theta0->count && pid_args.Kcfact->count && pid_args.Tifact->count && pid_args.Tdfact->count) {

		PIDSetParams(pid_args.Ki->dval[0],pid_args.Theta0->dval[0],pid_args.Kcfact->dval[0],pid_args.Tifact->dval[0],pid_args.Tdfact->dval[0]);
	}
    return 0;
}

static void register_set_pid_params(void)
{
    pid_args.Ki = arg_dbl1(NULL, NULL, "<Ki>", "Open loop integrating gain. Ramp rate of the PID input in %/s divided by a fixed change in the PID output in % for an integrating process.");
    pid_args.Theta0 = arg_dbl1(NULL, NULL, "<Theta0>", "Total loop deadtime");
    pid_args.Kcfact = arg_dbl1(NULL, NULL, "<Kcfact>", "Factor when computing the PID gain, expressed as Kcfact/(Ki*Theta0). A value of 0.5 can be good");
    pid_args.Tifact = arg_dbl1(NULL, NULL, "<Tifact>", "Factor when computing the PID reset time, expressed as Tifact*Theta0. A value of 4 can be good");
    pid_args.Tdfact = arg_dbl1(NULL, NULL, "<Tdfact>", "Factor when computing the PID derivative time, expressed as Tdfact*Tehta0. A value of 0.5 can be good");
    pid_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_pid_params",
        .help = "Set PID parameter values",
        .hint = NULL,
        .func = &cmd_set_pid_params,
		.argtable = &pid_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
	struct arg_dbl *maxintegralval;
	struct arg_dbl *minintegralval;
	struct arg_end *end;
} pid_limit_args;

/* 'set_pid_limit_params' command */
static int cmd_set_pid_limit_params(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &pid_limit_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, pid_limit_args.end, argv[0]);
		return 1;
	}

	if(pid_limit_args.maxintegralval->count && pid_limit_args.minintegralval->count) {

		PIDSetLimitParams(pid_limit_args.maxintegralval->dval[0],pid_limit_args.minintegralval->dval[0]);
	}
    return 0;
}

static void register_set_pid_limit_params(void)
{
    pid_limit_args.maxintegralval = arg_dbl1(NULL, NULL, "<maxintegralval>", "Maximum integral value");
    pid_limit_args.minintegralval = arg_dbl1(NULL, NULL, "<minintegralval>", "Minimum integral value");
    pid_limit_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_pid_limit_params",
        .help = "Set PID limit parameter values",
        .hint = NULL,
        .func = &cmd_set_pid_limit_params,
		.argtable = &pid_limit_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_pid_derive_time' command */
static int cmd_set_pid_derive_time(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {

		if(output_args.output->dval[0]<0) {
			ESP_LOGE("", "PID derivative time value must be non-negative");
			return 1;
		}
		PIDSetDeriveTime(output_args.output->dval[0]);
	}
    return 0;
}

static void register_set_pid_derive_time(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<PID derivative time>", "PID derivative time");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_pid_derive_time",
        .help = "Set PID derivative time",
        .hint = NULL,
        .func = &cmd_set_pid_derive_time,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_pid_deadband' command */
static int cmd_set_pid_ramp_threshold(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {

		if(output_args.output->dval[0]<0) {
			ESP_LOGE("", "ramp threshold value must be non-negative");
			return 1;
		}
		PIDSetRampThreshold(output_args.output->dval[0]);
	}
    return 0;
}

static void register_set_pid_ramp_threshold(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<threshold>", "Ramp threshold value in C");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_pid_ramp_threshold",
        .help = "Set PID ramp threshold value where the ramp algorithm is used instead of PID",
        .hint = NULL,
        .func = &cmd_set_pid_ramp_threshold,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_pid_deadband' command */
static int cmd_set_pid_deadband(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {

		if(output_args.output->dval[0]<0) {
			ESP_LOGE("", "deadband value must be non-negative");
			return 1;
		}
		PIDSetDeadband(output_args.output->dval[0]);
	}
    return 0;
}

static void register_set_pid_deadband(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<deadband>", "Deadband value in C");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_pid_deadband",
        .help = "Set PID deadband value where integral updates are disabled",
        .hint = NULL,
        .func = &cmd_set_pid_deadband,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_pid_integral' command */
static int cmd_set_pid_integral(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {
		PIDSetIntegral(output_args.output->dval[0]);
	}
    return 0;
}

static void register_set_pid_integral(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<level>", "PID output sum");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_pid_integral",
        .help = "Set PID integral",
        .hint = NULL,
        .func = &cmd_set_pid_integral,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static struct {
	struct arg_int *i;
	struct arg_end *end;
} int_args;

/* 'set_pid_num_integral_estimate_cycles' command */
static int cmd_set_pid_num_integral_estimate_cycles(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &int_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, int_args.end, argv[0]);
		return 1;
	}

	if(int_args.i->count) {

		if(int_args.i->ival[0]<1) {
			ESP_LOGE("", "The number of integral estimate cycles must be larger than 0");
			return 1;
		}

		if(int_args.i->ival[0]>255) {
			ESP_LOGE("", "The number of integral estimate cycles must smaller than 256");
			return 1;
		}
		PIDSetNIntegralEstimateCycles(int_args.i->ival[0]);
	}
    return 0;
}

static void register_set_pid_num_integral_estimate_cycles(void)
{
    int_args.i = arg_int1(NULL, NULL, "<level>", "PID number of integral estimate cycles");
    int_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_pid_num_integral_estimate_cycles",
        .help = "Set PID the number of integral estimate cycles",
        .hint = NULL,
        .func = &cmd_set_pid_num_integral_estimate_cycles,
		.argtable = &int_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_temp_noise' command */
static int cmd_set_temp_noise(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {

		if(output_args.output->dval[0]<0) {
			ESP_LOGE("", "Noise value must be non-negative");
			return 1;
		}
		SetTempNoise(output_args.output->dval[0]);
	}
    return 0;
}

static void register_set_temp_noise(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<level>", "Initial noise level");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_temp_noise",
        .help = "Set temperature noise level",
        .hint = NULL,
        .func = &cmd_set_temp_noise,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_output_step' command */
static int cmd_set_output_step(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &output_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, output_args.end, argv[0]);
		return 1;
	}

	if(output_args.output->count) {

		if(output_args.output->dval[0]<0) {
			ESP_LOGE("", "Output step must be non-negative");
			return 1;
		}
		PIDATuneSetOutputStep(output_args.output->dval[0]);
	}
    return 0;
}

static void register_set_output_step(void)
{
    output_args.output = arg_dbl1(NULL, NULL, "<step>", "Output step value");
    output_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_output_step",
        .help = "Set output step for autotune",
        .hint = NULL,
        .func = &cmd_set_output_step,
		.argtable = &output_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

/* 'set_n_lookback_samples' command */
static int cmd_set_n_lookback_samples(int argc, char **argv)
{
	int nerrors = arg_parse(argc, argv, (void **) &int_args);
	if (nerrors != 0) {
		arg_print_errors(stderr, int_args.end, argv[0]);
		return 1;
	}

	if(int_args.i->count) {

		if(int_args.i->ival[0]<=0) {
			ESP_LOGE("", "Number of lookback samples must be greater than 0");
			return 1;
		}
		PIDATuneSetNLookbackSamples(int_args.i->ival[0]);
	}
    return 0;
}

static void register_set_n_lookback_samples(void)
{
    int_args.i = arg_int1(NULL, NULL, "<n>", "Number of lookback samples");
    int_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {
        .command = "set_n_lookback_samples",
        .help = "Set the number of lookback samples for autotune",
        .hint = NULL,
        .func = &cmd_set_n_lookback_samples,
		.argtable = &int_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int cmd_start_tuning(int argc, char **argv)
{
	PIDATuneStart();
	return 0;
}

static void register_start_tuning(void)
{
    const esp_console_cmd_t cmd = {
        .command = "start_tuning",
        .help = "Start PID autotune",
        .hint = NULL,
        .func = &cmd_start_tuning,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int cmd_stop_tuning(int argc, char **argv)
{
	PIDATuneStop();
	return 0;
}

static void register_stop_tuning(void)
{
    const esp_console_cmd_t cmd = {
        .command = "stop_tuning",
        .help = "Stop PID autotune",
        .hint = NULL,
        .func = &cmd_stop_tuning,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int cmd_status(int argc, char **argv)
{
	uint32_t temptime=TempTick(), newtemptime;
	float radcval, tempval;

	for(;;) {
		radcval=TempGetRelativeADCAve();
		tempval=TempGetTempAve();
		newtemptime=TempTick();

		if(newtemptime==temptime) break;
		temptime=newtemptime;
	}

	printf("Relative ADC average is %.6f\n",radcval);
	printf("Temperature average is %.4f C\n",tempval);
	printf("Temperature measurement time is %" PRIu32 "\n",temptime);
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

static int cmd_start_stats(int argc, char **argv)
{
	StartStats();
	return 0;
}

static void register_start_stats(void)
{
    const esp_console_cmd_t cmd = {
        .command = "start_stats",
        .help = "Start statistics",
        .hint = NULL,
        .func = &cmd_start_stats,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static int cmd_stop_stats(int argc, char **argv)
{
	StopStats();
	return 0;
}

static void register_stop_stats(void)
{
    const esp_console_cmd_t cmd = {
        .command = "stop_stats",
        .help = "Stop statistics",
        .hint = NULL,
        .func = &cmd_stop_stats,
		.argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

void register_system(void)
{
	register_kill();
	register_nokill();
	register_controller_init();
	register_controller_deinit();
	register_status();
	register_start_stats();
	register_stop_stats();
	register_set_target_temp();
	register_set_algo();
	register_pwm_set_output();
	register_set_init_output();
	register_set_pid_params();
	register_set_pid_limit_params();
	register_set_pid_derive_time();
	register_set_pid_ramp_threshold();
	register_set_pid_deadband();
	register_set_pid_integral();
	register_set_pid_num_integral_estimate_cycles();
	register_set_temp_noise();
	register_set_output_step();
	register_set_n_lookback_samples();
	register_start_tuning();
	register_stop_tuning();
}
