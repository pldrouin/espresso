#include <string.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"

#include "freertos/FreeRTOS.h"

#include "driver/timer.h"
#include "driver/gpio.h"

#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#include "cmd_system.h"

#include "Arduino.h"

#include "controller.h"
#include "pid_control.h"

#include "sdkconfig.h"

#ifdef CONFIG_ESP_CONSOLE_USB_CDC
#error This example is incompatible with USB CDC console. Please try "console_usb" example instead.
#endif // CONFIG_ESP_CONSOLE_USB_CDC

#define PROMPT_STR CONFIG_IDF_TARGET

void config_and_start()
{
	SetTargetTemp(127.75);
	ControllerSetAlgorithm(PIDControl, PIDControlInit);
	PIDSetParams(0.315, 3.5, 0.5, 5, 0.5);
	PIDSetDFilter(1);
	PIDSetDeriveTime(1);
	PIDSetRampThreshold(0.25);
	PIDSetDeadband(0.03);
	PIDSetLimitParams(0.20, 0.05);
	ControllerInit();
}

void initialize_console(void)
{
	/* Drain stdout before reconfiguring it */
	fflush(stdout);
	fsync(fileno(stdout));

	/* Disable buffering on stdin */
	setvbuf(stdin, NULL, _IONBF, 0);

	/* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
	esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
	/* Move the caret to the beginning of the next line on '\n' */
	esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);

	/* Configure UART. Note that REF_TICK is used so that the baud rate remains
	 * correct while APB frequency is changing in light sleep mode.
	 */
	const uart_config_t uart_config = {
			.baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE,
			.data_bits = UART_DATA_8_BITS,
			.parity = UART_PARITY_DISABLE,
			.stop_bits = UART_STOP_BITS_1,
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
			.source_clk = UART_SCLK_REF_TICK,
#else
			.source_clk = UART_SCLK_XTAL,
#endif
	};
	/* Install UART driver for interrupt-driven reads and writes */
	ESP_ERROR_CHECK( uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
			256, 0, 0, NULL, 0) );
	ESP_ERROR_CHECK( uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config) );

	/* Tell VFS to use UART driver */
	esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

	/* Initialize the console */
	esp_console_config_t console_config = {
			.max_cmdline_length = 256,
			.max_cmdline_args = 8,
#if CONFIG_LOG_COLORS
			.hint_color = atoi(LOG_COLOR_CYAN)
#endif
	};
	ESP_ERROR_CHECK( esp_console_init(&console_config) );

	/* Configure linenoise line completion library */
	/* Enable multiline editing. If not set, long commands will scroll within
	 * single line.
	 */
	linenoiseSetMultiLine(1);

	/* Tell linenoise where to get command completions and hints */
	linenoiseSetCompletionCallback(&esp_console_get_completion);
	linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

	/* Set command history size */
	linenoiseHistorySetMaxLen(100);

	/* Don't return empty lines */
	linenoiseAllowEmpty(false);

#if CONFIG_STORE_HISTORY
	/* Load command history from filesystem */
	linenoiseHistoryLoad(HISTORY_PATH);
#endif
}

extern "C" void app_main()
{
	initArduino();

	eg=xEventGroupCreate();
	ControllerSetup();
	config_and_start();

	initialize_console();

	/* Register commands */
	register_system();
	esp_console_register_help_command();

	/* Prompt to be printed before each line.
	 * This can be customized, made dynamic, etc.
	 */
	const char* prompt = LOG_COLOR_I PROMPT_STR "> " LOG_RESET_COLOR;

	/* Figure out if the terminal supports escape sequences */
	int probe_status = linenoiseProbe();
	if (probe_status) { /* zero indicates success */
		printf("\n"
				"Your terminal application does not support escape sequences.\n"
				"Line editing and history features are disabled.\n");
		linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
		/* Since the terminal doesn't support escape sequences,
		 * don't use color codes in the prompt.
		 */
		prompt = PROMPT_STR "> ";
#endif //CONFIG_LOG_COLORS
	}

	while(1) {
		/*
		// Blink off (output low)
		printf("Turning off the LED at %22.15e ms\n",millis());
		gpio_set_level(BLINK_GPIO, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		// Blink on (output high)
		printf("Turning on the LED\n");
		gpio_set_level(BLINK_GPIO, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		 */
	    //printf("ADC voltage is %.15f\n",ads.readADC_SingleEnded(0)*0.000125f);

		/* Get a line using linenoise.
		 * The line is returned when ENTER is pressed.
		 */
		char* line = linenoise(prompt);
		if (line == NULL) { /* Break on EOF or error */
			continue;
		}
		/* Add the command to the history if not empty*/
		if (strlen(line) > 0) {
			linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
			/* Save command history to filesystem */
			linenoiseHistorySave(HISTORY_PATH);
#endif
		}

		/* Try to run the command */
		int ret;
		esp_err_t err = esp_console_run(line, &ret);
		if (err == ESP_ERR_NOT_FOUND) {
			printf("Unrecognized command\n");
		} else if (err == ESP_ERR_INVALID_ARG) {
			// command was empty
		} else if (err == ESP_OK && ret != ESP_OK) {
			printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
		} else if (err != ESP_OK) {
			printf("Internal error: %s\n", esp_err_to_name(err));
		}
		/* linenoise allocates line buffer on the heap, so need to free it */
		linenoiseFree(line);

	}

	ESP_LOGE("", "Error or end-of-input, terminating console");
	esp_console_deinit();
}
