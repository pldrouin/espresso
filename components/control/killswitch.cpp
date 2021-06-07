/*
 * killswitch.cpp
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#include "killswitch.h"

void KillSwitchSetup()
{
	/* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
	 */
	gpio_pad_select_gpio((gpio_num_t)CONFIG_KILLSWITCH_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction((gpio_num_t)CONFIG_KILLSWITCH_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_pull_mode((gpio_num_t)CONFIG_KILLSWITCH_GPIO, GPIO_FLOATING);
}
