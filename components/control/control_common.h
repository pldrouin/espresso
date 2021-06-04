/*
 * control_common.h
 *
 *  Created on: May 28, 2021
 *      Author: pldrouin
 */

#ifndef COMPONENTS_CONTROL_CONTROL_COMMON_H_
#define COMPONENTS_CONTROL_CONTROL_COMMON_H_

extern EventGroupHandle_t eg;

#define TEMP_UPDATE_TASK_BIT (1)
#define PWM_TASK_BIT (1<<1)
#define CONTROLLER_UPDATE_TASK_BIT (1<<2)

#endif /* COMPONENTS_CONTROL_CONTROL_COMMON_H_ */
