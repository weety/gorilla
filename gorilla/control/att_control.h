/*
 * File      : att_control.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-02     weety    first version.
 */

#ifndef __ATT_CONTROL_H__
#define __ATT_CONTROL_H__

#include <stdbool.h>
#include "global.h"
#include "mpdc.h"

#define LOST_CONTROL_ANGLE deg_to_rad(30)

typedef struct {
	const char *name;
	mpdc_t *mpdc;
} ctrl_t;

#define CTRL_OBJ_INIT(_name) {#_name, NULL}

typedef struct {
	float pwm_l_out;
	float pwm_r_out;
} ctrl_param_t;


typedef struct {
	float target_speed;
	float target_turn;
	float pwm_out;
	float pwm_l_out;
	float pwm_r_out;
	float position;
	float speed;
	float speed_ctrl_out;
	int   speed_ctrl_period;
	bool lost_control_flag;
} att_control_t;

#endif

