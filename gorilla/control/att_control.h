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

#define LOST_CONTROL_ANGLE deg_to_rad(30)

typedef struct {
	float target_speed;
	float target_turn;
	float angle_kp;
	float angle_kd;
	float speed_kp;
	float speed_kd;
	float pwm_out;
	float pwm_l_out;
	float pwm_r_out;
	float position;
	float speed;
	bool lost_control_flag;
} att_control_t;

#endif

