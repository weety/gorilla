/*
 * File      : fuzzy_pid.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-06-10     weety    first version.
 */

#ifndef __FUZZY_PID_H__
#define __FUZZY_PID_H__

#include <stdbool.h>
#include "global.h"

#define LANGUAGE_DOMAIN_RANGE 6
#define E_EXPANSION_FACTOR  6
#define EC_EXPANSION_FACTOR 6

#define E_MAX  (0.09) /* max angle rad range:[-0.09,0.09]*/
#define EC_MAX (0.5)  /* max angular velocity rad/s range:[-0.5,0.5]*/

#define E_MAX_SCALER  (E_MAX * E_EXPANSION_FACTOR) /* Increasing the scaling factor increases the sensitivity*/
#define EC_MAX_SCALER (EC_MAX * EC_EXPANSION_FACTOR) /* Increasing the scaling factor increases the sensitivity*/

#define KE  ((float)LANGUAGE_DOMAIN_RANGE/E_MAX_SCALER)
#define KEC ((float)LANGUAGE_DOMAIN_RANGE/EC_MAX_SCALER)

typedef struct {
	float kp;
	float kd;
} fuzzy_pid_param;

void fuzzy_pid_angle(float e, float ec, fuzzy_pid_param *param);

#endif

