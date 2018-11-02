/*
 * File      : attitude_estimation.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-26     weety    first version.
 */

#ifndef __ATTITUDE_ESTIMATION_H__
#define __ATTITUDE_ESTIMATION_H__

#include "global.h"
#include "mpdc.h"

typedef struct {
	const char *name;
	mpdc_t *mpdc;
} att_t;

#define ATT_OBJ_INIT(_name) {#_name, NULL}

typedef struct {
	float angle;
	float speed;
} att_angle_t;

int angle_estimation_init(void);
void angle_estimation(float dt);

#endif

