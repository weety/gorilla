/*
 * File      : sensor.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-10-12     weety   	the first version
 */
 
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f4xx.h"
#include <rtthread.h>
#include "drv_quardencoder.h"
#include "mpdc.h"

#define SENSOR_ACC_DEVICE "mpu6050"
#define SENSOR_GYR_DEVICE "mpu6050"
#define SENSOR_QENC_DEVICE "qenc"

typedef struct {
	const char *name;
	mpdc_t *mpdc;
} sensor_t;

#define SENSOR_OBJ_INIT(_name) {#_name, NULL}

typedef struct {
	float x;
	float y;
	float z;
} sensor_acc_t;

typedef struct {
	float x;
	float y;
	float z;
} sensor_gyr_t;

typedef struct {
	int count_l;
	int count_r;
	float speed_l;
	float speed_r;
	uint32_t delta_t_l;
	uint32_t delta_t_r;
} sensor_qenc_t;

int gorilla_sensor_init(void);
void sensor_measure(void);

#endif
