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

#define SENSOR_ACC_DEVICE "mpu6050"
#define SENSOR_GYR_DEVICE "mpu6050"

int gorilla_sensor_init(void);

#endif
