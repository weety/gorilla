/*
 * File      : mpu6050.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-10-03     weety   	the first version
 */
 
#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "stm32f4xx.h"
#include <rtthread.h>

#define GYR_RAW_POS   0
#define GYR_SCALE_POS 1
#define ACC_RAW_POS   2
#define ACC_SCALE_POS 3

rt_err_t rt_mpu6050_init(char* i2c_dev_name);

#endif
