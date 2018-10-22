/*
 * moto.h
 */

#ifndef __MOTO_H__
#define __MOTO_H__

#include <rtdevice.h>
#include "pwm.h"

/* Define pwm channel for moto control */
#define PWMA_LEFT_MOTO 1
#define PWMB_RIGHT_MOTO 2

/* Define left and right moto direction pin */
#define LEFT_AIN1_PIN 2
#define LEFT_AIN2_PIN 3
#define RIGHT_BIN1_PIN 4
#define RIGHT_BIN2_PIN 5
#define MOTO_CTRL_STBY 38

/* Need confirm according hardware */
/* Define moto direction controller pin level */
#define LEFT_MOTO_AIN1_DIR_FRONT 0
#define LEFT_MOTO_AIN2_DIR_FRONT 1
#define LEFT_MOTO_AIN1_DIR_BACK 1
#define LEFT_MOTO_AIN2_DIR_BACK 0
#define RIGHT_MOTO_BIN1_DIR_FRONT 0
#define RIGHT_MOTO_BIN2_DIR_FRONT 1
#define RIGHT_MOTO_BIN1_DIR_BACK 1
#define RIGHT_MOTO_BIN2_DIR_BACK 0

int moto_init(void);
int moto_left_run(int dir, float speed);
int moto_left_set_speed(float speed);
int moto_left_stop(void);
int moto_right_run(int dir, float speed);
int moto_right_set_speed(float speed);
int moto_right_stop(void);

#endif // __MOTO_H__