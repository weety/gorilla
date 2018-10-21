/*
 * pwm.h
 */

#ifndef __PWM_H__
#define __PWM_H__

#include <rtdevice.h>
#include <drv_pwm.h>

int pwm_init(void);

int pwm_enalbe(int channel);
int pwm_disalbe(int channel);
int pwm_set_duty(int channel, int duty);
int pwm_get_duty(int channel);

int pwm_set_freq(int channel, int freq);
int pwm_get_freq(int channel);

#endif // __PWM_H__
