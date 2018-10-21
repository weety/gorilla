/*
 * File      : drv_pwm.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-12-05     weety        First implementation.
 */

#ifndef __DRV_PWM_H__
#define __DRV_PWM_H__

#include <rtthread.h>

#include "stm32f4xx.h"

#define RT_PWM_DEFAULT_FREQ 8000

/* Control command define */
#define RT_PWM_CTRL_SET_DUTY   0x110
#define RT_PWM_CTRL_GET_DUTY   0x111
#define RT_PWM_CTRL_SET_FREQ   0x120
#define RT_PWM_CTRL_GET_FREQ   0x121

#define RT_PWM_CTRL_ENABLE     0x100
#define RT_PWM_CTRL_DISABLE    0x101

struct rt_pwm_param 
{
	int channel;
	union
	{
		rt_uint32_t duty;
		rt_uint32_t freq;
	} p;
};

extern int rt_hw_pwm_init(void);

#endif // __DRV_PWM_H__
