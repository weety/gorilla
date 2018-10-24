/*
 * File      : drv_quardencoder.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-21     weety        First implementation.
 */

#ifndef __DRV_QUARDENC_H__
#define __DRV_QUARDENC_H__

#include <rtthread.h>

#include "stm32f4xx.h"

#define COUNT_PER_CIRCLE 4200

/* Control command define */
#define RT_QUARDENC_CTRL_GET_VAL   0x11
#define RT_QUARDENC_CTRL_RESET     0x20

struct rt_quardenc_param 
{
	int channel;
	int count;
	uint32_t delta_t;
};

extern int rt_hw_quardenc_init(void);

#endif // __DRV_QUARDENC_H__
