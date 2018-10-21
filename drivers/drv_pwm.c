/*
 * File      : drv_pwm.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2017 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-12-05     weety        First implementation.
 */
 
#include "drv_pwm.h"

#include <board.h>

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTF(...)   rt_kprintf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)   
#endif

struct rt_pwm_device {
	struct rt_device parent;
	TIM_TypeDef *TIM;
};

static struct rt_pwm_device _pwm_device;

static rt_size_t _pwm_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    return -RT_ENOSYS;
}

static rt_size_t _pwm_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    return -RT_ENOSYS;
}

static rt_err_t  _pwm_control(rt_device_t dev, int cmd, void *args)
{
	rt_err_t ret = RT_EOK;
    struct rt_pwm_param *param;
    struct rt_pwm_device *pwm = (struct rt_pwm_device *)dev->user_data;

    /* check parameters */
    RT_ASSERT(pwm != RT_NULL);

	switch (cmd)
	{
		case RT_PWM_CTRL_ENABLE:
		{
			int channel = (int)args;
			if ((channel < 1) || (channel > 2))
			{
				ret = -RT_EINVAL;
				break;
			}
			switch (channel)
			{
				case 1:
					TIM_CCxCmd(pwm->TIM, TIM_Channel_1, TIM_CCx_Enable);
					break;
				case 2:
					TIM_CCxCmd(pwm->TIM, TIM_Channel_2, TIM_CCx_Enable);
					break;
				default:
					break;
			}
			break;
		}
		case RT_PWM_CTRL_DISABLE:
		{
			int channel = (int)args;
			if ((channel < 1) || (channel > 2))
			{
				ret = -RT_EINVAL;
				break;
			}
			switch (channel)
			{
				case 1:
					TIM_CCxCmd(pwm->TIM, TIM_Channel_1, TIM_CCx_Disable);
					break;
				case 2:
					TIM_CCxCmd(pwm->TIM, TIM_Channel_2, TIM_CCx_Disable);
					break;
				default:
					break;
			}
			break;
		}
		case RT_PWM_CTRL_SET_DUTY:
		{
			rt_uint32_t pulse = 0;
			param = (struct rt_pwm_param *)args;
			if ((param->channel < 1) || (param->channel > 2) || 
				(param->p.duty > 100))
			{
				ret = -RT_EINVAL;
				break;
			}
			pulse = pwm->TIM->ARR/100 * param->p.duty;
			switch (param->channel)
			{
				case 1:
					pwm->TIM->CCR1 = pulse;
					break;
				case 2:
					pwm->TIM->CCR2 = pulse;
					break;
				default:
					break;
			}
			break;
		}
		case RT_PWM_CTRL_GET_DUTY:
		{
			rt_uint32_t pulse = 0;
			param = (struct rt_pwm_param *)args;
			if ((param->channel < 1) || (param->channel > 2))
			{
				ret = -RT_EINVAL;
				break;
			}
			switch (param->channel)
			{
				case 1:
					pulse = pwm->TIM->CCR1;
					break;
				case 2:
					pulse = pwm->TIM->CCR2;
					break;
				default:
					break;
			}
			param->p.duty = pulse * 100 / pwm->TIM->ARR;
			break;
		}
		case RT_PWM_CTRL_SET_FREQ:
		{
			param = (struct rt_pwm_param *)args;
			if ((param->channel < 1) || (param->channel > 4) || 
				(param->p.freq > ((SystemCoreClock /2) >> 2)))
			{
				ret = -RT_EINVAL;
				break;
			}
			pwm->TIM->ARR = (SystemCoreClock /2) / param->p.freq - 1;
			break;
		}
		case RT_PWM_CTRL_GET_FREQ:
		{
			param = (struct rt_pwm_param *)args;
			if ((param->channel < 1) || (param->channel > 4))
			{
				ret = -RT_EINVAL;
				break;
			}
			param->p.freq = (SystemCoreClock /2) / (pwm->TIM->ARR + 1);
			break;
		}
		default:
			rt_kprintf("[pwm] Inalid argument\n");
			ret = -RT_EINVAL;
			break;
	}

    return ret;
}

int rt_hw_pwm_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//TIM_OC_InitTypeDef sConfig = {0};

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	_pwm_device.TIM = TIM3;
	/* GPIOC and GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration: TIM9 CH1 (PA2) and TIM9 CH2 (PA3) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	/* Connect TIM9 pins to AF3 */  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM9); 

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = (SystemCoreClock /2) / RT_PWM_DEFAULT_FREQ - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(_pwm_device.TIM, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(_pwm_device.TIM, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(_pwm_device.TIM, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(_pwm_device.TIM, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(_pwm_device.TIM, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(_pwm_device.TIM, ENABLE);

	/* TIM9 enable counter */
	TIM_Cmd(_pwm_device.TIM, ENABLE);

	_pwm_device.parent.type         = RT_Device_Class_Miscellaneous;
	_pwm_device.parent.rx_indicate  = RT_NULL;
	_pwm_device.parent.tx_complete  = RT_NULL;

	_pwm_device.parent.init         = RT_NULL;
	_pwm_device.parent.open         = RT_NULL;
	_pwm_device.parent.close        = RT_NULL;
	_pwm_device.parent.read         = _pwm_read;
	_pwm_device.parent.write        = _pwm_write;
	_pwm_device.parent.control      = _pwm_control;

	_pwm_device.parent.user_data    = &_pwm_device;

	/* register a character device */
    rt_device_register(&_pwm_device.parent, "pwm", RT_DEVICE_FLAG_RDWR);

	return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_pwm_init);

