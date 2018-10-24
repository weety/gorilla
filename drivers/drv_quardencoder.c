/*
 * File      : drv_quardencoder.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2017 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-21     weety        First implementation.
 */
 
#include "drv_quardencoder.h"

#include <board.h>

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTF(...)   rt_kprintf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)   
#endif

struct _quardenc_priv {
	TIM_TypeDef *TIM;
	int totol_count;
	uint64_t last_time;
};

struct rt_quardenc_device {
	struct rt_device parent;
	struct _quardenc_priv tim_l;
	struct _quardenc_priv tim_r;
};

static struct rt_quardenc_device _quardenc_device;

static rt_size_t _quardenc_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    return -RT_ENOSYS;
}

static rt_size_t _quardenc_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    return -RT_ENOSYS;
}

static rt_err_t  _quardenc_control(rt_device_t dev, int cmd, void *args)
{
	rt_err_t ret = RT_EOK;
    struct rt_quardenc_param *param;
    struct rt_quardenc_device *quardenc = (struct rt_quardenc_device *)dev->user_data;

    /* check parameters */
    RT_ASSERT(quardenc != RT_NULL);

	switch (cmd)
	{
		case RT_QUARDENC_CTRL_GET_VAL:
		{
			param = args;
			int channel = param->channel;
			if ((channel < 1) || (channel > 2))
			{
				ret = -RT_EINVAL;
				break;
			}
			switch (channel)
			{
				case 1:
					param->count = quardenc->tim_l.totol_count * COUNT_PER_CIRCLE + quardenc->tim_l.TIM->CNT;
					param->delta_t = time_get_us() - quardenc->tim_l.last_time;
					break;
				case 2:
					param->count = quardenc->tim_r.totol_count * COUNT_PER_CIRCLE + quardenc->tim_r.TIM->CNT;
					param->delta_t = time_get_us() - quardenc->tim_r.last_time;
					break;
				default:
					break;
			}
			break;
		}
		case RT_QUARDENC_CTRL_RESET:
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
					quardenc->tim_l.TIM->CNT = 0;
					quardenc->tim_l.totol_count = 0;
					quardenc->tim_l.last_time = time_get_us();
					break;
				case 2:
					quardenc->tim_r.TIM->CNT = 0;
					quardenc->tim_r.totol_count = 0;
					quardenc->tim_r.last_time = time_get_us();
					break;
				default:
					break;
			}
			break;
		}
		default:
			rt_kprintf("[quardenc] Inalid argument\n");
			ret = -RT_EINVAL;
			break;
	}

    return ret;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	uint16_t cr1 = TIM1->CR1;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		if (cr1 & TIM_CR1_DIR)
			_quardenc_device.tim_l.totol_count--;
		else
			_quardenc_device.tim_l.totol_count++;
	}

}

void TIM3_IRQHandler(void)
{
	uint16_t cr1 = TIM3->CR1;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		if (cr1 & TIM_CR1_DIR)
			_quardenc_device.tim_r.totol_count--;
		else
			_quardenc_device.tim_r.totol_count++;
	}

}

int rt_hw_quardenc_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	_quardenc_device.tim_l.TIM = TIM1;
	_quardenc_device.tim_r.TIM = TIM3;
	/* GPIOE and GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* GPIOE Configuration: TIM1 CH1 (PE9) and TIM1 CH2 (PE11) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
	/* Connect TIM1 pins to AF1 */  
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 

	/* GPIOB Configuration: TIM3 CH1 (PB4) and TIM1 CH2 (PB5) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	/* Connect TIM3 pins to AF2 */  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); 

	/* Time1 base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(_quardenc_device.tim_l.TIM, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(_quardenc_device.tim_l.TIM, TIM_EncoderMode_TI12, 
		TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(_quardenc_device.tim_l.TIM, &TIM_ICInitStructure);

	// Clear all pending interrupts  
	TIM_ClearFlag(_quardenc_device.tim_l.TIM, TIM_FLAG_Update);
	TIM_ITConfig(_quardenc_device.tim_l.TIM, TIM_IT_Update, ENABLE);
	//Reset counter
	_quardenc_device.tim_l.TIM->CNT = 0;

	/* Enable the TIM1 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM1 enable counter */
	TIM_Cmd(_quardenc_device.tim_l.TIM, ENABLE);

	/* Time3 base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(_quardenc_device.tim_r.TIM, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(_quardenc_device.tim_r.TIM, TIM_EncoderMode_TI12, 
		TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(_quardenc_device.tim_r.TIM, &TIM_ICInitStructure);

	// Clear all pending interrupts  
	TIM_ClearFlag(_quardenc_device.tim_r.TIM, TIM_FLAG_Update);
	TIM_ITConfig(_quardenc_device.tim_r.TIM, TIM_IT_Update, ENABLE);
	//Reset counter
	_quardenc_device.tim_r.TIM->CNT = 0;

	/* Enable the TIM3 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 enable counter */
	TIM_Cmd(_quardenc_device.tim_r.TIM, ENABLE);

	_quardenc_device.parent.type         = RT_Device_Class_Miscellaneous;
	_quardenc_device.parent.rx_indicate  = RT_NULL;
	_quardenc_device.parent.tx_complete  = RT_NULL;

	_quardenc_device.parent.init         = RT_NULL;
	_quardenc_device.parent.open         = RT_NULL;
	_quardenc_device.parent.close        = RT_NULL;
	_quardenc_device.parent.read         = _quardenc_read;
	_quardenc_device.parent.write        = _quardenc_write;
	_quardenc_device.parent.control      = _quardenc_control;

	_quardenc_device.parent.user_data    = &_quardenc_device;

	/* register a character device */
    rt_device_register(&_quardenc_device.parent, "qenc", RT_DEVICE_FLAG_RDWR);

	return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_quardenc_init);

