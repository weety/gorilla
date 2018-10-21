/*
 * pwm.c
 */

#include <rtdevice.h>
#include "pwm.h"

static rt_device_t pwm_dev = RT_NULL;

int pwm_init(void)
{
	pwm_dev = rt_device_find("pwm");
	if (!pwm_dev)
	{
		rt_kprintf("Device pwm not found\n");
		return -1;
	}

	return 0;
}

int pwm_enalbe(int channel)
{
	if (!pwm_dev)
	{
		return -1;
	}
	return rt_device_control(pwm_dev, RT_PWM_CTRL_ENABLE, (void *)channel);
}

int pwm_disalbe(int channel)
{
	if (!pwm_dev)
	{
		return -1;
	}
	return rt_device_control(pwm_dev, RT_PWM_CTRL_DISABLE, (void *)channel);
}

int pwm_set_duty(int channel, int duty)
{
	struct rt_pwm_param param;
	if (!pwm_dev)
	{
		return -1;
	}
	param.channel = channel;
	param.p.duty = duty;
	return rt_device_control(pwm_dev, RT_PWM_CTRL_SET_DUTY, &param);
}

int pwm_get_duty(int channel)
{
	int ret = 0;
	struct rt_pwm_param param;
	if (!pwm_dev)
	{
		return -1;
	}
	param.channel = channel;
	ret = rt_device_control(pwm_dev, RT_PWM_CTRL_GET_DUTY, &param);

	return ret ? ret : param.p.duty;
}

int pwm_set_freq(int channel, int freq)
{
	struct rt_pwm_param param;
	if (!pwm_dev)
	{
		return -1;
	}
	param.channel = channel;
	param.p.freq = freq;
	return rt_device_control(pwm_dev, RT_PWM_CTRL_SET_FREQ, &param);
}

int pwm_get_freq(int channel)
{
	int ret = 0;
	struct rt_pwm_param param;
	if (!pwm_dev)
	{
		return -1;
	}
	param.channel = channel;
	ret = rt_device_control(pwm_dev, RT_PWM_CTRL_GET_FREQ, &param);

	return ret ? ret : param.p.freq;
}


#ifdef RT_USING_FINSH
#include <finsh.h>

FINSH_FUNCTION_EXPORT(pwm_init, pwm init);

int cmd_pwm_init(int argc, char **argv)
{
    return pwm_init();
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwm_init, __cmd_pwm_init, pwm init);

int cmd_pwm_enable(int argc, char **argv)
{
	int channel;

	channel = strtoul(argv[1], RT_NULL, RT_NULL);
	return pwm_enalbe(channel);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwm_enable, __cmd_pwm_enable, pwm enable channel);

int cmd_pwm_disable(int argc, char **argv)
{
	int channel;

	channel = strtoul(argv[1], RT_NULL, RT_NULL);
	return pwm_disalbe(channel);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwm_disable, __cmd_pwm_disable, pwm disable channel);

int cmd_pwm_set_duty(int argc, char **argv)
{
	int channel, duty;

	channel = strtoul(argv[1], RT_NULL, RT_NULL);
	duty = strtoul(argv[2], RT_NULL, RT_NULL);
	rt_kprintf("pwm chn[%d] set duty %d%\n", channel, duty);
	return pwm_set_duty(channel, duty);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwm_set_duty, __cmd_pwm_set_duty, pwm set duty);

int cmd_pwm_get_duty(int argc, char **argv)
{
	int channel;

	channel = strtoul(argv[1], RT_NULL, RT_NULL);
	rt_kprintf("pwm get duty %d%\n", pwm_get_duty(channel));

	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwm_get_duty, __cmd_pwm_get_duty, pwm get duty);

int cmd_pwm_set_freq(int argc, char **argv)
{
	int channel, freq;

	channel = strtoul(argv[1], RT_NULL, RT_NULL);
	freq = strtoul(argv[2], RT_NULL, RT_NULL);
	rt_kprintf("pwm chn[%d] set freq %d Hz\n", channel, freq);
	return pwm_set_freq(channel, freq);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwm_set_freq, __cmd_pwm_set_freq, pwm set freq);

int cmd_pwm_get_freq(int argc, char **argv)
{
	int channel;

	channel = strtoul(argv[1], RT_NULL, RT_NULL);
	rt_kprintf("pwm freq %d Hz\n", pwm_get_freq(channel));

	return 0;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_pwm_get_freq, __cmd_pwm_get_freq, pwm get freq);

#endif
