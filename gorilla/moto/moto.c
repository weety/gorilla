/*
 * moto.c
 */

#include <rtdevice.h>
#include "moto.h"

int moto_init(void)
{
	int ret = 0;
	ret = pwm_init();
	if (ret)
		return ret;

	rt_pin_mode(MOTO_CTRL_STBY, PIN_MODE_OUTPUT);
	rt_pin_mode(LEFT_AIN1_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(LEFT_AIN2_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RIGHT_BIN1_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(RIGHT_BIN2_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(MOTO_CTRL_STBY, 0);
	rt_pin_write(LEFT_AIN1_PIN, 0);
	rt_pin_write(LEFT_AIN2_PIN, 0);
	rt_pin_write(RIGHT_BIN1_PIN, 0);
	rt_pin_write(RIGHT_BIN2_PIN, 0);
	rt_pin_write(MOTO_CTRL_STBY, 1);

	return 0;
}

/* dir : 0,front; 1, back */
int moto_left_run(int dir, float speed)
{
	int ret = 0;

	if (!dir) {
		rt_pin_write(LEFT_AIN1_PIN, LEFT_MOTO_AIN1_DIR_FRONT);
		rt_pin_write(LEFT_AIN2_PIN, LEFT_MOTO_AIN2_DIR_FRONT);
	} else {
		rt_pin_write(LEFT_AIN2_PIN, LEFT_MOTO_AIN2_DIR_BACK);
		rt_pin_write(LEFT_AIN1_PIN, LEFT_MOTO_AIN1_DIR_BACK);
	}
	pwm_enalbe(PWMA_LEFT_MOTO);
	ret = pwm_set_duty(PWMA_LEFT_MOTO, speed * 100);
	if (ret)
		pwm_disalbe(PWMA_LEFT_MOTO);

	return ret;
}

int moto_left_set_speed(float speed)
{
	int ret = 0;
	ret = pwm_set_duty(PWMA_LEFT_MOTO, speed * 100);
	if (ret)
		pwm_disalbe(PWMA_LEFT_MOTO);
	
	return ret;
}

int moto_left_stop(void)
{
	return pwm_set_duty(PWMA_LEFT_MOTO, 0);
}

/* dir: 0,front; 1,back */
int moto_right_run(int dir, float speed)
{
	int ret = 0;

	if (!dir) {
		rt_pin_write(RIGHT_BIN1_PIN, RIGHT_MOTO_BIN1_DIR_FRONT);
		rt_pin_write(RIGHT_BIN2_PIN, RIGHT_MOTO_BIN2_DIR_FRONT);
	} else {
		rt_pin_write(RIGHT_BIN2_PIN, RIGHT_MOTO_BIN2_DIR_BACK);
		rt_pin_write(RIGHT_BIN1_PIN, RIGHT_MOTO_BIN1_DIR_BACK);
	}
	pwm_enalbe(PWMB_RIGHT_MOTO);
	ret = pwm_set_duty(PWMB_RIGHT_MOTO, speed * 100);
	if (ret)
		pwm_disalbe(PWMB_RIGHT_MOTO);

	return ret;
}

int moto_right_set_speed(float speed)
{
	int ret = 0;
	ret = pwm_set_duty(PWMB_RIGHT_MOTO, speed * 100);
	if (ret)
		pwm_disalbe(PWMB_RIGHT_MOTO);

	return ret;
}

int moto_right_stop(void)
{
	return pwm_set_duty(PWMB_RIGHT_MOTO, 0);
}

#ifdef RT_USING_FINSH
#include <finsh.h>

int cmd_moto_init(int argc, char **argv)
{
    return moto_init();
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_moto_init, __cmd_moto_init, moto init);

int cmd_moto_left_run(int argc, char **argv)
{
	int dir;
	float speed;

	if (argc != 3)
	{
		rt_kprintf("arg error\n");
	}
	dir = strtoul(argv[1], RT_NULL, RT_NULL);
	speed = strtof(argv[2], RT_NULL);
	return moto_left_run(dir, speed);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_moto_left_run, __cmd_moto_left_run, moto left run);

int cmd_moto_left_stop(int argc, char **argv)
{
	return moto_left_stop();
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_moto_left_stop, __cmd_moto_left_stop, moto left stop);

int cmd_moto_right_run(int argc, char **argv)
{
	int dir;
	float speed;

	if (argc != 3)
	{
		rt_kprintf("arg error\n");
	}
	dir = strtoul(argv[1], RT_NULL, RT_NULL);
	speed = strtof(argv[2], RT_NULL);
	return moto_right_run(dir, speed);
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_moto_right_run, __cmd_moto_right_run, moto right run);

int cmd_moto_right_stop(int argc, char **argv)
{
	return moto_right_stop();
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_moto_right_stop, __cmd_moto_right_stop, moto right stop);

int cmd_read_pin(int argc, char **argv)
{
	int pin;

	if (argc != 2)
	{
		rt_kprintf("arg error\n");
	}
	pin = strtoul(argv[1], RT_NULL, RT_NULL);
	rt_kprintf("pin[%d] = %d\n", pin, rt_pin_read(pin));
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_read_pin, __cmd_read_pin, read pin status);

#endif



