/*
 * File      : att_control.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-02     weety    first version.
 */

#include "moto.h"
#include "sensor.h"
#include "filter.h"
#include "att_control.h"

extern sensor_t sensor_qenc;
extern att_t att_angle;

att_control_t att_control;

void angle_control(void)
{
	att_angle_t  angle;
	mpdc_pull_data(att_angle.mpdc, &angle);

	if (fabs(angle.angle) >= LOST_CONTROL_ANGLE) {
		att_control.lost_control_flag = true;
	} else {
		att_control.lost_control_flag = false;
	}
	att_control.pwm_out = att_control.angle_kp * angle.angle + att_control.angle_kd * angle.speed;
}

void speed_control(void)
{
	float speed;
	sensor_qenc_t qenc;
	mpdc_pull_data(sensor_qenc.mpdc, &qenc);
	speed = (qenc.speed_l + qenc.speed_r) * 0.5f;
	att_control.speed = first_order_lowpass_filter(att_control.speed, speed, 0.3);
	att_control.position += att_control.speed;
	att_control.position += att_control.target_speed;

	att_control.pwm_out += att_control.speed_kp * att_control.position + att_control.speed_kd * att_control.speed;
}

void direction_control(void)
{
	att_control.pwm_l_out = att_control.pwm_out - att_control.target_turn;
	att_control.pwm_r_out = att_control.pwm_out + att_control.target_turn;
}

int moto_control(void)
{
	int l_dir, r_dir;
	float l_speed, r_speed;

	if (att_control.lost_control_flag) {
		moto_left_stop();
		moto_right_stop();
		return -1;
	}

	if (att_control.pwm_l_out < 0) {
		l_dir = 1;
	} else {
		l_dir = 0;
	}

	l_speed = fabs(att_control.pwm_l_out);

	if (att_control.pwm_r_out < 0) {
		r_dir = 1;
	} else {
		r_dir = 0;
	}

	r_speed = fabs(att_control.pwm_r_out);

	if (l_speed > 0.6) {
		l_speed = 0.6;
	}

	if (r_speed > 0.6) {
		r_speed = 0.6;
	}

	moto_left_run(l_dir, l_speed);
	moto_right_run(r_dir, r_speed);

	return 0;
}

int att_control_main(void)
{
	int ret;
	angle_control();
	speed_control();
	direction_control();
	ret = moto_control();

	return ret;
}

int att_control_init(void)
{
	int ret;

	ret = moto_init();
	if (ret)
		return ret;

	memset(&att_control, 0, sizeof(att_control));

	return 0;
}

