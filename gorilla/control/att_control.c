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
#include "attitude_estimation.h"
#include "param.h"
#include "att_control.h"
#include <math.h>

extern sensor_t sensor_qenc;
extern att_t att_angle;

ctrl_t ctrl_param = CTRL_OBJ_INIT(ctrl_param);

att_control_t att_control;

void angle_control(void)
{
	float lost_ctrl_angle = 0.0f;
	float angle_kp = 0.0f;
	float angle_kd = 0.0f;
	att_angle_t  angle;
	mpdc_pull_data(att_angle.mpdc, &angle);
	param_get_by_idx(LOST_CTRL_ANGLE, &lost_ctrl_angle);
	param_get_by_idx(PID_ANGLE_KP, &angle_kp);
	param_get_by_idx(PID_ANGLE_KD, &angle_kd);

	if (fabs(angle.angle) >= deg_to_rad(lost_ctrl_angle)) {
		att_control.lost_control_flag = true;
	} else {
		att_control.lost_control_flag = false;
	}
	att_control.pwm_out = angle_kp * angle.angle + angle_kd * angle.speed;
}

void speed_control(void)
{
	float speed_kp = 0.0f;
	float speed_ki = 0.0f;
	float speed_ctrl_limit = 0.0f;
	float speed;
	sensor_qenc_t qenc;
	mpdc_pull_data(sensor_qenc.mpdc, &qenc);
	param_get_by_idx(PID_SPEED_KP, &speed_kp);
	param_get_by_idx(PID_SPEED_KI, &speed_ki);
	speed = (qenc.speed_l + qenc.speed_r) * 0.5f;
	att_control.speed = first_order_lowpass_filter(att_control.speed, speed, 0.3);
	att_control.position += att_control.speed;
	att_control.position += att_control.target_speed;

	param_get_by_idx(SPEED_CTRL_LIMIT, &speed_ctrl_limit);
	if (att_control.position > speed_ctrl_limit) {
		att_control.position = speed_ctrl_limit;
	} else if (att_control.position < -speed_ctrl_limit) {
		att_control.position = -speed_ctrl_limit;
	}

	att_control.speed_ctrl_out = (speed_kp * att_control.speed + speed_ki * att_control.position);
}

void speed_control_out(void)
{
	int speed_sample_interval;

	param_get_by_idx(SPEED_SAMPLE_INTERVAL, &speed_sample_interval);

	if (att_control.speed_ctrl_period == 0) {
		speed_control();
	}

	att_control.pwm_out += att_control.speed_ctrl_out * (att_control.speed_ctrl_period + 1) / (speed_sample_interval + 1);

	att_control.speed_ctrl_period++;
	if (att_control.speed_ctrl_period > speed_sample_interval) {
		att_control.speed_ctrl_period = 0;
	}
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
	float moto_l_pwm_bd = 0.0f;
	float moto_r_pwm_bd = 0.0f;
	ctrl_param_t param;

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

	param_get_by_idx(MOTO_L_PWM_BD, &moto_l_pwm_bd);
	param_get_by_idx(MOTO_R_PWM_BD, &moto_r_pwm_bd);

	l_speed += moto_l_pwm_bd;
	r_speed += moto_r_pwm_bd;

	if (l_speed > 0.99f) {
		l_speed = 0.99f;
	}

	if (r_speed > 0.99f) {
		r_speed = 0.99f;
	}

	moto_left_run(l_dir, l_speed);
	moto_right_run(r_dir, r_speed);

	param.pwm_l_out = l_speed;
	param.pwm_r_out = r_speed;
	mpdc_push_data(ctrl_param.mpdc, &param);

	return 0;
}

int att_control_main(void)
{
	int ret;
	angle_control();
	speed_control_out();
	direction_control();
	ret = moto_control();

	return ret;
}

int att_control_init(void)
{
	int ret;

	ctrl_param.mpdc = mpdc_advertise(ctrl_param.name, sizeof(ctrl_param_t));
	mpdc_subscribe(ctrl_param.name, sizeof(ctrl_param_t));

	ret = moto_init();
	if (ret)
		return ret;

	memset(&att_control, 0, sizeof(att_control));

	return 0;
}

