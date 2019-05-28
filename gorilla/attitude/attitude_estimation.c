/*
 * File      : attitude_estimation.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-26     weety    first version.
 */

#include "sensor.h"
#include "attitude_estimation.h"
#include "kalman.h"
#include <math.h>
#include <stdio.h>

extern sensor_t sensor_acc;
extern sensor_t sensor_gyr;
att_t att_angle = ATT_OBJ_INIT(att_angle);

float Angle_gy;          //由角速度计算的倾斜角度
float Accel_y;           //Y轴加速度值暂存
float Angle_ay;          //由加速度计算的倾斜角度
extern float Gyro_x;
extern float Angle;

//*********************************************************
// 倾角计算（卡尔曼融合）
//*********************************************************

void angle_estimation(float dt)
{
	sensor_acc_t acc;
	sensor_gyr_t gyr;
	att_angle_t  angle;

	mpdc_pull_data(sensor_acc.mpdc, &acc);
	Accel_y  = acc.y;    //读取X轴加速度
	//Angle_ay = acc.y;  //小角度下 sin(sita) ~= sita
	Angle_ay = atan(acc.y / sqrt(acc.x * acc.x + acc.z * acc.z));
	if (isnan(Angle_ay)) {
		printf("detect angle is nan[%f]\n", acc.y);
	}

	mpdc_pull_data(sensor_gyr.mpdc, &gyr);
	Gyro_x = gyr.x;
	//Gyro_y = GetData(GYRO_YOUT_H);       //静止时角速度Y轴输出为-30左右
	//Gyro_y = -(Gyro_y+11)/16.4;         //去除零点偏移，计算角速度值,负号为方向处理 
	Angle_gy = Angle_gy + Gyro_x*dt;  //角速度积分得到倾斜角度.	

	kalman_filter(Angle_ay, Gyro_x, dt);       //卡尔曼滤波计算倾角

	angle.angle_acc = Angle_ay;
	angle.angle = Angle;
	angle.speed = Gyro_x;
	mpdc_push_data(att_angle.mpdc, &angle);
}


int angle_estimation_init(void)
{
	att_angle.mpdc = mpdc_advertise(att_angle.name, sizeof(att_angle_t));
	mpdc_subscribe(att_angle.name, sizeof(att_angle_t));

	return 0;
}

INIT_COMPONENT_EXPORT(angle_estimation_init);

int cmd_att(int argc, char *argv[])
{
	att_angle_t angle;
	mpdc_pull_data(att_angle.mpdc, &angle);

	printf("Current attitude angle %f\n", rad_to_deg(angle.angle));
	return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

MSH_CMD_EXPORT_ALIAS(cmd_att, att, dump att info);

#endif

