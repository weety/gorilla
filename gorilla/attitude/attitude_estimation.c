/*
 * File      : attitude_estimation.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-26     weety    first version.
 */

#include "sensor.h"
#include "attitude_estimation.h"

extern sensor_t sensor_acc;
extern sensor_t sensor_gyr;
//sensor_t sensor_angle;

float Angle_gy;          //由角速度计算的倾斜角度
float Accel_y;           //Y轴加速度值暂存
float Angle_ay;          //由加速度计算的倾斜角度
extern float Gyro_x;

//*********************************************************
// 倾角计算（卡尔曼融合）
//*********************************************************

void angle_estimation(float dt)
{
	sensor_acc_t acc;
	sensor_gyr_t gyr;
	mpdc_pull_data(sensor_acc.mpdc, &acc);
	Accel_y  = acc.y;    //读取X轴加速度
	Angle_ay = acc.y;    //小角度下 sin(sita) ~= sita
	//Angle_ax = (Accel_x-570) /16570;   //去除零点偏移,计算得到角度（弧度）
	//Angle_ax = Angle_ax*1.1*180/3.14;     //弧度转换为度,


	//-------角速度-------------------------

	//范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)

	mpdc_pull_data(sensor_gyr->mpdc, &gyr);
	Gyro_x = gyr.x;
	//Gyro_y = GetData(GYRO_YOUT_H);       //静止时角速度Y轴输出为-30左右
	//Gyro_y = -(Gyro_y+11)/16.4;         //去除零点偏移，计算角速度值,负号为方向处理 
	Angle_gy = Angle_gy + Gyro_x*dt;  //角速度积分得到倾斜角度.	

	kalman_filter(Angle_ay, Gyro_x, dt);       //卡尔曼滤波计算倾角
}


int angle_estimation_init(void)
{
	sensor_acc.mpdc = mpdc_subscribe(sensor_acc.name, sizeof(sensor_acc_t));
	sensor_gyr.mpdc = mpdc_subscribe(sensor_gyr.name, sizeof(sensor_gyr_t));

	return 0;
}

