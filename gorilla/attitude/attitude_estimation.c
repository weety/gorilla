/*
 * File      : attitude_estimation.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-26     weety    first version.
 */

#include "attitude_estimation.h"

//*********************************************************
// 倾角计算（卡尔曼融合）
//*********************************************************

void angle_estimation(void)
{
	Accel_x  = GetData(ACCEL_XOUT_H);	  //读取X轴加速度
	Angle_ax = (Accel_x-570) /16570;   //去除零点偏移,计算得到角度（弧度）
	Angle_ax = Angle_ax*1.1*180/3.14;     //弧度转换为度,


	//-------角速度-------------------------

	//范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)

	Gyro_y = GetData(GYRO_YOUT_H);	      //静止时角速度Y轴输出为-30左右
	Gyro_y = -(Gyro_y+11)/16.4;         //去除零点偏移，计算角速度值,负号为方向处理 
	Angle_gy = Angle_gy + Gyro_y*dt;  //角速度积分得到倾斜角度.	

	kalman_filter(Angle_ax, Gyro_y);       //卡尔曼滤波计算倾角
}


