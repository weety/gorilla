/*
 * File      : kalman.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-26     weety    first version.
 */

#include "kalman.h"

/********************角度参数**************/ 

float Gyro_y;            //Y轴陀螺仪数据暂存
float Angle_gy;          //由角速度计算的倾斜角度
float Accel_x;           //X轴加速度值暂存
float Angle_ax;          //由加速度计算的倾斜角度
float Angle;             //小车最终倾斜角度
unsigned char value;     //角度正负极性标记


//******卡尔曼参数************
		
float  Q_angle = 0.001;            //小，消除尖锋，但会使波形滞后
float  Q_gyro = 0.003;              //小，调节波形的跟踪速度，但不相信预测
float  R_angle = 0.5;
float  dt = 0.01;                    //dt为kalman滤波器采样时间;
char   C_0 = 1;
float  Q_bias, Angle_err;
float  PCt_0, PCt_1, E;
float  K_0, K_1, t_0, t_1;
float  Pdot[4] = {0,0,0,0};
float  PP[2][2] = { { 1, 0 },{ 0, 1 } };

//*********************************************************
// 卡尔曼滤波
//*********************************************************


void kalman_filter(float Accel, float Gyro)
{
	Angle += (Gyro - Q_bias) * dt; //先验估计

	
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;  //zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;     //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	Angle   += K_0 * Angle_err;  //后验估计
	Q_bias  += K_1 * Angle_err;  //后验估计
	Gyro_y   = Gyro - Q_bias;    //输出值(后验估计)的微分=角速度

}

