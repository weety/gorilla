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
float Accel_x;	         //X轴加速度值暂存
float Angle_ax;          //由加速度计算的倾斜角度
float Angle;             //小车最终倾斜角度
unsigned char value;		 //角度正负极性标记


//******卡尔曼参数************
		
float  Q_angle=0.001;            //小，消除尖锋，但会使波形滞后
float  Q_gyro=0.003;              //小，调节波形的跟踪速度，但不相信预测
float  R_angle=0.5;
float  dt=0.01;	                  //dt为kalman滤波器采样时间;
char   C_0 = 1;
float  Q_bias, Angle_err;
float  PCt_0, PCt_1, E;
float  K_0, K_1, t_0, t_1;
float  Pdot[4] ={0,0,0,0};
float  PP[2][2] = { { 1, 0 },{ 0, 1 } };

//*********************************************************
// 卡尔曼滤波
//*********************************************************


void Kalman_Filter(float Accel,float Gyro)		
{
	Angle+=(Gyro - Q_bias) * dt; //先验估计

	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度

}

//*********************************************************
// 倾角计算（卡尔曼融合）
//*********************************************************

void Angle_Calcu(void)
{
	
	Accel_x  = GetData(ACCEL_XOUT_H);	  //读取X轴加速度
	Angle_ax = (Accel_x-570) /16570;   //去除零点偏移,计算得到角度（弧度）
	Angle_ax = Angle_ax*1.1*180/3.14;     //弧度转换为度,


    //-------角速度-------------------------

	//范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)

	Gyro_y = GetData(GYRO_YOUT_H);	      //静止时角速度Y轴输出为-30左右
	Gyro_y = -(Gyro_y+11)/16.4;         //去除零点偏移，计算角速度值,负号为方向处理 
 	Angle_gy = Angle_gy + Gyro_y*dt;  //角速度积分得到倾斜角度.	

	Kalman_Filter(Angle_ax,Gyro_y);       //卡尔曼滤波计算倾角

															  
}   

