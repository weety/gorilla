/*
 * File      : sensor.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-12     weety    first version.
 */
 
#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "board.h"
#include "sensor.h"
#include "mpu6050.h"
#include "param.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

sensor_t sensor_acc = SENSOR_OBJ_INIT(sensor_acc);
sensor_t sensor_orig_acc = SENSOR_OBJ_INIT(sensor_orig_acc);
sensor_t sensor_gyr = SENSOR_OBJ_INIT(sensor_gyr);
sensor_t sensor_orig_gyr = SENSOR_OBJ_INIT(sensor_orig_gyr);
sensor_t sensor_qenc = SENSOR_OBJ_INIT(sensor_qenc);

static rt_device_t sensor_acc_dev;
static rt_device_t sensor_gyr_dev;
static rt_device_t sensor_qenc_dev;

int gorilla_sensor_init(void)
{
	rt_mpu6050_init("i2c1");

	sensor_acc_dev = rt_device_find(SENSOR_ACC_DEVICE);
	if (!sensor_acc_dev)
	{
		rt_kprintf("Device:%s not found\n", SENSOR_ACC_DEVICE);
		return -ENODEV;
	}
	rt_device_open(sensor_acc_dev , RT_DEVICE_OFLAG_RDWR);

	sensor_gyr_dev = rt_device_find(SENSOR_GYR_DEVICE);
	if (!sensor_gyr_dev)
	{
		rt_kprintf("Device:%s not found\n", SENSOR_GYR_DEVICE);
		return -ENODEV;
	}
	rt_device_open(sensor_gyr_dev , RT_DEVICE_OFLAG_RDWR);

	sensor_qenc_dev = rt_device_find(SENSOR_QENC_DEVICE);
	if (!sensor_qenc_dev)
	{
		rt_kprintf("Device:%s not found\n", SENSOR_QENC_DEVICE);
		return -ENODEV;
	}
	rt_device_open(sensor_qenc_dev , RT_DEVICE_OFLAG_RDWR);

	sensor_acc.mpdc = mpdc_advertise(sensor_acc.name, sizeof(sensor_acc_t));
	sensor_orig_acc.mpdc = mpdc_advertise(sensor_orig_acc.name, sizeof(sensor_acc_t));
	sensor_gyr.mpdc = mpdc_advertise(sensor_gyr.name, sizeof(sensor_gyr_t));
	sensor_orig_gyr.mpdc = mpdc_advertise(sensor_orig_gyr.name, sizeof(sensor_gyr_t));

	sensor_qenc.mpdc = mpdc_advertise(sensor_qenc.name, sizeof(sensor_qenc_t));

	mpdc_subscribe(sensor_orig_acc.name, sizeof(sensor_acc_t));
	mpdc_subscribe(sensor_acc.name, sizeof(sensor_acc_t));
	mpdc_subscribe(sensor_orig_gyr.name, sizeof(sensor_gyr_t));
	mpdc_subscribe(sensor_gyr.name, sizeof(sensor_gyr_t));
	mpdc_subscribe(sensor_qenc.name, sizeof(sensor_qenc_t));

	return 0;
}

INIT_COMPONENT_EXPORT(gorilla_sensor_init);

rt_err_t sensor_acc_raw_measure(int16_t acc[3])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(sensor_acc_dev, ACC_RAW_POS, (void*)acc, 6);
	
	return r_byte == 6 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_acc_measure(sensor_acc_t *acc)
{
	rt_size_t r_byte;
	float data[3] = {0.0f, 0.0f, 0.0f};
	r_byte = rt_device_read(sensor_acc_dev, ACC_SCALE_POS, (void*)data, 12);
	acc->x = data[0];
	acc->y = data[1];
	acc->z = data[2];
	
	return r_byte == 12 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_acc_get_calibrated_data(sensor_acc_t *acc)
{
	int i = 0;
	sensor_acc_t acc_m;
	rt_err_t res;
	float acc_f[3];
	float ofs[3];
	float trans_mat[3][3];
	float ofs_acc[3];
	
	res = sensor_acc_measure(&acc_m);
	
	// push non-calibrated data for calibration
	mpdc_push_data(sensor_orig_acc.mpdc, &acc_m);

	acc_f[0] = acc_m.x;
	acc_f[1] = acc_m.y;
	acc_f[2] = acc_m.z;

	param_get_by_idx(ACC_X_OFFSET, &ofs[0]);
	param_get_by_idx(ACC_Y_OFFSET, &ofs[1]);
	param_get_by_idx(ACC_Z_OFFSET, &ofs[2]);
	param_get_by_idx(ACC_TRANS_MAT00, &trans_mat[0][0]);
	param_get_by_idx(ACC_TRANS_MAT01, &trans_mat[0][1]);
	param_get_by_idx(ACC_TRANS_MAT02, &trans_mat[0][2]);
	param_get_by_idx(ACC_TRANS_MAT10, &trans_mat[1][0]);
	param_get_by_idx(ACC_TRANS_MAT11, &trans_mat[1][1]);
	param_get_by_idx(ACC_TRANS_MAT12, &trans_mat[1][2]);
	param_get_by_idx(ACC_TRANS_MAT20, &trans_mat[2][0]);
	param_get_by_idx(ACC_TRANS_MAT21, &trans_mat[2][1]);
	param_get_by_idx(ACC_TRANS_MAT22, &trans_mat[2][2]);

	for (i = 0; i < 3; i++) {
		ofs_acc[i] = acc_f[i] - ofs[i];
	}
	for (i = 0; i < 3; i++) {
		acc_f[i] = ofs_acc[0]*trans_mat[0][i] + ofs_acc[1]*trans_mat[1][i] + ofs_acc[2]*trans_mat[2][i];
	}

	acc->x = acc_f[0];
	acc->y = acc_f[1];
	acc->z = acc_f[2];

	return res;
}


rt_err_t sensor_gyr_raw_measure(int16_t gyr[3])
{
	rt_size_t r_size;
	r_size = rt_device_read(sensor_gyr_dev, GYR_RAW_POS, (void*)gyr, 6);
	
	return r_size == 6 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_gyr_measure(sensor_gyr_t *gyr)
{
	rt_size_t r_size;
	float data[3] = {0.0f, 0.0f, 0.0f};
	r_size = rt_device_read(sensor_gyr_dev, GYR_SCALE_POS, (void*)data, 12);
	gyr->x = data[0];
	gyr->y = data[1];
	gyr->z = data[2];
	
	return r_size == 12 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_gyr_get_calibrated_data(sensor_gyr_t *gyr)
{
	sensor_gyr_t gyr_m;
	rt_err_t res;
	
	float gyr_offset[3];
	float gyr_gain[3];

	param_get_by_idx(GYR_X_OFFSET, &gyr_offset[0]);
	param_get_by_idx(GYR_Y_OFFSET, &gyr_offset[1]);
	param_get_by_idx(GYR_Z_OFFSET, &gyr_offset[2]);
	param_get_by_idx(GYR_X_GAIN, &gyr_gain[0]);
	param_get_by_idx(GYR_Y_GAIN, &gyr_gain[1]);
	param_get_by_idx(GYR_Z_GAIN, &gyr_gain[2]);
	
	res = sensor_gyr_measure(&gyr_m);
						 
	// push non-calibrated data for calibration
	mpdc_push_data(sensor_orig_gyr.mpdc, &gyr_m);

	gyr->x = (gyr_m.x + gyr_offset[0]) * gyr_gain[0];
	gyr->y = (gyr_m.y + gyr_offset[1]) * gyr_gain[1];
	gyr->z = (gyr_m.z + gyr_offset[2]) * gyr_gain[2];

	return res;
}

rt_err_t sensor_qenc_measure(sensor_qenc_t *qenc)
{
	rt_err_t err;
	struct rt_quardenc_param param;
	param.channel = 1;
	err = rt_device_control(sensor_qenc_dev, RT_QUARDENC_CTRL_GET_VAL, &param);
	if (err) {
		return err;
	}
	qenc->count_l = param.count;
	qenc->speed_l = (float)param.count / COUNT_PER_CIRCLE / ((float) param.delta_t * 1e-6);
	qenc->delta_t_l = param.delta_t;

	err = rt_device_control(sensor_qenc_dev, RT_QUARDENC_CTRL_RESET, (void *)param.channel);
	if (err) {
		return err;
	}

	param.channel = 2;
	err = rt_device_control(sensor_qenc_dev, RT_QUARDENC_CTRL_GET_VAL, &param);
	if (err) {
		return err;
	}

	qenc->count_r = param.count;
	qenc->speed_r = (float)param.count / COUNT_PER_CIRCLE / ((float) param.delta_t * 1e-6);
	qenc->delta_t_r = param.delta_t;

	err = rt_device_control(sensor_qenc_dev, RT_QUARDENC_CTRL_RESET, (void *)param.channel);

	return err;
}


void sensor_measure(void)
{
	sensor_acc_t acc;
	sensor_gyr_t gyr;
	sensor_qenc_t qenc;

	if (sensor_acc_get_calibrated_data(&acc) == RT_EOK) {
		mpdc_push_data(sensor_acc.mpdc, &acc);
	}

	if (sensor_gyr_get_calibrated_data(&gyr) == RT_EOK) {
		mpdc_push_data(sensor_gyr.mpdc, &gyr);
	}

	if (sensor_qenc_measure(&qenc) == RT_EOK) {
		mpdc_push_data(sensor_qenc.mpdc, &qenc);
	}
}

int cmd_sensor(int argc, char *argv[])
{
	uint8_t sensor_type = 0;
	uint32_t interval = 1000;	//default is 1s
	uint32_t cnt = 1;
	uint8_t raw_data = 0;
	uint8_t no_cali = 0;

	if(argc > 1){
		if(strcmp(argv[1], "acc") == 0) {
			sensor_type = 1;
		} else if(strcmp(argv[1], "gyr") == 0) {
			sensor_type = 2;
		} else if(strcmp(argv[1], "qenc") == 0) {
			sensor_type = 3;
		} else {
			rt_kprintf("unknow parameter:%s\n", argv[1]);
			return 1;
		}
		
		for(uint16_t i = 2 ; i < argc ; i++) {
			if(strcmp(argv[i], "-t") == 0) {
				i++;
				if(i >= argc) {
					rt_kprintf("wrong cmd format.\n");
					return 2;
				}
				interval = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-n") == 0) {
				i++;
				if(i >= argc) {
					rt_kprintf("wrong cmd format.\n");
					return 2;
				}
				cnt = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-r") == 0) {
				raw_data = 1;
			}
			if(strcmp(argv[i], "-nc") == 0) {
				no_cali = 1;
			}
		}

		switch(sensor_type)
		{
			case 1:	//acc
			{
				for(uint32_t i = 0 ; i < cnt ; i++) {
					if(raw_data){
						int16_t raw_acc[3];
						sensor_acc_raw_measure(raw_acc);
						rt_kprintf("raw acc:%d %d %d\n", raw_acc[0], raw_acc[1], raw_acc[2]);
					} else if(no_cali) {
						sensor_acc_t acc;
						sensor_acc_measure(&acc);
						printf("acc:%f %f %f\n", acc.x, acc.y, acc.z);
					} else {
						sensor_acc_t acc;
						mpdc_pull_data(sensor_acc.mpdc, &acc);
						printf("cali acc:%f %f %f\n", acc.x, acc.y, acc.z);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}
			} break;
			case 2:	//gyr
			{
				for(uint32_t i = 0 ; i < cnt ; i++) {
					if(raw_data) {
						int16_t raw_gyr[3];
						sensor_gyr_raw_measure(raw_gyr);
						rt_kprintf("raw gyr:%d %d %d\n", raw_gyr[0], raw_gyr[1], raw_gyr[2]);
					} else if(no_cali) {
						sensor_gyr_t gyr;
						sensor_gyr_measure(&gyr);
						printf("gyr:%f %f %f\n", gyr.x, gyr.y, gyr.z);
					} else {
						sensor_gyr_t gyr;
						mpdc_pull_data(sensor_gyr.mpdc, &gyr);
						printf("cali acc:%f %f %f\n", gyr.x, gyr.y, gyr.z);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}	
			} break;
			case 3:	//qenc
			{
				for(uint32_t i = 0 ; i < cnt ; i++) {
					if(raw_data || no_cali) {
						sensor_qenc_t qenc;
						mpdc_pull_data(sensor_qenc.mpdc, &qenc);
						printf("qenc:%d-%lu-%f %d-%lu-%f\n", qenc.count_l, qenc.delta_t_l, qenc.speed_l, 
							qenc.count_r, qenc.delta_t_r, qenc.speed_r);
					} else {
						sensor_qenc_t qenc;
						mpdc_pull_data(sensor_qenc.mpdc, &qenc);
						printf("cali qenc:%d-%lu-%f %d-%lu-%f\n", qenc.count_l, qenc.delta_t_l, qenc.speed_l, 
							qenc.count_r, qenc.delta_t_r, qenc.speed_r);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}	
			} break;
			default:
				break;
		}
	}

	return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

MSH_CMD_EXPORT_ALIAS(cmd_sensor, sensor, sensor measure);

#endif

