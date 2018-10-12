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

static rt_device_t sensor_acc_dev;
static rt_device_t sensor_gyr_dev;

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

	return 0;
}

INIT_COMPONENT_EXPORT(gorilla_sensor_init);

rt_err_t sensor_acc_raw_measure(int16_t acc[3])
{
	rt_size_t r_byte;
	
	r_byte = rt_device_read(sensor_acc_dev, ACC_RAW_POS, (void*)acc, 6);
	
	return r_byte == 6 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_acc_measure(float acc[3])
{
	rt_size_t r_byte;
	r_byte = rt_device_read(sensor_acc_dev, ACC_SCALE_POS, (void*)acc, 12);
	
	return r_byte == 12 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_gyr_raw_measure(int16_t gyr[3])
{
	rt_size_t r_size;
	r_size = rt_device_read(sensor_gyr_dev, GYR_RAW_POS, (void*)gyr, 6);
	
	return r_size == 6 ? RT_EOK : RT_ERROR;
}

rt_err_t sensor_gyr_measure(float gyr[3])
{
	rt_size_t r_size;
	r_size = rt_device_read(sensor_gyr_dev, GYR_SCALE_POS, (void*)gyr, 12);
	
	return r_size == 12 ? RT_EOK : RT_ERROR;
}

int cmd_sensor(int argc, char *argv[])
{
	uint8_t sensor_type = 0;
	uint32_t interval = 1000;	//default is 1s
	uint32_t cnt = 1;
	uint8_t raw_data = 0;
	uint8_t no_cali = 0;

	if(argc > 1){
		if(strcmp(argv[1], "acc") == 0){
			sensor_type = 1;
		}
		else if(strcmp(argv[1], "gyr") == 0){
			sensor_type = 2;
		}else{
			rt_kprintf("unknow parameter:%s\n", argv[1]);
			return 1;
		}
		
		for(uint16_t i = 2 ; i < argc ; i++){
			if(strcmp(argv[i], "-t") == 0){
				i++;
				if(i >= argc){
					rt_kprintf("wrong cmd format.\n");
					return 2;
				}
				interval = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-n") == 0){
				i++;
				if(i >= argc){
					rt_kprintf("wrong cmd format.\n");
					return 2;
				}
				cnt = atoi(argv[i]);
			}
			if(strcmp(argv[i], "-r") == 0){
				raw_data = 1;
			}
			if(strcmp(argv[i], "-nc") == 0){
				no_cali = 1;
			}
		}

		switch(sensor_type)
		{
			case 1:	//acc
			{
				for(uint32_t i = 0 ; i < cnt ; i++){
					if(raw_data){
						int16_t raw_acc[3];
						sensor_acc_raw_measure(raw_acc);
						rt_kprintf("raw acc:%d %d %d\n", raw_acc[0], raw_acc[1], raw_acc[2]);
					}else if(no_cali){
						float acc[3];
						sensor_acc_measure(acc);
						printf("acc:%f %f %f\n", acc[0], acc[1], acc[2]);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}
			}break;
			case 2:	//gyr
			{
				for(uint32_t i = 0 ; i < cnt ; i++){
					if(raw_data){
						int16_t raw_gyr[3];
						sensor_gyr_raw_measure(raw_gyr);
						rt_kprintf("raw gyr:%d %d %d\n", raw_gyr[0], raw_gyr[1], raw_gyr[2]);
					}else if(no_cali){
						float gyr[3];
						sensor_gyr_measure(gyr);
						printf("gyr:%f %f %f\n", gyr[0], gyr[1], gyr[2]);
					}
					if(cnt > 1)
						rt_thread_delay(interval);
				}	
			}break;
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

