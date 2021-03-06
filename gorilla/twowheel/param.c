/*
 * File      : param.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-28     weety    first version.
 */

#include "param.h"
#include <stdio.h>
#include <stdlib.h>
#include <rtthread.h>

static param_info_t param_info[] = {
	/* sensor params */
	PARAM_DEFINE_FLOAT(GYR_X_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(GYR_Y_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(GYR_Z_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(GYR_X_GAIN, 1.0),
	PARAM_DEFINE_FLOAT(GYR_Y_GAIN, 1.0),
	PARAM_DEFINE_FLOAT(GYR_Z_GAIN, 1.0),
	PARAM_DEFINE_INT32(GYR_CALIB, 0),
	PARAM_DEFINE_FLOAT(ACC_X_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(ACC_Y_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(ACC_Z_OFFSET, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT00, 1.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT01, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT02, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT10, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT11, 1.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT12, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT20, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT21, 0.0),
	PARAM_DEFINE_FLOAT(ACC_TRANS_MAT22, 1.0),
	PARAM_DEFINE_INT32(ACC_CALIB, 0),

	PARAM_DEFINE_FLOAT(PID_ANGLE_KP, 0.0f),
	PARAM_DEFINE_FLOAT(PID_ANGLE_KD, 0.0f),
	PARAM_DEFINE_FLOAT(PID_SPEED_KP, 0.0f),
	PARAM_DEFINE_FLOAT(PID_SPEED_KI, 0.0f),

	PARAM_DEFINE_INT32(FUZZY_PID_EN, 0),
	PARAM_DEFINE_FLOAT(FUZZY_PID_KP, 9.0f),
	PARAM_DEFINE_FLOAT(FUZZT_PID_KD, 0.1f),

	PARAM_DEFINE_FLOAT(LOST_CTRL_ANGLE, 40),
	PARAM_DEFINE_FLOAT(MOTO_L_PWM_BD, 0.0f),
	PARAM_DEFINE_FLOAT(MOTO_R_PWM_BD, 0.0f),
	PARAM_DEFINE_FLOAT(SPEED_CTRL_LIMIT, 0.0f),
	PARAM_DEFINE_INT32(SPEED_SAMPLE_INTERVAL, 0),
};

param_info_t *param_get(char *param_name)
{
	param_info_t *p = param_info;
	for (int i = 0 ; i < sizeof(param_info) / sizeof(param_info_t); i++) {
		if (strcmp(param_name, p->name) == 0)
			return p;
		p++;
	}

	return NULL;
}

int param_set(char *param_name, char *val)
{
	param_info_t* p = param_get(param_name);
	if (p != NULL) {
		if (p->type == PARAM_TYPE_INT32) {
			p->val.i = atoi(val);
		}
		if (p->type == PARAM_TYPE_FLOAT) {
			p->val.f = atof(val);
		}
		return 0;
	} else {
		return -1;
	}
}

int param_set_float(char *param_name, float val)
{
	param_info_t* p = param_get(param_name);
	if (p != NULL) {
		if (p->type == PARAM_TYPE_FLOAT) {
			p->val.f = val;
		} else {
			return -2;
		};
		return 0;
	} else {
		return -1;
	}
}

int param_set_int32(char *param_name, int32_t val)
{
	param_info_t* p = param_get(param_name);
	if (p != NULL) {
		if (p->type == PARAM_TYPE_INT32) {
			p->val.i = val;
		} else {
			return -2;
		}
		return 0;
	} else {
		return -1;
	}
}

int param_get_by_idx(uint32_t idx, float *val)
{
	param_info_t *param;

	if (idx > PARAM_MAX_IDX) {
		rt_kprintf("Invalid param %d\n", idx);
		return -1;
	}
	param = &param_info[idx];
	switch (param->type) {
		case PARAM_TYPE_FLOAT:
			*val = param->val.f;
			break;
		case PARAM_TYPE_INT32:
			memcpy(val, &(param->val.i), sizeof(param->val.i));
			break;
		default:
			*val = param->val.f;
			break;
	}

	return 0;
}

int param_set_by_idx(uint32_t idx, float val)
{
	param_info_t *param;

	if (idx > PARAM_MAX_IDX) {
		return -1;
	}
	param = &param_info[idx];
	switch (param->type) {
		case PARAM_TYPE_FLOAT:
			param->val.f = val;
			break;
		case PARAM_TYPE_INT32:
			memcpy(&(param->val.i), &val, sizeof(param->val.i));
			break;
		default:
			param->val.f = val;
			break;
	}

	return 0;
}

int param_get_by_info(param_info_t *param, float *val)
{
	switch (param->type) {
		case PARAM_TYPE_FLOAT:
			*val = param->val.f;
			break;
		case PARAM_TYPE_INT32:
			memcpy(&val, &(param->val.i), sizeof(param->val.i));
			break;
		default:
			*val = param->val.f;
			break;
	}

	return 0;
}

int param_set_by_info(param_info_t* param, float val)
{
	switch (param->type) {
		case PARAM_TYPE_FLOAT:
			param->val.f = val;
			break;
		case PARAM_TYPE_INT32:
			memcpy(&(param->val.i), &val, sizeof(param->val.i));
			break;
		default:
			param->val.f = val;
			break;
	}

	return 0;
}

uint32_t param_get_info_count(void)
{
	return sizeof(param_info)/sizeof(param_info_t);
}

uint32_t param_get_info_index(char *param_name)
{
	uint32_t index = 0;
	param_info_t *p = param_info;
	for (int i = 0 ; i < sizeof(param_info)/sizeof(param_info_t); i++) {
		if (strcmp(param_name, p->name) == 0)
			return index;
		p++;
		index++;
	}

	return index;
}

void param_dump(void)
{
	param_info_t *p = param_info;
	for(int i = 0 ; i < sizeof(param_info)/sizeof(param_info_t); i++){
		printf("%25s: ", p->name);
		if(p->type == PARAM_TYPE_INT32){
			printf("%ld\n", p->val.i);
		}
		if(p->type == PARAM_TYPE_FLOAT){
			printf("%f\n", p->val.f);
		}
		p++;
	}
}

int param_dump_info(char *param_name)
{
	param_info_t* p = param_get(param_name);

	if (p != NULL) {
		printf("%25s: ", p->name);
		if (p->type == PARAM_TYPE_INT32) {
			printf("%ld\n", p->val.i);
		}
		if (p->type == PARAM_TYPE_FLOAT) {
			printf("%f\n", p->val.f);
		}

		return -1;
	}

	return 0;
}

int param_init(void)
{
	param_set_float("GYR_X_OFFSET", 0.174777);
	param_set_float("GYR_Y_OFFSET", -0.002860);
	param_set_float("GYR_Z_OFFSET", 0.008812);
	param_set_int32("GYR_CALIB", 1);
	param_set_float("ACC_X_OFFSET", 0.081311);
	param_set_float("ACC_Y_OFFSET", 0.136616);
	param_set_float("ACC_Z_OFFSET", -0.354648);
	param_set_float("ACC_TRANS_MAT00", 0.996216);
	param_set_float("ACC_TRANS_MAT01", -0.000395);
	param_set_float("ACC_TRANS_MAT02", 0.002893);
	param_set_float("ACC_TRANS_MAT10", -0.000395);
	param_set_float("ACC_TRANS_MAT11", 0.996788);
	param_set_float("ACC_TRANS_MAT12", 0.000157);
	param_set_float("ACC_TRANS_MAT20", 0.002893);
	param_set_float("ACC_TRANS_MAT21", 0.000157);
	param_set_float("ACC_TRANS_MAT22", 0.992257);
	param_set_int32("ACC_CALIB", 1);

	return 0;
}

INIT_COMPONENT_EXPORT(param_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

int cmd_param(int argc, char *argv[])
{
	if (argc > 1) {
		if (strcmp(argv[1], "dump") == 0) {
			param_dump();
		}

		if (strcmp(argv[1], "get") == 0 && argc == 3) {
			return param_dump_info(argv[2]);
		}

		if (strcmp(argv[1], "set") == 0 && argc == 4) {
			if (param_set(argv[2], argv[3]))
				printf("fail, can not find %s\n", argv[2]);
			else
				printf("success, %s is set to %s\n", argv[2], argv[3]);
		}
	}

	return 0;
}

MSH_CMD_EXPORT_ALIAS(cmd_param, param, param info);

#endif

