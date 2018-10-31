/*
 * File      : param.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-28     weety    first version.
 */

#ifndef __PARAM_H__
#define __PARAM_H__

#include <stdint.h>
#include <string.h>

typedef enum {
	/* sensor params */
	GYR_X_OFFSET = 0,
	GYR_Y_OFFSET,
	GYR_Z_OFFSET,
	GYR_X_GAIN,
	GYR_Y_GAIN,
	GYR_Z_GAIN,
	GYR_CALIB,
	ACC_X_OFFSET,
	ACC_Y_OFFSET,
	ACC_Z_OFFSET,
	ACC_TRANS_MAT00,
	ACC_TRANS_MAT01,
	ACC_TRANS_MAT02,
	ACC_TRANS_MAT10,
	ACC_TRANS_MAT11,
	ACC_TRANS_MAT12,
	ACC_TRANS_MAT20,
	ACC_TRANS_MAT21,
	ACC_TRANS_MAT22,
	ACC_CALIB,

	PARAM_MAX_IDX,
} param_index_e;

typedef enum param_type_e {
	/* globally-known parameter types */
	PARAM_TYPE_INT32 = 0,
	PARAM_TYPE_FLOAT,

	/* structure parameters; size is encoded in the type value */
	PARAM_TYPE_STRUCT = 100,
	PARAM_TYPE_STRUCT_MAX = 16384 + PARAM_TYPE_STRUCT,

	PARAM_TYPE_UNKNOWN = 0xffff
} param_type_t;

union param_value_u {
	void      *p;
	int32_t    i;
	float      f;
};

typedef struct {
	const char    *name;
	param_type_t  type;
	union param_value_u val;
} param_info_t;

#define PARAM_DEFINE_INT32(_name, _default) \
		{ \
			.name = #_name, \
			.type = PARAM_TYPE_INT32, \
			.val.i = _default \
		}

#define PARAM_DEFINE_FLOAT(_name, _default) \
		{ \
			.name = #_name, \
			.type = PARAM_TYPE_FLOAT, \
			.val.f = _default \
		}

#define PARAM_DEFINE_STRUCT(_name, _default)

param_info_t *param_get(char *param_name);
int param_set(char *param_name, char *val);
int param_set_float(char *param_name, float val);
int param_set_int32(char *param_name, int32_t val);
int param_get_by_idx(uint32_t idx, float *val);
int param_set_by_idx(uint32_t idx, float val);
int param_get_by_info(param_info_t *param, float *val);
int param_set_by_info(param_info_t* param, float val);
uint32_t param_get_info_count(void);
uint32_t param_get_info_index(char *param_name);

#endif

