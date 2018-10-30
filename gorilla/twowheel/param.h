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
int param_get_by_info(param_info_t *param, float *val);
int param_set_by_info(param_info_t* param, float val);
uint32_t param_get_info_count(void);
uint32_t param_get_info_index(char *param_name);

#endif

