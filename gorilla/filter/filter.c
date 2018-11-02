/*
 * File      : filter.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-02     weety    first version.
 */

#include "filter.h"

float first_order_lowpass_filter(float data, float new_data, float coff)
{
	return data * (1 - coff) + new_data * coff;
}


