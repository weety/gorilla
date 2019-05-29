/*
 * File      : sbus.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-29     weety    first version.
 */


#ifndef __SBUS_H__
#define __SBUS_H__
#include <stdio.h>
#include <stdbool.h>

unsigned sbus_dropped_frames(void);

void sbus1_output(uint16_t *values, uint16_t num_values);
void sbus2_output(uint16_t *values, uint16_t num_values);
bool sbus_input(uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, 
		bool *sbus_frame_drop, uint16_t max_channels);
int sbus_process(void);

int sbus_init(void);
#endif
