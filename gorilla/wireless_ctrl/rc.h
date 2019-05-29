/*
 * File      : rc.h
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-29     weety    first version.
 */

#ifndef __GORILLA_RC_H__
#define __GORILLA_RC_H__

#include <stdint.h>
#include <stdbool.h>

#define RC_CHAN_NUM    12

typedef enum
{
	CHAN_ROLL = 0,	//channel 1
	CHAN_PITCH,
	CHAN_THROTTLE,
	CHAN_YAW,
	CHAN_CTRL_MODE,
	CHAN_THROTTLE_SWITCH,
} RC_CHANEL_E;

int32_t rc_get_raw_value(RC_CHANEL_E rc_chan);
float rc_get_value(RC_CHANEL_E rc_chan);
bool rc_get_connect_status(void);
void rc_input_radio_signal(uint16_t* chan_val);

#endif

