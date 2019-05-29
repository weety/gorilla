/*
 * File      : rc.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-05-29     weety    first version.
 */

#include <stdint.h>
#include <string.h>
#include <rtthread.h>
#include "board.h"
#include "rc.h"

/* If more than RC_LOST_SIGNAL_TIME ms we don't receive ppm signal, then we think rc is disconnected */
#define RC_LOST_SIGNAL_TIME    300

static uint16_t _rc_chan_val[RC_CHAN_NUM];
static bool _rc_connect = false;
static uint32_t _time_last_receive = 0;

int32_t rc_get_raw_value(RC_CHANEL_E rc_chan)
{
	uint16_t chan_val;
	if((int)rc_chan >= RC_CHAN_NUM)
		return -1;
	
	rt_enter_critical();
	chan_val = _rc_chan_val[rc_chan];
	rt_exit_critical();
	
	return (int32_t)chan_val;
}

float rc_get_value(RC_CHANEL_E rc_chan)
{
	float chan_val;
	if((int)rc_chan >= RC_CHAN_NUM)
		return -1;
	
	rt_enter_critical();
	chan_val = _rc_chan_val[rc_chan];
	rt_exit_critical();

	if (chan_val > 2000)
		chan_val = 2000;
	if (chan_val < 1000)
		chan_val = 1000;
	return (float)(chan_val - 1000)/1000;
}

bool rc_get_connect_status(void)
{
	if(time_get_ms() - _time_last_receive > RC_LOST_SIGNAL_TIME)
	{
		_rc_connect = false;
	}
	else
	{
		_rc_connect = true;
	}
	
	return _rc_connect;
}

void rc_input_radio_signal(uint16_t* chan_val)
{
	for(int i = 0 ; i < RC_CHAN_NUM ; i++)
	{
		_rc_chan_val[i] = chan_val[i];
	}
	
	_time_last_receive = time_get_ms();
}

int cmd_rc(int argc, char** argv)
{
	if(argc > 1)
	{
		if(strcmp(argv[1], "status") == 0)
		{
			rt_kprintf("rc connected: %s\n", rc_get_connect_status() ? "true" : "false");
			for(int i = 0 ; i < RC_CHAN_NUM ; i++)
			{
				rt_kprintf("ch%d:%d ", i+1, _rc_chan_val[i]);
			}
			rt_kprintf("\n");
		}
	}
	
	return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

MSH_CMD_EXPORT_ALIAS(cmd_rc, rc, rc status);
#endif

