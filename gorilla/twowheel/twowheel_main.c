#include "global.h"
#include "sensor.h"

rt_thread_t fastloop_thread;

void fastloop_entry(void *parameter)
{
	while(1) {
		sensor_measure();
		rt_thread_mdelay(FASTLOOP_THREAD_PERIOD);
	}
}

int twowheel_main(void)
{
	fastloop_thread = rt_thread_create("fastloop", fastloop_entry, RT_NULL, FASTLOOP_THREAD_STACKSIZE, FASTLOOP_THREAD_PRIO, 2);
	if (fastloop_thread) {
		rt_thread_startup(fastloop_thread);
	}
}

INIT_APP_EXPORT(twowheel_main);

