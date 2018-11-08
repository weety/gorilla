#include "global.h"
#include "sensor.h"
#include "attitude_estimation.h"

rt_thread_t fastloop_thread;
rt_thread_t twowheel_thread;

void fastloop_entry(void *parameter)
{
	while(1) {
		sensor_measure();
		rt_thread_mdelay(FASTLOOP_THREAD_PERIOD);
	}
}

void twowheel_entry(void *parameter)
{
	att_control_init();
	while(1) {
		angle_estimation(0.01f);
		att_control_main();
		rt_thread_mdelay(TWOWHEEL_THREAD_PERIOD);
	}
}

int twowheel_main(void)
{
	fastloop_thread = rt_thread_create("fastloop", fastloop_entry, RT_NULL, FASTLOOP_THREAD_STACKSIZE, FASTLOOP_THREAD_PRIO, 2);
	if (fastloop_thread) {
		rt_thread_startup(fastloop_thread);
	}

	twowheel_thread = rt_thread_create("twowheel", twowheel_entry, RT_NULL, TWOWHEEL_THREAD_STACKSIZE, TWOWHEEL_THREAD_PRIO, 2);
	if (twowheel_thread) {
		rt_thread_startup(twowheel_thread);
	}

	return 0;
}

INIT_APP_EXPORT(twowheel_main);

