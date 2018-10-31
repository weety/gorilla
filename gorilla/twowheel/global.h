
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define MS_TO_TICKS(ms)   (ms * RT_TICK_PER_SECOND / 1000)
#define TICKS_TO_MS(tick) (tick * 1000 / RT_TICK_PER_SECOND)

#define FASTLOOP_THREAD_PERIOD    10 /* 10ms per loop */
#define FASTLOOP_THREAD_STACKSIZE 2048
#define FASTLOOP_THREAD_PRIO      3

#define TWOWHEEL_THREAD_PERIOD    10 /* 10ms per loop */
#define TWOWHEEL_THREAD_STACKSIZE 2048
#define TWOWHEEL_THREAD_PRIO      4

#define rad_to_deg(x)        ((x)*57.2957795f)
#define deg_to_rad(x)        ((x)*0.0174533f)
#define GRAVITY_ACC          9.81f

#ifndef PI
	#define PI               3.14159265358979f
#endif


#endif

