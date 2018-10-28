
#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#define MS_TO_TICKS(ms)   (ms * RT_TICK_PER_SECOND / 1000)
#define TICKS_TO_MS(tick) (tick * 1000 / RT_TICK_PER_SECOND)

#define FASTLOOP_THREAD_PERIOD    10 /* 10ms per loop */
#define FASTLOOP_THREAD_STACKSIZE 2048
#define FASTLOOP_THREAD_PRIO      3


#endif

