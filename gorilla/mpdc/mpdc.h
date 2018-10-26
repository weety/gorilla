/*
 * File      : mpdc.h
 * This file implement the component for Multiprocess data communication
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-15     weety      first implementation
 */
#ifndef __MPDC_H__
#define __MPDC_H__

#include <stdint.h>
#include <stdbool.h>
#include <rtthread.h>
#include <board.h>

#define MPDC_DEV_NAME "mpdc"

#define MPDC_MAX_NAME_LEN 32

typedef struct {
	struct rt_mutex  lock;
	rt_list_t mpdc_head;
} mpdc_meta_t;

#define MPDC_FLAG_ADVERTISE (1u << 0)
#define MPDC_FLAG_SUBSCRIBE (1u << 1)
#define MPDC_FLAG_UPDATED   (1u << 2)

#define MPDC_EVENT_POLLIN   (1u << 0)
#define MPDC_EVENT_POLLOUT  (1u << 1)

typedef struct {
	char     name[MPDC_MAX_NAME_LEN];
	uint32_t objsize;
	void     *data;
	uint32_t flag;
	struct rt_event evt;
	rt_list_t node;
	uint64_t timestamp;
} mpdc_t;

typedef enum {
	MPDC_OK  = 0,
	MPDC_ERR = 1,
	MPDC_ERR_TIMEOUT = 2,
	MPDC_ERR_NO_ADVERTISE = 3,
	MPDC_ERR_NO_SUBSCRIBE = 4,
} mpdc_errno;

#define MPDC_MALLOC(s) rt_malloc(s)
#define MPDC_FREE(a)   rt_free(a)
#define MPDC_LOCK()    rt_enter_critical()
#define MPDC_UNLOCK()  rt_exit_critical()

mpdc_t *mpdc_advertise(const char *name, uint32_t size);
mpdc_t *mpdc_subscribe(const char *name, uint32_t size);
int mpdc_unsubscribe(mpdc_t *mpdc);
int mpdc_push_data(mpdc_t *mpdc, void *data);
int mpdc_pull_data(mpdc_t *mpdc, void *data);
int mpdc_poll(mpdc_t *mpdc, int timeout);
int mpdc_check(mpdc_t *mpdc, bool *updated);
int mpdc_stat(mpdc_t *mpdc, uint64_t *time);
void mpdc_dump(void);
int mpdc_init(void);

#endif
