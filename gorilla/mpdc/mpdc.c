/*
 * File      : mpdc.c
 * This file implement the component for Multiprocess data communication
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-15     weety      first implementation
 */

#include <stdio.h>
#include <string.h>
#include "mpdc.h"

#define MS_TO_TICKS(ms) (ms * RT_TICK_PER_SECOND / 1000)

static mpdc_meta_t mpdc_meta;

static mpdc_t *mpdc_alloc(const char *name, uint32_t size)
{
	mpdc_t *mpdc = NULL;
	rt_list_t *pos;

	rt_mutex_take(&(mpdc_meta.lock), RT_WAITING_FOREVER);
	rt_list_for_each(pos, &(mpdc_meta.mpdc_head)) {
		mpdc = rt_list_entry(pos, mpdc_t, node);
		if (!strncmp(name, mpdc->name, MPDC_MAX_NAME_LEN)) {
			rt_mutex_release(&(mpdc_meta.lock));
			return mpdc;
		}
	}

	mpdc = MPDC_MALLOC(sizeof(*mpdc));
	if (!mpdc) {
		goto err;
	}
	memset(mpdc, 0, sizeof(*mpdc));
	mpdc->data = MPDC_MALLOC(size);
	if (!mpdc->data) {
		MPDC_FREE(mpdc);
		goto err;
	}
	memset(mpdc->data, 0, size);

	strncpy(mpdc->name, name, MPDC_MAX_NAME_LEN);
	mpdc->objsize = size;
	rt_event_init(&mpdc->evt, name, RT_IPC_FLAG_FIFO);
	rt_list_insert_before(&(mpdc_meta.mpdc_head), &mpdc->node);
	rt_mutex_release(&(mpdc_meta.lock));

	return mpdc;
err:
	rt_mutex_release(&(mpdc_meta.lock));
	return NULL;
}

mpdc_t *mpdc_advertise(const char *name, uint32_t size)
{
	mpdc_t *mpdc = NULL;

	mpdc = mpdc_alloc(name, size);
	RT_ASSERT(mpdc->objsize == size);
	MPDC_LOCK();
	mpdc->flag |= MPDC_FLAG_ADVERTISE;
	MPDC_UNLOCK();

	return mpdc;
}


mpdc_t *mpdc_subscribe(const char *name, uint32_t size)
{
	mpdc_t *mpdc = NULL;

	mpdc = mpdc_alloc(name, size);
	RT_ASSERT(mpdc->objsize == size);
	MPDC_LOCK();
	mpdc->flag |= MPDC_FLAG_SUBSCRIBE;
	MPDC_UNLOCK();

	return mpdc;
}

int mpdc_unsubscribe(mpdc_t *mpdc)
{
	MPDC_LOCK();
	mpdc->flag &= ~MPDC_FLAG_SUBSCRIBE;
	MPDC_UNLOCK();

	return 0;
}

int mpdc_push_data(mpdc_t *mpdc, void *data)
{
	int ret = MPDC_OK;

	if (!(mpdc->flag & MPDC_FLAG_ADVERTISE)) {
			ret = -MPDC_ERR_NO_ADVERTISE;
			goto out;
	}

	MPDC_LOCK();
	memcpy(mpdc->data, data, mpdc->objsize);
	mpdc->timestamp = time_get_us();
	mpdc->flag |= MPDC_FLAG_UPDATED;
	MPDC_UNLOCK();
	rt_event_send(&mpdc->evt, MPDC_EVENT_POLLIN);

out:
	return ret;

}

int mpdc_pull_data(mpdc_t *mpdc, void *data)
{
	int ret = MPDC_OK;

	if (!(mpdc->flag & MPDC_FLAG_ADVERTISE)) {
		ret = -MPDC_ERR_NO_ADVERTISE;
		goto out;
	}
	if (!(mpdc->flag & MPDC_FLAG_SUBSCRIBE)) {
		ret = -MPDC_ERR_NO_SUBSCRIBE;
		goto out;
	}
	MPDC_LOCK();
	memcpy(data, mpdc->data, mpdc->objsize);
	mpdc->flag &= ~MPDC_FLAG_UPDATED;
	MPDC_UNLOCK();

out:
	return ret;
}

int mpdc_poll(mpdc_t *mpdc, int timeout)
{
	rt_err_t res;
	int ret = MPDC_OK;
	rt_uint32_t recv_set = 0;
	rt_uint32_t wait_set = MPDC_EVENT_POLLIN;

	if (timeout == 0) {
		return (mpdc->flag & MPDC_FLAG_UPDATED) ? MPDC_OK : -MPDC_ERR_TIMEOUT;
	}

	res = rt_event_recv(&mpdc->evt, wait_set, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
						MS_TO_TICKS(timeout), &recv_set);
	if (res == RT_EOK) {
		if (recv_set & MPDC_EVENT_POLLIN) {
			ret = (mpdc->flag & MPDC_FLAG_UPDATED) ? MPDC_OK : -MPDC_ERR;
		}
	} else {
		ret = -MPDC_ERR_TIMEOUT;
	}

	return ret;
}

int mpdc_check(mpdc_t *mpdc, bool *updated)
{
	int ret = MPDC_OK;

	if (!(mpdc->flag & MPDC_FLAG_ADVERTISE)) {
		*updated = false;
		ret = -MPDC_ERR_NO_ADVERTISE;
		goto out;
	}
	if (!(mpdc->flag & MPDC_FLAG_SUBSCRIBE)) {
		*updated = false;
		ret = -MPDC_ERR_NO_SUBSCRIBE;
		goto out;
	}

	if (mpdc->flag & MPDC_FLAG_UPDATED)
		*updated = true;
	else
		*updated = false;
out:
	return ret;
}

int mpdc_stat(mpdc_t *mpdc, uint64_t *time)
{
	int ret = MPDC_OK;

	if (!(mpdc->flag & MPDC_FLAG_ADVERTISE)) {
		*time = 0;
		ret = -MPDC_ERR_NO_ADVERTISE;
		goto out;
	}
	if (!(mpdc->flag & MPDC_FLAG_SUBSCRIBE)) {
		*time = 0;
		ret = -MPDC_ERR_NO_SUBSCRIBE;
		goto out;
	}

	MPDC_LOCK();
	*time = mpdc->timestamp;
	MPDC_UNLOCK();
out:
	return ret;

}

void mpdc_dump(void)
{
	mpdc_t *mpdc = NULL;
	rt_list_t *pos;

	rt_kprintf("mpdc info:\n");
	rt_mutex_take(&(mpdc_meta.lock), RT_WAITING_FOREVER);
	rt_list_for_each(pos, &(mpdc_meta.mpdc_head)) {
		mpdc = rt_list_entry(pos, mpdc_t, node);
		rt_kprintf("Node [%s] has %d objsize\n", mpdc->name, mpdc->objsize);
		rt_kprintf("Status: %s - %s - %s\n", (mpdc->flag & MPDC_FLAG_ADVERTISE) ? "advertised" : "not advertised",
			(mpdc->flag & MPDC_FLAG_SUBSCRIBE) ? "subscribed" : "not subscribed", 
			(mpdc->flag & MPDC_FLAG_UPDATED) ? "updated" : "not updated");
		printf("Last update at time %lld us\n\n", mpdc->timestamp);
	}
	rt_mutex_release(&(mpdc_meta.lock));
}

int mpdc_init(void)
{
	rt_mutex_init(&(mpdc_meta.lock), "mpdc", RT_IPC_FLAG_FIFO);
	rt_list_init(&(mpdc_meta.mpdc_head));

	return 0;
}

INIT_DEVICE_EXPORT(mpdc_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

int cmd_mpdc_dump(int argc, char *argv[])
{
	mpdc_dump();
	return 0;
}

FINSH_FUNCTION_EXPORT_ALIAS(cmd_mpdc_dump, __cmd_mpdc_dump, dump mpdc info);

#endif

