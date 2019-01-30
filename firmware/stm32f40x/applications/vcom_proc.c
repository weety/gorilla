
#include <board.h>
#include <rtthread.h>
#include <rtdevice.h>

#define VCOM_BUFSZ 2048
static rt_uint32_t vcom_buffer[VCOM_BUFSZ/4];
static struct rt_semaphore vcom_sem;
static rt_device_t vcom_dev;

rt_err_t vcom_rx_ind(rt_device_t dev, rt_size_t size)
{
	rt_sem_release(&vcom_sem);

	return RT_EOK;
}

void vcom_thread_entry(void* parameter)
{
	rt_err_t err;
	rt_size_t size = 0;
	rt_uint8_t *buf;

	buf = (rt_uint8_t *)vcom_buffer;

	rt_device_set_rx_indicate(vcom_dev, vcom_rx_ind);

	while(1)
	{
		err = rt_sem_take(&vcom_sem, RT_WAITING_FOREVER);
		if (err != RT_EOK)
		{
			rt_kprintf("wait sem vcom error\n");
			break;
		}

		size = rt_device_read(vcom_dev, 0, buf, VCOM_BUFSZ);
		if (size > 0)
		{
			rt_device_write(vcom_dev, 0, buf, size);
			//rt_device_write(rt_console_get_device(), 0, buf, size);
		}

	}

	rt_device_set_rx_indicate(vcom_dev, RT_NULL);
	rt_device_close(vcom_dev);
	rt_sem_detach(&vcom_sem);
}

int vcom_init(void)
{
	rt_err_t err;
	rt_thread_t vcom_thread;
	struct serial_configure config;

	rt_sem_init(&vcom_sem, "vcom", 0, RT_IPC_FLAG_FIFO);
	vcom_dev = rt_device_find("vcom");

	if (!vcom_dev)
	{
		rt_kprintf("open device vcom failed\n");
	}

	config.baud_rate    = BAUD_RATE_115200;
	config.data_bits    = DATA_BITS_8;
	config.stop_bits    = STOP_BITS_1;
	config.parity       = PARITY_NONE;
	config.bit_order    = BIT_ORDER_LSB;
	config.invert       = NRZ_NORMAL;
	config.bufsz        = 1024;

	rt_device_control(vcom_dev, RT_DEVICE_CTRL_CONFIG, &config);

	err = rt_device_open(vcom_dev, RT_DEVICE_OFLAG_RDWR 
						| RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);
	if (err != RT_EOK)
	{
		rt_kprintf("open vcom failed\n");
		return -1;
	}

	vcom_thread = rt_thread_create("vcom",
								vcom_thread_entry, RT_NULL,
								512, 17, 20);

	if (vcom_thread != RT_NULL)
		rt_thread_startup(vcom_thread);

	return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

MSH_CMD_EXPORT_ALIAS(vcom_init, vcom_init, vcom app init);
#endif


