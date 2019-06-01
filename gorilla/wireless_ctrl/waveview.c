#include <rtthread.h>
#include <rtdevice.h>
#include "ring_buffer.h"
#include "waveview.h"
#include "mpdc.h"
#include "sensor.h"
#include "attitude_estimation.h"
#include "att_control.h"

#define WAVEVIEW_DEVICE "uart3"
static rt_device_t waveview_dev;
#define WAVE_EVT_RX      (1u << 0)
#define WAVE_EVT_TIMEOUT (1u << 1)
#define WAVE_EVT_TX_DONE (1u << 2)
static struct rt_event wave_evt;
static struct rt_timer wave_timer;
#define WAVE_SEND_PERIOD (10) /* 10ms */
#define WAVE_UART_SEND_TIMEOUT (10) /* 10ms timeout for send 10 bytes data */
#define WAVE_BUF_SIZE 64

struct wave_buffer {
	struct ringbuffer rb;
	uint8_t buffer[WAVE_BUF_SIZE];
	struct rt_mutex lock;
};

static struct wave_buffer _wave_buf;

extern sensor_t sensor_acc;
extern sensor_t sensor_gyr;
extern sensor_t sensor_qenc;
extern att_t att_angle;
extern ctrl_t ctrl_param;

rt_err_t wave_write_data(uint8_t *buf, uint32_t size);

uint16_t CRC16_CHECK(uint8_t *Buf, uint8_t CRC_CNT)
{
	uint16_t CRC_Temp;
	uint8_t i,j;
	CRC_Temp = 0xffff;

	for (i = 0; i < CRC_CNT; i++)
	{
		CRC_Temp ^= Buf[i];
		for (j = 0; j < 8; j++)
		{
			if (CRC_Temp & 0x01)
				CRC_Temp = (CRC_Temp >> 1 ) ^ 0xa001;
			else
				CRC_Temp = CRC_Temp >> 1;
		}
	}
	return CRC_Temp;
}

void OutPut_Data(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4)
{
	uint16_t temp[4] = {0};
	uint8_t databuf[10] = {0};
	uint8_t i;
	uint16_t CRC16 = 0;
	temp[0] = ch1;
	temp[1] = ch2;
	temp[2] = ch3;
	temp[3] = ch4;

	for(i = 0; i < 4; i++) 
	{
		databuf[i<<1]   = (unsigned char)(temp[i] & 0xff);
		databuf[(i<<1)+1] = (unsigned char)((temp[i] & 0xff00) >> 8);
	}

	CRC16 = CRC16_CHECK(databuf,8);
	databuf[8] = CRC16 & 0xff;
	databuf[9] =(CRC16 & 0xff00) >> 8;

	wave_write_data(databuf, 10);
}

rt_err_t wave_write_data(uint8_t *buf, uint32_t size)
{
	rt_size_t count = 0;
	rt_err_t  err = RT_EOK;
	count = rt_device_write(waveview_dev, 0, buf, size);
	err = rt_event_recv(&wave_evt, WAVE_EVT_TX_DONE, 
		RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 
		rt_tick_from_millisecond(WAVE_UART_SEND_TIMEOUT), 
		RT_NULL);
	if (err != RT_EOK)
	{
		rt_kprintf("wave write data timeout\n");
		return 0;
	}

	return count;
}

static void wave_timer_update(void* parameter)
{
	rt_event_send(&wave_evt, WAVE_EVT_TIMEOUT);
}

rt_err_t wave_tx_done(rt_device_t dev, void *buffer)
{
	return rt_event_send(&wave_evt, WAVE_EVT_TX_DONE);
}

rt_err_t wave_rx_ind(rt_device_t dev, rt_size_t size)
{
	rt_event_send(&wave_evt, WAVE_EVT_RX);

	return RT_EOK;
}

void wave_thread_entry(void* parameter)
{
	rt_err_t err;
	rt_size_t size = 0;
	rt_uint32_t evt_recved = 0;
	rt_uint8_t buf[32];
	uint32_t ret = 0;
	uint32_t pAddr1 = 0, pAddr2 = 0, pAddr3 = 0, pAddr4 = 0;
	uint16_t CRC_RX, CRC_Tmp;
	uint16_t ch1 = 0;
	uint16_t ch2 = 0;
	uint16_t ch3 = 0;
	uint16_t ch4 = 0;

	union {
		sensor_gyr_t gyr;
		sensor_acc_t acc;
		sensor_qenc_t qenc;
		att_angle_t  angle;
		ctrl_param_t param;
	} chn_data;

	rt_device_set_rx_indicate(waveview_dev, wave_rx_ind);
	rt_device_set_tx_complete(waveview_dev, wave_tx_done);

	while(1)
	{
		err = rt_event_recv(&wave_evt, WAVE_EVT_RX | WAVE_EVT_TIMEOUT, 
			RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, 
			&evt_recved);
		if (err != RT_EOK)
		{
			rt_kprintf("wait evt wave error\n");
			continue;
		}

		if (evt_recved & WAVE_EVT_TIMEOUT)
		{
			mpdc_pull_data(att_angle.mpdc, &chn_data.angle);
			ch1 = (uint16_t)(chn_data.angle.angle_acc * 40000);
			ch2 = (uint16_t)(chn_data.angle.angle * 40000);
			ch3 = (uint16_t)(chn_data.angle.speed * 40000);
			mpdc_pull_data(ctrl_param.mpdc, &chn_data.param);
			ch4 = (uint16_t)(chn_data.param.pwm_l_out * 40000);
			OutPut_Data(ch1, ch2, ch3, ch4);
		}

		if (evt_recved & WAVE_EVT_RX)
		{

			size = rt_device_read(waveview_dev, 0, buf, 32);
			if (size > 0)
			{
				ret = ringbuffer_put(&_wave_buf.rb, buf, size);
				if (ret < size)
				{
					rt_kprintf("wave rb full\n");
				}
			}

			ret = ringbuffer_data_len(&_wave_buf.rb);
			if (ret >= 18)
			{
				ringbuffer_get(&_wave_buf.rb, buf, 18);
				CRC_Tmp =  CRC16_CHECK(buf, 16);	//CRC Calculation
				CRC_RX = ((unsigned short)buf[17] << 8) + buf[16];
				if(CRC_Tmp == CRC_RX)
				{
					pAddr1 = ((uint32_t)(buf[0x3])<<24)|((uint32_t)(buf[0x2])<<16)|((uint32_t)(buf[0x1])<<8)|buf[0x0];
					pAddr2 = ((uint32_t)(buf[0x7])<<24)|((uint32_t)(buf[0x6])<<16)|((uint32_t)(buf[0x5])<<8)|buf[0x4];
					pAddr3 = ((uint32_t)(buf[0xB])<<24)|((uint32_t)(buf[0xA])<<16)|((uint32_t)(buf[0x9])<<8)|buf[0x8];
					pAddr4 = ((uint32_t)(buf[0xF])<<24)|((uint32_t)(buf[0xE])<<16)|((uint32_t)(buf[0xD])<<8)|buf[0xC];
					rt_kprintf("pAddr1=%08x, pAddr2=%08x, pAddr3=%08x, pAddr4=%08x\n", pAddr1, pAddr2, pAddr3, pAddr4);
				}
			}
		}

	}

	rt_device_set_rx_indicate(waveview_dev, RT_NULL);
	rt_device_set_tx_complete(waveview_dev, RT_NULL);
	rt_device_close(waveview_dev);
	rt_event_detach(&wave_evt);
}

static void wave_buf_init(void)
{
	ringbuffer_init(&(_wave_buf.rb), _wave_buf.buffer, WAVE_BUF_SIZE);
	rt_mutex_init(&(_wave_buf.lock), "wave", RT_IPC_FLAG_FIFO);
}


int waveview_init(void)
{
	rt_err_t err;
	rt_thread_t wave_thread;
	struct serial_configure config;

	wave_buf_init();
	rt_event_init(&wave_evt, "wave", RT_IPC_FLAG_FIFO);
	waveview_dev = rt_device_find(WAVEVIEW_DEVICE);

	if (!waveview_dev)
	{
		rt_kprintf("open device %s failed\n", WAVEVIEW_DEVICE);
	}

	config.baud_rate    = BAUD_RATE_57600;
	config.data_bits    = DATA_BITS_8;
	config.stop_bits    = STOP_BITS_1;
	config.parity       = PARITY_NONE;
	config.bit_order    = BIT_ORDER_LSB;
	config.invert       = NRZ_NORMAL;
	config.bufsz        = 128;

	rt_device_control(waveview_dev, RT_DEVICE_CTRL_CONFIG, &config);

	err = rt_device_open(waveview_dev, RT_DEVICE_OFLAG_RDWR 
						| RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX);
	if (err != RT_EOK)
	{
		rt_kprintf("open %s failed\n", WAVEVIEW_DEVICE);
		return -1;
	}

	wave_thread = rt_thread_create("wave",
								wave_thread_entry, RT_NULL,
								1024, 20, 20);

	if (wave_thread != RT_NULL)
		rt_thread_startup(wave_thread);

	/* register timer event */
	rt_timer_init(&wave_timer, "wave",
	              wave_timer_update,
	              RT_NULL,
	              rt_tick_from_millisecond(WAVE_SEND_PERIOD),
	              RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);
	rt_timer_start(&wave_timer);

	return 0;
}


#ifdef RT_USING_FINSH
#include <finsh.h>

MSH_CMD_EXPORT_ALIAS(waveview_init, waveview_init, wave view app init);
#endif

