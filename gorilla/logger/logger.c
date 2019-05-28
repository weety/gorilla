#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <dfs_fs.h>
#include "ring_buffer.h"
#include "logger.h"

const struct logger_structure logger_struct[] = {
	{HEAD_BYTE1, HEAD_BYTE2, MSG_ID_FMT, MSG_ID_FMT, sizeof(struct logger_structure), "FMT", 
	"BBnNZ", "Type,Length,Format,lables"},
	{HEAD_BYTE1, HEAD_BYTE2, MSG_ID_FMT, MSG_ID_IMU, sizeof(log_imu), "IMU", 
	"Qffffff", "TimeUs,AccX,AccY,AccZ,GyrX,GyrY,GyrZ"},
	{HEAD_BYTE1, HEAD_BYTE2, MSG_ID_FMT, MSG_ID_QENC, sizeof(log_qenc), "QENC", 
	"Qfff", "TimeUs,lSpeed,rSpeed,Speed"},
	{HEAD_BYTE1, HEAD_BYTE2, MSG_ID_FMT, MSG_ID_PARM, sizeof(log_parm), "PARM", 
	"QNf", "TimeUs,Name,Value"},
	{HEAD_BYTE1, HEAD_BYTE2, MSG_ID_FMT, MSG_ID_KF, sizeof(log_kf), "KF", 
	"Qffff", "TimeUs,Angle,Speed,QBias,AngleAcc"},
};


#define LOG_BUF_SIZE 2048

struct logger_buffer {
	struct ringbuffer rb;
	uint8_t buffer[LOG_BUF_SIZE];
	struct rt_mutex lock;
};

static struct logger_buffer _log_buf;

int logger_write(uint8_t msg_id, uint8_t len ,uint8_t *data)
{
	struct PACKED logger_msg_head
	{
		LOG_PACKET_HEADER;
		uint8_t type;
		uint8_t length;
	};

	struct logger_msg_head msg_head = {HEAD_BYTE1, HEAD_BYTE2, msg_id, len};

	rt_mutex_take(&(_log_buf.lock), RT_WAITING_FOREVER);
	ringbuffer_put(&(_log_buf.rb), (const uint8_t *)&msg_head, sizeof(msg_head));
	ringbuffer_put(&(_log_buf.rb), data, len);
	rt_mutex_release(&(_log_buf.lock));

	return len;
}

int logger_init(void)
{
	ringbuffer_init(&(_log_buf.rb), _log_buf.buffer, LOG_BUF_SIZE);
	rt_mutex_init(&(_log_buf.lock), "logger", RT_IPC_FLAG_FIFO);
}

