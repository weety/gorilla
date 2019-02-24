#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <stdint.h>

#define UDISK_DEV_NAME "ud0-0"
#define LOGGER_FILE_PATH "/"

#define IMU "imu"
#define KF  "kf"
#define CONTROL "control"

#define MSG_NAME_FMT    "FMT"
#define MSG_NAME_IMU    "IMU"
#define MSG_NAME_QENC   "QENC"
#define MSG_NAME_PARM   "PARM"
#define MSG_NAME_KF     "KF"
#define MSG_NAME_CNTL   "CNTL"

#define MSG_ID_FMT   0x10
#define MSG_ID_IMU   0x12
#define MSG_ID_QENC  0x13
#define MSG_ID_PARM  0x14
#define MSG_ID_KF    0x15
#define MSG_ID_CNTL  0x16

/*
Format characters in the format string for binary log messages
  a   : int16_t[32]
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  d   : double
  n   : char[4]
  N   : char[16]
  Z   : char[64]
  c   : int16_t * 100
  C   : uint16_t * 100
  e   : int32_t * 100
  E   : uint32_t * 100
  q   : int64_t
  Q   : uint64_t
 */

typedef enum {
	LOG_UINT8 = 0,
	LOG_INT8,
	LOG_UINT16,
	LOG_INT16,
	LOG_UINT32,
	LOG_INT32,
	LOG_UINT64,
	LOG_INT64,
	LOG_FLOAT,
	LOG_DOUBLE,
} LOG_TYPE_E;

#define PACKED __attribute__((packed))

struct PACKED logger_structure {
	uint8_t msg_type;
	uint8_t msg_len;
	const char name[6];
	const char format[16];
	const char labels[64];
};

#define LOG_PACKET_HEADER_LEN 3
#define LOG_PACKET_HEADER         uint8_t head1, head2, msgid;
/* once the logging code is all converted we will remove these from
 * this header
 */
#define HEAD_BYTE1  0xE1
#define HEAD_BYTE2  0x96

struct PACKED logger_format
{
    LOG_PACKET_HEADER;
    uint8_t type;
    uint8_t length;
    char name[4];
    char format[16];
    char labels[64];
};

struct PACKED log_imu {
	uint64_t timeUs;
	float accx;
	float accy;
	float accz;
	float gyrx;
	float gyry;
	float gyrz;
};

struct PACKED log_qenc {
	uint64_t timeUs;
	float lspeed;
	float rspeed;
	float speed;
};

struct PACKED log_parm {
	uint64_t timeUs;
	char name[16];
	float value;
};

struct PACKED log_kf {
	uint64_t timeUs;
	float angle;
	float speed;
	float Q_bias;
	float angle_acc;
};


#endif

