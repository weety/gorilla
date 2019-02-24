#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <dfs_fs.h>
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

struct ringbuffer
{
    uint8_t *buffer;
    /* use the msb of the {read,write}_index as mirror bit. You can see this as
     * if the buffer adds a virtual mirror and the pointers point either to the
     * normal or to the mirrored buffer. If the write_index has the same value
     * with the read_index, but in a different mirror, the buffer is full.
     * While if the write_index and the read_index are the same and within the
     * same mirror, the buffer is empty. The ASCII art of the ringbuffer is:
     *
     *          mirror = 0                    mirror = 1
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Full
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     *  read_idx-^                   write_idx-^
     *
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 | Empty
     * +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
     * read_idx-^ ^-write_idx
     *
     * The tradeoff is we could only use 32KiB of buffer for 16 bit of index.
     * But it should be enough for most of the cases.
     *
     * Ref: http://en.wikipedia.org/wiki/Circular_buffer#Mirroring */
    uint32_t read_mirror : 1;
    uint32_t read_index : 31;
    uint32_t write_mirror : 1;
    uint32_t write_index : 31;
    /* as we use msb of index as mirror bit, the size should be signed and
     * could only be positive. */
    uint32_t buffer_size;
};

enum ringbuffer_state
{
    RINGBUFFER_EMPTY,
    RINGBUFFER_FULL,
    /* half full is neither full nor empty */
    RINGBUFFER_HALFFULL,
};

uint32_t ringbuffer_get_size(struct ringbuffer *rb)
{
    return rb->buffer_size;
}

/** return the size of empty space in rb */
#define ringbuffer_space_len(rb) ((rb)->buffer_size - ringbuffer_data_len(rb))

enum ringbuffer_state ringbuffer_status(struct ringbuffer *rb)
{
    if (rb->read_index == rb->write_index)
    {
        if (rb->read_mirror == rb->write_mirror)
            return RINGBUFFER_EMPTY;
        else
            return RINGBUFFER_FULL;
    }
    return RINGBUFFER_HALFFULL;
}

uint32_t ringbuffer_data_len(struct ringbuffer *rb)
{
    switch (ringbuffer_status(rb))
    {
    case RINGBUFFER_EMPTY:
        return 0;
    case RINGBUFFER_FULL:
        return rb->buffer_size;
    case RINGBUFFER_HALFFULL:
    default:
        if (rb->write_index > rb->read_index)
            return rb->write_index - rb->read_index;
        else
            return rb->buffer_size - (rb->read_index - rb->write_index);
    }
}


void ringbuffer_init(struct ringbuffer *rb,
                        uint8_t           *buf,
                        uint32_t            size)
{
    /* initialize read and write index */
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;

    /* set buffer pool and size */
    rb->buffer = buf;
    rb->buffer_size = size);
}

struct ringbuffer* ringbuffer_alloc(uint32_t size)
{
    struct ringbuffer *rb;
    uint8_t *buf;

    rb = malloc(sizeof(struct ringbuffer));
    if (rb == NULL)
        goto exit;

    buf = malloc(size);
    if (buf == NULL)
    {
        free(rb);
        rb = NULL;
        goto exit;
    }
    ringbuffer_init(rb, buf, size);

exit:
    return rb;
}

void ringbuffer_free(struct ringbuffer *rb)
{
    if (rb != NULL)
    {
        rt_free(rb->buffer);
        rt_free(rb);
    }
}

void ringbuffer_reset(struct ringbuffer *rb)
{
    rb->read_mirror = 0;
    rb->read_index = 0;
    rb->write_mirror = 0;
    rb->write_index = 0;
}

uint32_t ringbuffer_put(struct ringbuffer *rb,
                            const uint8_t     *ptr,
                            uint32_t           length)
{
    uint32_t size;

    /* whether has enough space */
    size = ringbuffer_space_len(rb);

    /* no space */
    if (size == 0)
        return 0;

    /* drop some data */
    if (size < length)
        length = size;

    if (rb->buffer_size - rb->write_index > length)
    {
        /* read_index - write_index = empty space */
        memcpy(&rb->buffer[rb->write_index], ptr, length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rb->write_index += length;
        return length;
    }

    memcpy(&rb->buffer[rb->write_index],
           &ptr[0],
           rb->buffer_size - rb->write_index);
    memcpy(&rb->buffer[0],
           &ptr[rb->buffer_size - rb->write_index],
           length - (rb->buffer_size - rb->write_index));

    /* we are going into the other side of the mirror */
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index = length - (rb->buffer_size - rb->write_index);

    return length;
}

uint32_t ringbuffer_put_force(struct ringbuffer *rb,
                            const uint8_t     *ptr,
                            uint32_t           length)
{
    uint32_t space_length;

    space_length = ringbuffer_space_len(rb);

    if (length > rb->buffer_size)
    {
        ptr = &ptr[length - rb->buffer_size];
        length = rb->buffer_size;
    }

    if (rb->buffer_size - rb->write_index > length)
    {
        /* read_index - write_index = empty space */
        memcpy(&rb->buffer[rb->write_index], ptr, length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rb->write_index += length;

        if (length > space_length)
            rb->read_index = rb->write_index;

        return length;
    }

    memcpy(&rb->buffer[rb->write_index],
           &ptr[0],
           rb->buffer_size - rb->write_index);
    memcpy(&rb->buffer[0],
           &ptr[rb->buffer_size - rb->write_index],
           length - (rb->buffer_size - rb->write_index));

    /* we are going into the other side of the mirror */
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index = length - (rb->buffer_size - rb->write_index);

    if (length > space_length)
    {
        rb->read_mirror = ~rb->read_mirror;
        rb->read_index = rb->write_index;
    }

    return length;
}

uint32_t ringbuffer_get(struct ringbuffer *rb,
                            uint8_t           *ptr,
                            uint32_t           length)
{
    uint32_t size;

    /* whether has enough data  */
    size = ringbuffer_data_len(rb);

    /* no data */
    if (size == 0)
        return 0;

    /* less data */
    if (size < length)
        length = size;

    if (rb->buffer_size - rb->read_index > length)
    {
        /* copy all of data */
        memcpy(ptr, &rb->buffer[rb->read_index], length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rb->read_index += length;
        return length;
    }

    memcpy(&ptr[0],
           &rb->buffer[rb->read_index],
           rb->buffer_size - rb->read_index);
    memcpy(&ptr[rb->buffer_size - rb->read_index],
           &rb->buffer[0],
           length - (rb->buffer_size - rb->read_index));

    /* we are going into the other side of the mirror */
    rb->read_mirror = ~rb->read_mirror;
    rb->read_index = length - (rb->buffer_size - rb->read_index);

    return length;
}

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

