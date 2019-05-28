#include <stdlib.h>
#include <string.h>
#include "ring_buffer.h"

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
    rb->buffer_size = size;
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
        free(rb->buffer);
        free(rb);
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

