#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <stdint.h>

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

uint32_t ringbuffer_get_size(struct ringbuffer *rb);
enum ringbuffer_state ringbuffer_status(struct ringbuffer *rb);
uint32_t ringbuffer_data_len(struct ringbuffer *rb);
void ringbuffer_init(struct ringbuffer *rb,
                        uint8_t           *buf,
                        uint32_t            size);
struct ringbuffer* ringbuffer_alloc(uint32_t size);
void ringbuffer_free(struct ringbuffer *rb);
void ringbuffer_reset(struct ringbuffer *rb);
uint32_t ringbuffer_put(struct ringbuffer *rb,
                            const uint8_t     *ptr,
                            uint32_t           length);
uint32_t ringbuffer_put_force(struct ringbuffer *rb,
                            const uint8_t     *ptr,
                            uint32_t           length);
uint32_t ringbuffer_get(struct ringbuffer *rb,
                            uint8_t           *ptr,
                            uint32_t           length);

#endif

