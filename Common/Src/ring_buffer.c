/**
 * @file ring_buffer.c
 * @brief Byte-oriented circular buffer implementation.
 *
 * The UART bridge uses ring buffers between interrupt callbacks and the main
 * loop: ISR code pushes received bytes, and the main loop pops bytes when it
 * is ready to forward them.
 */
#include "ring_buffer.h"

/**
 * @brief Initialize a ring buffer with caller-provided storage.
 *
 * The caller owns the storage array so the buffer can be used without dynamic
 * memory allocation.
 */
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buf, uint16_t size)
{
    rb->buffer = buf;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief Push one byte into the ring buffer.
 *
 * This is used by UART RX callbacks to store newly received bytes. If the
 * buffer is full, the byte is dropped by design and the function returns 0.
 */
uint8_t RingBuffer_Push(RingBuffer_t *rb, uint8_t data)
{
    uint16_t next_head = rb->head + 1U;

    if (next_head >= rb->size)
    {
        next_head = 0;
    }

    if (RingBuffer_IsFull(rb))
    {
        return 0;
    }

    rb->buffer[rb->head] = data;
    rb->head = next_head;

    return 1;
}

/**
 * @brief Pop one byte from the ring buffer.
 *
 * This is used by the main loop to remove queued bytes before forwarding them
 * to the opposite UART transmit queue.
 */
uint8_t RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data)
{
    if (RingBuffer_IsEmpty(rb))
    {
        return 0;
    }

    *data = rb->buffer[rb->tail];
    rb->tail++;
    if (rb->tail >= rb->size)
    {
        rb->tail = 0;
    }

    return 1;
}

/**
 * @brief Return the number of bytes currently stored in the ring buffer.
 */
uint16_t RingBuffer_Available(const RingBuffer_t *rb)
{
    uint16_t head = rb->head;
    uint16_t tail = rb->tail;

    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(rb->size - tail + head);
}

/**
 * @brief Return whether the ring buffer cannot accept another byte.
 *
 * One slot is intentionally left unused so full and empty states can be
 * distinguished using only head and tail indexes.
 */
uint8_t RingBuffer_IsFull(const RingBuffer_t *rb)
{
    uint16_t next_head = rb->head + 1U;

    if (next_head >= rb->size)
    {
        next_head = 0;
    }

    return (next_head == rb->tail);
}

/**
 * @brief Return whether the ring buffer has no queued bytes.
 */
uint8_t RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    return (rb->head == rb->tail);
}
