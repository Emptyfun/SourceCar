#ifndef RING_BUFFER_H
#define RING_BUFFER_H

/**
 * @file ring_buffer.h
 * @brief Public interface for byte-oriented circular buffers.
 *
 * The Common layer holds reusable code that does not depend on board-specific
 * peripherals. Ring buffers decouple interrupt-driven UART reception from
 * main-loop forwarding: UART callbacks push bytes into RX buffers, and
 * App_Loop() processing pops bytes later without blocking the ISR.
 */

#include <stdint.h>

/**
 * @brief Ring buffer state and caller-owned storage.
 *
 * head and tail are volatile because they can be touched from both UART
 * callbacks and main-loop code.
 */
typedef struct
{
    uint8_t *buffer;
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t size;
} RingBuffer_t;

/**
 * @brief Initialize a ring buffer with caller-owned storage.
 */
void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buf, uint16_t size);

/**
 * @brief Push one byte; returns 0 when the buffer is full.
 */
uint8_t RingBuffer_Push(RingBuffer_t *rb, uint8_t data);

/**
 * @brief Pop one byte; returns 0 when the buffer is empty.
 */
uint8_t RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data);

/**
 * @brief Return the number of queued bytes.
 */
uint16_t RingBuffer_Available(const RingBuffer_t *rb);

/**
 * @brief Return whether the buffer has no free write slot.
 */
uint8_t RingBuffer_IsFull(const RingBuffer_t *rb);

/**
 * @brief Return whether the buffer has no queued bytes.
 */
uint8_t RingBuffer_IsEmpty(const RingBuffer_t *rb);

#endif /* RING_BUFFER_H */
