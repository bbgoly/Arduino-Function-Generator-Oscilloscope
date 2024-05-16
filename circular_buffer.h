/* Simple implementation of a circular buffer (aka ring buffer or circular queue)
 * By Yousif Kndkji
 *
 * Implemented in mostly C, but makes use of C++ dynamic allocation. Being able to 
 * use dynamic allocation despite it usually being avoided in embedded systems is that 
 * there is never any intention to free the memory allocated by the buffer at any point in
 * the lifetime of the program, thus making it almost like a static allocation of an array.
 * 
 * Because of the intention to never free that memory, the allocated space for the buffer
 * will stay in memory throughout the lifetime of the program, which is perfectly fine
 * in cases where the buffer is meant to be continuously used like in this case.
 *
 * Additionally, the circular buffer will only ever be used by one consumer and
 * one producer in my case, so no lock is needed at all, which considerably simplifies
 * the implementation. I also do not need to worry about making the buffer full and
 * potentially losing data as I continuously read the buffer as it's being written to.
 */

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <string.h>

// Cicular buffer queue will only be used by one consumer and one producer in my case, so no lock is needed
typedef struct {
    unsigned int head, tail;
    const unsigned int maxSize;
    uint16_t* const buffer;
} circular_buffer_t;


circular_buffer_t* circular_buffer_init(unsigned int maxSize);

bool circular_buffer_isEmpty(circular_buffer_t* cBuffer);
bool circular_buffer_isFull(circular_buffer_t* cBuffer);
void circular_buffer_clear(circular_buffer_t* cBuffer);

int circular_buffer_enqueue(circular_buffer_t* cBuffer, uint16_t data);
void circular_buffer_enqueue(circular_buffer_t* cBuffer, uint16_t data[], unsigned int size);
int16_t circular_buffer_dequeue(circular_buffer_t* cBuffer);
void circular_buffer_dequeue(circular_buffer_t *cBuffer, uint16_t data[], unsigned int size);

#endif // CIRCULAR_BUFFER_H