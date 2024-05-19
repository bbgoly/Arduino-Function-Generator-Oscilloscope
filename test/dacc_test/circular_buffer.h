#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// buffer holds constant pointer to an array of unsigned 16 bit integers to allow
// changes to be made to buffer, without accidentally orphaning the pointer
typedef struct {
    unsigned int head, tail;
    const unsigned int maxSize;
    uint16_t* const buffer;
} circular_buffer_t;

// Cicular buffer queue will only be used by one consumer and one producer in my case, so no lock is needed

circular_buffer_t* circular_buffer_init(unsigned int maxSize);

// unsigned int circular_buffer_maxSize(circular_buffer_t* cBuffer);
// unsigned int circular_buffer_size(circular_buffer_t* cBuffer);
bool circular_buffer_isEmpty(circular_buffer_t* cBuffer);
bool circular_buffer_isFull(circular_buffer_t* cBuffer);
void circular_buffer_clear(circular_buffer_t* cBuffer);

void circular_buffer_enqueue(circular_buffer_t* cBuffer, uint16_t data);
void circular_buffer_enqueue(circular_buffer_t* cBuffer, uint16_t data[], unsigned int size);
uint16_t circular_buffer_dequeue(circular_buffer_t* cBuffer);
// uint16_t* circular_buffer_dequeue(circular_buffer_t *cBuffer)

#endif // CIRCULAR_BUFFER_H