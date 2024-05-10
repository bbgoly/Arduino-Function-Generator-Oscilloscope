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