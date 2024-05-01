#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    unsigned int head;
    unsigned int tail;
    uint16_t* buffer;
    const unsigned int size;
} ring_buffer_t;

ring_buffer_t* ring_buffer_init(uint16_t* buffer, const unsigned int size);

int ring_buffer_push(ring_buffer_t*, uint16_t data) {
    
}

#endif // RING_BUFFER_H