/* Implementation of a Circular/Ring Buffer
 * By Yousif Kndkji
 */

#ifndef CIRC_BUFFER_H
#define CIRC_BUFFER_H

#include <stdint.h>

#define CIRC_BUFFER_DEF(name, size)     \
    uint16_t name##_data[size];         \
    circ_buffer_t name = {              \
        .head = 0,                      \
        .tail = 0,                      \
        .buffer = name##_data,          \
        .len = size                     \
    }

typedef struct {
    int head;
    int tail;
    uint16_t* const buffer;
    const unsigned int len;
} circ_buffer_t;

int circ_buffer_push(circ_buffer_t* c, uint16_t data) {
    int next = c->head + 1;
    if (next >= c->len) {
        next = 0;
    }

    if (next == c->tail) {
        return -1;
    }
    
    c->buffer[c->head] = data;
    c->head = next;
    return 0;
}

int circ_buffer_pop(circ_buffer_t* c, uint16_t* data) {
    if (c->head == c->tail) {
        return -1;
    }

    int next = c->tail + 1;
    if (next >= c->len) {
        next = 0;
    }

    *data = c->buffer[c->tail];
    c->tail = next;
    return 0;
}

bool circ_buffer_is_empty(circ_buffer_t* c) {
    return c->head == c->tail;
}

#endif // CIRC_BUFFER_H