#include "circular_buffer.h"

circular_buffer_t* circular_buffer_init(unsigned int maxSize) {
    circular_buffer_t* cBuffer = new circular_buffer_t{ 0, 0, maxSize, new uint16_t[maxSize] };
    // cBuffer->buffer = malloc(sizeof(*cBuffer->buffer) * maxSize);
    // cBuffer->maxSize = maxSize;
    return cBuffer;
}

// https://www.youtube.com/watch?v=-5rAjOjTGtc&list=RDouUeRx0OHPw&index=27

bool circular_buffer_isEmpty(circular_buffer_t* cBuffer) {
    return cBuffer->head == cBuffer->tail;
}

bool circular_buffer_isFull(circular_buffer_t* cBuffer) {
    return cBuffer->head + 1 == cBuffer->tail;
}

void circular_buffer_clear(circular_buffer_t* cBuffer) {
    cBuffer->head = 0;
    cBuffer->tail = 0;
}

int circular_buffer_enqueue(circular_buffer_t* cBuffer, uint16_t data) {
    int nextPosition = (cBuffer->head + 1) & (cBuffer->maxSize - 1);  // assumes maxSize is power of 2
    if (nextPosition == cBuffer->tail) return -1;

    cBuffer->buffer[cBuffer->head] = data;
    cBuffer->head = nextPosition;
    return 0;
}

void circular_buffer_enqueue(circular_buffer_t* cBuffer, uint16_t data[], unsigned int size) {
    // no need to check if adding array of size n will overflow since we will be appending size that is multiple of size of buffer
    memcpy(&cBuffer->buffer[cBuffer->head], data, 2 * size);  // memcpy works on bytes, we're working with 2 bytes
    cBuffer->head = (cBuffer->head + size) & (cBuffer->maxSize - 1);
}

int16_t circular_buffer_dequeue(circular_buffer_t* cBuffer) {
    if (cBuffer->head == cBuffer->tail) return -1;

    uint16_t data = cBuffer->buffer[cBuffer->tail];
    cBuffer->tail = (cBuffer->tail + 1) & (cBuffer->maxSize - 1);
    return data;
}

void circular_buffer_dequeue(circular_buffer_t* cBuffer, uint16_t data[], unsigned int size) {
    memcpy(data, &cBuffer->buffer[cBuffer->tail], 2 * size);
    cBuffer->tail = (cBuffer->tail + size) & (cBuffer->maxSize - 1);
}