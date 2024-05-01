#include "RingBuffer.h"

// ring_buffer_t* ring_buffer_init(uint16_t* buffer, const unsigned int size) {
//     ring_buffer_t buffer = {0, 0, buffer, size};
//     return &buffer;
// }

int ring_buffer_push(ring_buffer_t*, uint16_t data);