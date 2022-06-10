#ifndef __CIRCULAR_BUFFER__
#define __CIRCULAR_BUFFER__

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

struct circular_buf_t {
  uint16_t *buffer;
  uint16_t capacity;
  uint16_t head, tail;
  uint8_t full;
};

typedef struct circular_buf_t circular_buf_t;

circular_buf_t* circular_buf_init(uint16_t *buffer, uint16_t capacity);

void circular_buf_reset(circular_buf_t *cbuf);

void circular_buf_free(circular_buf_t *cbuf);

uint8_t circular_buf_full(circular_buf_t *cbuf);

uint8_t circular_buf_empty(circular_buf_t *cbuf);

uint16_t circular_buf_size(circular_buf_t *cbuf);

void advance_pointer(circular_buf_t *cbuf);

void retreat_pointer(circular_buf_t *cbuf);

void circular_buf_put(circular_buf_t *cbuf, uint16_t data);

uint16_t circular_buf_get(circular_buf_t *cbuf);

//int circular_buf_get(circular_buf_t *cbuf, uint16_t *data);

#endif
