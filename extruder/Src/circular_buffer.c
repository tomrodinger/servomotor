#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "circular_buffer.h"

circular_buf_t* circular_buf_init(uint16_t *buffer, uint16_t capacity) {
  circular_buf_t *cbuf = malloc(sizeof(circular_buf_t));
  cbuf->buffer = buffer;
  cbuf->capacity = capacity;
  cbuf->head = 0;
  cbuf->tail = 0;
  cbuf->full = 0;
  return cbuf;
}

void circular_buf_reset(circular_buf_t *cbuf) {
  cbuf->head = 0;
  cbuf->tail = 0;
  cbuf->full = 0;
}

void circular_buf_free(circular_buf_t *cbuf) {
  free(cbuf);
}

uint8_t circular_buf_full(circular_buf_t *cbuf) {
  return (cbuf->full);
}

uint8_t circular_buf_empty(circular_buf_t *cbuf) {
  return (!cbuf->full && (cbuf->head == cbuf->tail));
}

uint16_t circular_buf_size(circular_buf_t *cbuf) {
  uint16_t size = cbuf->capacity;
  if(!cbuf->full) {
    if(cbuf->head >= cbuf->tail) {
      size = cbuf->head - cbuf->tail;
    }
    else {
      size = (cbuf->capacity + cbuf->head - cbuf->tail);
    }
  }
  return size;
}

void advance_pointer(circular_buf_t *cbuf) {
	if(cbuf->full) {
		if(++(cbuf->tail) == cbuf->capacity) {
			cbuf->tail = 0;
		}
	}
	if(++(cbuf->head) == cbuf->capacity) {
		cbuf->head = 0;
	}
	cbuf->full = (cbuf->head == cbuf->tail);
}

void retreat_pointer(circular_buf_t *cbuf) {
	cbuf->full = 0;
	if(++(cbuf->tail) == cbuf->capacity) {
		cbuf->tail = 0;
	}
}

void circular_buf_put(circular_buf_t *cbuf, uint16_t data) {
  cbuf->buffer[cbuf->head] = data;
  advance_pointer(cbuf);
}

uint16_t circular_buf_get(circular_buf_t *cbuf) {
	if(!circular_buf_empty(cbuf)) {
		return cbuf->buffer[cbuf->tail];
	}
	return 0;
}

//int circular_buf_get(circular_buf_t *cbuf, uint16_t *data) {
//  uint8_t res = -1;
//  if(!circular_buf_empty(cbuf)) {
//    *data = cbuf->buffer[cbuf->tail];
//    retreat_pointer(cbuf);
//    res = 1;
//  } else {
//    printf("buffer is empty\n");
//  }
//  return res;
//}
