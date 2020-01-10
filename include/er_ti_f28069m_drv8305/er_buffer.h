#ifndef ER_BUFFER_H
#define ER_BUFFER_H

// **************************************************************************
// the includes
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

typedef volatile struct _er_buffer_struct_ {
    uint16_t *buf;
    size_t buff_size;
    bool buff_is_changed;
    uint16_t length;
} ER_Buffer;

//! \brief Initialization values of global variables
//!
#define ER_Buffer_INIT {NULL, \
                        0, \
                        0, \
                        0}

//typedef unsigned char uint16_t;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes
void        buffer__add_to_buffer(ER_Buffer *channel_buff, uint16_t value);
void        buffer__remove_from_buffer (ER_Buffer *channel_buff, uint16_t remove_size);
void        buffer__destroy(ER_Buffer *channel_buff);
ER_Buffer   buffer__create(size_t buff_size);
void        buffer__resize(ER_Buffer *channel_buff, size_t size);
uint16_t    buffer__median(ER_Buffer *channel_buff);

#endif
