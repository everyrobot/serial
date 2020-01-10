#ifndef ER_BUFFER_H
#define ER_BUFFER_H

// **************************************************************************
// the includes
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include <er_ti_f28069m_drv8305/er_msg.h>

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//typedef unsigned char uint16_t;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes
void buffer__add_to_buffer(volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr,  volatile uint16_t * _flag_buffer_rx_is_changed, uint16_t value);
void buffer__remove_from_buffer (volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr, volatile uint16_t * _flag_buffer_rx_is_changed, uint16_t remove_signs);
bool buffer__analyse_buffer(volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr,  volatile uint16_t * _flag_buffer_rx_is_changed, volatile uint16_t * node_id, volatile ER_Msg * message);

#endif
