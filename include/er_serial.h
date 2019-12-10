#ifndef ER_SERIAL_H
#define ER_SERIAL_H

// **************************************************************************
// the includes
#include <string.h>
#include <stdlib.h>

#include "sw/drivers/sci/src/32b/f28x/f2806x/sci.h"
#include "main_position.h"

#include <er_globals.h>
#include <er_ti_f28069m_drv8305/er_buffer.h>
#include <er_comm.h>

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

//typedef unsigned char uint16_t;

// **************************************************************************
// the globals
//extern USER_Params gUserParams;
//extern volatile ER_Msg            gMsgCommand;
//extern volatile ER_Msg            gMsgResponse;

//extern volatile uint16_t rx_b_buf[1024];    // buffer RX
//extern volatile uint16_t rx_b_ptr                  = 0;
//extern volatile uint16_t flag_buffer_rx_is_changed = 0;

// **************************************************************************
// the function prototypes
//void serial__read(SCI_Handle handle, volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr);
void serial__write(SCI_Handle handle, char * msg, uint16_t length);
//void serial__add_to_buffer(uint16_t value);
//void serial__analyse_buffer(SCI_Handle handle);

#endif
