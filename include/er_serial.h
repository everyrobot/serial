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
//#include <er_comm.h>

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs

// **************************************************************************
// the globals
//extern volatile unsigned char      gRX_SCI_buf[1024];    // buffer RX
//extern volatile uint16_t           gRX_SCI_ptr;
//extern volatile uint16_t           g_flag_buffer_rx_is_changed;

extern ER_Buffer gBufferSerial;

//extern USER_Params gUserParams;
//extern volatile ER_Msg            gMsgCommand;
//extern volatile ER_Msg            gMsgResponse;

//extern volatile uint16_t rx_b_buf[1024];    // buffer RX
//extern volatile uint16_t rx_b_ptr                  = 0;
//extern volatile uint16_t flag_buffer_rx_is_changed = 0;
extern HAL_Handle halHandle;

// **************************************************************************
// the function prototypes
//void serial__configure(void);
void serial__scib_init(void);
//void serial__enable(bool enable);
interrupt void serial__scib_rx_isr(void);
interrupt void serial__scib_tx_isr(void);
//void serial__configure(void);
//void serial__enable(bool enable);
void serial__write(SCI_Handle handle, char * msg, uint16_t length);

#endif
