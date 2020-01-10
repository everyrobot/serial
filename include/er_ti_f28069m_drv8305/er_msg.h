#ifndef ER_MSG_H
#define ER_MSG_H

// **************************************************************************
// the includes

#include <stdint.h>   // needed for C99 data types
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include <er_ti_f28069m_drv8305/er_registers.h>
#include <er_ti_f28069m_drv8305/er_buffer.h>

// **************************************************************************
// the defines

#define MSG__COMMAND      0b00000000
#define MSG__RESPONSE     0b10000000

#define MSG__OK             0b00000000
#define MSG__BAD_REGISTER   0b01000000
#define MSG__BAD_BODY_CRC   0b00100000 // command body has incorrect CRC
#define MSG__BAD_BODY       0b00010000 // command body has incorrect data for register
#define MSG__ALLOC_FAILED   0b00001000 // body allocation failed
#define MSG__BAD_HEADER_CRC 0b00000100 // command header has incorrect CRC

#define ER_Msg_INIT {0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         0, \
                         NULL, \
                         0}

// **************************************************************************
// the typedefs
typedef struct _ER_Msg_t_
{
    uint16_t source_id;
    uint16_t destination_id;
    uint16_t register_id;
    uint16_t flags;
    uint16_t body_size;
    uint16_t header_crc;
    char * body;
    uint16_t body_crc;
} ER_Msg;

// **************************************************************************
// the globals
//extern USER_Params gUserParams;
//extern volatile ER_Msg            gMsgCommand;
//extern volatile ER_Msg            gMsgResponse;
extern ER_Buffer gBufferSerial;
extern volatile  uint16_t          gFlag;

// **************************************************************************
// the function prototypes
//void      msg__clearMessages();
void      msg__clearMessage(volatile ER_Msg * message);

uint16_t  msg__calculate_header_crc(volatile ER_Msg * message);
uint16_t  msg__calculate_body_crc(volatile ER_Msg * message);
void      msg__set_header(volatile ER_Msg * message, uint16_t source_id, uint16_t destination_id, uint16_t register_id, uint16_t flags, uint16_t body_size);
void      msg__set_body(volatile ER_Msg * message, char * body, uint16_t body_size);

char    * msg__to_byte_array(volatile ER_Msg * message);
uint16_t  msg__get_length(volatile ER_Msg * message);

bool      msg__check(volatile ER_Msg * message);
bool      msg__create(volatile ER_Msg * message);

uint16_t msg__get_uint16_from_body(volatile ER_Msg *message, uint16_t start);
//int16_t   msg__get_int16_from_body(uint16_t start);
int32_t   msg__get_int32_from_body(volatile ER_Msg * message, uint16_t start);
float     msg__get_float_from_body(volatile ER_Msg * message, uint16_t start);

void      msg__add_uint16_to_body(volatile ER_Msg * message,uint16_t value);
//void      msg__add_int16_to_body(int16_t value);
void      msg__add_int32_to_body(volatile ER_Msg * message, int32_t value);

int32_t   msg__convert_float_to_iq(float float_data, int n);
float     msg__convert_iq_to_float(long iq_data, int n);

bool      msg__analyse_buffer(volatile uint16_t * node_id, volatile ER_Msg * message);

#endif
