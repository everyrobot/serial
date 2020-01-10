#ifndef ER_COMMAND_H
#define ER_COMMAND_H

// **************************************************************************
// the includes

//#include <stdint.h>   // needed for C99 data types
//#include <stdlib.h>
//#include <stdbool.h>
//#include <math.h>
//#include <string.h>

#include <er_ti_f28069m_drv8305/er_registers.h>
#include <er_ti_f28069m_drv8305/er_msg.h>

// **************************************************************************
// the defines

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes
ER_Msg      command__create_msg_set_position(uint16_t source_id, uint16_t node_id, float position);
ER_Msg      command__create_msg_get_position(uint16_t source_id, uint16_t node_id);
ER_Msg      command__create_msg_ping(uint16_t source_id, uint16_t node_id);

// Tactile Module
ER_Msg command__create_msg_tactile_pcb1_get_median(uint16_t source_id, uint16_t node_id);
ER_Msg command__create_msg_tactile_pcb2_get_median(uint16_t source_id, uint16_t node_id);
ER_Msg command__create_msg_tactile_finger1_get_median(uint16_t source_id, uint16_t node_id);
ER_Msg command__create_msg_tactile_pcb3_get_median(uint16_t source_id, uint16_t node_id);
ER_Msg command__create_msg_tactile_pcb4_get_median(uint16_t source_id, uint16_t node_id);
ER_Msg command__create_msg_tactile_finger2_get_median(uint16_t source_id, uint16_t node_id);
ER_Msg command__create_msg_tactile_gripper1_get_median(uint16_t source_id, uint16_t node_id);

uint16_t command__check_msg_flags(volatile ER_Msg *_msg);
uint16_t    command__check_response_msg(volatile ER_Msg * _msg);

#endif
