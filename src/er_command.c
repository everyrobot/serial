// **************************************************************************
// the includes

#include <er_ti_f28069m_drv8305/er_command.h>

// **************************************************************************
// the defines


// **************************************************************************
// the globals

// **************************************************************************
// the functions
ER_Msg  command__create_msg_set_position(uint16_t source_id, uint16_t node_id, float position) {
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command,  1, node_id, ER_REG_SET_POSITION_SP, MSG__COMMAND, 0);

    msg__add_int32_to_body(&_command, msg__convert_float_to_iq(position, 15) );

    return _command;
}

ER_Msg  command__create_msg_get_position(uint16_t source_id, uint16_t node_id) {
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command,  1, node_id, ER_REG_GET_POSITION, MSG__COMMAND, 0);

    return _command;
}

ER_Msg  command__create_msg_ping(uint16_t source_id, uint16_t node_id) {
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command,  1, node_id, ER_REG_PING, MSG__COMMAND, 0);

    return _command;
}

ER_Msg command__create_msg_tactile_pcb1_get_median(uint16_t source_id, uint16_t node_id)
{
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command, 1, node_id, ER_TACTILE_PCB1_GET_MEDIAN, MSG__COMMAND, 0);

    return _command;
}

ER_Msg command__create_msg_tactile_pcb2_get_median(uint16_t source_id, uint16_t node_id)
{
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command, 1, node_id, ER_TACTILE_PCB2_GET_MEDIAN, MSG__COMMAND, 0);

    return _command;
}

ER_Msg command__create_msg_tactile_finger1_get_median(uint16_t source_id, uint16_t node_id)
{
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command, 1, node_id, ER_TACTILE_FINGER1_GET_MEDIAN, MSG__COMMAND, 0);

    return _command;
}

ER_Msg command__create_msg_tactile_pcb3_get_median(uint16_t source_id, uint16_t node_id)
{
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command, 1, node_id, ER_TACTILE_PCB3_GET_MEDIAN, MSG__COMMAND, 0);

    return _command;
}

ER_Msg command__create_msg_tactile_pcb4_get_median(uint16_t source_id, uint16_t node_id)
{
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command, 1, node_id, ER_TACTILE_PCB4_GET_MEDIAN, MSG__COMMAND, 0);

    return _command;
}

ER_Msg command__create_msg_tactile_finger2_get_median(uint16_t source_id, uint16_t node_id)
{
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command, 1, node_id, ER_TACTILE_FINGER2_GET_MEDIAN, MSG__COMMAND, 0);

    return _command;
}

ER_Msg command__create_msg_tactile_gripper1_get_median(uint16_t source_id, uint16_t node_id)
{
    ER_Msg _command = ER_Msg_INIT;
    msg__set_header(&_command, 1, node_id, ER_TACTILE_GRIPPER1_GET_MEDIAN, MSG__COMMAND, 0);

    return _command;
}

uint16_t  command__check_msg_flags(volatile ER_Msg * _msg) {
    return (* _msg).flags & 0b01111111;
}

uint16_t  command__check_command_msg(volatile ER_Msg * _msg) {
    // check body CRC
    uint16_t errors = MSG__OK;


    switch ((* _msg).register_id) {
        case ER_REG_PING: // PING message
        case ER_REG_GET_POSITION: { // Set position setpoint
            if ((* _msg).body_size != 0) errors |= MSG__BAD_BODY;
            break;
        }

    case ER_REG_SET_POSITION_SP: { // Set position setpoint
            if ((* _msg).body_size != 4) errors |= MSG__BAD_BODY;
            break;
        }

        default: {
            errors |= MSG__BAD_REGISTER;

        }

    }

    return errors;
}

uint16_t  command__check_response_msg(volatile ER_Msg * _msg) {
    uint16_t errors = MSG__OK;

    // check body CRC

    switch ((* _msg).register_id) {
        case ER_REG_PING: // PING message
        case ER_REG_SET_POSITION_SP: { // Set position setpoint
            if ((* _msg).body_size != 0) errors |= MSG__BAD_BODY;
            break;
        }

        case ER_REG_GET_POSITION: { // Set position setpoint
            if ((* _msg).body_size != 4) errors |= MSG__BAD_BODY;
            break;
        }

        default: {
            errors |= MSG__BAD_REGISTER;

        }

    }

    return errors;
}
