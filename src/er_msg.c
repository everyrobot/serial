// **************************************************************************
// the includes

#include <er_ti_f28069m_drv8305/er_msg.h>

// **************************************************************************
// the defines


// **************************************************************************
// the globals

// **************************************************************************
// the functions

//void msg__clearMessages(){
//
//    if(gMsgCommand.body_size > 0){
//        free(gMsgCommand.body);
//    }
//    gMsgCommand.source_id = 0;
//    gMsgCommand.destination_id = 0;
//    gMsgCommand.register_id = 0;
//    gMsgCommand.flags = 0;
//    gMsgCommand.body_size = 0;
//    gMsgCommand.header_crc = 0;
//    gMsgCommand.body = NULL;
//    gMsgCommand.body_crc = 0;
//
//    if(gMsgResponse.body_size > 0){
//        free(gMsgResponse.body);
//    }
//   (* message).source_id = 0;
//   (* message).destination_id = 0;
//   (* message).register_id = 0;
//   (* message).flags = 0;
//   (* message).body_size = 0;
//   (* message).header_crc = 0;
//   (* message).body = NULL;
//   (* message).body_crc = 0;
//
//}

void msg__clearMessage(volatile ER_Msg * message){

    if((* message).body_size > 0){
        free((* message).body);
    }
    (* message).source_id = 0;
    (* message).destination_id = 0;
    (* message).register_id = 0;
    (* message).flags = 0;
    (* message).body_size = 0;
    (* message).header_crc = 0;
    (* message).body = NULL;
    (* message).body_crc = 0;

}

uint16_t msg__calculate_header_crc(volatile ER_Msg * message) {
    uint16_t header_crc = ( ( message->source_id + message->destination_id + message->register_id + message->flags + message->body_size ) ^ 0xFF ) & 0xFF;
    return header_crc;
}

uint16_t msg__calculate_body_crc(volatile ER_Msg * message) {
    uint16_t body_crc = 0;
    uint16_t i;

    for (i = 0; i < message->body_size; i++) {
        body_crc += (message->body[i] & 0xFF);
    }
    body_crc ^= 0x00FF; // XOR
    body_crc &= 0x00FF; // AND
    return (uint16_t) body_crc;
}

void msg__set_header(volatile ER_Msg * message, uint16_t source_id, uint16_t destination_id, uint16_t register_id, uint16_t flags, uint16_t body_size) {
    message->source_id = source_id;
    message->destination_id = destination_id;
    message->register_id = register_id;
    message->flags = flags;
    message->body_size = body_size;
    // wyliczenie CRC nag��wka
    message->header_crc = msg__calculate_header_crc(message);
    message->body = NULL;
}


void msg__set_body(volatile ER_Msg * message, char * body, uint16_t body_size) {

    if(message->body != NULL) {

        free(message->body);
    }

    message->body = (char *) malloc( sizeof(char) * body_size );

    if(message->body == NULL){  // ALLOCATION FAILED

        message->flags |= MSG__ALLOC_FAILED;
        message->header_crc = msg__calculate_header_crc(message);
        return;

    }
    else {

        memcpy(message->body, body, sizeof(char) * body_size);

        message->body_size = body_size;

        // recalculate header CRC
        message->header_crc = msg__calculate_header_crc(message);
        // calculate body CRC
        message->body_crc  = msg__calculate_body_crc(message);
//        free(body); // FIXME !!!! - has to be checked
    }

}


char * msg__to_byte_array(volatile ER_Msg * message) {
    char * byte_array;
    // byte array size - header (6) + body_size + body_crc if body_size > 0
    uint16_t msg_length = 8;
    if (message->body_size > 0) {
        msg_length++; // additional byte for body CRC
    }
    msg_length += message->body_size;

    byte_array = (char *) malloc( sizeof(char) * msg_length); // allocate memory for array
    uint16_t i;
    byte_array[0] = 0x45;
    byte_array[1] = 0x52;
    byte_array[2] = message->source_id;
    byte_array[3] = message->destination_id;
    byte_array[4] = message->register_id;
    byte_array[5] = message->flags;
    byte_array[6] = message->body_size;
    byte_array[7] = message->header_crc;
    if (message->body_size > 0) {
        for(i = 0; i < message->body_size; i++){
            byte_array[8+i] =  message->body[i];
        }
        byte_array[8 + message->body_size] =  message->body_crc;
    }
    return byte_array;
}


uint16_t msg__get_length(volatile ER_Msg * message) {
    uint16_t length = 8;
    if (message->body_size > 0) {
        length += message->body_size;
        length++;
    }
    return length;
}

bool      msg__check(volatile ER_Msg * message) {

    switch ((* message).register_id) {

        case ER_REG_BLDC_SET_POSITION_SP: {
            if ( (* message).body_size == 4 ) return true;
            break;

        }
    }

    return false;

}

bool      msg__create(volatile ER_Msg * message) {

    switch ((* message).register_id) {

        case ER_REG_BLDC_SET_POSITION_SP: { // Set position setpoint
//            if ( (* message).body_size == 4 ) return true;
            break;

        }
    }

    return false;

}

uint16_t msg__get_uint16_from_body(volatile ER_Msg *message, uint16_t start)
{
    uint16_t uint_v = 0;
    char char_g = (*message).body[start];
    char char_d = (*message).body[start + 1];
    uint_v = (char_g << 8) & 0xFF00;
    uint_v |= (char_d & 0x00FF);
    return uint_v;
}

//int16_t   msg__get_int16_from_body(uint16_t start){
//    int16_t int_v = 0;
//    char char_g = gMsgCommand.body[start];
//    char char_d = gMsgCommand.body[start+1];
//    int_v = (char_g << 8) & 0xFF00;
//    int_v |= (char_d & 0x00FF);
//
//    return int_v;
//}

int32_t   msg__get_int32_from_body(volatile ER_Msg * message, uint16_t start){
    uint32_t uint_g = 0;
    uint32_t uint_d = 0;
    int32_t  int_v = 0;

    char char_msb =  (* message).body[start];
    char char_1b  =  (* message).body[start+1];
    char char_2b  =  (* message).body[start+2];
    char char_lsb =  (* message).body[start+3];
    uint_g = (char_msb << 8) & 0xFF00;
    uint_g |= (char_1b & 0x00FF);
    uint_g &= 0x0000FFFF;
    uint_d = (char_2b << 8) & 0xFF00;
    uint_d |= (char_lsb & 0x00FF);
    uint_d &= 0x0000FFFF;

    int_v = (uint_g << 16);
    int_v &= 0xFFFF0000;
    int_v |= uint_d;


    return int_v;
}

float   msg__get_float_from_body(volatile ER_Msg * message, uint16_t start){

    int32_t value = msg__get_int32_from_body(message, start);
    float   value_f = msg__convert_iq_to_float(value, 15);

    return value_f;
}

//int32_t   msg__get_int32_from_body(uint16_t start){
//    uint32_t uint_g = 0;
//    uint32_t uint_d = 0;
//    int32_t  int_v = 0;
//
//    char char_msb =  gMsgCommand.body[start];
//    char char_1b  =  gMsgCommand.body[start+1];
//    char char_2b  =  gMsgCommand.body[start+2];
//    char char_lsb =  gMsgCommand.body[start+3];
//
//    uint_g = (char_msb << 8) & 0xFF00;
//    uint_g |= (char_1b & 0x00FF);
//    uint_g &= 0x0000FFFF;
//    uint_d = (char_2b << 8) & 0xFF00;
//    uint_d |= (char_lsb & 0x00FF);
//    uint_d &= 0x0000FFFF;
//
//    int_v = (uint_g << 16);
//    int_v &= 0xFFFF0000;
//    int_v |= uint_d;
//
//
//    return int_v;
//}

void     msg__add_uint16_to_body(volatile ER_Msg * message, uint16_t value){

    uint16_t body_size = (* message).body_size;
    if((* message).body == NULL) {
       (* message).body = (char *) malloc( sizeof(char) * (body_size + 2));
    }
    else {
       (* message).body = (char *) realloc((* message).body, sizeof(char) * (body_size + 2));
    }

    if((* message).body != NULL) { // uda�o si� zaalokowac

       (* message).body[body_size] = (char) ((value >> 8) & 0xFF);
       (* message).body[body_size + 1] = (char) (value & 0xFF);
       (* message).body_size = body_size + 2;
       (* message).body_crc            = msg__calculate_body_crc(message);
       (* message).header_crc          = msg__calculate_header_crc(message);
    }
    else { // blad alokacji
       (* message).flags |= MSG__ALLOC_FAILED;
       (* message).header_crc          = msg__calculate_header_crc(message);
    }

}

//void     msg__add_int16_to_body(int16_t value){
//
//    uint16_t body_size =(* message).body_size;
//    if(gMsgResponse.body == NULL) {
//       (* message).body = (char *) malloc( sizeof(char) * (body_size + 2));
//    }
//    else {
//       (* message).body = (char *) realloc(&gMsgResponse.body, sizeof(char) * (body_size + 2));
//    }
//
//    if(gMsgResponse.body != NULL) { // uda�o si� zaalokowac
//
//       (* message).body[body_size] = (char) ((value >> 8) & 0xFF);
//       (* message).body[body_size + 1] = (char) (value & 0xFF);
//       (* message).body_size = body_size + 2;
//       (* message).body_crc = msg__calculate_body_crc(&gMsgResponse);
//    }
//    else { // blad alokacji
//       (* message).flags |= MSG__ALLOC_FAILED;
//       (* message).header_crc = msg__calculate_header_crc(&gMsgResponse);
//            return;
//    }
//
//}

void     msg__add_int32_to_body(volatile ER_Msg * message, int32_t value){

    uint16_t body_size = (* message).body_size;

    // memory allocation for next variable
    if((* message).body == NULL) {
        (* message).body = (char *) malloc( sizeof(char) * (body_size + 4));
    }
    else {
        (* message).body = (char *) realloc((* message).body, sizeof(char) * (body_size + 4)); // FIXME !!!!
    }

    if((* message).body != NULL) { // uda�o si� zaalokowac

        (* message).body[body_size]     = (char) ((value >> 24) & 0xFF);
        (* message).body[body_size + 1] = (char) ((value >> 16) & 0xFF);
        (* message).body[body_size + 2] = (char) ((value >> 8) & 0xFF);
        (* message).body[body_size + 3] = (char) (value & 0xFF);
        (* message).body_size           = body_size + 4;
        (* message).body_crc            = msg__calculate_body_crc(message);
        (* message).header_crc          = msg__calculate_header_crc(message);
    }
    else { // blad alokacji pami�ci
        (* message).flags              |= MSG__ALLOC_FAILED;
        (* message).header_crc          = msg__calculate_header_crc(message);
    }
}

void     msg__add_float_to_body(volatile ER_Msg * message, float value){
    int32_t value_int32 = msg__convert_float_to_iq(value, 24); // FIXME - static 24 bits od fraction
    msg__add_int32_to_body(message, value_int32);
}

//void     msg__add_int32_to_body(volatile ER_Msg * message, int32_t value){
//
//    uint16_t body_size =(* message).body_size;
//    if(gMsgResponse.body == NULL) {
//       (* message).body = (char *) malloc( sizeof(char) * (body_size + 4));
//    }
//    else {
//       (* message).body = (char *) realloc(&gMsgResponse.body, sizeof(char) * (body_size + 4));
//    }
//
//    if(gMsgResponse.body != NULL) { // uda�o si� zaalokowac
//
//       (* message).body[body_size] = (char) ((value >> 24) & 0xFF);
//       (* message).body[body_size + 1] = (char) ((value >> 16) & 0xFF);
//       (* message).body[body_size + 2] = (char) ((value >> 8) & 0xFF);
//       (* message).body[body_size + 3] = (char) (value & 0xFF);
//       (* message).body_size = body_size + 4;
//       (* message).body_crc = msg__calculate_body_crc(&gMsgResponse);
//    }
//    else { // blad alokacji
//       (* message).flags |= MSG__ALLOC_FAILED;
//       (* message).header_crc = msg__calculate_header_crc(&gMsgResponse);
//            return;
//    }
//
//}

int32_t  msg__convert_float_to_iq(float float_data, int n){
    float tmp_data = float_data * pow(2, n);
    int32_t iq_data = (long) tmp_data;
    return iq_data;
}

float      msg__convert_iq_to_float(long iq_data, int n){
    float float_data = (float) iq_data;
    float_data = float_data * pow(2, -n);
    return float_data;
}

bool      msg__analyse_buffer(volatile uint16_t * node_id, volatile ER_Msg * message) {
    ER_Buffer * channel_buff = &gBufferSerial;
    // Quit if no changes in buffer
    if (!channel_buff->buff_is_changed) return false;

    // check RX serial buffer for incoming messages
    if ((channel_buff->length) > 0) {
        channel_buff->buff_is_changed = false;

        if (channel_buff->buf[0] != 0x45) { // E
            // BAD FIRST SIGN
            buffer__remove_from_buffer (channel_buff, 1);
            return false;
        }

        if ( channel_buff->length > 1 && channel_buff->buf[1] != 0x52) { // R
            // BAD SECOND SIGN
            buffer__remove_from_buffer (channel_buff, 1);
            return false;
        }

        // FIXME - bad package explode TI
        if ( channel_buff->length > 3 && channel_buff->buf[3] != (* node_id) ) { // node_id is different from destination_id
            buffer__remove_from_buffer (channel_buff, 1);
            return false;
        }
    }

    if (channel_buff->length >= 8) {
        uint16_t hcrc;

        msg__set_header(message,  channel_buff->buf[2], channel_buff->buf[3], channel_buff->buf[4], channel_buff->buf[5], channel_buff->buf[6]);

        hcrc = msg__calculate_header_crc(message);
        if (hcrc == channel_buff->buf[7]) {
            (* message).header_crc = channel_buff->buf[7]; // set header CRC
        }
        else
        { // header CRC error - skipping 1 byte FIXME - create response with information about issue

            // BAD HEADER CRC
            buffer__remove_from_buffer (channel_buff, 1);
            gFlag |= MSG__BAD_HEADER_CRC;
            return false;
        }

        if (channel_buff->buf[6] == 0) { // message without no body
            // remove message from buffer
            buffer__remove_from_buffer (channel_buff, 8);
            return true; // Send response
        }

        if (channel_buff->length >= 2 + 6 + 1 + channel_buff->buf[6]) { // message with body
            uint16_t bsize = channel_buff->buf[6]; // body size
            uint16_t mcrc;
            uint16_t bcrc;
            msg__set_body(message, (char *) channel_buff->buf + 8 * sizeof(char), bsize);

            mcrc = msg__calculate_body_crc(message); // CRC after recalculation
            bcrc = channel_buff->buf[8 + bsize]; // CRC in buffer

            if (bcrc == mcrc) {
                (* message).body_crc = bcrc;

            } else { // inccorect body CRC
//               (* message).flags |= MSG__BAD_BODY_CRC; // set ERROR flag
                gFlag |=  MSG__BAD_BODY_CRC;
            }

            buffer__remove_from_buffer (channel_buff, 8 + bsize + 1);
            return true;
        }

    }

    return false;
}
