#include <er_ti_f28069m_drv8305/er_buffer.h>


// 3 zmienne globalne musz� by� dost�pne:
// - gRX_SCI_buf
// - rx_b_ptr
// - flag_buffer_rx_is_changed

void buffer__add_to_buffer(volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr, volatile uint16_t * _flag_buffer_rx_is_changed, uint16_t value)
{
    // set buffer changed flag
    * _flag_buffer_rx_is_changed = true;

    // Add value to the buffer and move pointer
    _rx_buf[(* _rx_ptr)++] = value;
}

void buffer__remove_from_buffer (volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr, volatile uint16_t * _flag_buffer_rx_is_changed, uint16_t remove_signs) {
    (* _rx_ptr) -= remove_signs;

    memcpy((void *) _rx_buf, (void *) _rx_buf + remove_signs * sizeof(unsigned char), (* _rx_ptr) * sizeof(unsigned char)); // OK - remove message from buffer

    * _flag_buffer_rx_is_changed = true; // set to analyse buffer in another iteration

}

bool buffer__analyse_buffer(volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr, volatile uint16_t * _flag_buffer_rx_is_changed, volatile uint16_t * _node_id, volatile ER_Msg * message)
{
    // Quit if no changes in buffer
    if (! * _flag_buffer_rx_is_changed) return false;

    // check RX serial buffer for incoming messages
    if ((* _rx_ptr) > 0) {
        * _flag_buffer_rx_is_changed = false;

        if (_rx_buf[0] != 0x45) { // E
            // BAD FIRST SIGN
            buffer__remove_from_buffer (_rx_buf, _rx_ptr,_flag_buffer_rx_is_changed, 1);
            return false;
        }

        if ( (* _rx_ptr) > 1 && _rx_buf[1] != 0x52) { // R
            // BAD SECOND SIGN
            buffer__remove_from_buffer (_rx_buf, _rx_ptr,_flag_buffer_rx_is_changed, 1);
            return false;
        }

        // FIXME - bad package explode TI
        if ( (* _rx_ptr) > 3 && _rx_buf[3] != (* _node_id) ) { // node_id is different from destination_id
            buffer__remove_from_buffer (_rx_buf, _rx_ptr,_flag_buffer_rx_is_changed, 1);
            return false;
        }
    }

    if ((* _rx_ptr) >= 8) {
        uint16_t hcrc;

        msg__set_header(message,  _rx_buf[2], _rx_buf[3], _rx_buf[4], _rx_buf[5], _rx_buf[6]);

        hcrc = msg__calculate_header_crc(message);
        if (hcrc == _rx_buf[7]) {
            (* message).header_crc = _rx_buf[7]; // set header CRC
        }
        else
        { // header CRC error - skipping 1 byte FIXME - create response with information about issue
            // BAD HEADER CRC
            buffer__remove_from_buffer (_rx_buf, _rx_ptr,_flag_buffer_rx_is_changed, 1);

            return false;
        }

        if (_rx_buf[6] == 0) { // message without no body
            // remove message from buffer
            buffer__remove_from_buffer (_rx_buf, _rx_ptr,_flag_buffer_rx_is_changed, 8);
            return true; // Send response
        }

        if ((* _rx_ptr) >= 2 + 6 + 1 + _rx_buf[6]) { // message with body
            uint16_t bsize = _rx_buf[6]; // body size
            uint16_t mcrc;
            uint16_t bcrc;
            msg__set_body(message, (char *) _rx_buf + 8 * sizeof(char), bsize);

            mcrc = msg__calculate_body_crc(message); // CRC after recalculation
            bcrc = _rx_buf[8 + bsize]; // CRC in buffer

            if (bcrc == mcrc) {
                (* message).body_crc = bcrc;

            } else { // inccorect body CRC
//                gMsgResponse.flags |= MSG__BAD_BODY_CRC; // set ERROR flag
            }

            buffer__remove_from_buffer (_rx_buf, _rx_ptr,_flag_buffer_rx_is_changed, 8 + bsize + 1);
            return true;
        }

    }

    return false;

}

//bool buffer__analyse_buffer(volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr, volatile uint16_t * _flag_buffer_rx_is_changed)
//{
//    // Quit if no changes in buffer
//    if (! * _flag_buffer_rx_is_changed) return false;
//
//    // check RX serial buffer for incoming messages
//    if (gRX_SCI_ptr >= 8) {
//        * _flag_buffer_rx_is_changed = false;
//
//        if (gRX_SCI_buf[0] != 0x45) { // E
//            // BAD FIRST SIGN
//            memcpy ((void *) &gRX_SCI_buf, (void *) &gRX_SCI_buf + 1 * sizeof(uint16_t), --gRX_SCI_ptr * sizeof(uint16_t));
////            g_flag_buffer_rx_is_changed = true;
//            return false;
//        }
//
//        if (gRX_SCI_ptr > 1 && gRX_SCI_buf[1] != 0x52) { // R
//            // BAD SECOND SIGN
//            memcpy ((void *) &gRX_SCI_buf, (void *) &gRX_SCI_buf + 1 * sizeof(uint16_t), --gRX_SCI_ptr * sizeof(uint16_t));
////            g_flag_buffer_rx_is_changed = true;
//            return false;
//        }
//
//        uint16_t hcrc;
//
//        msg__set_header(&gMsgCommand,  gRX_SCI_buf[2], gRX_SCI_buf[3], gRX_SCI_buf[4], gRX_SCI_buf[5], gRX_SCI_buf[6]);
//        msg__set_header(&gMsgResponse, gRX_SCI_buf[3], gRX_SCI_buf[2], gRX_SCI_buf[4], MSG__RESPONSE, 0);
//
//        hcrc = msg__calculate_header_crc(&gMsgCommand);
//        if (hcrc == gRX_SCI_buf[7]) {
//            gMsgCommand.header_crc = gRX_SCI_buf[7]; // set header CRC
//        }
//        else
//        { // header CRC error - skipping 1 byte FIXME - create response with information about issue
//            // BAD HEADER CRC
//            memcpy ((void *) &gRX_SCI_buf, (void *) &gRX_SCI_buf  + 1 * sizeof(uint16_t), --gRX_SCI_ptr * sizeof(uint16_t));
////            g_flag_buffer_rx_is_changed = true;
//            return false;
//        }
//
//        if (gRX_SCI_buf[6] == 0) { // message without no body
//            // remove message from buffer
//            gRX_SCI_ptr -= 8;
//            memcpy ((void *) &gRX_SCI_buf, (void *) &gRX_SCI_buf + 8 * sizeof(uint16_t), gRX_SCI_ptr * sizeof(uint16_t));
//
////            g_flag_buffer_rx_is_changed = false;
//            return true; // Send response
//        }
//
//        if (gRX_SCI_ptr >= 2 + 6 + 1 + gRX_SCI_buf[6]) { // message with body
//            uint16_t bsize = gRX_SCI_buf[6]; // body size
//            uint16_t mcrc;
//            uint16_t bcrc;
//
//            msg__set_body(&gMsgCommand, (char *) &gRX_SCI_buf + 8 * sizeof(char), bsize);
//
//            mcrc = msg__calculate_body_crc(&gMsgCommand); // CRC after recalculation
//            bcrc = gRX_SCI_buf[8 + bsize]; // CRC in buffer
//
//            if (bcrc == mcrc) {
//                gMsgCommand.body_crc = bcrc;
//
//            } else { // inccorect body CRC
//                gMsgResponse.flags |= MSG__BAD_BODY_CRC; // set ERROR flag
//            }
//
//            gRX_SCI_ptr -= (8 + bsize + 1);
//            memcpy((void *) &gRX_SCI_buf, (void *) &gRX_SCI_buf + (8 + bsize + 1) * sizeof(uint16_t), gRX_SCI_ptr); // remove message from buffer
//
////            g_flag_buffer_rx_is_changed = false;
//            return true;
//        }
//
//    }
//
//    return false;
//
//}
