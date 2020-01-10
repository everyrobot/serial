#include <er_serial.h>

//void serial__read(SCI_Handle sciHandle, volatile unsigned char * _rx_buf, volatile uint16_t * _rx_ptr)
//{
//    uint16_t success;
//    uint16_t dataRx;
//
//    dataRx = SCI_getDataNonBlocking(sciHandle, &success);
//    if (success)
//    {
//        gRX_SCI_buf[gRX_SCI_ptr++] = dataRx;
//    }
//
//}

//void serial__read(SCI_Handle sciHandle)
//{
//    uint16_t success;
//    uint16_t dataRx;
//
//    dataRx = SCI_getDataNonBlocking(sciHandle, &success);
//    if (success)
//    {
//        gRX_SCI_buf[gRX_SCI_ptr++] = dataRx;
//    }
//
//}

void serial__write(SCI_Handle sciHandle, char * msg, uint16_t length)
{
//    uint16_t success;
    uint16_t i;
    for (i = 0; i < length; i++)
    {
        SCI_putDataBlocking(sciHandle, msg[i]);

    }
}

