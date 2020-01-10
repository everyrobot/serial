#include <er_serial.h>

//void serial__configure(void) {
//}

void serial__scib_init(void) {
    gBufferSerial = buffer__create(1024);

    #pragma CODE_SECTION(serial__scib_rx_isr, "ramfuncs");

    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_Obj *pie = (PIE_Obj *)obj->pieHandle;

//    HAL_setParams
    //    CLK_disableSciaClock(obj->clkHandle);
    CLK_enableScibClock(obj->clkHandle);

    //    HAL_setParams
    SCI_reset(obj->sciBHandle);
    SCI_enableTx(obj->sciBHandle);
    SCI_enableRx(obj->sciBHandle);
    SCI_disableParity(obj->sciBHandle);
    SCI_setNumStopBits(obj->sciBHandle, SCI_NumStopBits_One);
    SCI_setCharLength(obj->sciBHandle, SCI_CharLength_8_Bits);
    // set baud rate to 115200
    SCI_setBaudRate(obj->sciBHandle, (SCI_BaudRate_e) (0x0061));
    SCI_setPriority(obj->sciBHandle, SCI_Priority_FreeRun);
    SCI_enable(obj->sciBHandle);

    //  // UARTB RX
    GPIO_setMode(obj->gpioHandle,GPIO_Number_15,GPIO_15_Mode_SCIRXDB);
    // UARTB TX
    GPIO_setMode(obj->gpioHandle,GPIO_Number_58,GPIO_58_Mode_SCITXDB);

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    pie->SCIRXINTB  = &serial__scib_rx_isr;
    pie->SCITXINTB  = &serial__scib_tx_isr;
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    //    HAL_enableSciInts
    // enable the PIE interrupts associated with the SCI interrupts
    // enable SCIB RX interrupt in PIE
    PIE_enableInt(obj->pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_SCIBRX);
//    PIE_enableInt(obj->pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_SCIBTX);
    // enable SCIB RX interrupt
    SCI_enableRxInt(obj->sciBHandle);
//    SCI_enableTxInt(obj->sciBHandle);
    // enable the cpu interrupt for SCI interrupts
    CPU_enableInt(obj->cpuHandle, CPU_IntNumber_9);


}

//void serial__enable(bool enable) {
//}

void serial__scib_rx_isr(void){
    uint16_t success;
    uint16_t dataRx;

    HAL_Obj *obj = (HAL_Obj *)halHandle;
    dataRx  = SCI_getDataNonBlocking(halHandle->sciBHandle, &success);
    // acknowledge interrupt from SCI group so that SCI interrupt
    // is not received twice
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);

    buffer__add_to_buffer(&gBufferSerial, dataRx);

} // end of sciBRxISR() function

void serial__scib_tx_isr(void)
{
    PIE_clearInt(halHandle->pieHandle, PIE_GroupNumber_9);
}

void serial__write(SCI_Handle sciHandle, char * msg, uint16_t length)
{
//    uint16_t success;
    uint16_t i;
    for (i = 0; i < length; i++)
    {
        SCI_putDataBlocking(sciHandle, msg[i]);

    }
}

