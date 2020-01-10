#include <er_tactile.h>

// tactile functions
void tactile__GPIO_setMode(HAL_Handle handle)
{
HAL_Obj *obj = (HAL_Obj *)handle;
 // PWM4A
    //GPIO_setMode(obj->gpioHandle,GPIO_Number_6,GPIO_6_Mode_EPWM4A);
    GPIO_setMode(obj->gpioHandle,GPIO_Number_6,GPIO_6_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle,GPIO_Number_6,GPIO_Direction_Output);
    GPIO_setHigh(obj->gpioHandle,GPIO_Number_6);
    // PWM4B
    //GPIO_setMode(obj->gpioHandle,GPIO_Number_7,GPIO_7_Mode_EPWM4B);
    GPIO_setMode(obj->gpioHandle,GPIO_Number_7,GPIO_7_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle,GPIO_Number_7,GPIO_Direction_Output);
    GPIO_setHigh(obj->gpioHandle,GPIO_Number_7);
    // PWM5A
    //GPIO_setMode(obj->gpioHandle,GPIO_Number_8,GPIO_8_Mode_EPWM5A);
    GPIO_setMode(obj->gpioHandle,GPIO_Number_8,GPIO_8_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle,GPIO_Number_8,GPIO_Direction_Output);
    GPIO_setHigh(obj->gpioHandle,GPIO_Number_8);
    // PWM5B
   // GPIO_setMode(obj->gpioHandle,GPIO_Number_9,GPIO_9_Mode_EPWM5B);
    GPIO_setMode(obj->gpioHandle,GPIO_Number_9,GPIO_9_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle,GPIO_Number_9,GPIO_Direction_Output);
    GPIO_setHigh(obj->gpioHandle,GPIO_Number_9);
    // PWM6A
    //GPIO_setMode(obj->gpioHandle,GPIO_Number_10,GPIO_10_Mode_EPWM6A);
    GPIO_setMode(obj->gpioHandle,GPIO_Number_10,GPIO_10_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle,GPIO_Number_10,GPIO_Direction_Output);
    // PWM6B
    //GPIO_setMode(obj->gpioHandle,GPIO_Number_11,GPIO_11_Mode_EPWM6B);
    GPIO_setMode(obj->gpioHandle,GPIO_Number_11,GPIO_11_Mode_GeneralPurpose);
    GPIO_setDirection(obj->gpioHandle,GPIO_Number_11,GPIO_Direction_Output);

    // SPIB CLK
    GPIO_setMode(obj->gpioHandle,GPIO_Number_14,GPIO_14_Mode_SPICLKB);
    GPIO_setPullup(obj->gpioHandle,GPIO_Number_14,GPIO_Pullup_Disable);

    // SPIB SIMO
    GPIO_setMode(obj->gpioHandle,GPIO_Number_24,GPIO_24_Mode_SPISIMOB);
    GPIO_setPullup(obj->gpioHandle,GPIO_Number_24,GPIO_Pullup_Disable);
    // SPIB SOMI
    GPIO_setMode(obj->gpioHandle,GPIO_Number_25,GPIO_25_Mode_SPISOMIB);
    GPIO_setPullup(obj->gpioHandle,GPIO_Number_25,GPIO_Pullup_Disable);
} //end of tactile__GPIO_setMode


void tactile__setupSpiB(HAL_Handle handle)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  ENABLE_PROTECTED_REGISTER_WRITE_MODE;
  obj->pieHandle->SPITXINTB = &tactile__isrTx;
  obj->pieHandle->SPIRXINTB = &tactile__isrRx;
  DISABLE_PROTECTED_REGISTER_WRITE_MODE;

  SPI_reset(obj->spiBHandle);
  // -------------SCI RX INIT-------------------------------------------------
  SPI_resetRxFifo(obj->spiBHandle);
  SPI_enableRxFifo(obj->spiBHandle);
  SPI_enableRxFifoInt(obj->spiBHandle);
  SPI_setRxFifoIntLevel(obj->spiBHandle, SPI_FifoLevel_1_Word);
  // -------------------------------------------------------------------------
  // -------------SCI TX INIT-------------------------------------------------
  SPI_enableChannels(obj->spiBHandle);
  SPI_resetTxFifo(obj->spiBHandle);
  SPI_enableTxFifo(obj->spiBHandle);
  SPI_enableTxFifoInt(obj->spiBHandle);
  SPI_enableTxFifoEnh(obj->spiBHandle);
  SPI_disableOverRunInt(obj->spiBHandle);
  SPI_setTxFifoIntLevel(obj->spiBHandle, SPI_FifoLevel_Empty);
  //--------------------------------------------------------------------------
  SPI_setMode(obj->spiBHandle,SPI_Mode_Master);
  SPI_setClkPolarity(obj->spiBHandle,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
  SPI_setClkPhase(obj->spiBHandle,SPI_ClkPhase_Delayed);
  SPI_enableTx(obj->spiBHandle);
  SPI_setTxDelay(obj->spiBHandle,0x0);
  SPI_setBaudRate(obj->spiBHandle,(SPI_BaudRate_e)(30 << 0));
  SPI_setCharLength(obj->spiBHandle,SPI_CharLength_16_Bits);
  SPI_setSuspend(obj->spiBHandle,SPI_TxSuspend_free);
  SPI_enable(obj->spiBHandle);

  // -------------------------------------------------------------------------
  PIE_enableInt(obj->pieHandle, PIE_GroupNumber_6, PIE_InterruptSource_SPIBRX);
  PIE_enableInt(obj->pieHandle, PIE_GroupNumber_6, PIE_InterruptSource_SPIBTX);
  SPI_enableInt(obj->spiBHandle);
  CPU_enableInt(obj->cpuHandle, CPU_IntNumber_6);
  // -------------------------------------------------------------------------
  return;
}  // end of tactile__setupSpiB() function

void tactile__SPIwrite(HAL_Handle handle, uint16_t data_to_write)
{
    HAL_Obj  *obj = (HAL_Obj *)handle;
    uint16_t n;

    SPI_write(obj->spiBHandle, data_to_write);

    // wait for registers to update
    for(n=0;n<0xf;n++)
        asm(" NOP");

} //end of tactile__SPIwrite


void tactile__change_leg(){

    uint16_t temp = channel;
    actual_mode = ((temp << 4) + resolution_shift + polar_bit) << 8;
    channel = (channel+1) % 9;

}// end of tactile__change_leg() function

void tactile__init()
{

    tactile__GPIO_setMode(halHandle);
    result = 0;
    cs = GPIO_Number_53;
    channel = 0x00;
    resolution_shift = (0x03 << 2);
    polar_bit = 0x00;
    tactile__setupSpiB(halHandle);
    number_of_interrupt = 0;
    number_of_fsrs=0;
    number_of_channels=0;
    first_scan = true;
    recived = true;
}// end of tactile__init() function


void tactile__send() {

    if (number_of_channels == 0) // start of each pcb
    {
        channel = 0x00;
        uint16_t temp = channel;
        actual_mode = ((temp << 4) + resolution_shift + polar_bit) << 8;
        channel = (channel+1) % 9;

        HAL_setGpioLow(halHandle, cs);
    }

    tactile__SPIwrite(halHandle, actual_mode);

} // end of tactile__send() function

// tactile Interrupt
interrupt void tactile__isrRx(void){
    HAL_Obj   *obj = (HAL_Obj *)halHandle;

    if (first_scan){
        gBufferTactile[number_of_fsrs][number_of_channels] = buffer__create(NO_OF_SCANS);
    }

    result = SPI_read(obj->spiBHandle);
    result = result >> 4;

    if(number_of_channels >= 1){
        buffer__add_to_buffer(&gBufferTactile[number_of_fsrs][number_of_channels-1], result);
    }
    tactile__change_leg();
    number_of_channels +=1;
    if (number_of_channels == NO_OF_CHANNELS+1){ //reading next pcb
                number_of_fsrs+=1;
                HAL_setGpioHigh(halHandle, cs);
                number_of_channels = 0;
    }
    if (number_of_fsrs == NO_OF_PCBS ){ // reading from first pcb
        number_of_fsrs = 0;
        first_scan = false;
    }
    recived = true;
    SPI_clearRxFifoInt(halHandle->spiBHandle);
    SPI_clearRxFifoOvf(halHandle->spiBHandle);
    SPI_clearTxFifoInt(halHandle->spiBHandle);
    PIE_clearInt(halHandle->pieHandle,PIE_GroupNumber_6);
} // end of tactile__recive(void) interrupt function

interrupt void tactile__isrTx(void){
    if(recived){
         tactile__send();
        SPI_clearTxFifoInt(halHandle->spiBHandle);
        recived = false;
    }
    PIE_clearInt(halHandle->pieHandle,PIE_GroupNumber_6);
} // end of tactile__transmit(void) interrupt function
