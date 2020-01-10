#ifndef ER_TACTILE_H
#define ER_TACTILE__H

// **************************************************************************
// the includes
#include <stdint.h> //uint16_t def
#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h" // GPIO def
#include "sw/drivers/pie/src/32b/f28x/f2806x/pie.h" // PIE clear int
#include "hal.h" //HAL def for  GPIO_setMode
#include <er_ti_f28069m_drv8305/er_buffer.h> // Buffer for reading
// **************************************************************************
// **************************************************************************
// the defines
#define NO_OF_CHANNELS 9
#define NO_OF_PCBS  10
#define NO_OF_SCANS 5

// **************************************************************************
// **************************************************************************
// the typedefs

// **************************************************************************
// **************************************************************************
// the globals
extern uint16_t result;
extern GPIO_Number_e cs;
extern int number_of_interrupt ;
extern int number_of_fsrs;
extern int number_of_channels;
extern int av_temp;
extern int av_i;
//SPI for TLV2553 Communication
extern uint16_t channel;
extern uint16_t resolution_shift ;
extern uint16_t polar_bit;
extern uint16_t actual_mode;
extern bool first_scan;
extern uint16_t recived;

// HAL object
extern HAL_Handle halHandle;

// Tactile Buffer
extern ER_Buffer gBufferTactile[NO_OF_PCBS][NO_OF_CHANNELS];

// **************************************************************************
// **************************************************************************
// the function prototypes

// tactile functions
void tactile__SPIwrite(HAL_Handle handle, uint16_t data_to_write);
void tactile__setupSpiB(HAL_Handle handle);
void tactile__GPIO_setMode(HAL_Handle handle);
void tactile__init();
void tactile__send();
void tactile__change_leg(); // function for changing commands for DAC

// tactile Interrupt
interrupt void tactile__isrTx(void);
interrupt void tactile__isrRx(void);
//******************************************************************************
//******************************************************************************
#endif
