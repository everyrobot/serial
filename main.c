/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
* *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//! \file   solutions/instaspin_motion/src/proj_lab13b.c
//! \brief  mooth Position Transitions with SpinTAC Move
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.

//! \defgroup PROJ_LAB13B PROJ_LAB13B
//@{

//! \defgroup PROJ_LAB13B_OVERVIEW Project Overview
//!
//! Smooth Position Transitions with SpinTAC Move
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main_position.h"
#include <er_globals.h>
#include <er_serial.h>
#include <er_bldc.h>
#include <er_comm.h>
#include <er_adc.h>
#include <er_tactile.h>
#include <er_watchdog.h>
#include <Microchip_25AA02E48.h>
#include <er_ti_f28069m_drv8305/er_buffer.h>
#include <er_ti_f28069m_drv8305/er_msg.h>

#ifdef FLASH
#pragma CODE_SECTION(mainISR, "ramfuncs");
//#pragma CODE_SECTION(serial__isr, "ramfuncs");
#pragma CODE_SECTION(bldc__isr, "ramfuncs");
#pragma CODE_SECTION(comm__isr, "ramfuncs");
#pragma CODE_SECTION(adc__isr, "ramfuncs");
#pragma CODE_SECTION(tactile__isrRx, "ramfuncs");
#pragma CODE_SECTION(tactile__isrTx, "ramfuncs");
#pragma CODE_SECTION(watchdog__isr, "ramfuncs");
#endif

// Include header files used in the main function


// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   5
//#define M_2PI               (M_PI * 2)        /* 2pi */
//#define M_1_2PI             1 / (M_2PI)  /* 1/2pi */
#define GLOBAL_Q            24

// **************************************************************************
// the globals

volatile uint_least16_t gCounter_updateGlobals = 0;

volatile bool gFlag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

HAL_Handle halHandle;

USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

volatile _iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else
CTRL_Obj ctrl;				//v1p7 format
#endif

ENC_Handle encHandle;
ENC_Obj enc;

SLIP_Handle slipHandle;
SLIP_Obj slip;

ST_Obj st_obj;
ST_Handle stHandle;

uint16_t gLEDcnt        = 0;
//uint16_t gUART_main     = 0;

// Global variables
volatile MOTOR_Vars_t       gMotorVars = MOTOR_Vars_INIT;
volatile ER_Configurations  gConfigurationsVars = ER_Configurations_INIT;

volatile ER_BLDC_Configurations  gBLDCConfigurationsVars = ER_BLDC_Configurations_INIT;
volatile ER_BLDC_Parameters      gBLDCParametersVars = ER_BLDC_Parameters_INIT;
volatile ER_BLDC_Results         gBLDCResultsVars = ER_BLDC_Results_INIT;
volatile ER_ADC_Results          gADCResultsVars = ER_ADC_Results_INIT;

EEPROM25AA02_Handle              eeprom_handle;
EEPROM25AA02_Obj                 eeprom;

extern HAL_Obj                   hal;

// messages
volatile ER_Msg             gMsgCommand = ER_Msg_INIT;
volatile ER_Msg             gMsgResponse = ER_Msg_INIT;
volatile  uint16_t          gFlag;

// buffer for RX
ER_Buffer                   gBufferSerial;
ER_Buffer                   gBufferAdc;

uint8_t               write_eeprom[32];    // buffer RX
uint8_t               read_eeprom[32];    // buffer RX

volatile uint16_t           _node_id = 100;

// **************************************************************************
//* tactile global*//
ER_Buffer                   gBufferTactile[NO_OF_PCBS][NO_OF_CHANNELS];
bool tactile_median_request;

bool first_scan;
int number_of_interrupt ;
int number_of_fsrs;
int number_of_channels;
int av_temp;
int av_i;
int watchdog_counter;
//SPI Globals
uint16_t result;
GPIO_Number_e cs ;
//SPI for TLV2553 Communication
uint16_t channel;               // from which channel it should start
uint16_t resolution_shift;  // 0x03 - 16bit SPI
uint16_t polar_bit;               // 00 - unipolar, MSB out; 01 - bipolar, MSB out; 10 - unipolar, LSB out, 11 - bipolar, LSB out
uint16_t actual_mode;                     // full command for DAC
uint16_t recived;
// **************************************************************************

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif


// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

//* tactile global*//
ER_Buffer                   gBufferTactile[NO_OF_PCBS][NO_OF_CHANNELS];
bool tactile_median_request;

bool first_scan;
int number_of_interrupt ;
int number_of_fsrs;
int number_of_channels;
int av_temp;
int av_i;
//SPI Globals
uint16_t result;
GPIO_Number_e cs ;
//SPI for TLV2553 Communication
uint16_t channel;               // from which channel it should start
uint16_t resolution_shift;  // 0x03 - 16bit SPI
uint16_t polar_bit;               // 00 - unipolar, MSB out; 01 - bipolar, MSB out; 10 - unipolar, LSB out, 11 - bipolar, LSB out
uint16_t actual_mode;                     // full command for DAC

// **************************************************************************

void main(void)
{

  // Only used if running from FLASH
  // Note that the variable FLASH is defined by the project
  #ifdef FLASH
  // Copy time critical code and Flash setup code to RAM
  // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
  // symbols are created by the linker. Refer to the linker files.
  memCopy((uint16_t *) &RamfuncsLoadStart, (uint16_t *) &RamfuncsLoadEnd, (uint16_t *) &RamfuncsRunStart);
  #endif

  // initialize the hardware abstraction layer
  halHandle = HAL_init(&hal,sizeof(hal));

  // EEPROM
  eeprom_handle = EEPROM25AA02_begin(&eeprom, sizeof(eeprom), GPIO_Number_44, halHandle->gpioHandle, halHandle->spiAHandle);

  // HAL configuration
  HAL_setParams(halHandle);

  //-- PO TEJ LINI URUCHAMIANE S� INTERRUPTY - KONFIGURACJA MODU��W MUSI BY� PRZED T� LINI�

  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle); // Zostaje - uruchomienie main_isr

  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);

  // enable global interrupts
  HAL_enableGlobalInts(halHandle); // Zostaje

  // enable debug interrupts
  HAL_enableDebugInt(halHandle); // Zostaje

//  adc__init();

  serial__scib_init();

  bldc__init();

//  adc__init();

  tactile__init();

  watchdog__init();

//  uint8_t i;
//  for (i = 0; i < 32; i++) {
//      write_eeprom[i] = i+1;
////      EEPROM25AA02_writeRegister(eeprom_handle, i, write_eeprom[i]);
//      read_eeprom[i] = 0 + i;
////      read_eeprom[i] = EEPROM25AA02_readRegister(eeprom_handle, i);
//
//  }

  gFlag = MSG__RESPONSE;

  for(;;)
  {

      // analyse buffer from serial communication

      if (msg__analyse_buffer(&_node_id, &gMsgCommand) ) {
          // create response from command data
//          msg__set_header(&gMsgResponse, gMsgCommand.destination_id, gMsgCommand.source_id, gMsgCommand.register_id, MSG__RESPONSE, 0);

          msg__set_header(&gMsgResponse, gMsgCommand.destination_id, gMsgCommand.source_id, gMsgCommand.register_id, gFlag, 0);


          // FIXME - wykonywanie polecenia tylko w przypadku gdy nie ma flag bledy w response
          comm__execute(&gMsgCommand, &gMsgResponse);

          char * ba = msg__to_byte_array(&gMsgResponse);
          serial__write(halHandle->sciBHandle, ba, msg__get_length(&gMsgResponse));
          free(ba);

          gFlag = MSG__RESPONSE;

          msg__clearMessage(&gMsgCommand);
          msg__clearMessage(&gMsgResponse);
      }
      else{
          if(gFlag != MSG__RESPONSE){ // odebrano wiadomosc z bledem
              msg__set_header(&gMsgResponse, gMsgCommand.destination_id, gMsgCommand.source_id, gMsgCommand.register_id, gFlag, 0);
              char * ba = msg__to_byte_array(&gMsgResponse);
              serial__write(halHandle->sciBHandle, ba, msg__get_length(&gMsgResponse));
              free(ba);

              msg__clearMessage(&gMsgCommand);
              msg__clearMessage(&gMsgResponse);
          }
      }

      bldc__update_state(halHandle, ctrlHandle, stHandle);


     // if(number_of_channels >= 1){
       //      every_board[number_of_fsrs].results[(number_of_channels-1)%9][number_of_interrupt] = result;
          //        }

    // disable the PWM
//    HAL_disablePwm(halHandle);
//
//    // set the default controller parameters (Reset the control to re-identify the motor)
//    CTRL_setParams(ctrlHandle,&gUserParams);
//    gMotorVars.Flag_Run_Identify = false;
//
//    // setup the SpinTAC Components
//    ST_setupPosConv(stHandle);
//    ST_setupPosCtl(stHandle);
//    ST_setupPosMove(stHandle);
//    gMotorVars.MaxVel_krpm = 0;
  } // end of for(;;) loop

} // end of main() function


interrupt void mainISR(void)
{

  // toggle status LED
  if(++gLEDcnt >= (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle, (GPIO_Number_e) HAL_Gpio_LED2);
    gLEDcnt = 0;
    // ----
  }

  bldc__isr(halHandle, ctrlHandle, stHandle, encHandle);

  return;
} // end of mainISR() function

//@} //defgroup
// end of file
