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

#include <er_serial.h>
#include <er_globals.h>

#ifdef FLASH
#pragma CODE_SECTION(mainISR, "ramfuncs");
#pragma CODE_SECTION(sciBRxISR, "ramfuncs");
#pragma CODE_SECTION(spiISR, "ramfuncs");
#pragma CODE_SECTION(comm__isr, "ramfuncs");
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

//SPI for TLV2553 Communication

uint16_t channel = 0x00;                  // from which channel it should start

uint16_t resolution_shift = (0x03 << 2);  // 0x03 - 16bit SPI

uint16_t polar_bit = 0x00;                // 00 - unipolar, MSB out; 01 - bipolar, MSB out; 10 - unipolar, LSB out, 11 - bipolar, LSB out

uint16_t actual_mode;                     // full command for DAC

void change_leg(void);                    // function for changing commands for DAC

uint16_t result;

GPIO_Number_e cs = GPIO_Number_53;

int number_of_interrupt = 0;

int number_of_fsrs;

int number_of_channels;

int av_temp;

int av_i;

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

HAL_Handle halHandle;

USER_Params gUserParams;

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

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
uint16_t gUART_main     = 0;

// Global variables
volatile MOTOR_Vars_t       gMotorVars = MOTOR_Vars_INIT;
volatile ER_Configurations  gConfigurationsVars = ER_Configurations_INIT;
volatile ER_Parameters      gParametersVars = ER_Parameters_INIT;
volatile ER_Results         gResultsVars = ER_Results_INIT;
volatile ER_FSR             every_board[10] = {ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT,ER_FSR_INIT};

// messages
volatile ER_Msg             gMsgCommand = ER_Msg_INIT;
volatile ER_Msg             gMsgResponse = ER_Msg_INIT;

// buffer
volatile unsigned char      _RX_SCI_buf[1024];    // buffer RX
volatile uint16_t           _RX_SCI_ptr                 = 0;
volatile uint16_t           __flag_buffer_rx_is_changed = 0;

volatile uint16_t           _node_id = 100;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif


#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars;
#endif
#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

// **************************************************************************
// the functions

void main(void)
{
  uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

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


  // check for errors in user parameters
  USER_checkForErrors(&gUserParams);


  // store user parameter error in global variable
  gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);


  // do not allow code execution if there is a user parameter error
  if(gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
      for(;;)
        {
          gMotorVars.Flag_enableSys = false;
        }
    }


  // initialize the user parameters
  USER_setParams(&gUserParams);

  gConfigurationsVars.motor_pole_pairs = gUserParams.motor_numPolePairs;
  gConfigurationsVars.encoder_resolution_ppr = USER_MOTOR_ENCODER_LINES * 4;

  // set the hardware abstraction layer parameters
  HAL_setParams(halHandle,&gUserParams);


  // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);  		//v1p6 format (06xF and 06xM devices)
  controller_obj = (CTRL_Obj *)ctrlHandle;
#else
  ctrlHandle = CTRL_initCtrl(estNumber,&ctrl,sizeof(ctrl));	//v1p7 format default
#endif


  {
    CTRL_Version version;

    // get the version number
    CTRL_getVersion(ctrlHandle,&version);

    gMotorVars.CtrlVersion = version;
  }


  // set the default controller parameters
  CTRL_setParams(ctrlHandle,&gUserParams);


  // setup faults
  HAL_setupFaults(halHandle);


  // initialize the interrupt vector table
  HAL_initIntVectorTable(halHandle);


  // enable the ADC interrupts
  HAL_enableAdcInts(halHandle);

  // enable the SCI interrupts
  HAL_enableSciInts(halHandle);


  // enable global interrupts
  HAL_enableGlobalInts(halHandle);


  // enable debug interrupts
  HAL_enableDebugInt(halHandle);


  // disable the PWM
  HAL_disablePwm(halHandle);


  // initialize the ENC module
  encHandle = ENC_init(&enc, sizeof(enc));


  // setup the ENC module
  ENC_setup(encHandle, 1, USER_MOTOR_NUM_POLE_PAIRS, USER_MOTOR_ENCODER_LINES, 0, USER_IQ_FULL_SCALE_FREQ_Hz, USER_ISR_FREQ_Hz, 8000.0);


  // initialize the SLIP module
  slipHandle = SLIP_init(&slip, sizeof(slip));


  // setup the SLIP module
  SLIP_setup(slipHandle, _IQ(gUserParams.ctrlPeriod_sec));


  // initialize the SpinTAC Components
  stHandle = ST_init(&st_obj, sizeof(st_obj));
  
  
  // setup the SpinTAC Components
  ST_setupPosConv(stHandle);
  ST_setupPosCtl(stHandle);
  ST_setupPosMove(stHandle);
  gMotorVars.MaxVel_krpm = 0;

#ifdef DRV8301_SPI
  // turn on the DRV8301 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8301 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8301Vars);
#endif

#ifdef DRV8305_SPI
  // turn on the DRV8305 if present
  HAL_enableDrv(halHandle);
  // initialize the DRV8305 interface
  HAL_setupDrvSpi(halHandle,&gDrvSpi8305Vars);
#endif

  // enable DC bus compensation
  CTRL_setFlag_enableDcBusComp(ctrlHandle, true);


  // compute scaling factors for flux and torque calculations
  gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
  gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
  gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
  gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();


  // setup for spiB
  HAL_setupSpiB(halHandle);
  change_leg();
  HAL_setGpioHigh(halHandle ,cs);
  
  
  for(;;)
  {
    // Waiting for enable system flag to be set
    while(!(gMotorVars.Flag_enableSys))
    {
        // analyse buffer from serial communication
//        serial__analyse_buffer(halHandle->sciBHandle);
        if (buffer__analyse_buffer(_RX_SCI_buf, &_RX_SCI_ptr,  &__flag_buffer_rx_is_changed, &_node_id, &gMsgCommand) ) {
            // create response from command data
            msg__set_header(&gMsgResponse, gMsgCommand.destination_id, gMsgCommand.source_id, gMsgCommand.register_id, MSG__RESPONSE, 0);

            // FIXME - wykonywanie polecenia tylko w przypadku gdy nie ma flag bledy w response
            comm__execute(&gMsgCommand, &gMsgResponse);

            char * ba = msg__to_byte_array(&gMsgResponse);
            serial__write(halHandle->sciBHandle, ba, msg__get_length(&gMsgResponse));
            free(ba);

            msg__clearMessage(&gMsgCommand);
            msg__clearMessage(&gMsgResponse);
        }

    }

    // Dis-able the Library internal PI.  Iq has no reference now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, false);

    // loop while the enable system flag is true
    while(gMotorVars.Flag_enableSys)
      {
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;
        ST_Obj *stObj = (ST_Obj *)stHandle;

        // analyse buffer from serial communication
//        serial__analyse_buffer(halHandle->sciBHandle);
        if (buffer__analyse_buffer(_RX_SCI_buf, &_RX_SCI_ptr,  &__flag_buffer_rx_is_changed, &_node_id, &gMsgCommand) ) {

            // create response from command data
            msg__set_header(&gMsgResponse, gMsgCommand.destination_id, gMsgCommand.source_id, gMsgCommand.register_id, MSG__RESPONSE, 0);

            // FIXME - wykonywanie polecenia tylko w przypadku gdy nie ma flag bledy w response
            comm__execute(&gMsgCommand, &gMsgResponse);

            char * ba = msg__to_byte_array(&gMsgResponse);
            serial__write(halHandle->sciBHandle, ba, msg__get_length(&gMsgResponse));
            free(ba);

            msg__clearMessage(&gMsgCommand);
            msg__clearMessage(&gMsgResponse);

        }

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle,gMotorVars.Flag_enableUserParams);

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(obj->estHandle,gMotorVars.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc);


        if(CTRL_isError(ctrlHandle))
          {
            // set the enable controller flag to false
            CTRL_setFlag_enableCtrl(ctrlHandle,false);

            // set the enable system flag to false
            gMotorVars.Flag_enableSys = false;

            // disable the PWM
            HAL_disablePwm(halHandle);
          }
        else
          {
            // update the controller state
            bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

            // enable or disable the control
            CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

            if(flag_ctrlStateChanged)
              {
                CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                if(ctrlState == CTRL_State_OffLine)
                  {
                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_OnLine)
                  {
                    if(gMotorVars.Flag_enableOffsetcalc == true)
                    {
                      // update the ADC bias values
                      HAL_updateAdcBias(halHandle);
                    }
                    else
                    {
                      // set the current bias
                      HAL_setBias(halHandle,HAL_SensorType_Current, 0,_IQ(I_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current, 1,_IQ(I_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Current, 2,_IQ(I_C_offset));

                      // set the voltage bias
                      HAL_setBias(halHandle,HAL_SensorType_Voltage, 0,_IQ(V_A_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage, 1,_IQ(V_B_offset));
                      HAL_setBias(halHandle,HAL_SensorType_Voltage, 2,_IQ(V_C_offset));
                    }

                    // Return the bias value for currents
                    gMotorVars.I_bias.value[0] = HAL_getBias(halHandle, HAL_SensorType_Current,0);
                    gMotorVars.I_bias.value[1] = HAL_getBias(halHandle, HAL_SensorType_Current,1);
                    gMotorVars.I_bias.value[2] = HAL_getBias(halHandle, HAL_SensorType_Current,2);

                    // Return the bias value for voltages
                    gMotorVars.V_bias.value[0] = HAL_getBias(halHandle, HAL_SensorType_Voltage,0);
                    gMotorVars.V_bias.value[1] = HAL_getBias(halHandle, HAL_SensorType_Voltage,1);
                    gMotorVars.V_bias.value[2] = HAL_getBias(halHandle, HAL_SensorType_Voltage,2);

                    // enable the PWM
                    HAL_enablePwm(halHandle);
                  }
                else if(ctrlState == CTRL_State_Idle)
                  {
                    // disable the PWM
                    HAL_disablePwm(halHandle);
                    gMotorVars.Flag_Run_Identify = false;
                  }

                if((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                  (ctrlState > CTRL_State_Idle) &&
                  (gMotorVars.CtrlVersion.minor == 6))
                  {
                    // call this function to fix 1p6
                    USER_softwareUpdate1p6(ctrlHandle);
                  }

              }
          }


        if(EST_isMotorIdentified(obj->estHandle))
          {
            // set the current ramp
            EST_setMaxCurrentSlope_pu(obj->estHandle,gMaxCurrentSlope);
            gMotorVars.Flag_MotorIdentified = true;

            // set the speed reference
            CTRL_setSpd_ref_krpm(ctrlHandle, STPOSMOVE_getVelocityReference(stObj->posMoveHandle));

            // set the speed acceleration
            CTRL_setMaxAccel_pu(ctrlHandle, _IQmpy(MAX_ACCEL_KRPMPS_SF,gMotorVars.MaxAccel_krpmps));

            // enable the SpinTAC Position Controller
            STPOSCTL_setEnable(stObj->posCtlHandle, true);

            if(EST_getState(obj->estHandle) != EST_State_OnLine)
            {
            	// if the system is not running, disable SpinTAC Position Controller
        	    STPOSCTL_setEnable(stObj->posCtlHandle, false);
			    // If motor is not running, feed the position feedback into SpinTAC Position Move
				STPOSMOVE_setPositionStart_mrev(stObj->posMoveHandle, STPOSCONV_getPosition_mrev(stObj->posConvHandle));
            }

            if(Flag_Latch_softwareUpdate)
            {
              Flag_Latch_softwareUpdate = false;

              USER_calcPIgains(ctrlHandle);

              // initialize the watch window kp and ki current values with pre-calculated values
              gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle, CTRL_Type_PID_Id);
              gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle, CTRL_Type_PID_Id);


			  // initialize the watch window Bw value with the default value
              gMotorVars.SpinTAC.PosCtlBw_radps = STPOSCTL_getBandwidth_radps(stObj->posCtlHandle);

              // initialize the watch window with maximum and minimum Iq reference
              gMotorVars.SpinTAC.PosCtlOutputMax_A = _IQmpy(STPOSCTL_getOutputMaximum(stObj->posCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
              gMotorVars.SpinTAC.PosCtlOutputMin_A = _IQmpy(STPOSCTL_getOutputMinimum(stObj->posCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));
            }

          }
        else
          {
            Flag_Latch_softwareUpdate = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }


        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            updateGlobalVariables_motor(ctrlHandle, stHandle);
          }


        // update Kp and Ki gains
        updateKpKiGains(ctrlHandle);

        // set the SpinTAC (ST) bandwidth scale
        STPOSCTL_setBandwidth_radps(stObj->posCtlHandle, gMotorVars.SpinTAC.PosCtlBw_radps);

        // set the maximum and minimum values for Iq reference
        STPOSCTL_setOutputMaximums(stObj->posCtlHandle, _IQmpy(gMotorVars.SpinTAC.PosCtlOutputMax_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)), _IQmpy(gMotorVars.SpinTAC.PosCtlOutputMin_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

#ifdef DRV8301_SPI
        HAL_writeDrvData(halHandle, &gDrvSpi8301Vars);

        HAL_readDrvData(halHandle, &gDrvSpi8301Vars);
#endif
#ifdef DRV8305_SPI
        HAL_writeDrvData(halHandle, &gDrvSpi8305Vars);

        HAL_readDrvData(halHandle, &gDrvSpi8305Vars);
#endif
      } // end of while(gFlag_enableSys) loop


    // disable the PWM
    HAL_disablePwm(halHandle);

    // set the default controller parameters (Reset the control to re-identify the motor)
    CTRL_setParams(ctrlHandle,&gUserParams);
    gMotorVars.Flag_Run_Identify = false;

    // setup the SpinTAC Components
    ST_setupPosConv(stHandle);
    ST_setupPosCtl(stHandle);
    ST_setupPosMove(stHandle);
    gMotorVars.MaxVel_krpm = 0;

  } // end of for(;;) loop

} // end of main() function


interrupt void mainISR(void)
{

  static uint16_t stCnt = 0;
  CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

  // toggle status LED
  if(++gLEDcnt >= (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
  {
    HAL_toggleLed(halHandle, (GPIO_Number_e) HAL_Gpio_LED2);
    gLEDcnt = 0;
    // ----
//    serial__write(halHandle->sciBHandle, "0", 1);
    gUART_main = 1;
  }
    // ***************************************************************************************
    // Calculate motor position
    gResultsVars.__motor_prev_position = gResultsVars.__motor_position;
    gResultsVars.__motor_position      = HAL_getQepPosnCounts(halHandle);
    comm__isr(ctrlHandle);
   // ***************************************************************************************

    // compute the electrical angle
    ENC_calcElecAngle(encHandle, gResultsVars.__motor_position);


  // acknowledge the ADC interrupt
  HAL_acqAdcInt(halHandle,ADC_IntNumber_1);


  // convert the ADC data
  HAL_readAdcData(halHandle,&gAdcData);


  // Run the SpinTAC Components
  if(stCnt++ >= ISR_TICKS_PER_SPINTAC_TICK) {
	  ST_runPosConv(stHandle, encHandle, ctrlHandle);
	  ST_runPosMove(stHandle);
	  ST_runPosCtl(stHandle, ctrlHandle);
	  stCnt = 1;
  }


  if(USER_MOTOR_TYPE == MOTOR_Type_Induction) {
    // update the electrical angle for the SLIP module
    SLIP_setElectricalAngle(slipHandle, ENC_getElecAngle(encHandle));
    // compute the amount of slip
    SLIP_run(slipHandle);


    // run the controller
    CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,SLIP_getMagneticAngle(slipHandle));
  }
  else {
    // run the controller
    CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,ENC_getElecAngle(encHandle));
  }

  // write the PWM compare values
  HAL_writePwmData(halHandle,&gPwmData);


  // setup the controller
  CTRL_setup(ctrlHandle);

  // if we are forcing alignment, using the Rs Recalculation, align the eQEP angle with the rotor angle
  if((EST_getState(obj->estHandle) == EST_State_Rs) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
  {
	  ENC_setZeroOffset(encHandle, (uint32_t)(HAL_getQepPosnMaximum(halHandle) - HAL_getQepPosnCounts(halHandle)));
  }

  return;
} // end of mainISR() function

//! \brief the ISR for SCI-B receive interrupt
interrupt void sciBRxISR(void)
{
    uint16_t success;
    uint16_t dataRx;

    HAL_Obj *obj = (HAL_Obj *)halHandle;
    dataRx  = SCI_getDataNonBlocking(halHandle->sciBHandle, &success);
    // acknowledge interrupt from SCI group so that SCI interrupt
    // is not received twice
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);

    buffer__add_to_buffer(_RX_SCI_buf, &_RX_SCI_ptr,  &__flag_buffer_rx_is_changed, dataRx);

} // end of sciBRxISR() function

//! \brief the ISR for SPI-B send-receive interrupt
interrupt void spiISR(void) {
    HAL_acqTimer1Int(halHandle);
    number_of_fsrs = 0;
    number_of_channels = 0;

    for(number_of_fsrs; number_of_fsrs < 10; number_of_fsrs = number_of_fsrs+1){
        channel = 0x00;
        change_leg();
        HAL_setGpioLow(halHandle, cs);
        for(number_of_channels; number_of_channels < 10; number_of_channels = number_of_channels+1){
            result = HAL_SPIwrite(halHandle, actual_mode);
            result = result >> 4;
            change_leg();
            if(number_of_channels >= 1){
                every_board[number_of_fsrs].results[(number_of_channels-1)%9][number_of_interrupt] = result;
            }
        }
        HAL_setGpioHigh(halHandle, cs);
        number_of_channels = 0;
    }
    number_of_interrupt = number_of_interrupt +1;
    if(number_of_interrupt == 5){
        av_i = 0;
        av_temp = 0;
        number_of_fsrs = 0;
        for(number_of_fsrs; number_of_fsrs < 10; number_of_fsrs = number_of_fsrs+1){
            for(number_of_channels; number_of_channels < 9; number_of_channels = number_of_channels+1){
                for(av_i; av_i < 5; av_i = av_i + 1){
                    av_temp = av_temp + every_board[number_of_fsrs].results[number_of_channels][av_i];
                }
                every_board[number_of_fsrs].av_results[number_of_channels] = av_temp/5;
                av_temp = 0;
                av_i = 0;
            }
            number_of_channels = 0;
        }
        number_of_interrupt = 0;
    }
    return;
} // end of spiISR() function

void change_leg(void)
{
    uint16_t temp = channel;
    actual_mode = ((temp << 4) + resolution_shift + polar_bit) << 8;
    channel = (channel+1) % 9;
}

void updateGlobalVariables_motor(CTRL_Handle handle, ST_Handle sthandle)
{
  uint32_t ProTime_tick, ProTime_mtick;
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  ST_Obj *stObj = (ST_Obj *)sthandle;

  // get the speed estimate
  gMotorVars.Speed_krpm = _IQmpy(STPOSCONV_getVelocityFiltered(stObj->posConvHandle), _IQ(ST_SPEED_KRPM_PER_PU));

  // get the position error
  gMotorVars.PositionError_MRev = STPOSCTL_getPositionError_mrev(stObj->posCtlHandle);

  // get the torque estimate
  gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

  // get the magnetizing current
  gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);

  // get the rotor resistance
  gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);

  // get the stator resistance
  gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);

  // get the stator inductance in the direct coordinate direction
  gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);

  // get the stator inductance in the quadrature coordinate direction
  gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);

  // get the flux in V/Hz in floating point
  gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);

  // get the flux in Wb in fixed point
  gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

  // get the controller state
  gMotorVars.CtrlState = CTRL_getState(handle);

  // get the estimator state
  gMotorVars.EstState = EST_getState(obj->estHandle);

  // Get the DC buss voltage
  gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus,_IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

  // get the Iq reference from the position controller
  gMotorVars.IqRef_A = _IQmpy(STPOSCTL_getTorqueReference(stObj->posCtlHandle), _IQ(USER_IQ_FULL_SCALE_CURRENT_A));

  // gets the Position Controller status
  gMotorVars.SpinTAC.PosCtlStatus = STPOSCTL_getStatus(stObj->posCtlHandle);
	
  // get the inertia setting
  gMotorVars.SpinTAC.InertiaEstimate_Aperkrpm = _IQmpy(STPOSCTL_getInertia(stObj->posCtlHandle), _IQ(ST_SPEED_PU_PER_KRPM * USER_IQ_FULL_SCALE_CURRENT_A));

  // get the friction setting
  gMotorVars.SpinTAC.FrictionEstimate_Aperkrpm = _IQmpy(STPOSCTL_getFriction(stObj->posCtlHandle), _IQ(ST_SPEED_PU_PER_KRPM * USER_IQ_FULL_SCALE_CURRENT_A));

  // get the Position Controller error
  gMotorVars.SpinTAC.PosCtlErrorID = STPOSCTL_getErrorID(stObj->posCtlHandle);

  // get the Position Move status
  gMotorVars.SpinTAC.PosMoveStatus = STPOSMOVE_getStatus(stObj->posMoveHandle);

  // get the Position Move profile time
  STPOSMOVE_getProfileTime_tick(stObj->posMoveHandle, &ProTime_tick, &ProTime_mtick);
  gMotorVars.SpinTAC.PosMoveTime_ticks = ProTime_tick;
  gMotorVars.SpinTAC.PosMoveTime_mticks = ProTime_mtick;

  // get the Position Move error
  gMotorVars.SpinTAC.PosMoveErrorID = STPOSMOVE_getErrorID(stObj->posMoveHandle);
  
  // get the Position Converter error
  gMotorVars.SpinTAC.PosConvErrorID = STPOSCONV_getErrorID(stObj->posConvHandle);

  return;
} // end of updateGlobalVariables_motor() function


void updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_spd,gMotorVars.Kp_spd);
      CTRL_setKi(handle,CTRL_Type_PID_spd,gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle,CTRL_Type_PID_Id,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Id,gMotorVars.Ki_Idq);
      CTRL_setKp(handle,CTRL_Type_PID_Iq,gMotorVars.Kp_Idq);
      CTRL_setKi(handle,CTRL_Type_PID_Iq,gMotorVars.Ki_Idq);
	}

  return;
} // end of updateKpKiGains() function


void ST_runPosConv(ST_Handle handle, ENC_Handle encHandle, CTRL_Handle ctrlHandle)
{
	ST_Obj *stObj = (ST_Obj *)handle;

	// get the electrical angle from the ENC module
    STPOSCONV_setElecAngle_erev(stObj->posConvHandle, ENC_getElecAngle(encHandle));

    if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
      // The CurrentVector feedback is only needed for ACIM
      // get the vector of the direct/quadrature current input vector values from CTRL
      STPOSCONV_setCurrentVector(stObj->posConvHandle, CTRL_getIdq_in_addr(ctrlHandle));
    }

	// run the SpinTAC Position Converter
	STPOSCONV_run(stObj->posConvHandle);

	if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
	  // The Slip Velocity is only needed for ACIM
	  // update the slip velocity in electrical angle per second, Q24
	  SLIP_setSlipVelocity(slipHandle, STPOSCONV_getSlipVelocity(stObj->posConvHandle));
	}
}

void ST_runPosCtl(ST_Handle handle, CTRL_Handle ctrlHandle)
{
	ST_Obj *stObj = (ST_Obj *)handle;

	// provide the updated references to the SpinTAC Position Control
	STPOSCTL_setPositionReference_mrev(stObj->posCtlHandle, STPOSMOVE_getPositionReference_mrev(stObj->posMoveHandle));
	STPOSCTL_setVelocityReference(stObj->posCtlHandle, STPOSMOVE_getVelocityReference(stObj->posMoveHandle));
	STPOSCTL_setAccelerationReference(stObj->posCtlHandle, STPOSMOVE_getAccelerationReference(stObj->posMoveHandle));
	// provide the feedback to the SpinTAC Position Control
	STPOSCTL_setPositionFeedback_mrev(stObj->posCtlHandle, STPOSCONV_getPosition_mrev(stObj->posConvHandle));

	// Run SpinTAC Position Control
	STPOSCTL_run(stObj->posCtlHandle);
	
	// Provide SpinTAC Position Control Torque Output to the FOC
	CTRL_setIq_ref_pu(ctrlHandle, STPOSCTL_getTorqueReference(stObj->posCtlHandle));
}

void ST_runPosMove(ST_Handle handle)
{
	ST_Obj *stObj = (ST_Obj *)handle;

	// Run SpinTAC Position Profile Generator
	// If we are not running a profile, and command indicates we should has been modified
//	if((STPOSMOVE_getStatus(stObj->posMoveHandle) == ST_MOVE_IDLE) && (gMotorVars.RunPositionProfile == true)) {
//	if(gParametersVars.position_goal_changed_flag == true) {
//		// Get the configuration for SpinTAC Position Move
//		STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
//		STPOSMOVE_setPositionStep_mrev(stObj->posMoveHandle, gMotorVars.PosStepInt_MRev,  gMotorVars.PosStepFrac_MRev);
//		STPOSMOVE_setVelocityLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
//		STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
//		STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
//		STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
//		// Enable the SpinTAC Position Profile Generator
//		STPOSMOVE_setEnable(stObj->posMoveHandle, true);
//
//		// clear the position step command
//		gMotorVars.PosStepInt_MRev = 0;
//		gMotorVars.PosStepFrac_MRev = 0;
//		gMotorVars.RunPositionProfile = false;
//
//		gParametersVars.position_goal_changed_flag = false; // disable flag
//	}
//
//    // Run SpinTAC Position Profile Generator
//    // If we are not running a profile, and the PosStep_MRev has been modified
////    if(_IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)) != STPOSMOVE_getVelocityEnd(stObj->posMoveHandle)) {
//    if(gParametersVars.velocity_goal_changed_flag == true) {
//        // Get the configuration for SpinTAC Velocity Profile Generator
//        STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
//        STPOSMOVE_setProfileType(stObj->posMoveHandle, ST_POS_MOVE_VEL_TYPE);
//        STPOSMOVE_setVelocityEnd(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
//        STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
//        STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
//        STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
//        // Enable the SpinTAC Position Profile Generator
//        STPOSMOVE_setEnable(stObj->posMoveHandle, true);
//
//        gParametersVars.velocity_goal_changed_flag = false; // disable flag
//    }

    if(gParametersVars.goal_mode == 4) {
          // Get the configuration for SpinTAC Position Move
        STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
        STPOSMOVE_setProfileType(stObj->posMoveHandle, ST_POS_MOVE_POS_TYPE);
        STPOSMOVE_setPositionStep_mrev(stObj->posMoveHandle, gMotorVars.PosStepInt_MRev,  gMotorVars.PosStepFrac_MRev);
        STPOSMOVE_setVelocityLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
        // Enable the SpinTAC Position Profile Generator
        STPOSMOVE_setEnable(stObj->posMoveHandle, true);

        // clear the position step command
        gMotorVars.PosStepInt_MRev = 0;
        gMotorVars.PosStepFrac_MRev = 0;
        gMotorVars.RunPositionProfile = false;

        gParametersVars.goal_mode ^= 0b00000100;  // disable flag
    }

  if(gParametersVars.goal_mode == 5) {
      // Get the configuration for SpinTAC Velocity Profile Generator
      gParametersVars.positon_set_zero = true;
      STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
      STPOSMOVE_setProfileType(stObj->posMoveHandle, ST_POS_MOVE_VEL_TYPE);
      STPOSMOVE_setVelocityEnd(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
      STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
      STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
      STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
      // Enable the SpinTAC Position Profile Generator
      STPOSMOVE_setEnable(stObj->posMoveHandle, true);

      gParametersVars.goal_mode = 4;  // disable flag

  }

    if(gParametersVars.goal_mode == 6 || gParametersVars.goal_mode == 7) {
        // Get the configuration for SpinTAC Velocity Profile Generator
        STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
        STPOSMOVE_setProfileType(stObj->posMoveHandle, ST_POS_MOVE_VEL_TYPE);
        STPOSMOVE_setVelocityEnd(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
        // Enable the SpinTAC Position Profile Generator
        STPOSMOVE_setEnable(stObj->posMoveHandle, true);

        gParametersVars.goal_mode ^= 0b00000100;  // disable flag
      }

	STPOSMOVE_run(stObj->posMoveHandle);
}


//@} //defgroup
// end of file
