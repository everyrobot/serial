/*
 * er_bldc.c
 *
 *  Created on: 3 gru 2019
 *      Author: lukasz.lecki
 */

#include "er_bldc.h"

void bldc__init() {

    uint_least8_t cnt;
    uint_least8_t estNumber = 0;

    #ifdef FAST_ROM_V1p6
        uint_least8_t ctrlNumber = 0;
    #endif

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

    gBLDCConfigurationsVars.motor_pole_pairs = gUserParams.motor_numPolePairs;
    gBLDCConfigurationsVars.encoder_resolution_ppr = USER_MOTOR_ENCODER_LINES * 4;


    // !!!       HAL_setParams(halHandle);

    _iq beta_lp_pu = _IQ(gUserParams.offsetPole_rps / (float_t) gUserParams.ctrlFreq_Hz); // FIXME move to er_bldc
    HAL_setNumCurrentSensors(halHandle, gUserParams.numCurrentSensors); // FIXME move to er_bldc
    HAL_setNumVoltageSensors(halHandle, gUserParams.numVoltageSensors); // FIXME move to er_bldc
    for(cnt=0;cnt<HAL_getNumCurrentSensors(halHandle);cnt++) // FIXME move to er_bldc
    {
        HAL_setOffsetBeta_lp_pu(halHandle, HAL_SensorType_Current, cnt, beta_lp_pu); // FIXME move to er_bldc
        HAL_setOffsetInitCond(halHandle, HAL_SensorType_Current, cnt, _IQ(0.0)); // FIXME move to er_bldc
        HAL_setOffsetValue(halHandle, HAL_SensorType_Current, cnt, _IQ(0.0)); // FIXME move to er_bldc
    }

    for(cnt=0;cnt<HAL_getNumVoltageSensors(halHandle);cnt++) // FIXME move to er_bldc
    {
        HAL_setOffsetBeta_lp_pu(halHandle, HAL_SensorType_Voltage, cnt, beta_lp_pu); // FIXME move to er_bldc
        HAL_setOffsetInitCond(halHandle, HAL_SensorType_Voltage, cnt, _IQ(0.0)); // FIXME move to er_bldc
        HAL_setOffsetValue(halHandle, HAL_SensorType_Voltage, cnt, _IQ(0.0)); // FIXME move to er_bldc
    }

    // GPIO
    // PWM1
    GPIO_setMode(halHandle->gpioHandle, GPIO_Number_0, GPIO_0_Mode_EPWM1A);

    // PWM2
    GPIO_setMode(halHandle->gpioHandle, GPIO_Number_1, GPIO_1_Mode_EPWM1B);

    // PWM3
    GPIO_setMode(halHandle->gpioHandle, GPIO_Number_2, GPIO_2_Mode_EPWM2A);

    // PWM4
    GPIO_setMode(halHandle->gpioHandle, GPIO_Number_3, GPIO_3_Mode_EPWM2B);

    // PWM5
    GPIO_setMode(halHandle->gpioHandle, GPIO_Number_4, GPIO_4_Mode_EPWM3A);

    // PWM6
    GPIO_setMode(halHandle->gpioHandle, GPIO_Number_5, GPIO_5_Mode_EPWM3B);

    // GPIO
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_13,GPIO_13_Mode_GeneralPurpose);

    // SPIA SIMO
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_16,GPIO_16_Mode_SPISIMOA);

    // SPIA SOMI
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_17,GPIO_17_Mode_SPISOMIA);

    // SPIA CLK
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_18,GPIO_18_Mode_SPICLKA);

    // SPIA CS
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_19,GPIO_19_Mode_SPISTEA_NOT);

    // EQEP1A
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_20,GPIO_20_Mode_EQEP1A);
    GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_20,GPIO_Qual_Sample_3);

    // EQEP1B
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_21,GPIO_21_Mode_EQEP1B);
    GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_21,GPIO_Qual_Sample_3);

    // GPIO
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_22,GPIO_22_Mode_GeneralPurpose);

    // EQEP1I
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_23,GPIO_23_Mode_EQEP1I);
    GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_23,GPIO_Qual_Sample_3);

    // OCTWn
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_28,GPIO_28_Mode_TZ2_NOT);

    // FAULTn
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_29,GPIO_29_Mode_TZ3_NOT);

    // DRV8305 Enable Gate
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_50,GPIO_50_Mode_GeneralPurpose);
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_50);
    GPIO_setDirection(halHandle->gpioHandle,GPIO_Number_50,GPIO_Direction_Output);

    // DRV8305 DC Calibration
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_51,GPIO_51_Mode_GeneralPurpose);
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_51);
    GPIO_setDirection(halHandle->gpioHandle,GPIO_Number_51,GPIO_Direction_Output);

    // DRV8305 Enable Gate
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_52,GPIO_52_Mode_GeneralPurpose);
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_52);
    GPIO_setDirection(halHandle->gpioHandle,GPIO_Number_52,GPIO_Direction_Output);

    // DRV8305 Device Calibration
    GPIO_setMode(halHandle->gpioHandle,GPIO_Number_53,GPIO_53_Mode_GeneralPurpose);
    GPIO_setLow(halHandle->gpioHandle,GPIO_Number_53);
    GPIO_setDirection(halHandle->gpioHandle,GPIO_Number_53,GPIO_Direction_Output);

//        #ifdef QEP
//          // EQEP2A
//          GPIO_setMode(halHandle->gpioHandle,GPIO_Number_54,GPIO_54_Mode_EQEP2A);
//          GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_54,GPIO_Qual_Sample_3);
//        //
//        //  // EQEP2B
//          GPIO_setMode(halHandle->gpioHandle,GPIO_Number_55,GPIO_55_Mode_EQEP2B);
//          GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_55,GPIO_Qual_Sample_3);
//        //
//        //  // EQEP2I
//          GPIO_setMode(halHandle->gpioHandle,GPIO_Number_56,GPIO_56_Mode_EQEP2I);
//          GPIO_setQualification(halHandle->gpioHandle,GPIO_Number_56,GPIO_Qual_Sample_3);
//        #endif



    //configure the SOCs for boostxldrv8305_revB on J1 Connection FIXME
    // EXT IA-FB
    ADC_setSocChanNumber(halHandle->adcHandle,ADC_SocNumber_0,ADC_SocChanNumber_A0);
    ADC_setSocTrigSrc(halHandle->adcHandle,ADC_SocNumber_0,ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(halHandle->adcHandle,ADC_SocNumber_0,ADC_SocSampleDelay_9_cycles);

    // EXT IA-FB
    // Duplicate conversion due to ADC Initial Conversion bug (SPRZ342)
    ADC_setSocChanNumber(halHandle->adcHandle,ADC_SocNumber_1,ADC_SocChanNumber_A0);
    ADC_setSocTrigSrc(halHandle->adcHandle,ADC_SocNumber_1,ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(halHandle->adcHandle,ADC_SocNumber_1,ADC_SocSampleDelay_9_cycles);

    // EXT IB-FB
    ADC_setSocChanNumber(halHandle->adcHandle,ADC_SocNumber_2,ADC_SocChanNumber_B0);
    ADC_setSocTrigSrc(halHandle->adcHandle,ADC_SocNumber_2,ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(halHandle->adcHandle,ADC_SocNumber_2,ADC_SocSampleDelay_9_cycles);

    // EXT IC-FB
    ADC_setSocChanNumber(halHandle->adcHandle,ADC_SocNumber_3,ADC_SocChanNumber_A1);
    ADC_setSocTrigSrc(halHandle->adcHandle,ADC_SocNumber_3,ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(halHandle->adcHandle,ADC_SocNumber_3,ADC_SocSampleDelay_9_cycles);

    // ADC-Vhb1
    ADC_setSocChanNumber(halHandle->adcHandle,ADC_SocNumber_4,ADC_SocChanNumber_A7);
    ADC_setSocTrigSrc(halHandle->adcHandle,ADC_SocNumber_4,ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(halHandle->adcHandle,ADC_SocNumber_4,ADC_SocSampleDelay_9_cycles);

    // ADC-Vhb2
    ADC_setSocChanNumber(halHandle->adcHandle,ADC_SocNumber_5,ADC_SocChanNumber_B1);
    ADC_setSocTrigSrc(halHandle->adcHandle,ADC_SocNumber_5,ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(halHandle->adcHandle,ADC_SocNumber_5,ADC_SocSampleDelay_9_cycles);

    // ADC-Vhb3
    ADC_setSocChanNumber(halHandle->adcHandle,ADC_SocNumber_6,ADC_SocChanNumber_A2);
    ADC_setSocTrigSrc(halHandle->adcHandle,ADC_SocNumber_6,ADC_SocTrigSrc_EPWM1_ADCSOCA);
    ADC_setSocSampleDelay(halHandle->adcHandle,ADC_SocNumber_6,ADC_SocSampleDelay_9_cycles);

    // setup the PWMs
    HAL_setupPwms(halHandle,
                      (float_t) gUserParams.systemFreq_MHz,
                      gUserParams.pwmPeriod_usec,
                      USER_NUM_PWM_TICKS_PER_ISR_TICK);

    // setup the QEP
    bldc__HAL_setupQEP(halHandle, HAL_Qep_QEP1);
    //            bldc__HAL_setupQEP(halHandle, HAL_Qep_QEP2);

    // setup the spiA
    bldc__HAL_setupSpiA(halHandle);


    // setup the drv8305 interface
    bldc__HAL_setupGate(halHandle);


    // set the default current bias
    {
        uint_least8_t cnt;
        _iq bias = _IQ12mpy(ADC_dataBias,_IQ(gUserParams.current_sf));

        for(cnt=0;cnt<HAL_getNumCurrentSensors(halHandle);cnt++)
        {
            HAL_setBias(halHandle,HAL_SensorType_Current,cnt,bias);
        }
    }


    //  set the current scale factor
    {
         _iq current_sf = _IQ(gUserParams.current_sf);

        HAL_setCurrentScaleFactor(halHandle,current_sf);
    }


    // set the default voltage bias
   {
     uint_least8_t cnt;
     _iq bias = _IQ(0.0);

     for(cnt=0;cnt<HAL_getNumVoltageSensors(halHandle);cnt++)
       {
         HAL_setBias(halHandle,HAL_SensorType_Voltage,cnt,bias);
       }
   }


    //  set the voltage scale factor
   {
     _iq voltage_sf = _IQ(gUserParams.voltage_sf);

    HAL_setVoltageScaleFactor(halHandle,voltage_sf);
   }

   // END OF         HAL_setParams(halHandle); // Cz� dotycz�ca BLDC

    // initialize the controller - BLDC
    #ifdef FAST_ROM_V1p6
        ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);          //v1p6 format (06xF and 06xM devices)
        controller_obj = (CTRL_Obj *) ctrlHandle;
    #else
        ctrlHandle = CTRL_initCtrl(estNumber, &ctrl, sizeof(ctrl));   //v1p7 format default
    #endif

    { // BLDC
        CTRL_Version version;

        // get the version number
        CTRL_getVersion(ctrlHandle,&version);

        gMotorVars.CtrlVersion = version;
    }

    // set the default controller parameters
    CTRL_setParams(ctrlHandle, &gUserParams); // BLDC


    // setup faults
    HAL_setupFaults(halHandle); // BLDC

    // disable the PWM
    HAL_disablePwm(halHandle); // BLDC

    // initialize the ENC module
    encHandle = ENC_init(&enc, sizeof(enc)); // BLDC

    // setup the ENC module
    ENC_setup(encHandle, 1, USER_MOTOR_NUM_POLE_PAIRS, USER_MOTOR_ENCODER_LINES, 0, USER_IQ_FULL_SCALE_FREQ_Hz, USER_ISR_FREQ_Hz, 8000.0); // BLDC

    // initialize the SLIP module
    //          slipHandle = SLIP_init(&slip, sizeof(slip));
    // setup the SLIP module
    //          SLIP_setup(slipHandle, _IQ(gUserParams.ctrlPeriod_sec));

    // initialize the SpinTAC Components
    stHandle = ST_init(&st_obj, sizeof(st_obj));

    // setup the SpinTAC Components
    ST_setupPosConv(stHandle);
    ST_setupPosCtl(stHandle);
    ST_setupPosMove(stHandle);
    gMotorVars.MaxVel_krpm = 0;

    // turn on the DRV8305 if present
    HAL_enableDrv(halHandle);

    // initialize the DRV8305 interface
    bldc__HAL_setupDrvSpi(halHandle, &gDrvSpi8305Vars);

    // enable DC bus compensation
    CTRL_setFlag_enableDcBusComp(ctrlHandle, true);

    // compute scaling factors for flux and torque calculations
    gFlux_pu_to_Wb_sf             = USER_computeFlux_pu_to_Wb_sf();
    gFlux_pu_to_VpHz_sf           = USER_computeFlux_pu_to_VpHz_sf();
    gTorque_Ls_Id_Iq_pu_to_Nm_sf  = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
    gTorque_Flux_Iq_pu_to_Nm_sf   = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

    // Dis-able the Library internal PI.  Iq has no reference now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, false);
}

void bldc__update_state(HAL_Handle halHandle, CTRL_Handle ctrlHandle, ST_Handle stHandle) {
    if(gMotorVars.Flag_enableSys) {
        CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;
        ST_Obj *stObj = (ST_Obj *)stHandle;
//        CTRL_Obj *obj = (CTRL_Obj *) &ctrlHandle;
//        ST_Obj *stObj = (ST_Obj *) &stHandle;

//         analyse buffer from serial communication
//        if (buffer__analyse_buffer(_RX_SCI_buf, &_RX_SCI_ptr,  &__flag_buffer_rx_is_changed, &_node_id, &gMsgCommand) ) {
//
//            // create response from command data
//            msg__set_header(&gMsgResponse, gMsgCommand.destination_id, gMsgCommand.source_id, gMsgCommand.register_id, MSG__RESPONSE, 0);
//
//            // FIXME - wykonywanie polecenia tylko w przypadku gdy nie ma flag bledy w response
//            comm__execute(&gMsgCommand, &gMsgResponse);
//
//            char * ba = msg__to_byte_array(&gMsgResponse);
//            serial__write(halHandle->sciBHandle, ba, msg__get_length(&gMsgResponse));
//            free(ba);
//
//            msg__clearMessage(&gMsgCommand);
//            msg__clearMessage(&gMsgResponse);
//
//        }

        // increment counters
        gCounter_updateGlobals++;

        // enable/disable the use of motor parameters being loaded from user.h
        CTRL_setFlag_enableUserMotorParams(ctrlHandle, gMotorVars.Flag_enableUserParams);

        // enable/disable Rs recalibration during motor startup
        EST_setFlag_enableRsRecalc(obj->estHandle, gMotorVars.Flag_enableRsRecalc);

        // enable/disable automatic calculation of bias values
        CTRL_setFlag_enableOffset(ctrlHandle, gMotorVars.Flag_enableOffsetcalc);


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
            EST_setMaxCurrentSlope_pu(obj->estHandle, gMaxCurrentSlope);
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

            if(gFlag_Latch_softwareUpdate)
            {
              gFlag_Latch_softwareUpdate = false;

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
            gFlag_Latch_softwareUpdate = true;

            // the estimator sets the maximum current slope during identification
            gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
          }


        // when appropriate, update the global variables
        if(gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
          {
            // reset the counter
            gCounter_updateGlobals = 0;

            bldc__updateGlobalVariables_motor(ctrlHandle, stHandle);
          }


        // update Kp and Ki gains
        bldc__updateKpKiGains(ctrlHandle);

        // set the SpinTAC (ST) bandwidth scale
        STPOSCTL_setBandwidth_radps(stObj->posCtlHandle, gMotorVars.SpinTAC.PosCtlBw_radps);

        // set the maximum and minimum values for Iq reference
        STPOSCTL_setOutputMaximums(stObj->posCtlHandle, _IQmpy(gMotorVars.SpinTAC.PosCtlOutputMax_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)), _IQmpy(gMotorVars.SpinTAC.PosCtlOutputMin_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A)));

        // enable/disable the forced angle
        EST_setFlag_enableForceAngle(obj->estHandle,gMotorVars.Flag_enableForceAngle);

        // enable or disable power warp
        CTRL_setFlag_enablePowerWarp(ctrlHandle,gMotorVars.Flag_enablePowerWarp);

        bldc__HAL_writeDrvData(halHandle, &gDrvSpi8305Vars);

        bldc__HAL_readDrvData(halHandle, &gDrvSpi8305Vars);
    } // end of while(gFlag_enableSys) loop
}

void bldc__updateKpKiGains(CTRL_Handle handle)
{
  if((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (gFlag_Latch_softwareUpdate == false))
    {
      // set the kp and ki speed values from the watch window
      CTRL_setKp(handle, CTRL_Type_PID_spd, gMotorVars.Kp_spd);
      CTRL_setKi(handle, CTRL_Type_PID_spd, gMotorVars.Ki_spd);

      // set the kp and ki current values for Id and Iq from the watch window
      CTRL_setKp(handle, CTRL_Type_PID_Id, gMotorVars.Kp_Idq);
      CTRL_setKi(handle, CTRL_Type_PID_Id, gMotorVars.Ki_Idq);
      CTRL_setKp(handle, CTRL_Type_PID_Iq, gMotorVars.Kp_Idq);
      CTRL_setKi(handle, CTRL_Type_PID_Iq, gMotorVars.Ki_Idq);
    }

  return;
} // end of updateKpKiGains() function


void bldc__ST_runPosConv(ST_Handle handle, ENC_Handle encHandle, CTRL_Handle ctrlHandle)
{
    ST_Obj *stObj = (ST_Obj *)handle;

    // get the electrical angle from the ENC module
    STPOSCONV_setElecAngle_erev(stObj->posConvHandle, ENC_getElecAngle(encHandle));

    // run the SpinTAC Position Converter
    STPOSCONV_run(stObj->posConvHandle);

}

void bldc__ST_runPosCtl(ST_Handle handle, CTRL_Handle ctrlHandle)
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

void bldc__ST_runPosMove(ST_Handle handle)
{
    ST_Obj *stObj = (ST_Obj *)handle;

    // Run SpinTAC Position Profile Generator
    // If we are not running a profile, and command indicates we should has been modified
//  if((STPOSMOVE_getStatus(stObj->posMoveHandle) == ST_MOVE_IDLE) && (gMotorVars.RunPositionProfile == true)) {
//  if(gParametersVars.position_goal_changed_flag == true) {
//      // Get the configuration for SpinTAC Position Move
//      STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
//      STPOSMOVE_setPositionStep_mrev(stObj->posMoveHandle, gMotorVars.PosStepInt_MRev,  gMotorVars.PosStepFrac_MRev);
//      STPOSMOVE_setVelocityLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
//      STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
//      STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
//      STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
//      // Enable the SpinTAC Position Profile Generator
//      STPOSMOVE_setEnable(stObj->posMoveHandle, true);
//
//      // clear the position step command
//      gMotorVars.PosStepInt_MRev = 0;
//      gMotorVars.PosStepFrac_MRev = 0;
//      gMotorVars.RunPositionProfile = false;
//
//      gParametersVars.position_goal_changed_flag = false; // disable flag
//  }
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

    if(gBLDCParametersVars.goal_mode == 4) {
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

        gBLDCParametersVars.goal_mode ^= 0b00000100;  // disable flag
    }

  if(gBLDCParametersVars.goal_mode == 5) {
      // Get the configuration for SpinTAC Velocity Profile Generator
      gBLDCParametersVars.positon_set_zero = true;
      STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
      STPOSMOVE_setProfileType(stObj->posMoveHandle, ST_POS_MOVE_VEL_TYPE);
      STPOSMOVE_setVelocityEnd(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
      STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
      STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
      STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
      // Enable the SpinTAC Position Profile Generator
      STPOSMOVE_setEnable(stObj->posMoveHandle, true);

      gBLDCParametersVars.goal_mode = 4;  // disable flag

  }

    if(gBLDCParametersVars.goal_mode == 6 || gBLDCParametersVars.goal_mode == 7) {
        // Get the configuration for SpinTAC Velocity Profile Generator
        STPOSMOVE_setCurveType(stObj->posMoveHandle, gMotorVars.SpinTAC.PosMoveCurveType);
        STPOSMOVE_setProfileType(stObj->posMoveHandle, ST_POS_MOVE_VEL_TYPE);
        STPOSMOVE_setVelocityEnd(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxVel_krpm, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setAccelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxAccel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setDecelerationLimit(stObj->posMoveHandle, _IQmpy(gMotorVars.MaxDecel_krpmps, _IQ(ST_SPEED_PU_PER_KRPM)));
        STPOSMOVE_setJerkLimit(stObj->posMoveHandle, _IQ20mpy(gMotorVars.MaxJrk_krpmps2, _IQ20(ST_SPEED_PU_PER_KRPM)));
        // Enable the SpinTAC Position Profile Generator
        STPOSMOVE_setEnable(stObj->posMoveHandle, true);

        gBLDCParametersVars.goal_mode ^= 0b00000100;  // disable flag
      }

    STPOSMOVE_run(stObj->posMoveHandle);
}

void bldc__isr(HAL_Handle halHandle, CTRL_Handle ctrlHandle, ST_Handle stHandle, ENC_Handle encHandle) {
      static uint16_t stCnt = 0;
      CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

    // ***************************************************************************************
    // Calculate motor position
    gBLDCResultsVars.__motor_prev_position = gBLDCResultsVars.__motor_position;
    gBLDCResultsVars.__motor_position      = HAL_getQepPosnCounts(halHandle);
    comm__isr(ctrlHandle);
    // ***************************************************************************************

    // compute the electrical angle
    ENC_calcElecAngle(encHandle, gBLDCResultsVars.__motor_position);

    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle,ADC_IntNumber_1);

    // convert the ADC data
    HAL_readAdcData(halHandle, &gAdcData);

    // Run the SpinTAC Components
    if(stCnt++ >= ISR_TICKS_PER_SPINTAC_TICK) {
        bldc__ST_runPosConv(stHandle, encHandle, ctrlHandle);
        bldc__ST_runPosMove(stHandle);
        bldc__ST_runPosCtl(stHandle, ctrlHandle);
        stCnt = 1;
    }

    // run the controller
    CTRL_run(ctrlHandle,halHandle,&gAdcData,&gPwmData,ENC_getElecAngle(encHandle));

    // write the PWM compare values
    HAL_writePwmData(halHandle,&gPwmData);

    // setup the controller
    CTRL_setup(ctrlHandle);

    // if we are forcing alignment, using the Rs Recalculation, align the eQEP angle with the rotor angle
    if((EST_getState(obj->estHandle) == EST_State_Rs) && (USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
        ENC_setZeroOffset(encHandle, (uint32_t)(HAL_getQepPosnMaximum(halHandle) - HAL_getQepPosnCounts(halHandle)));
    }
}

void bldc__updateGlobalVariables_motor(CTRL_Handle handle, ST_Handle sthandle)
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

void bldc__HAL_writeDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8305_writeData(obj->drv8305Handle,Spi_8305_Vars);

  return;
}  // end of HAL_writeDrvData() function


void bldc__HAL_readDrvData(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8305_readData(obj->drv8305Handle,Spi_8305_Vars);

  return;
}  // end of HAL_readDrvData() function


void bldc__HAL_setupDrvSpi(HAL_Handle handle, DRV_SPI_8305_Vars_t *Spi_8305_Vars)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;

  DRV8305_setupSpi(obj->drv8305Handle,Spi_8305_Vars);

  return;
}  // end of HAL_setupDrvSpi() function

#ifdef QEP
void bldc__HAL_setupQEP(HAL_Handle handle,HAL_QepSelect_e qep)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;


  // hold the counter in reset
  QEP_reset_counter(obj->qepHandle[qep]);

  // set the QPOSINIT register
  QEP_set_posn_init_count(obj->qepHandle[qep], 0);

  // disable all interrupts
  QEP_disable_all_interrupts(obj->qepHandle[qep]);

  // clear the interrupt flags
  QEP_clear_all_interrupt_flags(obj->qepHandle[qep]);

  // clear the position counter
  QEP_clear_posn_counter(obj->qepHandle[qep]);

  // setup the max position
  QEP_set_max_posn_count(obj->qepHandle[qep], (4 * USER_MOTOR_ENCODER_LINES) - 1); // FIXME ????
//  QEP_set_max_posn_count(obj->qepHandle[qep], (USER_MOTOR_ENCODER_LINES - 1) ); // FIXME ????

  // setup the QDECCTL register
  QEP_set_QEP_source(obj->qepHandle[qep], QEP_Qsrc_Quad_Count_Mode);
  QEP_disable_sync_out(obj->qepHandle[qep]);
  QEP_set_swap_quad_inputs(obj->qepHandle[qep], QEP_Swap_Not_Swapped);
  QEP_disable_gate_index(obj->qepHandle[qep]);
  QEP_set_ext_clock_rate(obj->qepHandle[qep], QEP_Xcr_2x_Res);
  QEP_set_A_polarity(obj->qepHandle[qep], QEP_Qap_No_Effect);
  QEP_set_B_polarity(obj->qepHandle[qep], QEP_Qbp_No_Effect);
  QEP_set_index_polarity(obj->qepHandle[qep], QEP_Qip_No_Effect);

  // setup the QEPCTL register
  QEP_set_emu_control(obj->qepHandle[qep], QEPCTL_Freesoft_Unaffected_Halt);
  QEP_set_posn_count_reset_mode(obj->qepHandle[qep], QEPCTL_Pcrm_Max_Reset);
  QEP_set_strobe_event_init(obj->qepHandle[qep], QEPCTL_Sei_Nothing);
  QEP_set_index_event_init(obj->qepHandle[qep], QEPCTL_Iei_Nothing);
  QEP_set_index_event_latch(obj->qepHandle[qep], QEPCTL_Iel_Rising_Edge);
  QEP_set_soft_init(obj->qepHandle[qep], QEPCTL_Swi_Nothing);
  QEP_disable_unit_timer(obj->qepHandle[qep]);
  QEP_disable_watchdog(obj->qepHandle[qep]);

  // setup the QPOSCTL register
  QEP_disable_posn_compare(obj->qepHandle[qep]);

  // setup the QCAPCTL register
  QEP_disable_capture(obj->qepHandle[qep]);

  // renable the position counter
  QEP_enable_counter(obj->qepHandle[qep]);


  return;
}
#endif

void bldc__HAL_setupSpiA(HAL_Handle handle)
{
  HAL_Obj   *obj = (HAL_Obj *)handle;

  SPI_reset(obj->spiAHandle);
  SPI_setMode(obj->spiAHandle,SPI_Mode_Master);
  SPI_setClkPolarity(obj->spiAHandle,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
  SPI_enableTx(obj->spiAHandle);
  SPI_enableTxFifoEnh(obj->spiAHandle);
  SPI_enableTxFifo(obj->spiAHandle);
  SPI_setTxDelay(obj->spiAHandle,0x0018);
  SPI_setBaudRate(obj->spiAHandle,(SPI_BaudRate_e)(0x000d));
  SPI_setCharLength(obj->spiAHandle,SPI_CharLength_16_Bits);
  SPI_setSuspend(obj->spiAHandle,SPI_TxSuspend_free);
  SPI_enable(obj->spiAHandle);

  return;
}  // end of HAL_setupSpiA() function

void bldc__HAL_setupGate(HAL_Handle handle)
{
  HAL_Obj *obj = (HAL_Obj *)handle;

  DRV8305_setGpioHandle(obj->drv8305Handle,obj->gpioHandle);

  DRV8305_setSpiHandle(obj->drv8305Handle,obj->spiAHandle);
  DRV8305_setGpioNumber(obj->drv8305Handle,GPIO_Number_50);

  return;
} // HAL_setupGate() function


