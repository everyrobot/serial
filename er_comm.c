#include <er_comm.h>
#include <er_tactile.h>

void comm__execute(volatile ER_Msg * _msg_command, volatile ER_Msg * _msg_response) {
//
//    switch (gMsgCommand.register_id) {
      switch ((* _msg_command).register_id) {

//        // CONFIGURATION --------------------------------------------------------------------------------------

//
//        // ACTION --------------------------------------------------------------------------------------
          case ER_REG_BLDC_SET_POSITION_SP: { // Set position setpoint
              // limit IQ15 => -65535 ... 65535
              int32_t position = msg__get_int32_from_body(_msg_command, 0);
              gBLDCParametersVars.motor_position_sp = position;
              gBLDCParametersVars.goal_mode >>= 1;
              gBLDCParametersVars.goal_mode |= 0b00000100;
              break;
          }

          case ER_REG_BLDC_SET_SPEED_SP: {  // Set velocity setpoint in position mode
              // limit IQ15 => -65535 ... 65535
              int32_t speed = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gBLDCParametersVars.velocity_sp = speed;
              gBLDCParametersVars.goal_mode >>= 1;
              gBLDCParametersVars.goal_mode |= 0b00000110;
              break;
          }

          case ER_REG_BLDC_SET_ACCEL: {
              // limit IQ15 => -65535 ... 65535
              int32_t accel = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gBLDCParametersVars.motor_accel_limit = accel;
              break;
          }

          case ER_REG_BLDC_SET_DECEL: {
              // limit IQ15 => -65535 ... 65535
              int32_t decel = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gBLDCParametersVars.motor_decel_limit = decel;
              break;
          }

          case ER_REG_BLDC_SET_POSITION_TO_ZERO: {
              gBLDCParametersVars.positon_set_zero = true;
              break;
          }


          case ER_REG_BLDC_SET_SPEED_LIMIT: {  // Set velocity limit
              int32_t speed = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gBLDCParametersVars.motor_speed_limit = speed;
              break;
          }

          case ER_REG_BLDC_SET_CURRENT_LIMIT: {
              int32_t c_limit = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gBLDCParametersVars.motor_output_max_A = c_limit;
              gBLDCParametersVars.motor_output_min_A = -1 * c_limit;
              break;
          }

        case ER_REG_BLDC_INIT_MODULE: {
            gBLDCParametersVars.enable_controller_flag = true;
            break;
        }

        // RESULTS --------------------------------------------------------------------------------------
        case ER_REG_BLDC_GET_POSITION: { // Get position of the rotor
            msg__add_int32_to_body(_msg_response, gBLDCResultsVars.motor_position_rad);
            break;
        }

        case ER_REG_BLDC_GET_SPEED: { // Get speed of the rotor
            msg__add_int32_to_body(_msg_response, gBLDCResultsVars.motor_velocity_rad);
            break;
        }

        case ER_REG_BLDC_GET_TORQUE: { // Get torque of the rotor
            msg__add_int32_to_body(_msg_response, gBLDCResultsVars.motor_torque);
            break;
        }

        case ER_REG_BLDC_GET_ACCEL: { // Get acceleration of the rotor
            msg__add_int32_to_body(_msg_response, gBLDCResultsVars.motor_accel_rad);
            break;
        }

        case ER_REG_BLDC_GET_CURRENT: { // Get current of the rotor
            msg__add_int32_to_body(_msg_response, gBLDCResultsVars.motor_current);
            break;
        }

        case ER_REG_PING: {
            break;
        }
        
        // Tactile RESULTS --------------------------------------------------------------------------------------
        case ER_TACTILE_PCB1_GET_MEDIAN: {
            int j;
            for(j=0; j < NO_OF_CHANNELS;j++)
                msg__add_uint16_to_body(_msg_response,buffer__median(&gBufferTactile[0][j]));

             break;
                }
        case ER_TACTILE_PCB2_GET_MEDIAN: {
            int j;
            for( j=0;j<NO_OF_CHANNELS;j++)
                msg__add_uint16_to_body(_msg_response, buffer__median(&gBufferTactile[1][j]));
                    break;
                }
        case ER_TACTILE_FINGER1_GET_MEDIAN: {
            int i,j;
            for ( i=0;i<2;i++){
                for( j=0;j<NO_OF_CHANNELS;j++){
                    msg__add_uint16_to_body(_msg_response, buffer__median(&gBufferTactile[i][j]));
                         }
                     }
                    break;
                }

        case ER_TACTILE_PCB3_GET_MEDIAN: {
            int j;
            for( j=0;j<NO_OF_CHANNELS;j++)
                msg__add_uint16_to_body(_msg_response, buffer__median(&gBufferTactile[2][j]));
                break;
                }
        case ER_TACTILE_PCB4_GET_MEDIAN: {
            int j;
            for( j=0;j<NO_OF_CHANNELS;j++)
                msg__add_uint16_to_body(_msg_response, buffer__median(&gBufferTactile[3][j]));
            break;
                        }
        case ER_TACTILE_FINGER2_GET_MEDIAN: {
            int i,j;
            for ( i=2;i<4;i++){
                for( j=0;j<NO_OF_CHANNELS;j++){
                    msg__add_uint16_to_body(_msg_response, buffer__median(&gBufferTactile[i][j]));
                                   }
                            }
              break;
           }
        case ER_TACTILE_GRIPPER1_GET_MEDIAN: {
            int i,j;
            for ( i=0;i<4;i++){
                  for( j=0;j<NO_OF_CHANNELS;j++){
                      msg__add_uint16_to_body(_msg_response, buffer__median(&gBufferTactile[i][j]));
                                               }
                              }
             break;
         }
  // End of Tactile RESULTS --------------------------------------------------------------------------------------
  
        default: { // ERROR - register not exists
            (* _msg_response).flags |= MSG__BAD_REGISTER; // set BAD_REGISTER flag
        }
    }

    (* _msg_response).header_crc = msg__calculate_header_crc(_msg_response); // recalculate header CRC
}

void comm__isr(CTRL_Handle ctrlHandle) {
//    CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

    // Check errors

    // Get states from InstaSPIN ---------------------------------------------------------------
    gBLDCResultsVars.motor_velocity_rpm = _IQtoIQ15(gMotorVars.Speed_krpm); // Get speed
    gBLDCResultsVars.motor_velocity_rad = _IQ15mpy(_IQtoIQ15(_IQmpy(gMotorVars.Speed_krpm, _IQ(M_RPM2RADS))), _IQ15(1000)); // Get speed
    gBLDCResultsVars.motor_torque       = _IQtoIQ15(gMotorVars.Torque_Nm); // Get torque
    gBLDCResultsVars.motor_current = _IQtoIQ15(gMotorVars.RsOnLineCurrent_A);
    gBLDCResultsVars.motor_accel_rad = _IQtoIQ15(gMotorVars.MaxAccel_krpmps);

    gBLDCParametersVars.motor_position_sp_turns = _IQ15mpy(gBLDCParametersVars.motor_position_sp, _IQ15(M_1_2PI));
    _iq15 velocity_sp_krad = _IQ15mpy(gBLDCParametersVars.velocity_sp, _IQ15(0.001));
    gBLDCParametersVars.velocity_sp_krpm = _IQ15mpy(velocity_sp_krad, _IQ15(M_RADS2RPM));

    if (gBLDCParametersVars.positon_set_zero == true){
        gBLDCResultsVars.motor_position_ppr = _IQ15(0.0);
        gBLDCParametersVars.positon_set_zero = false;
    }

    // Calculate motor position
    uint16_t temp = 0;
    temp = gBLDCResultsVars.__motor_position; // przepisanie wartosci polozenia w poprzednim kroku

    // Check step size
    if ( abs(gBLDCResultsVars.__motor_prev_position - temp) >  (float) gBLDCConfigurationsVars.encoder_resolution_ppr / 4 ) { // big step - overroll
        if (gBLDCResultsVars.__motor_prev_position > temp) {
            temp += gBLDCConfigurationsVars.encoder_resolution_ppr;
        } else {
            gBLDCResultsVars.__motor_prev_position += gBLDCConfigurationsVars.encoder_resolution_ppr;
        }
    }
    gBLDCResultsVars.motor_position_ppr += (temp - gBLDCResultsVars.__motor_prev_position);
    gBLDCResultsVars.motor_position_turns = _IQ15div(gBLDCResultsVars.motor_position_ppr, gBLDCConfigurationsVars.encoder_resolution_ppr);
    gBLDCResultsVars.motor_position_rad   = _IQ15mpy(_IQ15div(gBLDCResultsVars.motor_position_ppr, gBLDCConfigurationsVars.encoder_resolution_ppr), _IQ15(M_2PI));


    // Change states in InstaSPIN ---------------------------------------------------------------
    gMotorVars.MaxAccel_krpmps   = _IQ15toIQ(gBLDCParametersVars.motor_accel_limit);
    gMotorVars.MaxDecel_krpmps   = _IQ15toIQ(gBLDCParametersVars.motor_decel_limit);
    gMotorVars.MaxJrk_krpmps2    = gBLDCParametersVars.motor_jerk_limit;

    gMotorVars.SpinTAC.PosCtlOutputMin_A = _IQ15toIQ(gBLDCParametersVars.motor_output_min_A);
    gMotorVars.SpinTAC.PosCtlOutputMax_A = _IQ15toIQ(gBLDCParametersVars.motor_output_max_A);

    // Enable controller
    if (gBLDCParametersVars.enable_controller_flag) {
        gMotorVars.Flag_enableSys = true;
        gMotorVars.Flag_Run_Identify = true;
    } else {
        gMotorVars.Flag_enableSys = false;
        gMotorVars.Flag_Run_Identify = false;
    }

    // recreate move if not achieve goal position - FIXME - check for errors!!!
//    if ( _IQ15abs(gParametersVars.motor_position_sp_turns - gResultsVars.motor_position_turns) > _IQ15( 2 / gConfigurationsVars.encoder_resolution_ppr) && !gParametersVars.position_goal_changed_flag ) {
//        gParametersVars.position_goal_changed_flag = true;
//        gMotorVars.MaxVel_krpm       = gParametersVars.motor_speed_limit;
//    }

//    // Set velocity
//    if (gParametersVars.velocity_goal_changed_flag && gMotorVars.EstState == EST_State_OnLine) {
//        gMotorVars.MaxVel_krpm = gParametersVars.velocity_sp_krpm;
//    }
//
//    // Set position (calculate goal position from actual position, and request for diff plan)
////    if (gParametersVars.position_goal_changed_flag && gMotorVars.EstState == EST_getState(obj->estHandle) ) {
//    if (gParametersVars.position_goal_changed_flag && gMotorVars.EstState == EST_State_OnLine ) {
////        gParametersVars.position_goal_changed_flag = false;
//        gMotorVars.MaxVel_krpm       = gParametersVars.motor_speed_limit; // set speed limit
//
//        _iq15 incremental_move =  gParametersVars.motor_position_sp_turns - gResultsVars.motor_position_turns;
//        gMotorVars.PosStepInt_MRev  = _IQint(incremental_move);
//        gMotorVars.PosStepFrac_MRev = _IQfrac(incremental_move);
//        gMotorVars.RunPositionProfile = true;
//    }

//    if (gMotorVars.SpinTAC.PosMoveStatus)
    // Set velocity
    if ((gBLDCParametersVars.goal_mode == 7) && gMotorVars.EstState == EST_State_OnLine) {  // velocity mode and previous velocity
        gMotorVars.MaxVel_krpm = _IQ15toIQ(gBLDCParametersVars.velocity_sp_krpm);
    }

    if ((gBLDCParametersVars.goal_mode == 6) && gMotorVars.EstState == EST_State_OnLine) { // velocity mode and previous position
        gMotorVars.MaxVel_krpm = _IQ15toIQ(gBLDCParametersVars.velocity_sp_krpm);
    }

    if ((gBLDCParametersVars.goal_mode == 4) && gMotorVars.EstState == EST_State_OnLine ) {  // position mode and previous position
    //        gParametersVars.position_goal_changed_flag = false;
        gMotorVars.MaxVel_krpm = _IQ15toIQ(gBLDCParametersVars.motor_speed_limit); // set speed limit
        _iq15 incremental_move =  gBLDCParametersVars.motor_position_sp_turns - gBLDCResultsVars.motor_position_turns;
        gMotorVars.PosStepInt_MRev  = _IQint(_IQ15toIQ(incremental_move));
        gMotorVars.PosStepFrac_MRev = _IQfrac(_IQ15toIQ(incremental_move));
//        gMotorVars.PosStepInt_MRev  = _IQ15int(incremental_move);
//        gMotorVars.PosStepFrac_MRev = _IQ15frac(incremental_move);
        gMotorVars.RunPositionProfile = true;
    }


    if ((gBLDCParametersVars.goal_mode == 5) && gMotorVars.EstState == EST_State_OnLine ) { // position mode and previous velocity
        gMotorVars.MaxVel_krpm = _IQ(0.0);
    }


}
