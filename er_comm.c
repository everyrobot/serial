#include <er_comm.h>

void comm__execute(volatile ER_Msg * _msg_command, volatile ER_Msg * _msg_response) {
//
//    switch (gMsgCommand.register_id) {
      switch ((* _msg_command).register_id) {

//        // CONFIGURATION --------------------------------------------------------------------------------------

//
//        // ACTION --------------------------------------------------------------------------------------
          case ER_REG_SET_POSITION_SP: { // Set position setpoint
              // limit IQ15 => -65535 ... 65535
              int32_t position = msg__get_int32_from_body(_msg_command, 0);
              gParametersVars.motor_position_sp = position;
              gParametersVars.goal_mode >>= 1;
              gParametersVars.goal_mode |= 0b00000100;
              break;
          }

          case ER_REG_SET_SPEED_SP: {  // Set velocity setpoint in position mode
              // limit IQ15 => -65535 ... 65535
              int32_t speed = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gParametersVars.velocity_sp = speed;
              gParametersVars.goal_mode >>= 1;
              gParametersVars.goal_mode |= 0b00000110;
              break;
          }

          case ER_REG_SET_ACCEL: {
              // limit IQ15 => -65535 ... 65535
              int32_t accel = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gParametersVars.motor_accel_limit = accel;
              break;
          }

          case ER_REG_SET_DECEL: {
              // limit IQ15 => -65535 ... 65535
              int32_t decel = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gParametersVars.motor_decel_limit = decel;
              break;
          }

          case ER_REG_SET_POSITION_TO_ZERO: {
              gParametersVars.positon_set_zero = true;
              break;
          }


          case ER_REG_SET_SPEED_LIMIT: {  // Set velocity limit
              int32_t speed = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gParametersVars.motor_speed_limit = speed;
              break;
          }

          case ER_REG_SET_CURRENT_LIMIT: {
              int32_t c_limit = msg__get_int32_from_body(_msg_command, 0);     // +/- 65535 + 1/32767 => 0.00003
              gParametersVars.motor_output_max_A = c_limit;
              gParametersVars.motor_output_min_A = -1 * c_limit;
              break;
          }

        case ER_REG_INIT_MODULE: {
            gParametersVars.enable_controller_flag = true;
            break;
        }

        // RESULTS --------------------------------------------------------------------------------------
        case ER_REG_GET_POSITION: { // Get position of the rotor
            msg__add_int32_to_body(_msg_response, gResultsVars.motor_position_rad);
            break;
        }

        case ER_REG_GET_SPEED: { // Get speed of the rotor
            msg__add_int32_to_body(_msg_response, gResultsVars.motor_velocity_rad);
            break;
        }

        case ER_REG_GET_TORQUE: { // Get torque of the rotor
            msg__add_int32_to_body(_msg_response, gResultsVars.motor_torque);
            break;
        }

        case ER_REG_GET_ACCEL: { // Get acceleration of the rotor
            msg__add_int32_to_body(_msg_response, gResultsVars.motor_accel_rad);
            break;
        }

        case ER_REG_GET_CURRENT: { // Get current of the rotor
            msg__add_int32_to_body(_msg_response, gResultsVars.motor_current);
            break;
        }

        case ER_REG_PING: {
            break;
        }

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
    gResultsVars.motor_velocity_rpm = _IQtoIQ15(gMotorVars.Speed_krpm); // Get speed
    gResultsVars.motor_velocity_rad = _IQ15mpy(_IQtoIQ15(_IQmpy(gMotorVars.Speed_krpm, _IQ(M_RPM2RADS))), _IQ15(1000)); // Get speed
    gResultsVars.motor_torque       = _IQtoIQ15(gMotorVars.Torque_Nm); // Get torque
    gResultsVars.motor_current = _IQtoIQ15(gMotorVars.RsOnLineCurrent_A);
    gResultsVars.motor_accel_rad = _IQtoIQ15(gMotorVars.MaxAccel_krpmps);

    gParametersVars.motor_position_sp_turns = _IQ15mpy(gParametersVars.motor_position_sp, _IQ15(M_1_2PI));
    _iq15 velocity_sp_krad = _IQ15mpy(gParametersVars.velocity_sp, _IQ15(0.001));
    gParametersVars.velocity_sp_krpm = _IQ15mpy(velocity_sp_krad, _IQ15(M_RADS2RPM));

    if (gParametersVars.positon_set_zero == true){
        gResultsVars.motor_position_ppr = _IQ15(0.0);
        gParametersVars.positon_set_zero = false;
    }

    // Calculate motor position
    uint16_t temp = 0;
    temp = gResultsVars.__motor_position; // przepisanie wartosci polozenia w poprzednim kroku

    // Check step size
    if ( abs(gResultsVars.__motor_prev_position - temp) >  (float) gConfigurationsVars.encoder_resolution_ppr / 4 ) { // big step - overroll
        if (gResultsVars.__motor_prev_position > temp) {
            temp += gConfigurationsVars.encoder_resolution_ppr;
        } else {
            gResultsVars.__motor_prev_position += gConfigurationsVars.encoder_resolution_ppr;
        }
    }
    gResultsVars.motor_position_ppr += (temp - gResultsVars.__motor_prev_position);
    gResultsVars.motor_position_turns = _IQ15div(gResultsVars.motor_position_ppr, gConfigurationsVars.encoder_resolution_ppr);
    gResultsVars.motor_position_rad   = _IQ15mpy(_IQ15div(gResultsVars.motor_position_ppr, gConfigurationsVars.encoder_resolution_ppr), _IQ15(M_2PI));
//    gResultsVars.motor_position_turns = _IQtoIQ15(_IQdiv(gResultsVars.motor_position_ppr, gConfigurationsVars.encoder_resolution_ppr));
//    gResultsVars.motor_position_rad   = _IQtoIQ15(_IQmpy(_IQdiv(gResultsVars.motor_position_ppr, gConfigurationsVars.encoder_resolution_ppr), _IQ(M_2PI)));


    // Change states in InstaSPIN ---------------------------------------------------------------
    gMotorVars.MaxAccel_krpmps   = _IQ15toIQ(gParametersVars.motor_accel_limit);
    gMotorVars.MaxDecel_krpmps   = _IQ15toIQ(gParametersVars.motor_decel_limit);
    gMotorVars.MaxJrk_krpmps2    = gParametersVars.motor_jerk_limit;

    gMotorVars.SpinTAC.PosCtlOutputMin_A = _IQ15toIQ(gParametersVars.motor_output_min_A);
    gMotorVars.SpinTAC.PosCtlOutputMax_A = _IQ15toIQ(gParametersVars.motor_output_max_A);

    // Enable controller
    if (gParametersVars.enable_controller_flag) {
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
    if ((gParametersVars.goal_mode == 7) && gMotorVars.EstState == EST_State_OnLine) {  // velocity mode and previous velocity
        gMotorVars.MaxVel_krpm = _IQ15toIQ(gParametersVars.velocity_sp_krpm);
    }

    if ((gParametersVars.goal_mode == 6) && gMotorVars.EstState == EST_State_OnLine) { // velocity mode and previous position
        gMotorVars.MaxVel_krpm = _IQ15toIQ(gParametersVars.velocity_sp_krpm);
    }

    if ((gParametersVars.goal_mode == 4) && gMotorVars.EstState == EST_State_OnLine ) {  // position mode and previous position
    //        gParametersVars.position_goal_changed_flag = false;
        gMotorVars.MaxVel_krpm = _IQ15toIQ(gParametersVars.motor_speed_limit); // set speed limit
        _iq15 incremental_move =  gParametersVars.motor_position_sp_turns - gResultsVars.motor_position_turns;
        gMotorVars.PosStepInt_MRev  = _IQint(_IQ15toIQ(incremental_move));
        gMotorVars.PosStepFrac_MRev = _IQfrac(_IQ15toIQ(incremental_move));
//        gMotorVars.PosStepInt_MRev  = _IQ15int(incremental_move);
//        gMotorVars.PosStepFrac_MRev = _IQ15frac(incremental_move);
        gMotorVars.RunPositionProfile = true;
    }


    if ((gParametersVars.goal_mode == 5) && gMotorVars.EstState == EST_State_OnLine ) { // position mode and previous velocity
        gMotorVars.MaxVel_krpm = _IQ(0.0);
    }


}
