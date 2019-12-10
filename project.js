expRemoveAll();
// ENABLE SYSTEM
expAdd ("gMotorVars.Flag_enableSys");
expAdd ("gMotorVars.Flag_Run_Identify");

// ACTION
expAdd ("gMotorVars.RunPositionProfile", getDecimal());
expAdd ("gMotorVars.PosStepInt_MRev");
expAdd ("gMotorVars.PosStepFrac_MRev", getQValue(24));

// CONFIGURATION
expAdd ("gMotorVars.Torque_Nm", getQValue(24));
expAdd ("gMotorVars.MaxVel_krpm", getQValue(24));
expAdd ("gMotorVars.MaxAccel_krpmps", getQValue(24));
expAdd ("gMotorVars.MaxDecel_krpmps", getQValue(24));
expAdd ("gMotorVars.MaxJrk_krpmps2", getQValue(20));


expAdd ("gMotorVars.SpinTAC.PosCtlBw_radps", getQValue(20));
expAdd ("gMotorVars.SpinTAC.PosCtlOutputMin_A", getQValue(24));
expAdd ("gMotorVars.SpinTAC.PosCtlOutputMax_A", getQValue(24));

// SYSTEM STATUS
// expAdd ("gMotorAbsolutePosition");
expAdd ("gParametersVars.motor_position_sp", getQValue(24));
expAdd ("gResultsVars.motor_position_ppr");
expAdd ("gResultsVars.motor_position_rad", getQValue(24));
expAdd ("gResultsVars.motor_position_turns", getQValue(24));



expAdd ("gConfigurationsVars");
expAdd ("gParametersVars");
expAdd ("gResultsVars");

expAdd ("gMotorVars.SpeedRef_krpm", getQValue(24));

expAdd ("gMotorVars.CtrlState");
expAdd ("gMotorVars.EstState");
expAdd ("gMotorVars.UserErrorCode");
expAdd ("gMotorVars.SpinTAC.PosMoveStatus");
expAdd ("gMotorVars.SpinTAC.VelPlanErrorID");
expAdd ("gMotorVars.SpinTAC.VelPlanCfgErrorCode");
expAdd ("gMotorVars.SpinTAC.VelPlanCfgErrorIdx");

expAdd ("gMotorVars.VdcBus_kV", getQValue(24));

expAdd ("gMotorVars.SpinTAC");










// expAdd ("gMotorVars.SpinTAC.VelCtlOutputMin_A", getQValue(24));
// expAdd ("gMotorVars.SpinTAC.VelCtlOutputMax_A", getQValue(24));

// expAdd ("st_obj.pos.ctl.PosRef_mrev", getQValue(24));
// expAdd ("st_obj.pos.ctl.PosFdb_mrev", getQValue(24));

expAdd ("gMotorVars");
// expAdd ("gMotorVars.CtrlVersion");
// expAdd ("gDrvSpi8305Vars");
// expAdd ("gMotorVars.Flag_enableOffsetcalc", getDecimal());
// expAdd ("gMotorVars.Flag_enablePowerWarp", getDecimal());
// expAdd ("gMotorVars.Flag_MotorIdentified", getDecimal());
// expAdd ("gMotorVars.SpinTAC.InertiaEstimate_Aperkrpm", getQValue(24));
// expAdd ("gMotorVars.SpinTAC.FrictionEstimate_Aperkrpm", getQValue(24));
// expAdd ("gMotorVars.SpinTAC.PosCtlErrorID");
// expAdd ("gMotorVars.SpinTAC.PosMoveStatus");
// expAdd ("gMotorVars.SpinTAC.PosMoveCurveType");
// expAdd ("gMotorVars.SpinTAC.PosMoveTime_ticks");
// expAdd ("gMotorVars.SpinTAC.PosMoveTime_mticks");
// expAdd ("gMotorVars.SpinTAC.PosMoveErrorID");

// expAdd ("gMotorVars.Kp_Idq", getQValue(24));
// expAdd ("gMotorVars.Ki_Idq", getQValue(24));

// 