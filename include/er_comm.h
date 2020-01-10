#ifndef ER_COMM_H
#define ER_COMM_H

// **************************************************************************
// the includes
#include <hal.h>
#include <stdlib.h>

#include <er_globals.h>
#include <er_ti_f28069m_drv8305/er_registers.h>
#include <er_ti_f28069m_drv8305/er_msg.h>
#include <er_bldc.h>
#include "main_position.h"

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the globals
extern volatile MOTOR_Vars_t            gMotorVars;
extern ER_BLDC_Configurations  gBLDCConfigurationsVars;
extern ER_BLDC_Parameters      gBLDCParametersVars;
extern ER_BLDC_Results         gBLDCResultsVars;
//extern volatile ER_Results        gResultsVars;
//extern volatile ER_Results        gResultsVars;
//extern volatile ER_Msg            gMsgCommand;
//extern volatile ER_Msg            gMsgResponse;


// **************************************************************************
// the function prototypes

void comm__execute(volatile ER_Msg * _msg_command, volatile ER_Msg * _msg_response);
void comm__isr(CTRL_Handle ctrlHandle);

#endif
