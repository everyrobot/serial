#ifndef ER_ADC_H
#define ER_ADC_H

// **************************************************************************
// the includes
#include <stdio.h>
#include <string.h>
#include "hal.h"
#include "sw/drivers/adc/src/32b/f28x/f2806x/adc.h"
#include <er_ti_f28069m_drv8305/er_buffer.h>
// **************************************************************************
// the defines

// **************************************************************************
// the typedefs
typedef volatile struct _ER_ADC_Results_t_
{
    int         counter;
    uint16_t    raw_current_sensor;
    uint16_t    no_kalman_current_sensor;
    uint16_t    no_conversion_current_sensor;
    _iq15       current_sensor;

} ER_ADC_Results;

#define ER_ADC_Results_INIT {0, \
                            _IQ15(0.0)}
// **************************************************************************

// the globals
extern          ER_ADC_Results  gADCResultsVars;
extern          HAL_Handle      halHandle;
extern          ER_Buffer       gBufferAdc;

// **************************************************************************
// the function prototypes
void adc__init();
void adc__read();
interrupt void adc__isr(void);

#endif
