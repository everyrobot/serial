#ifndef INCLUDE_ER_WATCHDOG_H_
#define INCLUDE_ER_WATCHDOG_H_


// **************************************************************************
// the includes
#include "hal.h"
// **************************************************************************
// **************************************************************************
// the defines

// **************************************************************************
// **************************************************************************
// the typedefs

// **************************************************************************
// **************************************************************************
// the globals
extern int watchdog_counter;

// HAL object
extern HAL_Handle halHandle;

// **************************************************************************
// **************************************************************************
// the function prototypes
static inline void HAL_acqTimer1Int(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    TIMER_clearFlag(obj->timerHandle[1]);

    return;
}

// watchdog functions
void HAL_enableTimer1Int(HAL_Handle handle);
void watchdog__init();

// watchdog Interrupt
interrupt void watchdog__isr(void);
//******************************************************************************
//******************************************************************************

#endif /* INCLUDE_ER_WATCHDOG_H_ */
