#include <er_watchdog.h>


void HAL_enableTimer1Int(HAL_Handle handle)
{
    HAL_Obj *obj = (HAL_Obj *)handle;

    TIMER_enableInt(obj->timerHandle[1]);

    CPU_enableInt(obj->cpuHandle,CPU_IntNumber_13);

    return;
}


void watchdog__init()
{
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_Obj *pie = (PIE_Obj *)obj->pieHandle;

    uint32_t  watchdogPeriod = (uint32_t)((float_t) 90 * (float_t)1000000.0) - 1; // 1000000.0 [us]

    // use timer 1 for CPU usage diagnostics - used for watchdog
    TIMER_setDecimationFactor(obj->timerHandle[1],0);
    TIMER_setEmulationMode(obj->timerHandle[1],TIMER_EmulationMode_RunFree);
    TIMER_setPeriod(obj->timerHandle[1],watchdogPeriod);
    TIMER_setPreScaler(obj->timerHandle[1],0);

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    pie->TINT1 = &watchdog__isr;
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    HAL_enableTimer1Int(halHandle);
    watchdog_counter = 0;
}


interrupt void watchdog__isr(void)
{
    HAL_acqTimer1Int(halHandle);

    watchdog_counter = watchdog_counter+1;

    return;
}

