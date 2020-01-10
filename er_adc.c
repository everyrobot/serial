#include <er_adc.h>

void adc__init(){
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_Obj *pie = (PIE_Obj *)obj->pieHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    pie->ADCINT2 = &adc__isr;
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    ADC_setIntMode(obj->adcHandle,ADC_IntNumber_2,ADC_IntMode_ClearFlag);
    ADC_setIntSrc(obj->adcHandle,ADC_IntNumber_2,ADC_IntSrc_EOC9);

    PIE_enableAdcInt(obj->pieHandle, ADC_IntNumber_2);
    ADC_enableInt(obj->adcHandle, ADC_IntNumber_2);

    // CURRENT SENSOR
    ADC_setSocChanNumber(obj->adcHandle,ADC_SocNumber_8,ADC_SocChanNumber_A3);
    ADC_setSocTrigSrc(obj->adcHandle,ADC_SocNumber_8,ADC_SocTrigSrc_CpuTimer_0);
    ADC_setSocSampleDelay(obj->adcHandle,ADC_SocNumber_8,ADC_SocSampleDelay_9_cycles);

    ADC_setSocChanNumber(obj->adcHandle,ADC_SocNumber_9,ADC_SocChanNumber_A3);
    ADC_setSocTrigSrc(obj->adcHandle,ADC_SocNumber_9,ADC_SocTrigSrc_CpuTimer_0);
    ADC_setSocSampleDelay(obj->adcHandle,ADC_SocNumber_9,ADC_SocSampleDelay_9_cycles);

    gBufferAdc = buffer__create(30);
}

void adc__read(){
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    gADCResultsVars.counter++;
    _iq voltage_sf = _IQ(3.3);
    uint16_t value = (uint16_t)ADC_readResult(obj->adcHandle,ADC_ResultNumber_9);     // divide by 2^numAdcBits = 2^12
    gADCResultsVars.raw_current_sensor = value;
    buffer__add_to_buffer(&gBufferAdc, value);
    if(gADCResultsVars.counter == 30) {
        gADCResultsVars.no_kalman_current_sensor = buffer__median(&gBufferAdc);
        gADCResultsVars.current_sensor = _IQtoIQ15(_IQ12mpy(gADCResultsVars.no_kalman_current_sensor,voltage_sf));
        gADCResultsVars.counter = 0;
    }
}

interrupt void adc__isr(void){
    HAL_acqAdcInt(halHandle,ADC_IntNumber_2);
    adc__read();
}
