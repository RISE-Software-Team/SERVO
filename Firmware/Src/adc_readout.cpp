#include "adc_readout.hpp"
#include "stm32l5xx_hal.h"
#include <cmath>

/*
 TODO:
 ~ theres an issue with ADC DMA missing in the setup
    > ts might require to change and generate new .ioc project file
*/

#define IPROPI_idx 0
#define T_FET_idx  1
#define V_BUS_idx  2
#define T_MOT_idx  3
#define POT_idx    4

extern ADC_HandleTypeDef hadc1;


void ADC::ADC_Init(){
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
        return false;

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_raw, 5) != HAL_OK)
        return false;

    HAL_Delay(2);
}

bool ADC::error_check(){
    return (Error_ != ERROR_NONE);
}

void ADC::set_config(const config_adc config_load){
    cfg_ = config_load;
}

void ADC::ADC_Update(){
    // Reset prior error state
    ADC_Error errors = ERROR_NONE;

    // I_DRV runs at the higher rate than the rest since its used in the innermost control loop
    voltage_.VDRV =  (ADC_raw[IPROPI_idx] * cfg_.VDDA)/ cfg_.ADC_RES; 
    if (std::isnan(result_.I_IPROPI)) errors |= ERROR_IDRV_INVALID_READ;
    result_.I_IPROPI = voltage_.VDRV / cfg_.IPROPI_resistor;
    result_.I_LOAD = result_.I_IPROPI * Igain;// I_load = I_IPROPI / A_IPROPI
    if (result_.I_LOAD > cfg_.DRV_I_limit) errors |= ERROR_IDRV_CURRENT_OVER;

// VT -> Temperature (Celcius)
    if(ADC_timer > 20000){
        ADC_slow();
        ADC_timer = 0;
        adc_slow = true;
    }

    Error_ = errors;
    ADC_timer += 1;

}

// Voltage - Temperature conversion using the beta equation https://www.ametherm.com/thermistor/ntc-thermistor-beta
float ADC::Thermistor_NCT(float v, Thermistor Tcfg_){
    float volt_raw = v / cfg_.VDDA;
    if(std::isnan(v) || volt_raw < 0.05 || volt_raw > 0.95){
        //error handling for invalid thermistor reading
        Error_ |= Tcfg_.FAULTYREAD;
        return NAN;
    }

    float R_NTC = (cfg_.T_FIX_RES/volt_raw) - cfg_.T_FIX_RES;
    float T2_inv = 1.0f/298.15f - logf(Tcfg_.R_INIT/R_NTC) / Tcfg_.Beta;

    return 1.0f/T2_inv - 273.15f;
}

void ADC::temperature_check(float temp, Thermistor Tcfg_){

    if (std::isnan(temp)) return Tcfg_.FAULTYREAD;

    if (temp > cfg_.FAILURE_TEMP) return Tcfg_.SHUTDOWN;
    
    if (temp > cfg_.WARNING_TEMP) return Tcfg_.WARNING;
    
    return ERROR_NONE;
}

/*

*/
void ADC::ADC_slow(){

    ADC_Error errors = ERROR_NONE;

    voltage_.VTFET = (ADC_raw[T_FET_idx]  * cfg_.VDDA)/ cfg_.ADC_RES;3
    voltage_.VBUS =  (ADC_raw[V_BUS_idx]  * cfg_.VDDA)/ cfg_.ADC_RES;  
    voltage_.VTMOT = (ADC_raw[T_MOT_idx]  * cfg_.VDDA)/ cfg_.ADC_RES;
    //voltage_.POT = 0; 

    // T_FET & T_MOT Temperature
    result_.T_FET = Thermistor_NCT(voltage_.VTFET, Thermistor_FET);
    result_.T_MOT = Thermistor_NCT(voltage_.VTMOT, Thermistor_MOT); 

    // Temp sensor error check
    errors |= temperature_check(result_.T_FET, Thermistor_FET); 
    errors |= temperature_check(result_.T_MOT, Thermistor_MOT);
    if (!std::isnan(result_.T_FET) &&
        !std::isnan(result_.T_MOT) &&
        std::fabs(result_.T_FET - result_.T_MOT) > 20.0f)
    {
        errors |= ERROR_TEMP_MISMATCH;
    }

    if (std::isnan(voltage_.VBUS)) errors |= ERROR_VBUS_INVALID_READ;

    result_.V_BUS = voltage_.VBUS * cfg_.V_BUS_DIVIDER; // VBUS = VBUS_ADC * (R1 + R2)/R2

    if (result_.V_BUS < cfg_.V_BUS_UVOLTAGE) errors |= ERROR_VBUS_UNDERVOLTAGE;
    if (result_.V_BUS > cfg_.V_BUS_OVOLTAGE) errors |= ERROR_VBUS_OVERVOLTAGE;

    Error_ = errors;
}


