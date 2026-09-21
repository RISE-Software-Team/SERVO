#include "pwm.hpp"
#include "motor.hpp"
#include "main.h"
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_gpio.h"
#include "utilities.hpp"
#include <cmath>
#include <cstdint>
#include <math.h>
#include <algorithm>




bool Motor::arm(){
    // Early Error Check
    if (isarmed_)               return false;
    if (Error_ != ERROR_NONE)   return false;
    //if !axis.docheck()    return false;

    PWM_init();
    HAL_GPIO_WritePin(DRV_OFF_GPIO_Port, DRV_OFF_Pin, GPIO_PIN_RESET);  // Disable DRV_OFF
    
    v_int = 0.0f;
    i_target = 0.0f;
    isarmed_ = true;
    return true;
}

void Motor::disarm(){
    HAL_GPIO_WritePin(DRV_OFF_GPIO_Port, DRV_OFF_Pin, GPIO_PIN_SET);
    PWM_shutdown();

    v_int = 0.0f;
    i_target = 0.0f;
    isarmed_ = false;
}

bool Motor::error_check(){
    const bool nfault = HAL_GPIO_ReadPin(DRV_FLT_GPIO_Port, DRV_FLT_Pin) == GPIO_PIN_RESET;

    if (nfault){
        Error_ |= ERROR_DRIVER_FAULT;
        nfault_read_pending = true;
    }
    return (Error_ != ERROR_NONE && nfault);
}

void Motor::set_duty_target(float desired_duty){
    duty_target = std::clamp(desired_duty, 0.0f, 1.0f);
}

void Motor::set_mode(Motor_mode mode){
    Mode = mode;
}

void Motor::set_config(const config_mot config_load){
    cfg_ = config_load;
    update_gain(cfg_.current_bandwidth);
}

bool Motor::update_gain(float BW){
    cfg_.current_bandwidth = BW;
    Kp = BW * cfg_.inductance;
    Ki = BW * cfg_.resistance;

    float PLACEHOLDER = 0.0f; // Placeholder for minimum gain value

    if (Kp <= PLACEHOLDER || Ki <= PLACEHOLDER){
        Error_ |= ERROR_INVALID_GAIN;
        return false;
    }
    return true;
}

bool Motor::Calculate_R(float V, float ILoad){
    const float R_upper = 0.0f;

    if (ILoad < 0.01f) {
        return false;
    }

    float R = V / ILoad;
    if(R <= 0.0f || R > R_upper){
        Error_ |= ERROR_INVALID_RESISTANCE;
        return false;
    }

    cfg_.resistance = R;
    return true;
}

bool Motor::Calculate_L(float V, float I_init, float I_final, float dt){
    
    const float L_upper = 0.0f;

    float deltaI = I_final - I_init;

    if (deltaI < 0.01f) {
        Error_ |= ERROR_INVALID_INDUCTANCE;
        return false;
    }

    float L = V * dt / deltaI;
    if(L <= 0.0f || L > L_upper){
        Error_ |= ERROR_INVALID_INDUCTANCE;
        return false;
    }

    cfg_.inductance = L;
    return true;
}

bool Motor::Is_calibrated(){
    if(isnan(cfg_.resistance)) return false;
    if(isnan(cfg_.inductance)) return false;
}


void Motor::current_inner_loop(float I_LOAD, float V_BUS){

    float v = 0.0f;

    switch (mode_){

        case MOTOR_IDLE:
            v_int = 0.0f;
            v_out = 0.0f;
            PWM_set_duty(0.0f);
            break;

        case MOTOR_OPEN_CONTROL:
            v_int = 0.0f;
            v_out = 0.0f;
            duty_steppoint += std::clamp(duty_target - duty_steppoint, -cfg_.duty_step, cfg_.duty_step);
            PWM_set_duty(duty_steppoint);
            break;     
            
        case MOTOR_CLOSED_CONTROL:
            float i_error = i_setpoint - I_LOAD;
            float dt = 0.0f;

            if (fabs(v_int) > cfg_.integrator_limit){
                v_int +=  cfg_.Ki * i_error * dt;
                v_int = std::clamp(v_int, -V_BUS, V_BUS);
            }
            float v_out = cfg_.Kp * i_error + v_int + v_feedforward;
            duty_steppoint = v_out / V_BUS; 
                  
    }

    PWM_set_duty(duty_steppoint);
}

void Motor::current_target(float torque, float timestep){

    i_target = std::clamp(torque / cfg_.torque_constant, -true_torque_limit, true_torque_limit);

    float step = cfg_.current_max_step * timestep;
    float delta_i = std::clamp(i_target - i_setpoint, -step, step);

    i_setpoint += delta_i;

    float di_dt = delta_i/timestep;
    
    float vel_estimate = 0.0f; // Placeholder for velocity estimation
    v_feedforward = i_setpoint * cfg_.resistance + cfg_.torque_constant * vel_estimate + cfg_.inductance * delta_i;

}

bool Motor::drv_spi(uint16_t tx, uint16_t *rx) {
    uint16_t in;
    HAL_GPIO_WritePin(DRV_CS_PORT, DRV_CS_PIN, GPIO_PIN_RESET);
    HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx,
                                                  (uint8_t*)&in, 1, 10);
    HAL_GPIO_WritePin(DRV_CS_PORT, DRV_CS_PIN, GPIO_PIN_SET);
    if (s != HAL_OK) return false;
    if ((in & 0xC000) != 0xC000) return false;  // top 2 bits always 11
    *rx = in;
    return true;
}

bool Motor::drv_read(uint8_t address, uint8_t *data, uint8_t *status){
    
    uint16_t package = 1u << 14 || (static_cast<uint16_t>(address & 0x3Fu) << 8);
    uint16_t rx; 

    if (!drv_spi(package, &rx)) return false;

    *status = static_cast<uint8_t>((rx >> 8) & 0xFFu);
    *data = static_cast<uint16_t>(rx & 0xFFu);

    return true;
}

bool Motor::drv_write(uint8_t address, uint8_t data){
    uint16_t package = (static_cast<uint16_t>(address & 0x3Fu) << 8) | data;
    uint16_t rx;

    return drv_spi(package, &rx);
}

bool Motor::read_nfault(){

    if(!drv_read(DrvReg::STATUS1, &STATUS1_vals, &motor_status)){
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }
    if(!drv_read(DrvReg::STATUS2, &STATUS2_vals, &motor_status)){
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }

    return true;
}

bool Motor::init(){
    uint8_t data_init = 0;

    disarm();
    
    if(!drv_read(DrvReg::DEVICE_ID, &data_init, &motor_status)){
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }
    uint8_t dev_id = static_cast<uint8_t>(data_init & 0x3F);

    if(dev_id != 0x25) {
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }

    if(!drv_write(DrvReg::CFG1_REG, CONFIG1_VAL)) {
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }

    if(!drv_write(DrvReg::CFG2_REG,  CONFIG2_VAL)){
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }
    
    if(!drv_write(DrvReg::CFG3_REG, CONFIG3_VAL)){
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }

    if(!drv_write(DrvReg::CFG4_REG,  CONFIG4_VAL)){
        Error_ = ERROR_SPI_INIT_FAIL;
        return false;
    }

    return true;
}