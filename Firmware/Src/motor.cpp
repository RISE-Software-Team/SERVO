#include "pwm.hpp"
#include "motor.hpp"
#include "main.h"
#include "stm32l5xx_hal.h"
#include <cmath>
#include <math.h>
#include <algorithm>


bool Motor::arm(){
    // Early Error Check
    if (isarmed_)               return false;
    if (Error_ == ERROR_NONE)   return false;
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
    return (Error_ != ERROR_NONE);
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

    i_target = std::clamp(torque / cfg_.torque_constant, -cfg_.current_limit, cfg_.current_limit);

    float step = cfg_.current_max_step * dt;
    i_setpoint += std::clamp(i_setpoint - i_target, -step, step);
    
    float vel_estimate = 0.0f; // Placeholder for velocity estimation
    v_feedforward = i_target * cfg_.resistance + cfg_.torque_constant * (2.0 * M_PI * vel_estimate);
}

