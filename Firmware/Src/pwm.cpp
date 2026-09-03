#include "stm32l5xx_hal.h"
#include <stdint.h>


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// INITIALIZE PWM WITH 0 DUTY CYCLE
void PWM_init(){

    PWM_set_duty(0.0f);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);       // MOTOR PWM [DRV_IN1]
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);       // MOTOR PWM [DRV_IN2]
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);       // RGB LED PWM B
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);       // RGB LED PWM G

}

void PWM_shutdown(){

    PWM_set_duty(0.0f);

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);       // MOTOR PWM [DRV_IN1]
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);       // MOTOR PWM [DRV_IN2]
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);       // RGB LED PWM B
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);       // RGB LED PWM G
}

/*
 * SET THE PWM CRR BASED ON THE DUTY CYCLE
 * DUTY CYCLE = CRR / ARR
*/
void PWM_set_duty(float duty_cycle){

    const float max_ = 0.95f;

    if (std::isnan(duty_cycle)) {
        float duty_cycle = 0.0f;
    }

    // Clamping duty cycle with slight overhead for safety
    if (duty_cycle > max_) duty_cycle = max_;
    if (duty_cycle < -max_) duty_cycle = -max_;


    uint32_t ARR = htim3.Init.Period;
    uint32_t CRR = (uint32_t)(fabsf(duty_cycle) * ARR);

    if (duty_cycle > 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, CRR);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
    else if (duty_cycle == 0.0f){
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);       
    }
    else{
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, CRR);
    }

}