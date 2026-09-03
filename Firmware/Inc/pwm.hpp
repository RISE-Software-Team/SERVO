// PWM output from the STM32 for motor control

/*
 * ACTIVE TIMER :
 * TIM2 CH3 : MOTOR CONTROL PWM POSITIVE [DRV_IN1]
 * TIM2 CH4 : MOTOR CONTROL PWM NEGATIVE [DRV_IN2]
 * 
 * TIM3 CH3 : LED RGB CONTROL BLUE
 * TIM3 CH4 : LED RGB CONTROL GREEN
 * LED RGB CONTROL RED MISSING (?)
*/

void PWM_init();
void PWM_set_duty(float duty_cycle);