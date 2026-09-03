#include "axis.hpp"
#include "control_system.hpp"
#include "adc_readout.hpp"
#include "encoder.hpp"
#include "flash_save.hpp"
#include <cstddef>

Axis Axis::Axis_create(Encoder encoder, Motor motor,
                       Controller Control, ADC ADC )
                       : encoder_(encoder), motor_(motor),
                       controller_(control), adc_(ADC)
                       {};

bool Axis::set_position(float p_target){
    return controller_.target_pos(p_target);
}

bool Axis::set_velocity(float v_target){
    return controller_.target_vel(v_target);
}

bool Axis::set_torque(float t_target){
    return controller_.target_torque(float t_target);
}

float Axis::current_magnitude(){
    float last_duty = motor_.duty_steppoint;

    int magnitude = (last_duty >= 0) ? 1 : -1;

    return adc_.get().I_LOAD * magnitude;
}

bool Axis::set_state(Axis_States state){
    switch (state) {
    case Axis_States::CLOSED_LOOP:
        if(!motor_.Is_calibrated()) return false;
        if(encoder_.get_config().direction == 0) return false;
        if(Error_ != ERROR_NONE) return false;

        __disable_irq();
        controller_.input_pos_ = encoder_.pos_estimate_;  // no startup jump
        controller_.vel_integrator_ = 0.0f;
        motor_.v_integral_ = 0.0f;
        states_ = Axis_States::CLOSED_LOOP;
        __enable_irq();

        return motor_.arm();

    case Axis_States::CALIBRATING:
        if(motor_.isarmed_) return false;
        calibration_ = FINISH;
        timer = 0.0f;
        states_ = Axis_States::CALIBRATING;

        return motor_.arm();
    }
}

void Axis::set_calibration(calibration calibration_state){
    calibration_ = calibration_state;
    timer = 0.0f;

    switch(calibration_state){
        case RESISTANCE:
            motor_.set_duty_target(calib_duty);
            break;

        case INDUCTANCE:
            I_init = adc_.get().I_LOAD;
            motor_.set_duty_target(calib_duty);
            break;

        case DIRECTION:
            enc_init = encoder_.get().count_true;
            motor_.set_duty_target(calib_duty);
            break;
        
        case FINISH:
            motor_.set_duty_target(0.0f);
            states_ = Axis_States::IDLE;

        case ERROR:
            motor_.set_duty_target(0.0f);
            states_ = Axis_States::AXIS_ERROR;
        
        }
}

void Axis::calibration_sequence(calibration calibration_status){

    switch (calibration_status){
    case RESISTANCE:

        timer += dt;

        // Input a waiting time here for inductance effect to dissapear
        if (timer > t_settle){
            float V_inject = adc_.get().V_BUS * calib_duty;
            float I_load = adc_.get().I_LOAD;

            if(motor_.Calculate_R(V_inject, I_load)){
                timer = 0.0f;
                set_calibration(FINISH);
            }
            else {set_calibration(ERROR);}
        }
        break;

    case INDUCTANCE:
        float I_finish = 0.0f;

        timer += dt;

        if (timer > t_measure){
            float I_finish = adc_.get().I_LOAD;
            float v_inject = adc_.get().V_BUS * calib_duty;
            if(motor_.Calculate_L(V_inject, I_init, I_finish, t_measure)){
                timer = 0.0f;
                set_calibration(FINISH);
            }    
            else{set_calibration(ERROR);}
        }
        break;
        
    case DIRECTION:
        timer += dt;

        if (timer > t_settle){
            enc_finish = encoder_.get().count_true;
            int delta = enc_finish - enc_init;
            if(encoder_.enc_direction(delta)){
                timer = 0.0f;
                set_calibration(FINISH);
            } 
            else{set_calibration(ERROR)};
        }
        break;
    }
}

void Axis::update(){

    bool test = error_check();

    switch (states_) {
    
        case IDLE:
            motor_.set_mode(Motor::MOTOR_IDLE);
        break;

        case CALIBRATING:
            motor_.set_mode(Motor::MOTOR_OPEN_CONTROL); 
            calibration_sequence(calibration_);
            break;

        case CLOSED_LOOP:
            controller_.update(dt);
            break;

        case AXIS_ERROR;
            motor_.disarm();
            break;
    }
}

bool Axis::error_check(){
    if (adc_.error_check() != ADC::ERROR_NONE) Error_ |= ERROR_ADC;

    if (controller_.get_errror() != Controller::ERROR_NONE) Error_ |= ERROR_CONTROLLER;
    
    if (encoder_.get_error() != Encoder::ERROR_NONE) Error_ |= ERROR_ENCODER;
    
    if (motor_.get_error() != Motor::ERROR_NONE) Error |= ERROR_MOTOR;
    
    if (Error_ != ERROR_NONE){
        states_= Axis_States::AXIS_ERROR
        return false;
    }
    return true;
    
}



bool Axis::save_config(){
    if (motor_.isarmed_) return false;

    config_axis data;
    data.valid = xvalid;
    data.layout = xlayout;
    data.adc_cfg = adc_.get_config();
    data.control_cfg = controller_.get_config()
    data.encoder_cfg = encoder_.get_config();
    data.motor_cfg = motor_.get_config();

    data.crc = compute_crc(&data, offsetof(config_axis, crc));

    return flash_write_config(&data);
}

bool Axis::load_config(){
    config_axis data;
    if (!flash_read_config(&data)) return false;
    if (data.valid  != xvalid)  return false;
    if (data.layout != xlayout) return false;
    if (compute_crc(&data, offsetof(config_axis, crc)) != data.crc) return false;

    adc_.set_config(data.adc_cfg);
    controller_.set_config(data.control_cfg);
    encoder_.set_config(data.encoder_cfg);
    motor_.set_config(data.motor_cfg);

    return true;
}


void Axis::axis_run(){
    if(encoder_.data_ready) {
        encoder_.data_ready = false;
        encoder_.enc_Update();
    }
    encoder_.ENC_SPI_START();
    adc_.ADC_Update();

    float i = current_magnitude();
    float v = adc_.get().V_BUS;

    switch (states_){
        
    case Axis_States::IDLE:
        break;

    case Axis_States::CALIBRATING:
        motor_.current_inner_loop(i, v);
        break;

    case Axis_States::CLOSED_LOOP:
        controller_.update(dt);
        motor_.current_target(controller_.torque_setpoint, dt);
        motor_.current_inner_loop(i, v);
        break;

    case Axis_States::AXIS_ERROR:
        break;
    }

}
