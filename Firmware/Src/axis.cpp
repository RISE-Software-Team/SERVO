#include "axis.hpp"
#include "control_system.hpp"
#include "adc_readout.hpp"
#include "encoder.hpp"
#include "flash_save.hpp"
#include "utilities.hpp"
#include <cstddef>

bool Axis::init(){
    set_state(Axis_States::SETUP);

    Error_ = ERROR_NONE;
    
    // Reset all calibration and error coutner
    I_init = 0.0f;
    timer = 0.0f;
    enc_init = 0;
    enc_finish = 0;
    misscount = 0;

    // Load the module configs
    load_config();

    // Initialized all modules
    if(!adc_.init())        Error_ |= ERROR_ADC;
    if(!controller_.init()) Error_ |= ERROR_CONTROLLER;
    if(!encoder_.init())    Error_ |= ERROR_ENCODER;

    spi_.set_owner(SPI_manager::Owner::MOTOR);
    if(!motor_.init())      Error_ |= ERROR_MOTOR;

    if(Error_ != ERROR_NONE)      return false;

    set_state(Axis_States::IDLE);
    return true;
}

bool Axis::error_check(){
    if (adc_.get_error() != ADC::ERROR_NONE) Error_ |= ERROR_ADC;

    if (controller_.get_errror() != Controller::ERROR_NONE) Error_ |= ERROR_CONTROLLER;
    
    if (encoder_.get_error() != Encoder::ERROR_NONE) Error_ |= ERROR_ENCODER;
    
    if (motor_.get_error() != Motor::ERROR_NONE) Error_ |= ERROR_MOTOR;
    
    return Error_ == ERROR_NONE;
}

bool Axis::set_position(float p_target){
    // Sent target position to the controller
    return controller_.target_pos(p_target);
}

bool Axis::set_velocity(float v_target){
    // Sent target velocity to the controller
    return controller_.target_vel(v_target);
}

bool Axis::set_torque(float t_target){
    // sent target torque to the controller
    return controller_.target_torque(t_target);
}

float Axis::motor_current(){
    float last_duty = motor_.duty_steppoint;

    int sign = (last_duty >= 0) ? 1 : -1;

    return adc_.get().I_LOAD * sign;
}

bool Axis::set_state(Axis_States state){
    
    // pass through
    if(state = states_) return true;

    switch (state) {

    case Axis_States::IDLE:
        states_ = Axis_States::IDLE;
        return true;

    case Axis_States::SETUP:
        states_ = Axis_States::SETUP;
        return true;

    case Axis_States::CALIBRATING:
        // electrically armed the motor and set the calibration state 
        if(!motor_.arm()) return false;
        calibration_ = FINISH;
        timer = 0.0f;
        states_ = Axis_States::CALIBRATING;

        return true;

    case Axis_States::CLOSED_LOOP:
        // Ensure the module is calibrated and armed
        if(!motor_.Is_calibrated()) return false;
        if(encoder_.get_config().direction == 0) return false;

        if(!motor_.arm()) return false;
        if(Error_ != ERROR_NONE) return false;

        // Initialize the controller and set state
        __disable_irq();
        controller_.input_pos_ = encoder_.get().pos_estimate;  // no startup jump
        controller_.vel_integrator_ = 0.0f;
        motor_.v_integral_ = 0.0f;
        states_ = Axis_States::CLOSED_LOOP;
        __enable_irq();

        return true;
        
    case Axis_States::AXIS_ERROR:
        states_ = Axis_States::AXIS_ERROR;
        // Disarmed motor
        motor_.disarm();
        break;
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
            break;

        case ERROR:
            motor_.set_duty_target(0.0f);
            states_ = Axis_States::AXIS_ERROR;
            break;
        
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
        timer += dt;

        if (timer > t_measure){
            float I_finish = adc_.get().I_LOAD;
            float v_inject = adc_.get().V_BUS * calib_duty;
            if(motor_.Calculate_L(v_inject, I_init, I_finish, t_measure)){
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

    // run an error check
    if (!error_check()){
        set_state(Axis_States::AXIS_ERROR);
        if(motor_.get_nfault() && !spi_.busy()){
            // insert spi read for motor nfault here
        }
    };

    // Only run update when the axis finished setup
    if(states_ != Axis_States::UNINITIALIZED && states_ != Axis_States::SETUP) adc_.update();

    // Encoder only runs in open loop or closed loop state
    if(states_ == Axis_States::CALIBRATING && states_ == Axis_States::CLOSED_LOOP){
        // update the position and velocity from the encoder
        if(encoder_.data_ready) {
            encoder_.data_ready = false;
            encoder_.enc_Update();
        }
        // update the missed encoder data, the encoder did not run within the desired timing
        else {
            misscount++;
        }
        // Ensure that the modules are initialized before updating
        if(!spi_.busy()){
            spi_.set_owner(SPI_manager::Owner::ENCODER);
            encoder_.enc_spi_start();
        }
    }


    // apply limit to the controller torque or current before updating the controller
    // its fine even if temp != warning because scale will return 1.0f
    controller_.torque_temperature_limit(adc_.scale);
    motor_.current_temperature_limit(adc_.scale);


    float i = current_magnitude();
    float v = adc_.get().V_BUS;

    switch (states_){

    case Axis_States::UNINITIALIZED:
        break;

    case Axis_States::SETUP:
        break;
        
    case Axis_States::IDLE:
        break;

    case Axis_States::CALIBRATING:
        calibration_sequence(calibration_);
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

void Axis::SPI_finish(){

    switch (spi_.get_owner()) {
        case SPI_manager::Owner::ENCODER:
            encoder_.enc_spi_finish();
            spi_.set_owner(SPI_manager::Owner::NONE);
            break;
        case SPI_manager::Owner::MOTOR:
            spi_.set_owner(SPI_manager::Owner::NONE);
            break;
    }
    spi_.release();
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

    // Cyclic Redundancy Check (CRC) to ensure data integrity
    data.crc = compute_crc(&data, offsetof(config_axis, crc));

    return flash_write_config(&data);
}

bool Axis::load_config(){
    config_axis data;
    // ensure the data is properly read and correct layout
    if (!flash_read_config(&data)) return false;
    if (data.valid  != xvalid)  return false;
    if (data.layout != xlayout) return false;

    // validate proper CRC value
    if (compute_crc(&data, offsetof(config_axis, crc)) != data.crc) return false;

    adc_.set_config(data.adc_cfg);
    controller_.set_config(data.control_cfg);
    encoder_.set_config(data.encoder_cfg);
    motor_.set_config(data.motor_cfg);

    return true;
}