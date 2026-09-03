#include "control_system.hpp"
#include <algorithm>
#include <cmath>


void Controller::set_config(const config_control config_load){
    cfg_ = config_load;
    update_gain(cfg_.bandwidth);
}

void Controller::update_gain(float BW){
    cfg_.bandwidth = BW;
    float p_kp = BW;
    float v_kp = cfg_.inertia * BW;
    float v_ki = v_kp * BW/6;

    if (!(BW > 0.0f) || BW > cfg_.max_bandwidth) {
        error_ |= ERROR_INVALID_GAIN;
        return;
    }

    cfg_.pos_KP = p_kp;
    cfg_.vel_KP = v_kp;
    cfg_.vel_KI = v_ki;

}
bool Controller::target_pos(float p_target){
    if (std::isnan(p_target)){
        error_ |= ERROR_INVALID_TARGET;
        return false;
    }

    float position = enc_->get().pos_estimate;
    float velocity = enc_->get().vel_estimate;

    if (fabsf(velocity) > cfg_.velocity_threshold){
        error_ |= ERROR_REPLANNING;
        return false;
    }

    trapezoid_planning(position, p_target, velocity, cfg_.A_max, cfg_.V_max);
    return true;
}

bool Controller::target_vel(float v_target){
    if (std::isnan(v_target)){
        error_ |= ERROR_INVALID_TARGET;
        return false;
    }

    vel_target = std::clamp(v_target, -cfg_.velocity_limit, cfg_.velocity_limit);
    return true;
}

bool Controller::target_torque(float t_target){
    if (std::isnan(v_target)){
        error_ |= ERROR_INVALID_TARGET;
        return false;
    }
    
    torque_target = std::clamp(t_target, -cfg_.torque_limit, cfg_.torque_limit);
}

void Controller::set_mode(Control_mode mode){
    mode_ = mode;
}

float Controller::position_control(float target, float position, float vel_ff){

    float vel_setpoint = cfg_.pos_KP * (target - position) + vel_ff;
    return std::clamp(vel_setpoint, -cfg_.velocity_limit, cfg_.velocity_limit);
}

float Controller::velocity_control(float vel_setpoint, float vel, float acc_ff, float dt){

    vel_int += cfg_.vel_KI * (vel_setpoint - vel) * dt;
    vel_int = std::clamp(vel_int, -cfg_.integrator_limit, cfg_.integrator_limit);

    return cfg_.vel_KP * (vel_setpoint - vel) + vel_int + acc_ff * cfg_.inertia * 2.0f * M_PI;
}

void Controller::update(float timestep){
    
    float pos_target = 0.0f;
    float vel_feedforward = 0.0f;
    float acc_feedforward = 0.0f;

    if (traj_ready){
        t_traj += dt;
        Trapezoid_component traj = trapezoid_running(t_traj);
        if (t_traj > t_tot) traj_ready = false;

        pos_target = traj.pos;
        vel_feedforward = traj.vel;
        acc_feedforward = traj.acc;
    } else{
        pos_target = enc_->get().pos_estimate;
        vel_feedforward = 0.0f;
        acc_feedforward = 0.0f;
    }

    float position = enc_.get().pos_estimate;
    float velocity = enc_.get().vel_estimate;

    switch (mode_){
        case CONTROL_TORQUE:
            torque_setpoint = torque_target;
            break;
        case CONTROL_VELOCITY:
            torque_setpoint = velocity_control(target_vel, velocity, 0.0f, dt);
            break;
        case CONTROL_POSITION:
            float vel_target = position_control(pos_target, position, vel_feedforward);
            torque_setpoint = velocity_control(vel_target, velocity, acc_feedforward, dt);
            break;
        }

    torque_setpoint = std::clamp(torque_setpoint, -cfg_.torque_limit, cfg_.torque_limit);
}

/*
 * Defining the time and position for each trapezoid phases
    Trapezoid profile phases equation
    https://www.firgelliauto.com/blogs/engineering-calculators/trajectory-planner-trapezoidal-velocity-profile
 */
void Controller::trapezoid_planning(float Xi, float Xf, float Vi, float Amax,
                                    float vel_peak){
    
    X_i = Xi;
    X_f = Xf;
    d_tot = fabs(X_f - X_i);
    if (d_tot < 1e-6f) return;
    direction = (X_f - X_i > 0) ? 1 : -1;

    float accel_time = vel_peak / Amax;

    // Assume symmetry between acceleration and deceleration phases
    float dis_a = 0.5f * Amax * accel_time * accel_time;
    float dis_d = 0.5f * Amax * accel_time * accel_time; 

    V_max = vel_peak;

    if (dis_a + dis_d <= d_tot){
        // Cruising phase time and distance
        float dis_c = d_tot - dis_a - dis_d;
        t_c = dis_c / vel_peak;
    }
    else{
        // Unsufficient distance, switch to triangular profile
        V_max = sqrtf(Amax * d_tot);
        t_a = V_max / Amax;
        t_c = 0.0f;
    }

    t_a = V_max / Amax;
    t_d = V_max / Amax;

    d_a = 0.5f * V_max * t_a;
    d_d = 0.5f * V_max * t_d;

    t_tot = t_a + t_c + t_d;
}

/*
 * Timesteps of the trapezoidal trajectory velocity planning
 */
Controller::Trapezoid_component Controller::trapezoid_running(float time){

    Trapezoid_component trap_traj;
    float p, v, a;

    if (time < 0.0f) {
        p = 0.0f;
        v = 0.0f;
        a = 0.0f;
    } 
    // Acceleration phase (0 <= t < t_a)
    else if (time < t_a) {
        p = 0.5f * A_max * (time*time);
        v = A_max * time;
        a = A_max;
    } 
    // Cruising phase (t_a <= t < t_a + t_c)
    else if (time < t_a + t_c) {
        p = d_a + V_max*(time - t_a);
        v = V_max;
        a = 0.0f;
    } 
    // Deceleration phase (t_a + t_c <= t < t_a + t_c + t_d)
    else if (time < t_tot) {
        float t = t_tot - time;
        p = d_tot - 0.5f * A_max * t * t;
        v = A_max * (time - t);
        a = -A_max;
    } else{
        p = d_tot;
        v = 0.0f;
        a = 0.0f;
    }

    trap_traj.pos = X_i + direction * p;
    trap_traj.vel = v * direction;
    trap_traj.acc = a * direction;

    traj_ready = true;
    return trap_traj;
}