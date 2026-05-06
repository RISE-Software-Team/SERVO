// MAIN CONTROL SYSTEM
// Utilize the PID for position and velocity
// but ive read that for torque control its easier to use Field-Oriented Control (FOC)
//

#include "sysconfig.hpp"

class Controller{

    struct Control_Config{
        CONTROL_MODE mode_;
        float P_gain = 0.0f;
        float I_gain = 0.0f;
        float D_gain = 0.0f;

        float integrator_limit = 0.0f;
        float velocity_limit = 0.0f;
    };

    Control_Config control_config_;

    void update_gain(float P, float I, float D);

    // Position Control
    void move_to_target(float p_target);
    

};