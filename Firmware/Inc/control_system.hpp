// MAIN CONTROL SYSTEM
// Utilize the PID for position and velocity
//

/*
Control system using the cascaded Position-Velocity P-PI controller
*/

#include "encoder.hpp"

/*
TODO:
    ~find bandwidth
    ~set int, vel, and torque limit
    ~maximum velocity threshold [whether or not the current speed is too fast to reach the target pos]
    ~find motor inertia
*/


class Controller{

    public:
        // Position Control
        enum Control_Mode{
            CONTROL_TORQUE,     // Torque control using inner current loop
            CONTROL_VELOCITY,   // Velocity controller
            CONTROL_POSITION,   // Position controller
        };

        enum Control_Error : uint32_t{
            ERROR_NONE = 0,
            ERROR_INVALID_GAIN      = 1U << 1,
            ERROR_INVALID_TARGET    = 1U << 2,
            ERROR_REPLANNING        = 1U << 3,
        };

        struct config_control{
            float pos_KP = 0.0f;

            float vel_KP = 0.0f;
            float vel_KI = 0.0f;

            float integrator_limit = 0.0f;      // Velocity Integrator limit
            float velocity_limit = 0.0f;        // basically Vmax ?
            float torque_limit = 0.0f;
            float velocity_threshold = 0.0f;    // Velocity threshold for replanning
            float bandwidth = 0.0f;             // System bandwidth
            float max_bandwidth = 0.0f;
            float inertia = 0.0f;               // Motor Inertia [DATASHEET]
        };

        // Velocity profile
        struct Trapezoid_component{
            float pos;
            float vel;
            float acc;
        };

        void update(float timestep);
        void update_gain(float BW);
        void set_mode (Control_Mode mode);

        bool target_pos(float p_target);
        bool target_vel(float v_target);
        bool target_torque(float t_target);

        Control_Error& get_errror() {return error_;};
        config_control& get_config() {return cfg_;};
        void set_config(const config_control& config_load);

        float pos_target;
        float vel_target;
        float torque_target;
        float torque_setpoint;
        int direction; 

    private:

        float V_max;
        float A_max;
        
        float X_i;      // Initial Position
        float X_f;      // Final Position
        float V_i;      // Initial Velocity

        float t_a;      // Acceleration time
        float t_c;      // Cruising time
        float t_d;      // Deceleration time
        float d_a;      // Acceleration distance
        float d_c;      // Cruising distance
        float d_d;      // Deceleration distance

        float t_tot;
        float d_tot;
        float t_traj;
        bool traj_ready = false;
        float vel_int;

        config_control cfg_;
        Control_Error error_ = ERROR_NONE;
        Control_Mode mode_ = CONTROL_TORQUE;

        // Velocity planning
        void trapezoid_planning(float Xi, float Xf, float Vi, float Amax, float Vmax);
        Trapezoid_component trapezoid_running(float time);

        float position_control(float target, float position, float vel_ff);
        float velocity_control(float vel_setpoint, float vel, float acc_ff, float dt);
        float torque_control();
        
        Encoder& enc_;
};