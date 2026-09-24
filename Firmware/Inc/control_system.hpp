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
        /// Controller modes
        enum Control_Mode{
            CONTROL_TORQUE,     ///< Torque control using inner current loop.
            CONTROL_VELOCITY,   ///< Velocity control.
            CONTROL_POSITION,   ///< Position control.
        };

        /// Controller Error classification
        enum Control_Error : uint32_t{
            ERROR_NONE = 0,
            ERROR_INVALID_GAIN      = 1U << 1,
            ERROR_INVALID_TARGET    = 1U << 2,
            ERROR_REPLANNING        = 1U << 3,
        };

        /// Controller class configuration
        struct config_control{
            float pos_KP = 0.0f;               ///< Position proportional gain.

            float vel_KP = 0.0f;               ///< Velocity proportional gain.
            float vel_KI = 0.0f;               ///< Velocity integral gain.

            float integrator_limit = 0.0f;     ///< Velocity integrator limit.
            float velocity_limit = 0.0f;       ///< Maximum velocity [rad/s].
            float torque_limit = 0.0f;         ///< Maximum torque [Nm].
            float velocity_threshold = 0.0f;   ///< Replanning threshold [rad/s].
            float bandwidth = 0.0f;            ///< Desired bandwidth [Hz].
            float max_bandwidth = 0.0f;        ///< Maximum permitted bandwidth [Hz].
            float inertia = 0.0f;              ///< System inertia.
        };

        /// Trajectory planning output component
        struct Trapezoid_component{
            float pos;      ///< Position [rad]
            float vel;      ///< Velocity [rad/s]
            float acc;      ///< Acceleration [rad/s^2]
        };

        /// Initializes controller runtime state.
        bool init();

        /**
          * @brief main update sequence to calculate the torque setpoint based on the previously planned trajectory
          * @param timestep based on the PWM frequency clock
        **/
        void update(float timestep);

        /** @brief customize the controller gain from the desired bandwidth **/
        void update_gain(float BW);

        /**
          * @brief customize the controller operating mode
          * @param mode choose between position, velocity, and torque
        **/
        void set_mode (Control_Mode mode);

        /**
        * @brief Sets the position target.
        * @param p_target Position target [rad].
        * @return true if the target is accepted.
        */
        bool target_pos(float p_target);

        /**
        * @brief Sets the velocity target.
        * @param v_target Velocity target [rad/s].
        * @return true if the target is accepted.
        */
        bool target_vel(float v_target);

        /**
        * @brief Sets the torque target.
        * @param t_target Torque target [Nm].
        * @return true if the target is accepted.
        */
        bool target_torque(float t_target);

        /// Returns the current controller error flags.
        Control_Error get_error() const { return error_; }

        /// Returns the active controller configuration.
        config_control& get_config() { return cfg_; }

        /// Loads controller configuration.
        void set_config(const config_control& config_load);

        /**
          * @brief torque limiter in the case of exceeding (or increasing) temperature
          * @param scale is used to ramp down the torque limit if the temperature still increasing
        **/
        void torque_temperature_limit(float scale);

        float pos_target;
        float vel_target;
        float torque_target;
        int direction; 

        float torque_setpoint;


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

        float true_torque_limit = 0.0f;

        // Controller structs
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