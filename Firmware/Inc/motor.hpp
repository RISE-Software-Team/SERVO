/*
TODO:
    ~find torque constant from motor

*/

class Axis;

class Motor{

    public:
        struct config_mot{
            float inductance = 0.0f;            // Calibrated in Calculate_L()  [Henries]
            float resistance = 0.0f;            // Calibrated in Calculate_R()  [Ohms]
            float torque_constant = 0.0f;       // MOTOR DATASHEET  [Nm/A]
            //float current_limit = 28.0f;      // MOTOR FULLSCALE CURRENT [Amps] 28A is unreachable from R_IPROPI
            float current_limit = 24.0f;        // MOTOR FULLSCALE CURRENT [Amps]
            float current_max_step = 0.0f;      // DEFINE THIS
            float duty_step = 0.0f;             // DEFINE THIS
            float torque_limit = 0.0f;          // DEFINE THIS
            float current_bandwidth = 0.0f;     // DEFINE THIS
            float Kp = 0.0f;                    // DEFINE THIS
            float Ki = 0.0f;                    // DEFINE THIS
            float integrator_limit = 0.0f;      // DEFINE THIS
        };

        enum Motor_mode{
            MOTOR_IDLE,     
            MOTOR_OPEN_CONTROL,
            MOTOR_CLOSED_CONTROL,
        };

        enum Motor_Error : uint32_t{
            ERROR_NONE = 0,
            ERROR_INVALID_RESISTANCE = 1U << 1,
            ERROR_INVALID_INDUCTANCE = 1U << 2,
            ERROR_INVALID_GAIN       = 1U << 3,
        };

        Motor_mode Mode = MOTOR_IDLE;

        bool Calculate_R(float V, float I);
        bool Calculate_L(float V, float I_init, float I_final, float dt);
        bool Is_calibrated();

        void set_duty_target(float duty);
        void set_mode(Motor_mode mode);
        void current_target(float torque, float timestep);

        void current_inner_loop(float I_LOAD, float V_BUS);
        bool error_check();

        bool update_gain(float BW);

        bool arm();
        void disarm();
        bool isarmed_;

        const config_m& get_config() const { return cfg_; };
        void set_config(const config_mot& config_load);
        Motor_mode get_mode() const { return Mode; };
        Motor_Error& get_error() { return Error_; };

        float duty_steppoint = 0.0f;

    private:
        config_mot cfg_;
        Motor_Error Error_ = ERROR_NONE;


        float v_int = 0.0f;
        float v_feedforward = 0.0f;
        float i_setpoint = 0.0f;
        float i_target = 0.0f;
        float duty_target = 0.0f;

        void current_inner_loop(float I_LOAD, float V_BUS);
        bool error_check();
};