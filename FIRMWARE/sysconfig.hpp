
enum class CONTROL_MODE{
    POSITION,
    VELOCITY,
    TORQUE
};

enum class ENCODER_MODE{
    SINCOS,     // need a lot of calibration for Field Oriented Controller (FOC) but is necessary for torque control so this is the default
    HALL        // this one seems to be pretty shit, might not even use it at all
};

struct System_Config{

    CONTROL_MODE controller_mode;
    ENCODER_MODE encoder_mode;
    float current_limit = 0.0f;
    float velocity_limit = 0.0f;
    float torque_limit = 0.0f;
    bool encoder_state = false;
    bool thermistor_state = false;
    bool is_ready = false;
};