// This is meant to be simple open control loop for calibration reasons
// 


class OpenLoop_controller{

    // Configuration
    float MAX_CURRENT_RAMP  = 0.0f;
    float MAX_VOLTAGE_RAMP  = 0.0f;
    float MAX_VELOCITY_RAMP = 0.0f;

    // Targets
    float TARGET_VELOCITY = 0.0f;
    float TARGET_CURRENT = 0.0f;
    float TARGET_VOLTAGE = 0.0f;

    // Outputs
    float TOTAL_DISTANCE = 0.0f;

};