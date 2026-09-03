// Main Servo function
// This is where everything is being ran on
// Task priority, calibration sequence, open and closed loop control

#include "encoder.hpp"
#include "motor.hpp"
#include "control_system.hpp"
#include "adc_readout.hpp"
#include "stm32l552xx.h"
#include "utilities.hpp"
#include <cstdint>
#include <cstddef>



struct Axis{
    public:
        enum class Axis_States{
            IDLE,           // System sleep
            CALIBRATING,    // Only utilize open loop control
            CLOSED_LOOP,    // POS/VEL/TORQUE closed loop
            AXIS_ERROR           // HALT SYSTEM and send error message
        };
        
        enum  Axis_Error : uint32_t{
            ERROR_NONE          = 0,
            ERROR_ADC           = 1U << 1,
            ERROR_ENCODER       = 1U << 2,
            ERROR_MOTOR         = 1U << 3,
            ERROR_CONTROLLER    = 1U << 4,
        };

        enum calibration {
            RESISTANCE,
            INDUCTANCE,
            DIRECTION,
            FINISH,
            ERROR
        }

        static constexpr float control_freq = 20000.0f;
        static constexpr float dt           = 1/control_freq;

        void update();
        void set_calibration(calibration set_calibration);
        void axis_run();

        bool set_position(float p_target);
        bool set_velocity(float v_target);
        bool set_torque(float t_target);

        float current_magnitude();

        bool save_config();
    private:

        float timer;
        float t_settle = 0.0f;
        float t_measure = 0.0f;
        float I_init;
        float calib_duty = 0.1f;
        int enc_init; 
        int enc_finish;


        // Subclass Inititalization
        Encoder encoder_;
        Motor motor_;
        Controller controller_;
        ADC adc_;

        // Subclasses flash saved config
        bool save_config();
        bool load_config();
        static constexpr uint32_t xvalid = 0x69674200;
        static constexpr uint32_t xlayout =
            sizeof(Encoder::config_adc)      * 1u
            + sizeof(Motor::config_control)  * 31u
            + sizeof(Controller::config_enc) * 131u
            + sizeof(ADC::config_mot)        * 521u;

        // Axis States
        Axis_States states_ = Axis_States::IDLE;
        Axis_Error  Error_  = ERROR_NONE;
        calibration calibration_ = FINISH;

        Axis Axes(Encoder encoder,
                  Motor motor,
                  Controller control,
                  ADC ADC);

        bool set_state(Axis_States states);

        void calibration_sequence(calibration calibration_state);

        bool error_check();
};