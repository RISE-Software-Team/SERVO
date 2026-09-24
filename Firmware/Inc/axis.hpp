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

/**
 * @brief Main top level controller for the servo axis

 * used to coordinates the servo state machine and error fault handling, 
 * which includes the ADC, Encoder, Motor, Controller and future desired module
 * run the update() with the pwm clock to do error_check()
*/
struct Axis{
    public:
        // Operating axis states
        enum class Axis_States{
            UNINITIALIZED,
            SETUP,
            IDLE,
            CALIBRATING,
            CLOSED_LOOP,
            AXIS_ERROR
        };
        
        // Axis error handling utilizing bitmask
        enum  Axis_Error : uint32_t{
            ERROR_NONE          = 0,
            ERROR_ADC           = 1U << 1,
            ERROR_ENCODER       = 1U << 2,
            ERROR_MOTOR         = 1U << 3,
            ERROR_CONTROLLER    = 1U << 4,
        };

        // Servo calibration procedure
        enum calibration {
            RESISTANCE,
            INDUCTANCE,
            DIRECTION,
            FINISH,
            ERROR
        };

        // Control loop frequency (based on PWM Timer)
        static constexpr float control_freq = 20000.0f;
        static constexpr float dt           = 1/control_freq;

        /**
         * @brief Initialize axis modules
         * @return True if all modules successfully initialized, false if error occured
         */
        bool init();

        /*
         * @brief Executes the error check and switch cases based on states_
         *
         * running at control_freq
         */
        void update();

        /**
         * @brief change and execute the initial condition for the desired calibration sequence
         */
        void set_calibration(calibration set_calibration);

        /**
         * @brief change and execute the initial conditions for the desired axis state
         * @param new desired state
         * @return TRUE if specified conditions is satisfied
        */
        bool set_state(Axis_States states);

        /**
         * @brief user interaction to set the position target
         * @param p_target target position in rad
         * @return  TRUE: if target is achievable
         * @return  FALSE: if the current velocity is above the threshold 
         */
        bool set_position(float p_target);

        /**
         * @brief user interaction to set the velocity target
         * @param v_target target velocity in rad/s
         * @return true if target is valid
         */
        bool set_velocity(float v_target);

        /**
         * @brief user interaction to set the torque target
         * @param t_target target torque in Nm
         * @return true if target is valid
         */
        bool set_torque(float t_target);

        /**
         * @brief get current value and direction
         * @return motor current in Amps 
         */
        float motor_current();

        /**
         * @brief save the axis and the other modules configuration
         * @return true if data sucessfully flashed
        */
        bool save_config();

    private:

        // Calibration state
        float timer;
        float t_settle = 0.0f;
        float t_measure = 0.0f;
        float I_init;
        float calib_duty = 0.1f;
        int enc_init; 
        int enc_finish;

        // Missed ADC / ENCODER error count
        int misscount;

        // Axis-owned subclasses
        Encoder encoder_;
        Motor motor_;
        Controller controller_;
        ADC adc_;
        SPI_manager spi_;

        bool load_config();

        // Magic number i made up to validate FLASH save attempts
        static constexpr uint32_t xvalid = 0x69674200;

        // FLASH saving layout to ensure no overlapping data
        static constexpr uint32_t xlayout =
            sizeof(Encoder::config_enc)             * 1u
            + sizeof(Motor::config_mot)             * 31u
            + sizeof(Controller::config_control)    * 131u
            + sizeof(ADC::config_adc)               * 521u;

        // Axis States
        Axis_States states_ = Axis::Axis_States::UNINITIALIZED;
        Axis_Error  Error_  = ERROR_NONE;
        calibration calibration_ = FINISH;

        // Motor resistance, inductance and encoder calibration sequence
        void calibration_sequence(calibration calibration_state);

        // Pull error from all modules get_error
        bool error_check();

        void SPI_finish();

};
