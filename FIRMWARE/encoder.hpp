#include "sysconfig.hpp"


class Encoder{

    public:

        struct Encoder_Config{
            ENCODER_MODE mode_;
            float calibration_acc = 0.0f;   // How much accuracy we want for the calibration
            int cpr = 0;                    // Magnetic Encoder Resolution
            float index_offset = 0.0f;      // used for absolute positioning i think
            float phase_offset = 0.0f;      // Difference between encoder mechanical to motor electrical
            int direction = 0;              // Direction of the encoder, relative to the motor
            
        };
        bool is_calibrated = false;

    private:
        
        Encoder_Config enc_config;
        void find_phase_offset();
        void find_direction();

        int count = 0;

};