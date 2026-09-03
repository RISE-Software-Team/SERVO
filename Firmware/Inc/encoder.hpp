#pragma once
#include <stdint.h>
#include "stm32l5xx_hal.h"

// External variable SPI connection
extern SPI_HandleTypeDef hspi1;


class Encoder{

    public:
        struct config_enc{
            int cpr = 16384;                // Magnetic Encoder Resolution
            int direction = 0;              // Direction of the encoder, relative to the motor
            
            float observer_alpha = 0.0f;       // Observer Proportional gain
            float observer_beta  = 0.0f;       // Observer Derivational gain
            float bandwidth      = 1.0f;
        };

        struct Encoder_State{
            int count_true;                 // True count from the encoder
            int count_wrap;                 // Count with wraparound
            float pos_estimate = 0.0f;      // Position estimate in counts
            float vel_estimate = 0.0f;      // Velocity estimate in counts/s
            float pos_cwrap = 0.0f;         // Position estimate in counts with 2pi wraparound
        };

        enum Encoder_Error : uint32_t{
            ERROR_NONE = 0,
            ERROR_SPI_PARITY = 1U << 1,
            ERROR_INVALID_DIRECTION = 1U << 2,
        };

        // Encoder SPI transmission
        bool data_ready;
        uint8_t SPI_raw;

        void update_observer_gain(float BW);
        void enc_direction(int delta_count);
        void enc_Update();

        const Encoder_State& get() const { return enc_state;};
        const config_enc& get_config() const { return cfg_; };
        void set_config(const config_enc& config_load);

        Encoder_Error get_error() {return error_;};
        void ENC_SPI_START();


    private:
        Encoder_Error error_ = ERROR_NONE;
        config_enc cfg_;
        uint8_t enc_tx[2] = {0xFF, 0xFF}; // 0x3FFF adress + read + parity
        uint8_t enc_rx[2];
        Encoder_State enc_state;
        void ENC_SPI_FINISH();
};