#pragma once
#include <stdint.h>
#include "stm32l5xx_hal.h"

// External variable SPI connection
extern SPI_HandleTypeDef hspi1;


class Encoder{

    public:
        // Encoder module configuration to FLASH
        struct config_enc{
            int cpr = 16384;                // Magnetic Encoder Resolution
            int direction = 0;              // Direction of the encoder, relative to the motor
            
            float observer_alpha = 0.0f;       // Observer Proportional gain
            float observer_beta  = 0.0f;       // Observer Derivational gain
            float bandwidth      = 1.0f;
        };

        // Encoder module state
        struct Encoder_State{
            uint32_t shadow_count;          // non-resetting true counter
            int count_true;                 // True count from the encoder
            int count_wrap;                 // Count with wraparound
            float pos_estimate = 0.0f;      // Position estimate in counts
            float vel_estimate = 0.0f;      // Velocity estimate in counts/s
            float pos_cwrap = 0.0f;         // Position estimate in counts with 2pi wraparound
        };

        // Error classifications
        enum Encoder_Error : uint32_t{
            ERROR_NONE = 0,
            ERROR_SPI_PARITY = 1U << 1,
            ERROR_INVALID_DIRECTION = 1U << 2,
        };

        // Encoder SPI transmission
        bool data_ready;
        uint8_t SPI_raw;

        // Timer frequency timestep [BASED ON TIM4]
        float frequency = 20000.0f;
        float dt = 1.0f/20000.0f;

        /**
          * @brief refresh encoder state that has the possibility to overflow
        **/
        bool init();

        // Apply loaded configuration from FLASH
        void set_config(const config_enc& config_load);

        /** 
          * @brief Update cfg observer gain and force it to be overdamped system
          * @param BW frequency 
        **/
        void update_observer_gain(float BW);

        /**
          * @brief encoder calibration sequence to update the positive and negative direction
          * @param delta_count the changes in observed count after certain t_settle
        **/
        void enc_direction(int delta_count);

        /**
          * @brief processed data and run the observer to update the encoder state
        **/
        void enc_Update();


        /**
          * @brief Initialize SPI Communications between MCU and encoder

          * pulldown the nCS to signal the desired SPI device and initiate transmit from the HAL library
        **/
        void enc_spi_start();

        /**
          * @brief functions called on the SPI_finish callback, used to validate the data and update raw state
          
          * The parity check is based on EVEN parity [DATASHEET] 
        **/
        void enc_spi_finish();

        // functions used to extract class private variables
        const Encoder_State& get() const { return enc_state;};
        const config_enc& get_config() const { return cfg_; };
        Encoder_Error get_error() {return error_;};

    private:
        // Encoder private configurations
        Encoder_Error error_ = ERROR_NONE;
        config_enc cfg_;
        uint16_t enc_tx = {0xFFFF}; // 0x3FFF adress + read + parity
        uint16_t enc_rx;
        Encoder_State enc_state;

        // Encoder temporary state
        float pos_estimate_counts;
        float pos_estimate_countwrap;
        float vel_estimate_counts;

        // Variable used to keep count on missing time cycles
        int missing_update;
};