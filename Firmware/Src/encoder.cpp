#include "encoder.hpp"
#include "stm32l5xx_hal.h"
#include "stm32l5xx_hal_gpio.h"
#include "utilities.hpp"
#include <cmath>
#include <stdint.h>
#include <math.h>

// Finding the rotational direction with respect to the motor

/*
TODO:
    ~ encoder - MCU SPI connection
    ~ enc_Update()
        > ADC read for position
        > velocity using observers (?)

*/

// SPI Data transfer initialization
void Encoder::enc_spi_start(){
    // Initialize SPI data transfer

    // SPI chip select ENC_CSn [active low]
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET)
    HAL_SPI_TransmitReceive_IT(&hspi1, (uint8_t*)enc_tx, (uint8_t*)enc_rx, 1);
}

// SPI Data transfer complete, data ready 
void Encoder::enc_spi_finish(){
    // Deactive SPI Chip select
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
    
    uint16_t w = enc_rx;

    // Package even parity check based on AS5047P Datasheet
    if (w & 0x4000 || __builtin_parity(w) != 0) {
        error_ |= ERROR_SPI_PARITY;   // don't publish a corrupt angle
        return;
    }

    SPI_raw = w & 0x3FFF;
    data_ready = true;
}

void Encoder::update_observer_gain(float BW){
    bandwidth = BW;
    float Kp = 2.0f * bandwidth;

    cfg_.observer_alpha = Kp;
    cfg_.observer_beta= 0.25f * (Kp*Kp);
}

bool Encoder::enc_direction(int delta_count){
    if (std::isnan(delta_count)){
        cfg_.direction = 0;
        error_ |= ERROR_INVALID_DIRECTION;
        return false;
    }
    else{
        cfg_.direction = (delta_count > 0) ? 1 : -1;
        return true;
    }

}

void Encoder::set_config(const config_enc& config_load){
    cfg_ = config_load;
    update_observer_gain(cfg_.bandwidth);
}


bool Encoder::init(){
    enc_state.count_true = 0;
    enc_state.pos_estimate = 0.0f;
    enc_state.vel_estimate = 0.0f;
    data_ready = false;
}

// Encoder State update
void Encoder::enc_Update(){
    // Encoder data state update from SPI
    // also do some observer shit for the velocity
    
    int delta_count = 0;
    // Update latest data
    int old_count = enc_state.count_wrap;

    //  Eliminate velocity jittering 
    if (fabs(vel_estimate_counts) < 0.5 * dt * cfg_.observer_beta) vel_estimate_counts = 0.0f;

    pos_estimate_counts += dt* vel_estimate_counts;
    pos_estimate_countwrap += dt * vel_estimate_counts;

    if(data_ready){
        data_ready = false;

    // Count Update
        int new_count = SPI_raw & 0x3FFF;
        delta_count = new_count - old_count;

    // modulus to wrap delta to [0 ~ cpr]
        delta_count = mod(delta_count, cfg_.cpr);

    // change wrapping to [-cpr/2 ~ cpr/2] to differentiate direction
        if(delta_count > cfg_.cpr * 0.5f) delta_count -= cfg_.cpr;

        enc_state.count_true += delta_count;
        enc_state.shadow_count += delta_count;
    // Observer filter
    /*
        The gain is defined using the 2nd order system equation 
        derived from the discrete equation of alpha-beta filter plant
    */
        float delta_pos = (float)(enc_state.count_true - (int)std::floor(pos_estimate_counts));

        pos_estimate_counts += dt * cfg_.observer_alpha * delta_pos;
        vel_estimate_counts += dt * cfg_.observer_beta * delta_pos;
    } else {
        // Encoder fail to update
        missing_update++
    }
    
    // Convert from cpr counts to turn [0 ~ 1]
        enc_state.vel_estimate = vel_estimate_counts / (float)cfg_.cpr;
        enc_state.pos_estimate = pos_estimate_counts / (float)cfg_.cpr;
}
