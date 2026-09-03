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

static Encoder* ENC1_SPI = nullptr;
// Custom callback function after SPI communication finishes
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    // Callback after SPI finished
    if (hspi->Instance != SPI1) return;
    ENC1_SPI->ENC_SPI_FINISH();
}

// SPI Data transfer initialization
void Encoder::ENC_SPI_START(){
    // Initialize SPI data transfer
    ENC1_SPI = this;
    // SPI chip select ENC_CSn [active low]
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_RESET)
    HAL_SPI_TransmitReceive_IT(&hspi1, enc_tx, enc_rx, 2);
}

// SPI Data transfer complete, data ready 
void Encoder::ENC_SPI_FINISH(){
    // Deactive SPI Chip select
    HAL_GPIO_WritePin(ENC_CS_GPIO_Port, ENC_CS_Pin, GPIO_PIN_SET);
    
    uint16_t w = ((uint16_t)enc_rx[0] << 8) | enc_rx[1];

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



// Encoder State update
void Encoder::enc_Update(){
    // Encoder data state update from SPI
    // also do some observer shit for the velocity
    
    int delta_count = 0;
    // Update latest data
    int old_count = enc_state.count_wrap;
    float old_posc   = enc_state.pos_estimate;
    float old_velc   = enc_state.vel_estimate;
    float BW_placeholder; // TODO: find this and move it to system config [hardware bounded]


    if(data_ready){
        data_ready = false;

        float dt = 0;

        // Count Update
        int new_count = SPI_raw & 0x3FFFF;
        delta_count = new_count - old_count;
        int wrap = enc_state.count_wrap + delta_count;

        enc_state.count_true += delta_count;
        enc_state.count_wrap = mod(wrap, cfg_.cpr);

        float enc_deg = enc_state.count_wrap * 360.0f / cfg_.cpr;
        float enc_rad = enc_state.count_wrap *2* M_PI / cfg_.cpr;

        // Observer filter
        /*
        The gain is defined using the 2nd order system equation 
        derived from the discrete equation of alpha-beta filter plant
        */

        //TODO:
        // If (Kp && Ki invalid put error check here)

        enc_state.pos_estimate += dt * enc_state.vel_estimate;
        enc_state.pos_cwrap += dt * enc_state.vel_estimate;

        //wrap est_pos_here

        float delta_pos = enc_state.count_true - enc_state.pos_estimate;

        enc_state.pos_estimate += dt * cfg_.observer_alpha * delta_pos;
        enc_state.pos_cwrap += dt * cfg_.observer_alpha * delta_pos;
        //wrap enc_state.pos_cwrap here

        enc_state.vel_estimate += dt * cfg_.observer_beta * delta_pos;
    }
}
