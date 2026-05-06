#include "encoder.hpp"

// Finding the rotational direction with respect to the motor
void Encoder::find_direction(){
    int init_enc = 0;
    /*
        Run the open loop controller, sweep it for a certain angle range that increase/decrease the counter
    */

    if (init_enc < count)
        enc_config.direction = -1; 
    else if (init_enc > count)
        enc_config.direction = 1;
    else
        enc_config.direction = 0; // Fail movement
}

// Finding the zero electrical angle based on the encoder reading
void Encoder::find_phase_offset(){

    int init_enc = 0;
    /*
        Run the open loop controller sweep it in pi/2 to -pi/2 to increase/decrease the counter
    */

    // OPEN LOOP CONTROL TO PI/2
    if (init_enc < count)
        enc_config.direction = -1; 
    else if (init_enc > count)
        enc_config.direction = 1;
    else
        enc_config.direction = 0; // Fail movement

    // OPEN LOOP CONTROL TO -PI/2
}