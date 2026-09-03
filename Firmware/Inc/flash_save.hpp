#include "adc_readout.hpp"
#include "control_system.hpp"
#include "encoder.hpp"
#include "motor.hpp"

struct config_axis{
    uint32_t valid;   // verifying non blank flash save or load
    uint32_t layout;

    ADC::config_adc             adc_cfg;
    Controller::config_control  control_cfg;
    Encoder::config_enc         encoder_cfg;
    Motor::config_mot           motor_cfg;

    uint32_t crc;
};

static constexpr uint32_t xvalid = 0x69674200;
static constexpr uint32_t xlayout =
    sizeof(Encoder::config_adc)      * 1u
    + sizeof(Motor::config_control)  * 31u
    + sizeof(Controller::config_enc) * 131u
    + sizeof(ADC::config_mot)        * 521u;

uint32_t compute_crc (const void* data, size_t len);

bool flash_write_config(const config_axis* cfg_);

