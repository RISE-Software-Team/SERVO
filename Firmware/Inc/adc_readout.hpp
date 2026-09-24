// ADC CH1 Input processing

/*
 * ACTIVE ADC CHANNEL1 :
 * CH1 IN5 : CURRENT SENSOR [DRV_IPROPI]
 * CH1 IN6 : TEMPERATURE SENSOR [T_FET]
 * CH1 IN7 : VOLTAGE SENSOR [V_BUS]
 * CH1 IN8 : TEMPERATURE SENSOR [T_MOT]
 * CH1 IN9 : POTENTIOMETER [POT] (im pretty sure we dont use this ?)
*/


/**
  * @brief module to process the ADC values
  
  * The module for now consist of the IPROPI from the motor driver [https://tinyurl.com/MODRV], VBUS [Schematics],
  * Temperature from Thermistor : 
    * TFET [https://tinyurl.com/TFET0]
    * TMOT [https://tinyurl.com/TMOT0]

**/
class ADC{
    public:
        // ADC module error classification
        enum ADC_Error : uint32_t{
            ERROR_NONE  = 0,
            ERROR_IDRV_INVALID_READ  = 1U << 1,
            ERROR_IDRV_CURRENT_OVER  = 1U << 2,
            ERROR_TFET_INVALID_READ  = 1U << 3,
            ERROR_TMOT_INVALID_READ  = 1U << 4,
            ERROR_TFET_TEMP_FAILURE  = 1U << 7,
            ERROR_TMOT_TEMP_FAILURE  = 1U << 8,
            ERROR_TEMP_MISMATCH      = 1U << 9,
            ERROR_VBUS_INVALID_READ  = 1U << 10,
            ERROR_VBUS_UNDERVOLTAGE  = 1U << 11,
            ERROR_VBUS_OVERVOLTAGE   = 1U << 12
        };

        // ADC module configuration
        struct config_adc{
            // Schematics OR datasheet based configuration
            float VDDA            = 3.3f;     // Volt
            float ADC_RES         = 4095.0f;  // ADC resolution
            float IPROPI_resistor = 680.0f;   // motor_DRV resistor               [SCHEMATICS]
            float DRV_I_gain      = 202e-6f;  // motor_DRV current gain           [DATASHEET]
            float T_FIX_RES       = 3.3e3f;   // Thermistor voltage divider resistor [SCHEMATICS]
            float V_BUS_DIVIDER   = 19;       // VBUS voltage divider resistor    [SCHEMATICS]
            // Customizable configuration 
            float DRV_I_limit     = 23.0f;    // V_IPROPI reaching 3.3V 
            float V_BUS_UVOLTAGE  = 4.25f;    // VBUS undervoltage threshold      [DATASHEET]   might have to change this
            float V_BUS_OVOLTAGE  = 62.0f;    // VBUS overvoltage threshold       [DATASHEET]   might have to change this
        };

        // ADC raw voltage reading
        struct ADC_Volts{
            float VDRV;
            float VTFET;
            float VBUS;
            float VTMOT;
            float POT = 0.0f;
        };

        // ADC processed values
        struct ADC_Result{
            float I_IPROPI;
            float I_LOAD;
            float T_FET;
            float T_MOT;
            float V_BUS;
        };

        // Thermal varying resistor structur
        struct Thermistor{
            float R_INIT;           // R Thermistor @25C [DATASHEET]
            float Beta;             // Resistance changes due to temp [DATASHEET]
            float WARNING_TEMP;
            float FAILURE_TEMP;  
            float derating_scale;
            bool WARNING;      // sent warning status          
            ADC_Error FAULTYREAD;   // ADC not working properly
            ADC_Error SHUTDOWN;     // overheat, shutdown
        };
        
        // Thermistor FET [datasheet: ]
        Thermistor Thermistor_FET{
            10000.0f,
            3453.0f,    // [PLACEHOLDER] MISSING DATASHEET
            80.0f,
            110.0f,
            fet_scale,
            TFET_WARNING,
            ERROR_TFET_INVALID_READ,
            ERROR_TFET_TEMP_FAILURE
        };

        // Motor board built-in Thermistor [datasheet: ]
        Thermistor Thermistor_MOT{
            10000.0f,   // [DATASHEET]
            4000.0f,   // [DATASHEET]
            80.0f,
            110.0f,
            mot_scale,
            TMOT_WARNING,
            ERROR_TMOT_INVALID_READ,
            ERROR_TMOT_TEMP_FAILURE
        };
        
        /**
          * @brief Calibration and initialization of the DMA
          * @return TRUE if DMA is calibrated and initialized
        **/
        bool init();

        /**
          * @brief converting the raw ADC reading to the observable values
          *
          * The fast loop update IPROPI/ILOAD for the current inner loop
          * the slow loop update TFET, TMOT, and VBUS for safety monitorization
        **/
        void update();

        /**
          * @brief check ADC status
          * @brief return error bitmask for monitoring
        **/
        bool error_check()

        // functions used to extract ADC private variables
        const ADC_Result& get() const {return result_;};
        const config_adc& get_config() const {return cfg_;};
        ADC_Error& get_error() {return Error_;};

        void set_config(const config_adc& config_load);

        // this is a one time thing just for debug to see if the fast and slow update is executed
        bool adc_slow;

        // Temperature warning trigger
        bool TMOT_WARNING;
        bool TFET_WARNING;
        
        // used to slowly decrease the torque/current limit based on current temperature
        float mot_scale;
        float fet_scale;
        float scale;
        

    private:
        // ADC raw value volatile storage
        volatile uint16_t ADC_raw[5];

        // ADC internal structs
        ADC_Volts  voltage_;
        ADC_Result result_;
        config_adc cfg_;
        ADC_Error Error_ = ERROR_NONE;

        float Igain = 1.0f/(cfg_.DRV_I_gain * cfg_.IPROPI_resistor);

        // ADC_slow trigger timer
        volatile uint32_t ADC_timer; 

        /**
          * @brief Voltage to temperature conversion using the beta equation
          * @param Tcfg_ thermistor configuration
          * @param v adc voltage readout
          * @return temperature in celcius
        **/
        float Thermistor_NCT(float v, Thermistor& Tcfg_);   

        /**
          * @brief temperature status check for overheat and warning
          * 
          * @return In the case of overheat returns an Error code for TFET or TMOT
          * @return In the case of warning, sent a boolean to limit the current / torque 
        **/
        ADC_Error temperature_check(float temp, Thermistor& Tcfg_);  

        /**
          * @brief ADC Slow route, update VBUS, TFET, and TMOT
          * 
          * The point is to prioritize the ILOAD conversion since the Volt - Temp 
          * using the beta equation is costly due to the logf 
        **/
        void ADC_slow();
};