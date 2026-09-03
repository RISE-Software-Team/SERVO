// ADC CH1 Input processing

/*
 * ACTIVE ADC CHANNEL1 :
 * CH1 IN5 : CURRENT SENSOR [DRV_IPROPI]
 * CH1 IN6 : TEMPERATURE SENSOR [T_FET]
 * CH1 IN7 : VOLTAGE SENSOR [V_BUS]
 * CH1 IN8 : TEMPERATURE SENSOR [T_MOT]
 * CH1 IN9 : POTENTIOMETER [POT] (im pretty sure we dont use this ?)
*/

/*
TODO:
    ~T_FET: BETA, WARNING, SHUTDOWN TEMP
    ~T_MOT: RESISTOR, BETA, TEMP
    ~look into the over and under voltage for the VBUS
*/

class ADC{
    public:
        friend void ::HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef*);

        enum ADC_Error : uint32_t{
            ERROR_NONE  = 0,
            ERROR_IDRV_INVALID_READ  = 1U << 1,
            ERROR_IDRV_CURRENT_OVER  = 1U << 2,
            ERROR_TFET_INVALID_READ  = 1U << 3,
            ERROR_TMOT_INVALID_READ  = 1U << 4,
            ERROR_TFET_TEMP_WARNING  = 1U << 5,
            ERROR_TMOT_TEMP_WARNING  = 1U << 6,
            ERROR_TFET_TEMP_FAILURE  = 1U << 7,
            ERROR_TMOT_TEMP_FAILURE  = 1U << 8,
            ERROR_TEMP_MISMATCH      = 1U << 9,
            ERROR_VBUS_INVALID_READ  = 1U << 10,
            ERROR_VBUS_UNDERVOLTAGE  = 1U << 11,
            ERROR_VBUS_OVERVOLTAGE   = 1U << 12
        };

        struct config_adc{
            float VDDA            = 3.3f;     // Volt
            float ADC_RES         = 4095.0f;  // ADC resolution
            float IPROPI_resistor = 680.0f;   // motor_DRV resistor               [SCHEMATICS]
            float DRV_I_gain      = 202e-6f;  // motor_DRV current gain           [DATASHEET]
            float DRV_I_limit     = 23.0f;    // V_IPROPI reaching 3.3V 
            float T_FIX_RES       = 3.3e3f;   // Thermistor voltage divider resistor [SCHEMATICS]
            float V_BUS_DIVIDER   = 19;       // VBUS voltage divider resistor    [SCHEMATICS]
            float V_BUS_UVOLTAGE  = 4.25f;    // VBUS undervoltage threshold      [DATASHEET]   might have to change this
            float V_BUS_OVOLTAGE  = 62.0f;    // VBUS overvoltage threshold       [DATASHEET]   might have to change this
        };

        struct ADC_Volts{
            float VDRV;
            float VTFET;
            float VBUS;
            float VTMOT;
            float POT = 0.0f;
        };

        struct ADC_Result{
            float I_IPROPI;
            float I_LOAD;
            float T_FET;
            float T_MOT;
            float V_BUS;
        };

        struct Thermistor{
            float R_INIT;           // R Thermistor @25C [DATASHEET]
            float Beta;             // Resistance changes due to temp [DATASHEET]
            float WARNING_TEMP;
            float FAILURE_TEMP;            
            ADC_Error FAULTYREAD;   // ADC not working properly
            ADC_Error WARNING;      // sent warning status
            ADC_Error SHUTDOWN;     // overheat, shutdown
        };
        
        Thermistor Thermistor_FET{
            10000.0f,
            3250.0f,    // [PLACEHOLDER] MISSING DATASHEET
            110.0f,
            130.0f,
            ERROR_TFET_INVALID_READ,
            ERROR_TFET_TEMP_WARNING,
            ERROR_TFET_TEMP_FAILURE
        };

        Thermistor Thermistor_MOT{
            0.0f,   // [PLACEHOLDER] MISSING DATASHEET
            0.0f,   // [PLACEHOLDER] MISSING DATASHEET
            110.0f,
            130.0f,
            ERROR_TMOT_INVALID_READ,
            ERROR_TMOT_TEMP_WARNING,
            ERROR_TMOT_TEMP_FAILURE
        };
        
        void ADC_Init();
        void ADC_Update();
        bool error_check() {return }
        bool adc_slow;

        const ADC_Result& get() const {return result_;};
        const config_adc& get_config() const {return cfg_;};
        void set_config(const config_adc& config_load);


        ADC_Error Error_ = ERROR_NONE;

        ADC_Error& get_error() {return Error_;};

    private:

        volatile uint16_t ADC_raw[5];
        ADC_Volts  voltage_;
        ADC_Result result_;
        config_adc cfg_;
        float Igain = 1.0f;/(cfg_.DRV_I_gain * cfg_.IPROPI_resistor);
        float Thermistor_NCT(float v, Thermistor& Tcfg_);   
        void temperature_check(float temp, Thermistor& Tcfg_);  
        void ADC_slow();
        bool error_check();
        volatile uint32_t ADC_timer; 
};