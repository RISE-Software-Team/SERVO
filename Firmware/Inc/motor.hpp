/*
TODO:
    ~find torque constant from motor

*/

#include <cstdint>
#define S_MODE_PWM      (0b10)

// CONFIG1 — bit positions from your Table 7-23
#define CFG1_EN_OLA     (1u << 7)       // enable open load detection in the active state.
#define CFG1_OTW_SEL    (1u << 6)       // Over Temperature Warning threshold 0b = 140C 1b = 120C
#define CFG1_OVSEL      (1u << 5)       // 0b: VMOV enabled     1b: VMOV disabled
#define CFG1_SSC_DIS    (1u << 4)       // Enables the spread spectrum clocking feature
#define CFG1_OCP_RTRY   (1u << 3)       // 1b to configure fault reaction to retry setting on the detection of overCURRENT
#define CFG1_TSD_RTRY   (1u << 2)       // 1b to configure fault reaction to retry setting on the detection of overTEMP
#define CFG1_OV_RTRY    (1u << 1)       // 1b to configure fault reaction to retry setting on the detection of overVOLT
#define CFG1_OLA_RTRY   (1u << 0)

// All *_RTRY clear = latched faults, which is what a servo wants.
#define CONFIG1     (CFG1_SSC_DIS)   // or 0 to enable spread spectrum 
static constexpr uint8_t CONFIG1_VAL = static_cast<uint8_t>(CONFIG1);

// CONFIG2
#define CFG2_EXTEND     (1u << 7)               // enable extend hi-z 
#define CFG2_S_DIAG(x)  (((x) & 0x3u) << 5)     // s_diag
#define CFG2_ISEL(x)    (((x) & 0x3u) << 3)     // current mirror or temperature 
#define CFG2_S_ITRIP(x) (((x) & 0x7u) << 0)     // ITRIP voltage regulator

#define CONFIG2    (CFG2_ISEL(0b11) | CFG2_S_ITRIP(0b000))    // activate current mirror, itrip disabled
static constexpr uint8_t CONFIG2_VAL = static_cast<uint8_t>(CONFIG2);

// CONFIG3
#define CFG3_TOFF(x)    (((x) & 0x3u) << 6)     // ITRIP shutdown time
#define CFG3_EN_POB     (1u << 5)               // powered off braking 
#define CFG3_TBLK       (1u << 4)               // blank time conf
#define CFG3_SR(x)      (((x) & 0x3u) << 2)     // slew rate conf
#define CFG3_S_MODE(x)  (((x) & 0x3u) << 0)     // device mode configuration

#define CONFIG3     (CFG3_TOFF(0b01) | CFG3_SR(0b01) | \
                                CFG3_S_MODE(S_MODE_PWM))    // activate driver PWM mode

static constexpr uint8_t CONFIG3_VAL = static_cast<uint8_t>(CONFIG3);

// CONFIG4
#define CFG4_OTW_REP    (1u << 7)               // 1b for overtemp in fault or not 0b
#define CFG4_TOCP       (1u << 6)               // filter time for overcurrent
#define CFG4_OLA_FLTR   (1u << 5)               // OLA filter
#define CFG4_OCP_SEL(x) (((x) & 0x3u) << 3)     // threshold current regulation
#define CFG4_DRV_SEL    (1u << 2)               // DRVPIN [0b OR] or [1b AND]
#define CFG4_ENIN1_SEL  (1u << 1)               // EN/IN 1 pin register
#define CFG4_PHIN2_SEL  (1u << 0)               // PH/IN 2 pin register

#define CONFIG4     (CFG4_TOCP | CFG4_OCP_SEL(0b00) | CFG4_DRV_SEL)

static constexpr uint8_t CONFIG4_VAL = static_cast<uint8_t>(CONFIG4);


namespace DrvReg {
    constexpr uint8_t DEVICE_ID = 0x00;     // val : Device name and ID value
    constexpr uint8_t FAULT     = 0x01;     // SPI_error, power-on-reset, logic fault, under/over voltage, overtemp
    constexpr uint8_t STATUS1   = 0x02;     // Device status and overcurrent warning
    constexpr uint8_t STATUS2   = 0x03;     // DRV_OFF and device status and overtemp warning
    constexpr uint8_t COMMAND   = 0x08;     // clear fault, SPI & REG lock
    constexpr uint8_t SPI_IN    = 0x09;
    constexpr uint8_t CFG1_REG  = 0x0A;
    constexpr uint8_t CFG2_REG  = 0x0B;
    constexpr uint8_t CFG3_REG  = 0x0C;
    constexpr uint8_t CFG4_REG  = 0x0D;
}


class Motor{

    public:
        // Motor module configuration
        struct config_mot{
            float inductance = 0.0f;            // Calibrated in Calculate_L()  [Henries]
            float resistance = 0.0f;            // Calibrated in Calculate_R()  [Ohms]
            float torque_constant = 0.0f;       // MOTOR DATASHEET  [Nm/A]
            //float current_limit = 28.0f;      // MOTOR FULLSCALE CURRENT [Amps] 28A is unreachable from R_IPROPI
            float current_limit = 24.0f;        // MOTOR FULLSCALE CURRENT [Amps]
            float current_max_step = 0.0f;      // DEFINE THIS
            float duty_step = 0.0f;             // DEFINE THIS
            float torque_limit = 0.0f;          // DEFINE THIS
            float current_bandwidth = 0.0f;     // DEFINE THIS
            float Kp = 0.0f;                    // DEFINE THIS
            float Ki = 0.0f;                    // DEFINE THIS
            float integrator_limit = 0.0f;      // DEFINE THIS
        };

        enum Motor_mode{
            MOTOR_IDLE,     
            MOTOR_OPEN_CONTROL,
            MOTOR_CLOSED_CONTROL,
        };

        enum Motor_Error : uint32_t{
            ERROR_NONE = 0,
            ERROR_INVALID_RESISTANCE = 1U << 1,
            ERROR_INVALID_INDUCTANCE = 1U << 2,
            ERROR_INVALID_GAIN       = 1U << 3,
            ERROR_DRIVER_FAULT       = 1U << 4,
            ERROR_SPI_INIT_FAIL      = 1U << 5 
        };

        Motor_mode Mode = MOTOR_IDLE;

        // Motor resistance calibration sequence
        bool Calculate_R(float V, float I);

        // Motor inductance calibration sequence
        bool Calculate_L(float V, float I_init, float I_final, float dt);

        /**
          * @brief checking motor config values to determine whether or not the calibration sequence is finished
          * @return true if resistance and inductance is defined
        **/
        bool Is_calibrated();

        /** 
          * @brief public function to change the desired duty cyccle for open loop controller
          * @param duty cycle in [0 ~ 1] 
        **/
        void set_duty_target(float duty);

        // Change motor mode
        void set_mode(Motor_mode mode);

        /**
          * @brief torque to current convertion used to determine the target current for the motor
          * @param torque calculation result from the controller
          * @param timestep calculating maximum changes of torque target

          * the current target can only change based on the achievable changes within the specified timestep
          * it also includes a feedforward component based on the motor resistance, current vel and the torque constant
        **/
        void current_target(float torque, float timestep);

        /**
          * @brief innermost control loop 
        **/
        void current_inner_loop(float I_LOAD, float V_BUS);
        bool error_check();

        bool update_gain(float BW);

        bool arm();
        void disarm();
        bool isarmed_;

        const config_m& get_config() const { return cfg_; };
        void set_config(const config_mot& config_load);
        Motor_mode get_mode() const { return Mode; };
        Motor_Error get_error() { return Error_; };
        bool get_nfault() {return nfault_read_pending;};
        bool read_nfault();
        void current_temperature_limit(float scale);


        float duty_steppoint = 0.0f;
        float true_torque_limit;

        bool init();
        

    private:
        config_mot cfg_;
        Motor_Error Error_ = ERROR_NONE;

        float v_int = 0.0f;
        float v_feedforward = 0.0f;
        float i_setpoint = 0.0f;
        float i_target = 0.0f;
        float duty_target = 0.0f;

        void current_inner_loop(float I_LOAD, float V_BUS);
        bool error_check();

        bool nfault_read_pending;

        bool drv_spi(uint16_t tx, uint16_t *rx);
        bool drv_read(uint8_t address, uint8_t *data, uint8_t *status);
        bool drv_write(uint8_t address, uint8_t *data);
        
        uint8_t motor_status = 0;
        uint8_t STATUS1_vals = 0;
        uint8_t STATUS2_vals = 0;

};