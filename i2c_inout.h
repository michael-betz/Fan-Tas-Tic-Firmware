#ifndef I2C_INOUT_H_
#define I2C_INOUT_H_
#include <stdint.h>

//*****************************************************************************
// Defines
//*****************************************************************************
#define N_BIT_PWM 3
// * The arrays with the I2C readback values and states of each input *
// How many PCL chips per channel (careful must be a multiple of 4)
#define PCF_MAX_PER_CHANNEL 8
// Lowest possible I2C Address of a PCF8574 IO extender (all address pins low)
#define PCF_LOWEST_ADDR 0x20

//--------------
// Custom types
//--------------
typedef struct {
    uint8_t flags;
    uint8_t i2c_addr;
    uint8_t last_value;
    uint8_t last_err_mcs;
    unsigned err_cnt;
    // -------------------
    //  Only for outputs:
    // -------------------
    // these 4 bytes will be written in sequence to the PCF
    // at 2^N time intervals (binary code modulation)
    uint8_t bcm_buffer[N_BIT_PWM];
} t_pcfState;

// bit masks for t_pcfState->flags
#define FPCF_RENABLED (1<<0)    // 1 = Read PCF
#define FPCF_WENABLED (1<<1)    // 1 = Write PCF

typedef struct {
    uint8_t flags;
    uint8_t i2c_addr;
    uint8_t nWrite;
    uint8_t nRead;
    uint8_t *readBuff;
    uint8_t *writeBuff;
} t_i2cCustom;

typedef enum{
    I2C_START,
    I2C_PCF,
    I2C_CUSTOM,
    I2C_IDLE
} t_i2cState;

typedef struct {
    uint32_t base_addr;
    uint8_t int_addr;
    t_i2cState i2c_state;
    uint8_t currentPcf;
    t_pcfState pcf_state[PCF_MAX_PER_CHANNEL];
} t_i2cChannelState;

// Holds the target hardware a hwIndex is referring to (Switch matrix, I2C port extender, HW. PWM channel)
typedef enum {
    HW_INDEX_INVALID, HW_INDEX_SWM, HW_INDEX_I2C, HW_INDEX_HWPWM
} t_hwIndexType;

// Holds all information of a decoded hwIndex
typedef struct {
    t_hwIndexType hwIndexType;
    uint8_t byteIndex;              // Refers to the g_SwitchOutBuffer.charValues array;
    int8_t pinIndex;                // which bit is relevant [0 - 7]
    int8_t i2cChannel;              // [0 - 3]
    uint8_t i2cAddress;             // Right shifted 7 bit I2C addr
} t_outputBit;

//--------------
// Functions
//--------------
// Inits I2C IOs, Hardware and data structures
void init_i2c_system();
// Shall be called every 1 ms to keep PCF transactions going
void trigger_i2c_cycle();
// Print table of state and error counts to UART
void print_pcf_state();
// i2c interrupt service routine
void i2c_isr(t_i2cChannelState *state);
// these just call i2c_isr with the right arg
void i2CIntHandler0(void);
void i2CIntHandler1(void);
void i2CIntHandler2(void);
void i2CIntHandler3(void);

#endif /* I2C_INOUT_H_ */
