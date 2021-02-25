#ifndef I2C_INOUT_H_
#define I2C_INOUT_H_
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Defines
//*****************************************************************************
// How many bits of BCM for the i2c output channels, the more the slower
#define N_BIT_PWM 3
// How many PCL chips per channel (careful must be a multiple of 4)
#define PCF_MAX_PER_CHANNEL 8
// Lowest possible I2C Address of a PCF8574 IO extender (all address pins low)
#define PCF_LOWEST_ADDR 0x20
// Time window to count I2C errors in [ms]
#define PCF_ERR_CHECK_CYCLE 5000
// Max. number of errors in the time window
#define PCF_ERR_CNT_DISABLE  4995
// Read channels get disabled if more than `PCF_ERR_CNT_DISABLE` errors happen
// in the first `PCF_ERR_CHECK_CYCLE` miliseconds.
// For all write channels, if more then `PCF_ERR_CNT_DISABLE` errors happen in
// any `PCF_ERR_CHECK_CYCLE` milisecond long window. 24 V will be disabled and
// the system will shut down.

//--------------
// Custom types
//--------------
typedef struct {
    uint8_t flags;
    uint8_t i2c_addr;
    // Values are read into this address
    uint8_t *value_target;
    // uint8_t value;
    uint8_t last_mcs;
    unsigned err_cnt;
    // -------------------
    //  Only for outputs:
    // -------------------
    // these 4 bytes will be written in sequence to the PCF
    // at 2^N time intervals (binary code modulation)
    uint8_t bcm_buffer[N_BIT_PWM];
} t_pcf_state;

// bit masks for t_pcf_state->flags
#define FPCF_RENABLED (1<<0)    // 1 = Read PCF
#define FPCF_WENABLED (1<<1)    // 1 = Write PCF

// t_i2cCustom.flags:
#define I2CC_W_ADR_NACK 0
#define I2CC_W_DAT_NACK 1
#define I2CC_R_ADR_NACK 2
#define I2CC_R_DAT_NACK 3

typedef struct {
    uint8_t channel;
    uint8_t i2c_addr;
    uint8_t nWrite;
    uint8_t nRead;
    uint8_t *readBuff;
    uint8_t *writeBuff;
    uint8_t flags;
} t_i2cCustom;

typedef enum{
    I2C_START,  // start scanning
    I2C_PCF,    // scan in progress, notify when done
    I2C_CUSTOM, // notify immediately
    I2C_IDLE    // does nothing (isr shouldn't be called)
} t_i2cState;

typedef struct {
    uint32_t base_addr;
    uint8_t int_addr;
    t_i2cState i2c_state;
    uint8_t currentPcf;
    t_pcf_state pcf_state[PCF_MAX_PER_CHANNEL];
} t_i2cChannelState;

#include "io_manager.h"

//--------------
// Functions
//--------------
// Inits I2C IOs, Hardware and data structures
void init_i2c_system(bool isr_init);
// Shall be called every 1 ms to keep PCF transactions going
void trigger_i2c_cycle();
// Print table of state and error counts to UART
void print_pcf_state();
// Return pointer to pcf_state instance of this pin
t_pcf_state *get_pcf(t_hw_index *pin);
// Update a bcm buffer with a new pwm value
void set_bcm(uint8_t *bcmBuffer, uint8_t pin, uint8_t pwmValue);
// Return PWM value of certain pin from bcmbuffer
uint8_t get_bcm(uint8_t *bcmBuffer, uint8_t pin);
// carry out a custom i2c transaction and yield until done
void handle_i2c_custom();
// Send / receive byte over i2c and yield until done (isr & freeRTOS must be setup)
void i2c_tx_rx_n(uint32_t b, t_i2cCustom *i2c);
// Wait for all bits to be set in notification value (and clear them)
void wait_for_noti_bits(uint32_t bits);
// i2c interrupt service routine
void i2c_isr(unsigned channel);
// these just call i2c_isr with the right arg
void i2CIntHandler0(void);
void i2CIntHandler1(void);
void i2CIntHandler2(void);
void i2CIntHandler3(void);

#endif /* I2C_INOUT_H_ */
