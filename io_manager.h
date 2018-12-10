// Anything related to processing the read input states
// like debouncing and firing solenoids

#ifndef IO_MANAGER_H_
#define IO_MANAGER_H_
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Update PCFs every 1 ms
#define DEBOUNCER_READ_PERIOD 1
// Char buffer size for reporting `input changed events`
#define REPORT_SWITCH_BUF_SIZE 90
// Max. number of output channels
#define OUT_WRITER_LIST_LEN 64
// How many uint32_t values to express all the input states
#define N_LONGS sizeof(t_switchState)/sizeof(uint32_t)
// How many uint8_t  values to express all the input states
#define N_CHARS sizeof(t_switchState)/sizeof(uint8_t)

// Holds the target hardware a hwIndex is referring to (Switch matrix, I2C port extender, HW. PWM channel)
typedef enum {
    C_I2C0, C_I2C1, C_I2C2, C_I2C3,
    C_FAST_PWM,
    C_SWITCH_MATRIX,
    C_INVALID
} t_channel;

// Holds all information of a decoded hwIndex
// For HW_INDEX_HWPWM only pinIndex is valid
typedef struct {
    t_channel channel;
    uint8_t byteIndex;  // Refers to the g_SwitchOutBuffer.charValues array;
    uint8_t pinIndex;   // which bit is relevant [0 - 7]
    uint8_t i2c_addr; // Right shifted 7 bit I2C addr
} t_hw_index;

// State of a pulsed solenoid driver output pin
typedef struct {
    int16_t tPulse;                 // Countdown counter, How long does the `high` pulse last [ms], -1 = invalid rule
    uint16_t lowPWM;                // PWM value after tPulse      (max. resolution is defined by N_BIT_PWM)
} t_BitModifyRules;

#include "i2c_inout.h"

// Holds the state of all input pins (all I2C extenders + Switch matrix)
typedef struct {
    uint8_t matrixData[8];
    uint8_t i2cReadData[4][PCF_MAX_PER_CHANNEL];
} t_switchState;

// Allows the input state to be read as bytes or 32 bit words for faster processing
typedef union {
    t_switchState switchState;
    uint32_t longValues[N_LONGS]; //Should be 10 long
    uint8_t charValues[N_CHARS];  //Should be 40 byte
} t_switchStateConverter;

// state of an PCF8574 IO extender configured as output
// also can keep state of a FAST_PWM output when channel == C_FAST_PWM
//   then pcf == NULL
typedef struct {
    t_channel channel;
    t_pcf_state *pcf;
    t_BitModifyRules bitRules[8];   // Hold the pulse state of each output pin
} t_PCLOutputByte;

//------------------------
// Global vars
//------------------------
extern QueueHandle_t g_i2c_queue;
extern TaskHandle_t hPcfInReader;
extern t_switchStateConverter g_SwitchStateSampled;
extern t_switchStateConverter g_SwitchStateDebounced;
extern t_switchStateConverter g_SwitchStateToggled;
extern t_switchStateConverter g_SwitchStateNoDebounce;
extern bool g_reDiscover;

//------------------------
// Global functs
//------------------------
// Set the power level and pulse settings of an output pin
//    tPulse    = duration of the pulse [ms]
//    highPower = PWM value during the pulse
//    lowPower  = PWM value after  the pulse
void setPCFOutput(t_hw_index *pin, int16_t tPulse, uint16_t highPower, uint16_t lowPower);

// Print active entries of out_writer_list to UART
void print_out_writer_list();

// Decode a hwIndex and fill the t_hw_index structure with details
// asInput: is this supposed to be an input (1) or output (0)
t_hw_index decodeHwIndex(uint16_t hwIndex, bool asInput);

// Orchestrates the periodic reading and writing of PCF chips over I2C
void task_pcf_io(void *pvParameters);

#endif
