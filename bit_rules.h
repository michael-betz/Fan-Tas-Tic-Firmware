// Anything related to processing the read input states
// like debouncing and firing solenoids

#ifndef BIT_RULES_H_
#define BIT_RULES_H_
#include "i2c_inout.h"

// Char buffer size for reporting `input changed events`
#define REPORT_SWITCH_BUF_SIZE 90
// How many quick-fire rules can be defined
#define MAX_QUICK_RULES 64
// Flags for quick-fire rules
#define QRF_TRIG_EDGE_POS 0
#define QRF_ENABLED       3
#define QRF_STATE_TRIG    4
// How many uint32_t values to express all the input states
#define N_LONGS sizeof(t_switchState)/sizeof(uint32_t)
// How many uint8_t  values to express all the input states
#define N_CHARS sizeof(t_switchState)/sizeof(uint8_t)

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

// Holds the state of a quick-fire rule
typedef struct {
    t_outputBit inputSwitchId;      // hwIndex specifying the input switch
    uint8_t triggerFlags;           // ([0] enable/disable, [1] trigger on positive/negative edge, [2] disable ouput on release, [4] check toggle)
    int16_t triggerHoldOffTime;     // post trigger hold-off time [ms]
    int16_t triggerHoldOffCounter;  // Counts down each tick
    t_outputBit outputDriverId;     // hwIndex specifying the driver output ID
    int16_t tPulse;                 // pulse duration [ms]
    uint16_t pwmHigh;               // power level during tPulse [0 - 15] for I2C, [0-MAX_PWM] for HWpwm
    uint16_t pwmLow;                // power level after  tPulse [0 - 15] for I2C, [0-MAX_PWM] for HWpwm
} t_quickRule;

// State of a pulsed solenoid driver output pin
typedef struct {
    int16_t tPulse;                 // Countdown counter, How long does the `high` pulse last [ms], -1 = invalid rule
    uint16_t lowPWM;                // PWM value after tPulse      (max. resolution is defined by N_BIT_PWM)
} t_BitModifyRules;

void setupQuickRule(uint8_t id, t_outputBit inputSwitchId,
        t_outputBit outputDriverId, uint16_t triggerHoldOffTime,
        uint16_t tPulse, uint16_t pwmHigh, uint16_t pwmLow, bool trigPosEdge );
void enableQuickRule(uint8_t id);
void disableQuickRule(uint8_t id);

extern void task_pcf_io(void *pvParameters);

#endif
