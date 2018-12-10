#ifndef QUICK_RULES_H
#define QUICK_RULES_H
#include "io_manager.h"

// How many quick-fire rules can be defined
#define MAX_QUICK_RULES 64
// Flags for quick-fire rules
#define QRF_TRIG_EDGE_POS 0
#define QRF_ENABLED       3
#define QRF_STATE_TRIG    4

// Holds the state of a quick-fire rule
typedef struct {
    t_hw_index inputSwitchId;      // hwIndex specifying the input switch
    uint8_t triggerFlags;           // ([0] enable/disable, [1] trigger on positive/negative edge, [2] disable ouput on release, [4] check toggle)
    int16_t triggerHoldOffTime;     // post trigger hold-off time [ms]
    int16_t triggerHoldOffCounter;  // Counts down each tick
    t_hw_index outputDriverId;     // hwIndex specifying the driver output ID
    int16_t tPulse;                 // pulse duration [ms]
    uint16_t pwmHigh;               // power level during tPulse [0 - 15] for I2C, [0-MAX_PWM] for HWpwm
    uint16_t pwmLow;                // power level after  tPulse [0 - 15] for I2C, [0-MAX_PWM] for HWpwm
} t_quickRule;

void setupQuickRule(uint8_t id, t_hw_index inputSwitchId,
        t_hw_index outputDriverId, uint16_t triggerHoldOffTime,
        uint16_t tPulse, uint16_t pwmHigh, uint16_t pwmLow, bool trigPosEdge );
void enableQuickRule(uint8_t id);
void disableQuickRule(uint8_t id);
void processQuickRules();

#endif
