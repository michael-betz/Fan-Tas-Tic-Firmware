/*
 * i2cHandlerTask.h
 *
 *  Created on: Dec 24, 2015
 *      Author: michael
 */

#ifndef I2CHANDLERTASK_H_
#define I2CHANDLERTASK_H_

//*****************************************************************************
// Defines
//*****************************************************************************
// * The arrays with the I2C readback values and states of each input *
// How many PCL chips per channel (careful must be a multiple of 4)
#define PCF_MAX_PER_CHANNEL 8
// Lowest possible I2C Address of a PCF8574 IO extender (all address pins low)
#define PCF_LOWEST_ADDR 0x20
// Read all PCF inputs and process quickRules every xxx ms
#define DEBOUNCER_READ_PERIOD 3
// How many uint32_t values to express all the input states
#define N_LONGS sizeof(t_switchState)/sizeof(uint32_t)
// How many uint8_t  values to express all the input states
#define N_CHARS sizeof(t_switchState)/sizeof(uint8_t)
// How many quick-fire rules can be defined
#define MAX_QUICK_RULES 64
// Char buffer size for reporting `input changed events`
#define REPORT_SWITCH_BUF_SIZE 90

// * Flags for quick-fire rules *
#define QRF_TRIG_EDGE_POS 0
#define QRF_ENABLED		  3
#define QRF_STATE_TRIG    4

// Define Port pins to the Switch Matrix column driver (shift register IC)
#define SM_COL_DAT GPIO_PIN_0
#define SM_COL_CLK GPIO_PIN_1

//resetSMcol() + 8 * advanceSMcol() should take ~ 500 us
// At 80 MHz We should spent 40000 instructions
// We got 320 + 160 + 60 = 530 useful instr.
// We got 6 + 16 + 32 = 54 delay calls which do n*3 instructions
// So each delay call should do ... delay units:
// (40000 - 530)/54/3 = 244
// 200 --> Switch matrix has ~ 8 us to settle
#define SM_COL_DELAY_CNT 100


//*****************************************************************************
// Custom types
//*****************************************************************************
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

// Complete state of all 8 outputs on a PCF8574 IO extender. HW outputs are indicated by i2cChannel == 100
typedef struct {
    int8_t i2cChannel;
    uint8_t i2cAddress;
    uint8_t bcmBuffer[N_BIT_PWM];   // To do binary code modulation, these 4 bytes will be written in sequence to the PCF
    t_BitModifyRules bitRules[8];   // Hold the state of each output pin
} t_PCLOutputByte;

//*****************************************************************************
// Global variables
//*****************************************************************************
// The I2C master driver instances (TI driver)
extern tI2CMInstance g_sI2CInst[4];

// Buffer bits of current I2C GPIO input state
extern t_switchStateConverter g_SwitchStateSampled;	    //Read values of last I2C scan
extern t_switchStateConverter g_SwitchStateDebounced;   //Debounced values (the same after 4 reads)
extern t_switchStateConverter g_SwitchStateToggled;		//Bits which changed
extern t_switchStateConverter g_SwitchStateNoDebounce;  //Debouncing-OFF flags
extern t_quickRule g_QuickRuleList[MAX_QUICK_RULES];	//List of Quickrules

//*****************************************************************************
// Global functions
//*****************************************************************************
void initMyI2C();
void ts_i2cTransfer(uint8_t channel, uint_fast8_t ui8Addr,
        const uint8_t *pui8WriteData, uint_fast16_t ui16WriteCount,
        uint8_t *pui8ReadData, uint_fast16_t ui16ReadCount,
        tSensorCallback *pfnCallback, void *pvCallbackData);
void i2cStartPCFL8574refresh();
void taskDebouncer(void *pvParameters);

void setupQuickRule(uint8_t id, t_outputBit inputSwitchId,
        t_outputBit outputDriverId, uint16_t triggerHoldOffTime,
        uint16_t tPulse, uint16_t pwmHigh, uint16_t pwmLow, bool trigPosEdge );
void enableQuickRule(uint8_t id);
void disableQuickRule(uint8_t id);

t_outputBit decodeHwIndex( uint16_t hwIndex, uint8_t asInput );
void setPCFOutput(t_outputBit outLocation, int16_t tPulse, uint16_t highPower, uint16_t lowPower);
void taskPCFOutWriter(void *pvParameters);

#endif /* I2CHANDLERTASK_H_ */
