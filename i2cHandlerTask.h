/*
 * i2cHandlerTask.h
 *
 *  Created on: Dec 24, 2015
 *      Author: michael
 */

#ifndef I2CHANDLERTASK_H_
#define I2CHANDLERTASK_H_

// The arrays with the readback values and states
#define MAX_PCLS_PER_CHANNEL 8			//Howm many PCL chips per channel (careful must be a multiple of 4)
#define N_LONGS sizeof(t_switchState)/sizeof(uint32_t)
#define N_CHARS sizeof(t_switchState)/sizeof(uint8_t)
#define MAX_QUICK_RULES 64

//ToDO: FREERTOS will crash when REPORT_SWITCH_BUF_SIZE > 400, why? --> as during the context switch it copied the whole thing on the stack :p
#define REPORT_SWITCH_BUF_SIZE 256		//Char buffer size for reporting events

//[0] enable/disable, [1] trigger on positive/negative edge, [2] disable ouput on release, [3] invert output, [4] apply now)
#define QRF_TRIG_EDGE_POS 0
#define QRF_OFF_ON_RELASE 1
#define QRF_LEVEL_TRIG    2
#define QRF_ENABLED		  3
#define QRF_STATE_TRIG    4

typedef struct {
    uint8_t matrixData[8];
    uint8_t i2cReadData[4][MAX_PCLS_PER_CHANNEL];
} t_switchState;

typedef union {
    t_switchState switchState;
    uint32_t longValues[N_LONGS]; //Should be 10 long
    uint8_t charValues[N_CHARS]; //Should be 40 byte
} t_switchStateConverter;

// Configure QuickRule:
//  * quickRuleId (0-64)
// 	* input switch ID
//  * trigger type flags ([0] enable/disable, [1] trigger on positive/negative edge, [2] disable ouput on release, [4] check Toggle)
//	* post trigger hold-off time [ms]
//  * driver output ID
//  * pulse duration 0 - 100 [ms]
//	* pulse pwm [only for output ID 0-3 which are the pwm channels]
//	* hold pwm  [only for output ID 0-3 which are the pwm channels]
typedef struct {
    t_outputBit inputSwitchId;
    uint8_t triggerFlags;//([0] enable/disable, [1] trigger on positive/negative edge, [2] disable ouput on release, [4] check toggle)
    uint16_t triggerHoldOffTime;
    uint16_t triggerHoldOffCounter;
    t_outputBit outputDriverId;
    uint16_t tPulse;
    uint8_t pwmHigh;
    uint8_t pwmLow;
} t_quickRule;

// The I2C master driver instances
extern tI2CMInstance g_sI2CInst[4];

// Buffer bits of current I2C GPIO input state
extern t_switchStateConverter g_SwitchStateSampled;	//Read values of last I2C scan
extern t_switchStateConverter g_SwitchStateDebounced;//Debounced values (the same after 4 reads)
extern t_switchStateConverter g_SwitchStateToggled;		//Bits which changed
extern t_quickRule g_QuickRuleList[MAX_QUICK_RULES];	//List of Quickrules

// The functions
void initMyI2C();
void ts_i2cTransfer(uint8_t channel, uint_fast8_t ui8Addr,
        const uint8_t *pui8WriteData, uint_fast16_t ui16WriteCount,
        uint8_t *pui8ReadData, uint_fast16_t ui16ReadCount,
        tSensorCallback *pfnCallback, void *pvCallbackData);
void i2cStartPCFL8574refresh();
void taskDebouncer(void *pvParameters);

void setupQuickRule(uint8_t id, t_outputBit inputSwitchId,
        t_outputBit outputDriverId, uint16_t triggerHoldOffTime,
        uint16_t tPulse, uint8_t pwmHigh, uint8_t pwmLow, bool trigPosEdge,
        bool outOffOnRelease, bool checkToggle);
void enableQuickRule(uint8_t id);
void disableQuickRule(uint8_t id);

#endif /* I2CHANDLERTASK_H_ */
