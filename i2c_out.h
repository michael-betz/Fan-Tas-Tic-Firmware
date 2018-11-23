#ifndef I2C_OUT_H_
#define I2C_OUT_H_
#include <stdint.h>

//------------------------
// Defines
//------------------------
#define N_BIT_PWM 3
#define OUT_WRITER_LIST_LEN 64                   //Max. number of output channels

//------------------------
// Custom types
//------------------------
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

//------------------------
// Global functions
//------------------------
t_outputBit decodeHwIndex( uint16_t hwIndex, uint8_t asInput );
void setPCFOutput(t_outputBit outLocation, int16_t tPulse, uint16_t highPower, uint16_t lowPower);
void taskPCFOutWriter(void *pvParameters);
void disableAllWriters(void);

#endif /* I2C_OUT_H_ */
