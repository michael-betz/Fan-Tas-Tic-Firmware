/*
 * tasks.h
 *
 *  Created on: Dec 22, 2015
 *      Author: michael
 */

#ifndef FAN_TAS_TIC_CONTROLLER_MYTASKS_H_
#define FAN_TAS_TIC_CONTROLLER_MYTASKS_H_

//*****************************************************************************
// Defines
//*****************************************************************************
#define N_BIT_PWM 4
#define CMD_PARSER_BUF_LEN 128
#define OUT_WRITER_LIST_LEN 32

//*****************************************************************************
// Custom types
//*****************************************************************************
typedef enum {
    HW_INDEX_INVALID, HW_INDEX_SWM, HW_INDEX_I2C
} t_hwIndexType;

typedef struct {
    t_hwIndexType hwIndexType;
    uint8_t byteIndex;	//Refers to the g_SwitchOutBuffer.charValues array;
    int8_t pinIndex;
    int8_t i2cChannel;
    uint8_t i2cAddress;
} t_outputBit;

typedef struct {				// State of a pulsed solenoid driver output pin
    int16_t tPulse; // Countdown counter, How long does the `high` pulse last [ms], -1 = invalid rule
    uint8_t lowPWM; // PWM value after tPulse	   (max. resolution is defined by N_BIT_PWM)
} t_BitModifyRules;

typedef struct {
    int8_t i2cChannel;
    uint8_t i2cAddress;
    uint8_t bcmBuffer[N_BIT_PWM];
    t_BitModifyRules bitRules[8];
} t_PCLOutputByte;

//*****************************************************************************
// Global vars
//*****************************************************************************
extern TaskHandle_t hUSBCommandParser;

//*****************************************************************************
// Function / Task declaations
//*****************************************************************************
void taskDemoLED(void *pvParameters);
void taskDemoSerial(void *pvParameters);
void taskUsbCommandParser(void *pvParameters);
void taskPCLOutWriter(void *pvParameters);
extern int Cmd_help(int argc, char *argv[]);
extern int Cmd_IDN(int argc, char *argv[]);
extern int Cmd_SW(int argc, char *argv[]);
extern int Cmd_OUT(int argc, char *argv[]);
extern int Cmd_RUL(int argc, char *argv[]);
extern int Cmd_RULD(int argc, char *argv[]);
extern int Cmd_RULE(int argc, char *argv[]);
extern int Cmd_LED(int argc, char *argv[]);
void usbReporter(void *pvParameters);
void ts_usbSend(uint8_t *data, uint16_t len);

t_outputBit decodeHwIndex(uint16_t hwIndex);
void setPclOutput(t_outputBit outLocation, int16_t tPulse, uint8_t highPower,
        uint8_t lowPower);

#endif /* FAN_TAS_TIC_CONTROLLER_MYTASKS_H_ */
