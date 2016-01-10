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
#define N_LEDS_MAX 512
//#define CMD_PARSER_BUF_LEN (N_LEDS_MAX*3+28)   //Need 3072 for LED data blob of 1 channel
#define CMD_PARSER_BUF_LEN 128                   //Most commands fit
#define OUT_WRITER_LIST_LEN 32
#define CUSTOM_I2C_BUF_LEN 64

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

typedef struct {				    // State of a pulsed solenoid driver output pin
    int16_t tPulse;                 // Countdown counter, How long does the `high` pulse last [ms], -1 = invalid rule
    uint8_t lowPWM;                 // PWM value after tPulse	   (max. resolution is defined by N_BIT_PWM)
} t_BitModifyRules;

typedef struct {                    // State of all outputs on a PCF8574 IO extender
    int8_t i2cChannel;
    uint8_t i2cAddress;
    uint8_t bcmBuffer[N_BIT_PWM];   // To do binary code modulation, these 4 bytes will be written in sequence to the PCF
    t_BitModifyRules bitRules[8];   // Hold the state of each output pin
} t_PCLOutputByte;

typedef struct {
    uint32_t baseAdr;
    uint8_t *currentByte;
    uint32_t nBytesLeft;
    bool doFirstNibbel;
    SemaphoreHandle_t semaToReleaseWhenFinished;
    uint8_t intNo;      //Hardware interrupt number
}t_spiTransferState;

//*****************************************************************************
// Global vars
//*****************************************************************************
extern TaskHandle_t hUSBCommandParser;
extern TaskHandle_t g_customI2cTask;        //Task to notify once custom i2c command is done
extern SemaphoreHandle_t g_MutexCustomI2C;  //To ensure the custom I2C is done before next one starts
extern const uint16_t g_ssi_lut[16];
extern uint8_t g_spiBuffer[3][N_LEDS_MAX*3];//3 channels * 3 colors --> 9.2 kByte
extern uint32_t g_LEDnBytesToCopy;
extern int8_t g_LEDChannel;

//*****************************************************************************
// Function / Task declaations
//*****************************************************************************
void taskDemoLED(void *pvParameters);
void taskDemoSerial(void *pvParameters);
void taskUsbCommandParser(void *pvParameters);
void taskPCLOutWriter(void *pvParameters);
void taskI2CCustomReporter(void *pvParameters);
extern int Cmd_help(int argc, char *argv[]);
extern int Cmd_IDN(int argc, char *argv[]);
extern int Cmd_SW(int argc, char *argv[]);
extern int Cmd_OUT(int argc, char *argv[]);
extern int Cmd_RUL(int argc, char *argv[]);
extern int Cmd_RULD(int argc, char *argv[]);
extern int Cmd_RULE(int argc, char *argv[]);
extern int Cmd_LED(int argc, char *argv[]);
extern int Cmd_I2C(int argc, char *argv[]);
void usbReporter(void *pvParameters);
void ts_usbSend(uint8_t *data, uint16_t len);
t_outputBit decodeHwIndex(uint16_t hwIndex);
void setPclOutput(t_outputBit outLocation, int16_t tPulse, uint8_t highPower, uint8_t lowPower);
void spiSend( uint8_t channel, uint32_t nBytes );
void spiSetup();
void spiISR( uint8_t channel );

#endif /* FAN_TAS_TIC_CONTROLLER_MYTASKS_H_ */
