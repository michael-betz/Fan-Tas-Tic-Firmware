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
#define N_BIT_PWM 3
//#define CMD_PARSER_BUF_LEN (N_LEDS_MAX*3+28)   //Need 3072 for LED data blob of 1 channel
#define CMD_PARSER_BUF_LEN 128                   //Most commands fit
#define OUT_WRITER_LIST_LEN 32
#define CUSTOM_I2C_BUF_LEN 64

//*****************************************************************************
// Custom types
//*****************************************************************************

//*****************************************************************************
// Global vars
//*****************************************************************************
extern TaskHandle_t hUSBCommandParser;
extern TaskHandle_t g_customI2cTask;        //Task to notify once custom i2c command is done
extern SemaphoreHandle_t g_MutexCustomI2C;  //To ensure the custom I2C is done before next one starts
extern uint8_t g_reportSwitchEvents;        //Flag: Should Switch events be reported on the serial port?

//*****************************************************************************
// Function / Task declaations
//*****************************************************************************
void taskDemoLED(void *pvParameters);
void taskDemoSerial(void *pvParameters);
void taskUsbCommandParser(void *pvParameters);
void taskI2CCustomReporter(void *pvParameters);
extern int Cmd_help(int argc, char *argv[]);
extern int Cmd_IDN(int argc, char *argv[]);
extern int Cmd_SW(int argc, char *argv[]);
extern int Cmd_SWE(int argc, char *argv[]);
extern int Cmd_OUT(int argc, char *argv[]);
extern int Cmd_RUL(int argc, char *argv[]);
extern int Cmd_RULE(int argc, char *argv[]);
extern int Cmd_LEC(int argc, char *argv[]);
extern int Cmd_LED(int argc, char *argv[]);
extern int Cmd_I2C(int argc, char *argv[]);
void usbReporter(void *pvParameters);
void ts_usbSend(uint8_t *data, uint16_t len);

#endif /* FAN_TAS_TIC_CONTROLLER_MYTASKS_H_ */
