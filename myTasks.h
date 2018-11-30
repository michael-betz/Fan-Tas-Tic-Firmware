/*
 * tasks.h
 *
 *  Created on: Dec 22, 2015
 *      Author: michael
 */

#ifndef FAN_TAS_TIC_CONTROLLER_MYTASKS_H_
#define FAN_TAS_TIC_CONTROLLER_MYTASKS_H_
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//*****************************************************************************
// Defines
//*****************************************************************************
//#define CMD_PARSER_BUF_LEN (N_LEDS_MAX*3+28)   //Need 3072 for LED data blob of 1 channel
#define CMD_PARSER_BUF_LEN 128                   //Most commands fit
#define CUSTOM_I2C_BUF_LEN 64

//*****************************************************************************
// Custom types
//*****************************************************************************

//*****************************************************************************
// Global vars
//*****************************************************************************
extern TaskHandle_t hUSBCommandParser;
extern TaskHandle_t hCustomI2cTask;        //Task to notify once custom i2c command is done
extern SemaphoreHandle_t g_SemaCustomI2C;   //To ensure the custom I2C is done before next one starts
extern uint8_t g_reportSwitchEvents;        //Flag: Should Switch events be reported on the serial port?
extern uint8_t g_errorBuffer[8];            //For reporting 'ER:1234\n' style errors over USB

#define REPORT_ERROR(errStr) {memcpy(g_errorBuffer,errStr,8); ts_usbSend(g_errorBuffer,8);}

//*****************************************************************************
// Function / Task declaations
//*****************************************************************************
void taskDemoLED(void *pvParameters);
void taskDemoSerial(void *pvParameters);
void taskUsbCommandParser(void *pvParameters);
void taskI2CCustomReporter(void *pvParameters);
int Cmd_help(int argc, char *argv[]);
int Cmd_IDN(int argc, char *argv[]);
int Cmd_IL(int argc, char *argv[]);
int Cmd_OL(int argc, char *argv[]);
int Cmd_IR(int argc, char *argv[]);
int Cmd_SW(int argc, char *argv[]);
int Cmd_SWE(int argc, char *argv[]);
int Cmd_DEB(int argc, char *argv[]);
int Cmd_SOE(int argc, char *argv[]);
int Cmd_OUT(int argc, char *argv[]);
int Cmd_RUL(int argc, char *argv[]);
int Cmd_RULE(int argc, char *argv[]);
int Cmd_LEC(int argc, char *argv[]);
int Cmd_LED(int argc, char *argv[]);
int Cmd_I2C(int argc, char *argv[]);
void usbReporter(void *pvParameters);
void ts_usbSend(uint8_t *data, uint16_t len);

#endif /* FAN_TAS_TIC_CONTROLLER_MYTASKS_H_ */
