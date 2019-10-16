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

//*****************************************************************************
// Custom types
//*****************************************************************************

//*****************************************************************************
// Global vars
//*****************************************************************************
extern TaskHandle_t hUSBCommandParser;
extern bool g_reportSwitchEvents;     //Flag: Should Switch events be reported on the serial port?
extern uint8_t g_errorBuffer[8];      //For reporting 'ER:1234\n' style errors over USB

#define REPORT_ERROR(errStr) {memcpy(g_errorBuffer,errStr,8); ts_usbSend(g_errorBuffer,8);}

//*****************************************************************************
// Function / Task declarations
//*****************************************************************************
void taskDemoLED(void *pvParameters);
void taskDemoSerial(void *pvParameters);
void taskUsbCommandParser(void *pvParameters);
void taskI2CCustomReporter(void *pvParameters);
void usbReporter(void *pvParameters);
void ts_usbSend(uint8_t *data, uint16_t len);

// Takes a string, returns a buff.
// Make sure to free it after use.
uint8_t *hexToBuff(char *str, unsigned *nWritten);

// Takes a buffer, writes hex string to dest
// returns pointer to \0 of \0 terminated string.
char *buffToHex(uint8_t *buf, unsigned len, char *dest);

#endif /* FAN_TAS_TIC_CONTROLLER_MYTASKS_H_ */
