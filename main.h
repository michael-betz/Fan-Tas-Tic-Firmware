/*
 * main.h
 *
 *  Created on: Mar 28, 2015
 *      Author: akobyljanec
 */

#ifndef FAN_TAS_TIC_CONTROLLER_MAIN_H_
#define FAN_TAS_TIC_CONTROLLER_MAIN_H_

#define VERSION_IDN  "ID:MB:V0.1\n"
#define VERSION_IDN_LEN 11
#define VERSION_INFO "\nFan-Tas-Tic pinball controller\nFirmware V0.1, M. Betz 01/2016\n"

// System clock rate, 80 MHz
#define SYSTEM_CLOCK    80000000U

// Set PWM frequency to 20 kHz, which gives a maximum PWM value for 100 % of 4000
#define MAX_PWM (SYSTEM_CLOCK/20000)

void configureTimer();
void startTimer();
uint32_t stopTimer();
uint32_t getTimer();
void setPwm( uint8_t channel, uint16_t pwmValue );
void ledOut( uint8_t ledVal );

#endif /* FAN_TAS_TIC_CONTROLLER_MAIN_H_ */
