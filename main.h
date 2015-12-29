/*
 * main.h
 *
 *  Created on: Mar 28, 2015
 *      Author: akobyljanec
 */

#ifndef FAN_TAS_TIC_CONTROLLER_MAIN_H_
#define FAN_TAS_TIC_CONTROLLER_MAIN_H_

#define VERSION_INFO "\nFan-Tas-Tic pinball controller\nFirmware V0.0, M. Betz 01/2016\n"

// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

void startTimer();
uint32_t stopTimer();

#endif /* FAN_TAS_TIC_CONTROLLER_MAIN_H_ */
