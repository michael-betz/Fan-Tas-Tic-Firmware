/*
 * main.h
 *
 *  Created on: Mar 28, 2015
 *      Author: M. Betz
 */

#ifndef FAN_TAS_TIC_CONTROLLER_MAIN_H_
#define FAN_TAS_TIC_CONTROLLER_MAIN_H_

#define VERSION_IDN  "ID:MB:V0.2\n"
#define VERSION_IDN_LEN 11
#define VERSION_INFO "\nFan-Tas-Tic pinball controller\nM. Betz, 11/2018\nGit: " GIT_VERSION "\n"

// System clock rate, 80 MHz
#define SYSTEM_CLOCK    80000000U

// Set PWM frequency to 20 kHz, which gives a maximum PWM value for 100 % of 4000
#define MAX_PWM (SYSTEM_CLOCK/20000)

// Solenoid 24 V power interlock relay
#define DISABLE_SOLENOIDS() ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0)
#define ENABLE_SOLENOIDS()  ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 1)

void configureTimer();
void startTimer();
uint32_t stopTimer();
uint32_t getTimer();
void setPwm( uint8_t channel, uint16_t pwmValue );
void ledOut( uint8_t ledVal );

#endif /* FAN_TAS_TIC_CONTROLLER_MAIN_H_ */
