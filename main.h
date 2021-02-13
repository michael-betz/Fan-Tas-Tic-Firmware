#ifndef FAN_TAS_TIC_CONTROLLER_MAIN_H_
#define FAN_TAS_TIC_CONTROLLER_MAIN_H_
#include <stdint.h>

#define VERSION_IDN  "ID:MB:V0.3\n"
#define VERSION_IDN_LEN 11
#ifndef GIT_VERSION
    #define GIT_VERSION "None"
#endif
#define VERSION_INFO "\nFan-Tas-Tic pinball controller\nM. Betz, 10/2019\nGit: " GIT_VERSION "\n"

// System clock rate, 80 MHz
#define SYSTEM_CLOCK    80000000U

// Set PWM frequency to 20 kHz, which gives a maximum PWM value for 100 % of 4000
#define MAX_PWM (SYSTEM_CLOCK / 20000)

// Solenoid 24 V power interlock relay
#define DISABLE_SOLENOIDS() ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0)
#define ENABLE_SOLENOIDS()  ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 1)

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

//---------------------
// Functions
//---------------------
void configureTimer();
void startTimer();
uint32_t stopTimer();
uint32_t getTimer();
void setPwm( uint8_t channel, uint16_t pwmValue );
void ledOut( uint8_t ledVal );

#endif /* FAN_TAS_TIC_CONTROLLER_MAIN_H_ */
