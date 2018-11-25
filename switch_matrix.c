// Scan the 8 x 8 Switch Matrix through a serial shift register

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/rom.h"
#include "driverlib/gpio.h"

#include "i2c_in.h"
#include "switch_matrix.h"

uint8_t getSMrow() {
    uint8_t temp = 0;
    // Read the state of the row of the switch matrix.
    // The inputs are distributed across 4 Ports :p
    //-------------------------------
    //7 6 5 4 3 2 1 0   Bit position
    //-------------------------------
    //        0 1 2     PORTE
    //4 3               PORTC
    //6 5               PORTD
    //      7           PORTF
    // Sample the state of each sense line in temp
    HWREGBITB( &temp, 0 ) = ROM_GPIOPinRead( GPIO_PORTE_BASE, GPIO_PIN_3 ) > 0;
    HWREGBITB( &temp, 1 ) = ROM_GPIOPinRead( GPIO_PORTE_BASE, GPIO_PIN_2 ) > 0;
    HWREGBITB( &temp, 2 ) = ROM_GPIOPinRead( GPIO_PORTE_BASE, GPIO_PIN_1 ) > 0;

    HWREGBITB( &temp, 3 ) = ROM_GPIOPinRead( GPIO_PORTC_BASE, GPIO_PIN_6 ) > 0;
    HWREGBITB( &temp, 4 ) = ROM_GPIOPinRead( GPIO_PORTC_BASE, GPIO_PIN_7 ) > 0;

    HWREGBITB( &temp, 5 ) = ROM_GPIOPinRead( GPIO_PORTD_BASE, GPIO_PIN_6 ) > 0;
    HWREGBITB( &temp, 6 ) = ROM_GPIOPinRead( GPIO_PORTD_BASE, GPIO_PIN_7 ) > 0;

    HWREGBITB( &temp, 7 ) = ROM_GPIOPinRead( GPIO_PORTF_BASE, GPIO_PIN_4 ) > 0;
    // A closed switch means it pulls the sense line to GND (active low)
    // just like with the the PCF IO expanders.
    return temp;
}

void resetSMrow() {
    uint8_t i;
//    Reset Shift register to 0x00
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    for (i = 0; i <= 7; i++) {            //Keep dat low and pulse clock 8 times
        ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_CLK);
        ROM_SysCtlDelay( SM_COL_DELAY_CNT);
        ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
        ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    }
    //Switch on first bit. Shift register data input = high
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_DAT);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    //Shift register clock = pos. edge
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_CLK | SM_COL_DAT);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    //Latch clock = pos. edge
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_DAT);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
}

void advanceSMrow() {
    //Shift register clock = pos. edge
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_CLK);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    //Shift register data input = low
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    //Latch clock = pos. edge
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_DAT);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
}

void readSwitchMatrix() {
    uint8_t nRow;
    resetSMrow();    //First col is active
    for (nRow = 0; nRow <= 7; nRow++) {
        ROM_SysCtlDelay(5 * SM_COL_DELAY_CNT);
        g_SwitchStateSampled.switchState.matrixData[nRow] = getSMrow();
        advanceSMrow();
    }
}
