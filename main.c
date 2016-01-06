/* Fan-Tas-Tic Pinball controller
 * M.Betz,   01/2016
 *
 * main.c
 *
 * Based on a demonstration project of FreeRTOS 8.2 on the Tiva Launchpad by Andy Kobyljanec
 * TivaWare driverlib sourcecode is included.
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"

#include "sensorlib/i2cm_drv.h"
#include "utils/uartstdio.h"

// TivaWare includes
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

//USB stuff
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"

//My stuff
#include "main.h"
#include "myTasks.h"
#include "i2cHandlerTask.h"

TaskHandle_t hUSBCommandParser = NULL;

//-------------------------------------------------------------------------
// Helper functions to Setup a hardware counter for simple cycle counting
// To see how fast crtical parts of the code can run
//-------------------------------------------------------------------------
void configureTimer() {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); 	    // Enable Timer 1 Clock
    ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT_UP); // Configure Timer Operation as one shot up counting
}
void startTimer() {
    ROM_TimerEnable(TIMER1_BASE, TIMER_A); // Start Timer 1A
}
uint32_t stopTimer() {
//  Returns the number of cycles since startTimer()
    uint32_t timerValue;
    ROM_TimerDisable(TIMER1_BASE, TIMER_A); // Stop Timer 1A
    timerValue = ROM_TimerValueGet(TIMER1_BASE, TIMER_A);
    ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, 0xFFFFFFFF);
    HWREG(TIMER1_BASE + TIMER_O_TAV) = 0;
    return (timerValue);
}

void ledOut( uint8_t ledVal ){
    // 0 = off, 1 = blue, 2 = green, 3 = blue & green
    GPIOPinWrite( GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2, ledVal<<2 );
}

//-------------------------------------------------------------------------
// Hardware initialization
//-------------------------------------------------------------------------
void initGpio(){
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Unlock NMI to use PD7 as normal GPIO
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
    // Configure GPIO Pins for UART mode (usb debug terminal).
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Configure DEVICE USB pins
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);
    // Enable the GPIO pins for the LED (PF2 & PF3).
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2);
}

//-------------------------------------------------------------------------
// Main function
//-------------------------------------------------------------------------
int main(void) {
    // Set the clocking to run at 80 MHz from the PLL.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    // Init GPIO pins
    initGpio();
    // Set up the UART which is connected to the virtual debugging COM port
    UARTStdioConfig(0, 115200, SYSTEM_CLOCK);
    UARTprintf("\n\n\n\n"
            "**************************************************\n"
            " Hi, here's the brain of Fan-Tas-Tic Pinball V0.0 \n"
            "**************************************************\n\n");

    configureTimer();   //Init HW timer for measuring processor cycles (%timeit)
    initMyI2C();        //Init the 4 I2C hardware channels
    spiSetup();         //Init 3 SPI channels for setting ws2811 LEDs

    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPULazyStackingEnable();

    //-------------------------------------------------------------------------
    // Init USB virtual serial port for communication to the host PC runing MPF
    //-------------------------------------------------------------------------
    // Configure the required pins for USB operation.
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_USB0 );
    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);
    // Set the USB stack mode to Device mode with VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);
    // Pass our device information to the USB library and place the device
    // on the bus.
    USBDCDCInit(0, &g_sCDCDevice);

    //-------------------------------------------------------------------------
    // Startup the FreeRTOS scheduler
    //-------------------------------------------------------------------------
    // Create demo tasks
    xTaskCreate(taskDemoLED, (const portCHAR *)"LEDr", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Report result of custom I2C transaction to commandline
    xTaskCreate(taskI2CCustomReporter, (const portCHAR *)"I2CcusRep", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    // Create USB command parser task
    xTaskCreate(taskUsbCommandParser, (const portCHAR *)"Parser", 256, NULL, 1, &hUSBCommandParser);

    // Create I2C / Matrix debouncer
    xTaskCreate(taskDebouncer, (const portCHAR *)"Debouncer", 256, NULL, 1, NULL);

    // Dispatch I2C write commands to PCL GPIO extenders every 1 ms
    xTaskCreate(taskPCLOutWriter, (const portCHAR *)"PCLwriter", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();  // This should never return!
    return 0;
}

//ASSERT() Error function failed ASSERTS() from driverlib/debug.h are executed in this function
void __error__(char *pcFilename, uint32_t ui32Line) {
    UARTprintf("__error__( %s, %d )", pcFilename, ui32Line);
    while (1)
        ; // Place a breakpoint here to capture errors until logging routine is finished
}
