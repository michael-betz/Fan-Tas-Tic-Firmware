/*
 * i2cHandlerTask.c
 *
 *  Created on: Dec 24, 2015
 *      Author: michael
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
#include "utils/i2cm_drv.h"
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
// TivaWare includes
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "utils/i2cm_drv.h"
// My stuff
#include "main.h"
#include "myTasks.h"
#include "i2cHandlerTask.h"

//*****************************************************************************
// Global vars.
//*****************************************************************************
tI2CMInstance g_sI2CInst[4];                         //Four TI I2C driver instances for 4 I2C channels
SemaphoreHandle_t g_i2cSemas[4];                     //Four binary semaphores for exclusive access to I2C driver

// I2C driver streams input state data into the below arrays
uint8_t g_I2CState[4][PCF_MAX_PER_CHANNEL];          //Status of last transmission
t_switchStateConverter g_SwitchStateSampled;         //Read values of last I2C scan
t_switchStateConverter g_SwitchStateDebounced;       //Debounced values (the same after 4 reads)
t_switchStateConverter g_SwitchStateToggled;         //Bits which changed
t_switchStateConverter g_SwitchStateNoDebounce;      //Debouncing-OFF flags

// Stuff for synchronizing TI I2C driver with freeRtos `read inputs` task
uint8_t g_reDiscover;                                      // Flag Rescann all I2C inputs
volatile SemaphoreHandle_t g_pcfReadsInProgressSema = NULL;// I2C read finished when == 0
static TaskHandle_t g_taskToNotifyI2CscanDone = NULL;      //send freeRtos task notification there once i2c scan is done

// Arrays to keep track of timed output pin states and quick-fire rules
t_quickRule g_QuickRuleList[MAX_QUICK_RULES];        //List of Quick fire rules
t_PCLOutputByte g_outWriterList[OUT_WRITER_LIST_LEN];// A list to keep track of the pulse state of all I2C output pins ever written to


//*****************************************************************************
// Functions
//*****************************************************************************

void i2CIntHandler0(void) {
    I2CMIntHandler(&g_sI2CInst[0]);
}
void i2CIntHandler1(void) {
    I2CMIntHandler(&g_sI2CInst[1]);
}
void i2CIntHandler2(void) {
    I2CMIntHandler(&g_sI2CInst[2]);
}
void i2CIntHandler3(void) {
    I2CMIntHandler(&g_sI2CInst[3]);
}

void initMyI2C() {
    uint8_t i;
    for (i = 0; i < N_LONGS; i++) {
        g_SwitchStateDebounced.longValues[i] = 0;
        g_SwitchStateSampled.longValues[i] = 0;
        g_SwitchStateToggled.longValues[i] = 0;
    }
    // ***********************************************
    //  Set up the i2c masters
    // ***********************************************
    // The I2C7 peripheral must be enabled before use.
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C0 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C1 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C2 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C3 );

    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C0 );
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C1 );
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C2 );
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C3 );

    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );   //I2C0
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );   //I2C1
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );   //I2C2
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );   //I2C3

    // Configure the pin muxing for I2C0 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    ROM_GPIOPinConfigure( GPIO_PB2_I2C0SCL );            //I2C0
    ROM_GPIOPinConfigure( GPIO_PB3_I2C0SDA );
    ROM_GPIOPinConfigure( GPIO_PA6_I2C1SCL );            //I2C1
    ROM_GPIOPinConfigure( GPIO_PA7_I2C1SDA );
    ROM_GPIOPinConfigure( GPIO_PE4_I2C2SCL );            //I2C2
    ROM_GPIOPinConfigure( GPIO_PE5_I2C2SDA );
    ROM_GPIOPinConfigure( GPIO_PD0_I2C3SCL );            //I2C3
    ROM_GPIOPinConfigure( GPIO_PD1_I2C3SDA );

    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    ROM_GPIOPinTypeI2CSCL( GPIO_PORTB_BASE, GPIO_PIN_2); //I2C0
    ROM_GPIOPinTypeI2C(    GPIO_PORTB_BASE, GPIO_PIN_3);
    ROM_GPIOPinTypeI2CSCL( GPIO_PORTA_BASE, GPIO_PIN_6); //I2C1
    ROM_GPIOPinTypeI2C(    GPIO_PORTA_BASE, GPIO_PIN_7);
    ROM_GPIOPinTypeI2CSCL( GPIO_PORTE_BASE, GPIO_PIN_4); //I2C2
    ROM_GPIOPinTypeI2C(    GPIO_PORTE_BASE, GPIO_PIN_5);
    ROM_GPIOPinTypeI2CSCL( GPIO_PORTD_BASE, GPIO_PIN_0); //I2C3
    ROM_GPIOPinTypeI2C(    GPIO_PORTD_BASE, GPIO_PIN_1);

    // Enable weak internal pullups on the SCK pin.
    // Also disabes open drain so remove this once external pullups are in place
//    ROM_GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU); //I2C0
//    ROM_GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
//    ROM_GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU); //I2C1
//    ROM_GPIOPadConfigSet( GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
//    ROM_GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU); //I2C2
//    ROM_GPIOPadConfigSet( GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
//    ROM_GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU); //I2C3
//    ROM_GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);

// Enable loopbakc mode (without pullups and without loopback the driver will hang! :( )
//    I2C0_MCR_R |= 0x01;

    // Initialize I2C0 peripheral driver.
    I2CMInit(&g_sI2CInst[0], I2C0_BASE, INT_I2C0, 0xff, 0xff, SYSTEM_CLOCK);
    I2CMInit(&g_sI2CInst[1], I2C1_BASE, INT_I2C1, 0xff, 0xff, SYSTEM_CLOCK);
    I2CMInit(&g_sI2CInst[2], I2C2_BASE, INT_I2C2, 0xff, 0xff, SYSTEM_CLOCK);
    I2CMInit(&g_sI2CInst[3], I2C3_BASE, INT_I2C3, 0xff, 0xff, SYSTEM_CLOCK);
    for( i=0; i<=3; i++ ){
        g_i2cSemas[i] = xSemaphoreCreateBinary();
        xSemaphoreGive( g_i2cSemas[i] );
    }
}

void ts_i2cTransfer(uint8_t channel, uint_fast8_t ui8Addr,
        const uint8_t *pui8WriteData, uint_fast16_t ui16WriteCount,
        uint8_t *pui8ReadData, uint_fast16_t ui16ReadCount,
        tSensorCallback *pfnCallback, void *pvCallbackData) {
    int8_t retVal;
//    Do a thread safe I2C transfer in background (add command to the i2c queue)
    if (channel > 3)
        return;
    if( xSemaphoreTake( g_i2cSemas[channel], 10 ) ){
    //    taskENTER_CRITICAL();
        retVal = I2CMRead(&g_sI2CInst[channel], ui8Addr, pui8WriteData, ui16WriteCount,
                pui8ReadData, ui16ReadCount, pfnCallback, pvCallbackData);
    //    taskEXIT_CRITICAL();
        xSemaphoreGive( g_i2cSemas[channel] );
        if( !retVal ){
            UARTprintf("%22s: !!! FATALITY !!! I2CMRead() failed. Buffer full? \n", "ts_i2cTransfer()");
//            configASSERT( 0 );
            initMyI2C();
        }
    } else {
        UARTprintf("%22s: !!! FATALITY !!! Could not TAKE Semaphore in time.\n", "ts_i2cTransfer");
        configASSERT( 0 );
    }

}

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
    HWREGBITB( &temp, 0 ) = ROM_GPIOPinRead( GPIO_PORTE_BASE, GPIO_PIN_3 ) > 0;
    HWREGBITB( &temp, 1 ) = ROM_GPIOPinRead( GPIO_PORTE_BASE, GPIO_PIN_2 ) > 0;
    HWREGBITB( &temp, 2 ) = ROM_GPIOPinRead( GPIO_PORTE_BASE, GPIO_PIN_1 ) > 0;

    HWREGBITB( &temp, 3 ) = ROM_GPIOPinRead( GPIO_PORTC_BASE, GPIO_PIN_6 ) > 0;
    HWREGBITB( &temp, 4 ) = ROM_GPIOPinRead( GPIO_PORTC_BASE, GPIO_PIN_7 ) > 0;

    HWREGBITB( &temp, 5 ) = ROM_GPIOPinRead( GPIO_PORTD_BASE, GPIO_PIN_6 ) > 0;
    HWREGBITB( &temp, 6 ) = ROM_GPIOPinRead( GPIO_PORTD_BASE, GPIO_PIN_7 ) > 0;

    HWREGBITB( &temp, 7 ) = ROM_GPIOPinRead( GPIO_PORTF_BASE, GPIO_PIN_4 ) > 0;
    return ~temp;
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
    //Switch on first bit
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_DAT);//Shift register data input = high
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT,
    SM_COL_CLK | SM_COL_DAT);        //Shift register clock = pos. edge
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_DAT);//Latch clock = pos. edge
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
}
void advanceSMrow() {
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_CLK);//Shift register clock = pos. edge
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);         //Shift register data input = low
    ROM_SysCtlDelay( SM_COL_DELAY_CNT);
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, SM_COL_DAT);//Latch clock = pos. edge
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

void i2cDoneCallback(void* pvCallbackData, uint_fast8_t ui8Status) {
    // pvCallbackData = pointer to a place in a global array, where the
    // status of the current transaction can be stored
//    uint8_t queueSize;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t *tempReadState = pvCallbackData;
    *tempReadState = ui8Status;                 //Store status value of this transfer in global array
    xSemaphoreTakeFromISR( g_pcfReadsInProgressSema, NULL );     //Decrement `jobs in progress` counter
    if ( xQueueIsQueueEmptyFromISR(g_pcfReadsInProgressSema) ) { //We're done with all jobs
        configASSERT( g_taskToNotifyI2CscanDone != NULL );
        vTaskNotifyGiveFromISR( g_taskToNotifyI2CscanDone, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

// Reads out 1 byte from the address range 0x20 - 0x27 (where PCFL can be) from all 4 I2C channels.
// Interrupt driven and runs in background. Writes to the global data and state arrays
//    Finished when g_readCounter == 4*MAX_PCLS_PER_CHANNEL
// The I2C status of a read is stored in the 2D array [channel][ADR] g_i2cReadStates
// Only the inputs are read which have not previously reported an I2C error
// e.g., the ones which are actually connected!
// @ToDo: only disable inputs on specific I2C error, which comes up when there is no dev. connected (maybe I2CM_STATUS_ADDR_NACK or I2CM_STATUS_ARB_LOST)
void i2cStartPCFL8574refresh() {
    uint8_t nPcf, nChannel, previousState;//, queueSize;
    xSemaphoreGive( g_pcfReadsInProgressSema );     //Make sure `I2C-DONE` cannot be triggered before all jobs are added
    for ( nPcf = 0; nPcf <= PCF_MAX_PER_CHANNEL - 1; nPcf++ ) {
        for ( nChannel = 0; nChannel <= 3; nChannel++ ) {
            previousState = g_I2CState[nChannel][nPcf];
            // If a previous read furing init succeeded, start another one
            if( previousState == I2CM_STATUS_DISABLED ){
                continue;
            }
            if( previousState != I2CM_STATUS_SUCCESS ){
                UARTprintf( "%22s: I2C hwIndex: 0x%2x, error %d\n", "i2cStartPCFL8574refresh()", 64 + nChannel*PCF_MAX_PER_CHANNEL*8 + nPcf*8, previousState );
//                g_I2CState[nChannel][nPcf] = I2CM_STATUS_DISABLED;
//                continue;
            }
            if( xSemaphoreGive( g_pcfReadsInProgressSema ) ) { //Increment reads in progress counter
                ts_i2cTransfer( nChannel, PCF_LOWEST_ADDR + nPcf, NULL, 0, &g_SwitchStateSampled.switchState.i2cReadData[nChannel][nPcf], 1, i2cDoneCallback, &g_I2CState[nChannel][nPcf] );
            } else {
                UARTprintf("i2cStartPCFL8574refresh(): Semaphore overflow!\n");
                configASSERT( 0 );
            }
        }
    }
    xSemaphoreTake( g_pcfReadsInProgressSema, 1 );     //Only now `I2C-DONE` can be triggered from the callback
    // In case the I2C callback was fired already, manually check if I2C is done here
    if ( uxQueueMessagesWaiting(g_pcfReadsInProgressSema) == 0 ) { //done with all reads
        configASSERT( g_taskToNotifyI2CscanDone != NULL );
        vTaskNotifyGiveFromISR( g_taskToNotifyI2CscanDone, NULL );
    }
}

void debounceAlgo( uint32_t *sample, uint32_t *state, uint32_t *toggle, uint32_t *noDebounce ) {
//  Takes the switch state as uint32_t array of length N_LONGS
//    Uses a 2 bit vertical counter algorithm to debounce each bit (needs 4 ticks)
//    toggle is an array indicating which bits changed
//  t:         t+1:
//  cnt1 cnt0  cnt1 cnt0 delta toggle noDebounce
//     0    0     0    0     0      0          0
//     0    0     0    1     1      0          0
//     0    1     1    0     1      0          0
//     1    0     1    1     1      0          0
//     1    1     0    0     1      1          0
// Setting the noDebounce flag
//     0    0     0    0     0      0          1
//     0    0     0    0     1      1          1
    uint8_t i;
    static uint32_t cnt0[N_LONGS], cnt1[N_LONGS];
    uint32_t delta[N_LONGS];
    for (i = 0; i < N_LONGS; i++) {
        delta[i] = sample[i] ^ state[i];               //Bits which are different to currently accepted state
                                                       //All bits of counter are reset if delta = 0
        cnt1[i]   = delta[i] &    (cnt1[i] ^ cnt0[i]); //Increment MSB if delta is SET, otherwise reset it
        cnt0[i]   = delta[i] &   ~(cnt0[i]);           //Increment LSB if delta is SET, otherwise reset it
        //Trigger condition 1: delta must be SET and both cnt must be zero (debounce counter overflow)
        //Trigger condition 2: delta must be SET and noDebounce overwrite must be SET
        toggle[i] = delta[i] & ( ~(cnt0[i]|cnt1[i]) | noDebounce[i] );
        state[i] ^= toggle[i];
    }
}

void reportSwitchStates() {
    // FREERTOS will crash when REPORT_SWITCH_BUF_SIZE > 400, why?
    // --> as during the context switch it copies the whole thing on the stack :p
    // Now we use static to make all large buffers invisible to the context switcher
    static char outBuffer[REPORT_SWITCH_BUF_SIZE];
    // Report changed switch states
    uint8_t i, j, switchValue;
    uint16_t charsWritten = 3;
    uint32_t tempValue;
    ustrncpy(outBuffer, "SE:", REPORT_SWITCH_BUF_SIZE); // SE = Switch event
    for ( i = 0; i < N_LONGS; i++ ) {                   // Go through all 32 bit Long-values
        if ( g_SwitchStateToggled.longValues[i] ) {     // If a bit is set
            tempValue = g_SwitchStateToggled.longValues[i];
            for ( j=0; j<=31; j++ ) {
                if ( tempValue & 0x00000001 ) {         //We found a bit that changed, report over serial USB
                    // 3rd switch changed to 0, 125th switch changed to 1 "SE:003=0;07D=1;"
                    switchValue = HWREGBITW( &g_SwitchStateDebounced.longValues[i], j );
                    charsWritten += usnprintf( &outBuffer[charsWritten],
                    REPORT_SWITCH_BUF_SIZE-charsWritten, "%03x=%01d ", i*32+j, switchValue );
                    if ( charsWritten >= REPORT_SWITCH_BUF_SIZE-10 ) {
                        UARTprintf("reportSwitchStates(): Too much changed, string buffer overflow!\n");
                        return;
                    }
                }
                tempValue = tempValue >> 1;
                if ( tempValue == 0 ) {    //No more bits are set
                    break;
                }
            }
        }
    }
    if (charsWritten > 3) {
        outBuffer[charsWritten] = '\n';
        outBuffer[charsWritten + 1] = '\r';
        ts_usbSend((uint8_t*) outBuffer, charsWritten + 2);
    }
}

#define TF(f) HWREGBITB(&currentRule->triggerFlags,f)

void processQuickRules() {
//    Trigger HoldOFF time (when is the trigger counted as not active?
//    OFF after release after holdoff
//    ---------------------
//     Logic for each rule
//    ---------------------
//    If a rule is enabled:
//        If it is currently triggered:
//            If holdOff time expired:
//                set Rule to untriggered state
//            Else:
//                decrement holdOff time
//        else:
//            Check if the input matches the trigger condition:
//                Set Rule to triggered state
//                switch output ON
    uint8_t i, bIndex, pinIndex;
    bool pinValue;
    t_quickRule *currentRule = g_QuickRuleList;
    for (i = 0; i < MAX_QUICK_RULES; i++) {
        if (TF(QRF_ENABLED)) {                                  //If a rule is enabled:
            bIndex = currentRule->inputSwitchId.byteIndex;
            pinIndex = currentRule->inputSwitchId.pinIndex;
            pinValue = HWREGBITB( &g_SwitchStateDebounced.charValues[bIndex], pinIndex );
            if (TF(QRF_STATE_TRIG)) {                           //    If it is currently triggered:
                if (currentRule->triggerHoldOffCounter <= 0) {  //      If holdOff time expired:
                    TF(QRF_STATE_TRIG) = 0;                     //            set Rule to untriggered state
                } else {                                        //        Else:
                                                                //            decrement holdOff time
                    currentRule->triggerHoldOffCounter -= DEBOUNCER_READ_PERIOD;
                }
            } else {                                            //    If it is not triggered:
                if ( HWREGBITB( &g_SwitchStateToggled.charValues[bIndex], pinIndex ) ) {      // Check if pin toggled
                    if ( pinValue == TF(QRF_TRIG_EDGE_POS) ) {  //    Check if the edge matches
                        TF( QRF_STATE_TRIG ) = 1;               //      Set Rule to triggered state
                        currentRule->triggerHoldOffCounter = currentRule->triggerHoldOffTime;
                        //UARTprintf( "%22s: [%d] Triggered, Outp. set\n", "processQuickRules()", i );
                        UARTprintf( "QR%02d ", i );
                        setPCFOutput( currentRule->outputDriverId,
                                      currentRule->tPulse, currentRule->pwmHigh,
                                      currentRule->pwmLow );
                    }
                }
            }
        }
        currentRule++;
    }
}

void disableQuickRule(uint8_t id) {
    t_quickRule *currentRule = &g_QuickRuleList[id];
    TF( QRF_ENABLED ) = 0;
    TF( QRF_STATE_TRIG ) = 0;
}

void enableQuickRule(uint8_t id) {
    t_quickRule *currentRule = &g_QuickRuleList[id];
    TF( QRF_ENABLED ) = 1;
}

void setupQuickRule(uint8_t id, t_outputBit inputSwitchId,
        t_outputBit outputDriverId, uint16_t triggerHoldOffTime,
        uint16_t tPulse, uint16_t pwmHigh, uint16_t pwmLow,
        bool trigPosEdge) {
//    Notes:
//     `checkToggle`     = False disables edge detecion and triggers on levels
//     `outOffOnRelease` = True  stays in triggered state and waits for a low level. Then disables the outputs and arms the trigger again
//      `outOffOnRelease` = False && levelTriggered = True leads to a periodic trigger with period `triggerHoldOffTime` as long as the level is there (not so good)
//
    t_quickRule *currentRule = &g_QuickRuleList[id];
    currentRule->triggerFlags = 0;    //Mark entry as invalid
    currentRule->inputSwitchId = inputSwitchId;
    currentRule->outputDriverId = outputDriverId;
    currentRule->pwmHigh = pwmHigh;
    currentRule->pwmLow = pwmLow;
    currentRule->tPulse = tPulse;
    currentRule->triggerHoldOffTime = triggerHoldOffTime;
    TF( QRF_TRIG_EDGE_POS ) = trigPosEdge;
    TF( QRF_ENABLED ) = 1;
}

void i2cDiscover(){
    uint8_t nPcf, nChannel;
    // Reset all read stati
    for ( nPcf=0; nPcf<=PCF_MAX_PER_CHANNEL-1; nPcf++ ) {
        for ( nChannel=0; nChannel<=3; nChannel++ ) {
            g_I2CState[nChannel][nPcf] = I2CM_STATUS_SUCCESS;
        }
    }
    // Read the inputs a bit to detect i2c errors
    i2cStartPCFL8574refresh();
    readSwitchMatrix();
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
    UARTprintf("%22s: hwIndex of PCF8574s [ ", "i2cDiscover()");
    for ( nPcf=0; nPcf<=PCF_MAX_PER_CHANNEL-1; nPcf++ ) {
        for ( nChannel=0; nChannel<=3; nChannel++ ) {
            if( g_I2CState[nChannel][nPcf] == I2CM_STATUS_SUCCESS ){
                UARTprintf( "0x%02x ", 64 + nChannel*PCF_MAX_PER_CHANNEL + nPcf*8 );
            } else {
                g_I2CState[nChannel][nPcf] = I2CM_STATUS_DISABLED;
            }
        }
    }
    UARTprintf("]\n");
}

void taskDebouncer(void *pvParameters) {
//    Read the state of all switches every 3 ms
//    Then we need to debounce each switch and send `state changed` events
//    uint32_t ticks;
    TickType_t xLastWakeTime;
    uint8_t i;
    UARTprintf("%22s: Started! I2CbufferSize = %d\n", "taskDebouncer()", NUM_I2CM_COMMANDS);
    for (i = 0; i < MAX_QUICK_RULES; i++) {
        disableQuickRule(i);
    }
    // Each of the 4*8 read operations takes a sema. + 1 extra sema for waiting during when the jobs are added
    // I2C-read is done when the queue behind the semaphore `g_pcfReadsInProgressSema` is empty
    g_pcfReadsInProgressSema = xSemaphoreCreateCounting( 4*PCF_MAX_PER_CHANNEL+1, 0 );
    g_taskToNotifyI2CscanDone = xTaskGetCurrentTaskHandle();
    g_reDiscover = 0;
    vTaskDelay( 1 );
    i2cDiscover();
    i2cStartPCFL8574refresh();
    readSwitchMatrix();
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (ulTaskNotifyTake( pdTRUE, portMAX_DELAY)) {    // Wait for i2c scanner ISR to finish
            // If rediscover flag is set, rescan all i2c inputs
            if( g_reDiscover ){
                i2cDiscover();
                g_reDiscover = 0;
            }
            // Run debounce algo (14 us)
            debounceAlgo( g_SwitchStateSampled.longValues, g_SwitchStateDebounced.longValues, g_SwitchStateToggled.longValues, g_SwitchStateNoDebounce.longValues );
            // Notify Mission pinball over serial port of all changed switches
            if( g_reportSwitchEvents ){
                reportSwitchStates();
            }
            processQuickRules();
//            ticks = stopTimer();
//            UARTprintf("readSwitchMatrix() %d ticks\n", ticks );
            //Run every 3 ms (333 Hz) --> 12 ms debounce latency
            vTaskDelayUntil(&xLastWakeTime, DEBOUNCER_READ_PERIOD);
//            startTimer();
            //Start background I2C scanner (takes ~ 600 us)
            i2cStartPCFL8574refresh();
            //Should take >= 500 us as it happens in parallel with the I2C scan
            readSwitchMatrix();
        }
    }
}

//------------------------------------------------------
// Functios to _WRITE_ to outputs
//------------------------------------------------------
t_outputBit decodeHwIndex(uint16_t hwIndex, uint8_t asInput) {
// Decode a hwIndex and fill the t_outputBit structure with details
// asInput: is this supposed to be an input (1) or output (0)
// Meaning of byteIndex:  HW_INDEX_SWM: column,  HW_INDEX_I2Cn: right shited I2C address
    int16_t i2cCh;
    t_outputBit tempResult;
    //-----------------------------------------
    // Check for HW. PWM outputs (60 - 63)
    //-----------------------------------------
    if( (!asInput) && (hwIndex>=60) && (hwIndex<=63) ){
        tempResult.hwIndexType = HW_INDEX_HWPWM;// Note: only pinIndex is valid. Identifies the HW pwmChannel!
        tempResult.pinIndex = hwIndex - 60;
        tempResult.i2cAddress = 0;
        tempResult.i2cChannel = 100;            // I2C channel 100 is reserved for HW PWM
        return tempResult;
    }
    tempResult.byteIndex = hwIndex / 8;
    tempResult.pinIndex = hwIndex % 8;          // Which bit of the byte is addressed
    tempResult.i2cChannel = -1;
    //-----------------------------------------
    // Check for a Switch Matrix Input (0 - 7)
    //-----------------------------------------
    if ( tempResult.byteIndex<=7 ) {
        if( asInput ){                          // And hence must be an input !
            tempResult.hwIndexType = HW_INDEX_SWM;
            return tempResult;
        }
        tempResult.hwIndexType = HW_INDEX_INVALID;
        return tempResult;
    }
    //-----------------------------------------
    // Check for I2C channel
    //-----------------------------------------
    i2cCh = (tempResult.byteIndex - 8) / 8;     // Only i2c channel 0-3 exists
    if (i2cCh <= 3) {
        tempResult.hwIndexType = HW_INDEX_I2C;
        tempResult.i2cChannel = i2cCh;      //I2C address, each channel has address 0x20 - 0x27
        tempResult.i2cAddress = (tempResult.byteIndex - 8) % 8 + PCF_LOWEST_ADDR;
        return tempResult;
    }
    tempResult.hwIndexType = HW_INDEX_INVALID;
    return tempResult;
}

void setBcm(uint8_t *bcmBuffer, uint8_t pin, uint8_t pwmValue) {
    // Pre calculate and SET the values which need to be output on a pin to get
    // Binary Code Modulation (BCM) with a certain power level.
    uint8_t j;
    taskENTER_CRITICAL();
    for (j = 0; j < N_BIT_PWM; j++) {
        HWREGBITB( bcmBuffer, pin ) = HWREGBITB(&pwmValue, j);
        bcmBuffer++;
    }
    taskEXIT_CRITICAL();
}

void handleBitRules( t_PCLOutputByte *outListPtr, uint8_t dt ) {
    // Handle the switchover from `Pulsed` state to `unpulsed` state for each output pin
    uint8_t i;
    t_BitModifyRules *bitRules = outListPtr->bitRules;
    for (i = 0; i <= 7; i++) {
        if ( bitRules->tPulse > 0 ) {                                  //Is the entry valid?
            bitRules->tPulse -= dt;                                    //Apply dt timestep
            if ( bitRules->tPulse <= 0 ) {                             //Did the countdown expire?
                if( outListPtr->i2cChannel == 100 ){                   // 100 = magic number meaning we need to set a HW PWM channel
                    setPwm( i, bitRules->lowPWM );                     // Apply low HW pwm
                } else {
                    setBcm( outListPtr->bcmBuffer, i, bitRules->lowPWM );//Then apply the I2C pulse_low bcm pattern
                }
            }
        }
        bitRules++;
    }
}

void taskPCFOutWriter(void *pvParameters) {
    // Dispatch I2C write commands to PCL GPIO extenders periodically
    // Use binary code modulation for N bit PWM
    UARTprintf("%22s: Started!\n", "taskPCLOutWriter()");
    uint8_t i, j, lastTickCount = 0;
    uint8_t bcmCycleCounter = 0;    //Which bit to output
//    uint32_t c = 0, ticks;
    TickType_t xLastWakeTime;
    t_PCLOutputByte *outListPtr = g_outWriterList;
//  -------------------------------------------------------
//   Init Data structure for caching the output values
//  -------------------------------------------------------
    for (i = 0; i < OUT_WRITER_LIST_LEN; i++) {
        for (j = 0; j < N_BIT_PWM; j++) {
            outListPtr->bcmBuffer[j] = 0x00;    //Clear all bits by default
        }
        outListPtr->i2cChannel = -1;            //This marks the entry as invalid
        outListPtr++;
    }
    vTaskDelay(1);
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        //------------------------------------------------------------------
        // This loop does binary code modulation
        // It executes periodically with increasing delay (d) like this:
        // d=1, d=2, d=4, d=8, d=1, ...
        //------------------------------------------------------------------
        outListPtr = g_outWriterList;
        for (i = 0; i < OUT_WRITER_LIST_LEN; i++) {
            if (outListPtr->i2cChannel < 0) {
                //This and all further entries in the array are invalid items.
                break;
            } else if ( outListPtr->i2cChannel <= 3 ) {
                //A valid item, simply output the current bcm buffer value over I2C
                ts_i2cTransfer(outListPtr->i2cChannel, outListPtr->i2cAddress,
                        &outListPtr->bcmBuffer[bcmCycleCounter], 1, NULL, 0,
                        NULL, NULL);
            }
            handleBitRules(outListPtr, lastTickCount);
            outListPtr++;
        }
        //---------------------------------
        // Measure ticks for profiling
        //---------------------------------
//        ticks = stopTimer();
//        c++;
//        if (c >= 3000) {
//            c = 0;
//            UARTprintf("%22s: %d ticks\n", "taskPCLOutWriter()", ticks);
//        }
        //-----------------------------------------
        // Delay until next bit needs to be output
        //-----------------------------------------
        // SET0, 1 ms, SET1, 2 ms, SET2, 4 ms, SET3, 8 ms, repeat
        lastTickCount = (1 << bcmCycleCounter);
        vTaskDelayUntil(&xLastWakeTime, lastTickCount);
//        startTimer();
        bcmCycleCounter++;
        if (bcmCycleCounter >= N_BIT_PWM) {
            bcmCycleCounter = 0;
        }
    }
}

void setPCFOutput(t_outputBit outLocation, int16_t tPulse, uint16_t highPower, uint16_t lowPower) {
// Set the power level and pulse settings of an output pin
// Will add a job to the output BCM list
// ToDo: Make sure that BCMing outputs are not reported as switch events.
//    tPulse    = duration of the pulse [ms]
//    highPower = PWM value during the pulse
//    lowPower  = PWM value after  the pulse
    uint8_t i;
    t_PCLOutputByte *outListPtr = g_outWriterList;
    t_BitModifyRules *bitRules;
    if ( !(outLocation.hwIndexType==HW_INDEX_I2C||outLocation.hwIndexType==HW_INDEX_HWPWM) )
        return;
    for (i = 0; i < OUT_WRITER_LIST_LEN; i++) {
        if (outListPtr->i2cChannel == -1) {
//            Create new entry!
            bitRules = &outListPtr->bitRules[outLocation.pinIndex];
            bitRules->tPulse = tPulse;
            bitRules->lowPWM = lowPower;
            outListPtr->i2cAddress = outLocation.i2cAddress;
            if( outLocation.hwIndexType == HW_INDEX_I2C ){
                setBcm(outListPtr->bcmBuffer, outLocation.pinIndex, highPower);
            } else if ( outLocation.hwIndexType == HW_INDEX_HWPWM ){
                setPwm( outLocation.pinIndex, highPower );
            }
            outListPtr->i2cChannel = outLocation.i2cChannel;//Mark the item as valid to the output routine
            UARTprintf("%22s: wrote to g_outWriterList[%d] (new)\n", "setPclOutput()", i );
            return;
        } else if (outListPtr->i2cChannel == outLocation.i2cChannel
                && outListPtr->i2cAddress == outLocation.i2cAddress) {
//            Found the right byte, change it
            bitRules = &outListPtr->bitRules[outLocation.pinIndex];
            bitRules->lowPWM = lowPower;
            if( outLocation.hwIndexType == HW_INDEX_I2C ){
                setBcm(outListPtr->bcmBuffer, outLocation.pinIndex, highPower);
            } else if ( outLocation.hwIndexType == HW_INDEX_HWPWM ){
                setPwm( outLocation.pinIndex, highPower );
            }
            bitRules->tPulse = tPulse;
//            UARTprintf("%22s: wrote to g_outWriterList[%d]\n", "setPclOutput()", i );
            return;
        }
        outListPtr++;
    }
//    Error, no more space in outList :(
    UARTprintf("%22s: Error, no more space in g_outWriterList :(\n", "setPclOutput()");
}
