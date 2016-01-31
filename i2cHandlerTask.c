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
#include "sensorlib/i2cm_drv.h"
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
// My stuff
#include "main.h"
#include "myTasks.h"
#include "i2cHandlerTask.h"

//*****************************************************************************
// Global vars.
//*****************************************************************************
tI2CMInstance g_sI2CInst[4];                         //Four TI I2C driver instances for 4 I2C channels

// I2C driver streams input state data into the below arrays
uint8_t g_i2cReadStates[4][PCF_MAX_PER_CHANNEL];     //All Error codes of last I2C scan
t_switchStateConverter g_SwitchStateSampled;         //Read values of last I2C scan
t_switchStateConverter g_SwitchStateDebounced;       //Debounced values (the same after 4 reads)
t_switchStateConverter g_SwitchStateToggled;         //Bits which changed

// Stuff for synchronizing TI I2C driver with freeRtos `read inputs` task
volatile uint8_t g_readCounter = 0;                  // I2C read finished when g_readCounter == 4*MAX_PCLS_PER_CHANNEL
static TaskHandle_t xTaskToNotifyI2CscanDone = NULL; //send freeRtos task notification there once i2c scan is done

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
}

void ts_i2cTransfer(uint8_t channel, uint_fast8_t ui8Addr,
        const uint8_t *pui8WriteData, uint_fast16_t ui16WriteCount,
        uint8_t *pui8ReadData, uint_fast16_t ui16ReadCount,
        tSensorCallback *pfnCallback, void *pvCallbackData) {
//    Do a thread safe I2C transfer in background (add command to the i2c queue)
    if (channel > 3)
        return;
    taskENTER_CRITICAL();
    I2CMRead(&g_sI2CInst[channel], ui8Addr, pui8WriteData, ui16WriteCount,
            pui8ReadData, ui16ReadCount, pfnCallback, pvCallbackData);
    taskEXIT_CRITICAL();
}

uint8_t getSMrow() {
    uint8_t temp;
    // Read the state of the row of the switch matrix.
    // The inputs are distributed across 4 Ports :p
    //         ,2,1,0   PORTE
    //     ,4,3         PORTC
    // ,6,5             PORTD
    //7                 PORTF
    temp  = GPIOPinRead( GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1) >> 1;
    temp |= GPIOPinRead( GPIO_PORTC_BASE, GPIO_PIN_7 | GPIO_PIN_6) >> 3;
    temp |= GPIOPinRead( GPIO_PORTD_BASE, GPIO_PIN_7 | GPIO_PIN_6) >> 1;
//    temp |= GPIOPinRead( GPIO_PORTF_BASE, GPIO_PIN_4) << 3;    //TODO: Renable this on the final hardware !
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
    ROM_GPIOPinWrite( GPIO_PORTB_BASE, SM_COL_CLK | SM_COL_DAT, 0);    //Shift register data input = low
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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t *tempReadState = pvCallbackData;
    *tempReadState = ui8Status;
    g_readCounter++;
    if (g_readCounter >= 4 * PCF_MAX_PER_CHANNEL) {
        configASSERT( xTaskToNotifyI2CscanDone != NULL );
        vTaskNotifyGiveFromISR(xTaskToNotifyI2CscanDone,
                &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Reads out 1 byte from the address range 0x20 - 0x27 (where PCFL can be) from all 4 I2C channels.
// Interrupt driven and runs in background. Writes to the global data and state arrays
//    Finished when g_readCounter == 4*MAX_PCLS_PER_CHANNEL
void i2cStartPCFL8574refresh() {
    uint8_t i;
    g_readCounter = 0;
    configASSERT( xTaskToNotifyI2CscanDone == NULL );
    xTaskToNotifyI2CscanDone = xTaskGetCurrentTaskHandle();
    for (i = 0; i <= PCF_MAX_PER_CHANNEL - 1; i++) {
        ts_i2cTransfer(0, PCF_LOWEST_ADDR + i, NULL, 0,
                g_SwitchStateSampled.switchState.i2cReadData[0], 1,
                i2cDoneCallback, &g_i2cReadStates[0][i]);
        ts_i2cTransfer(1, PCF_LOWEST_ADDR + i, NULL, 0,
                g_SwitchStateSampled.switchState.i2cReadData[1], 1,
                i2cDoneCallback, &g_i2cReadStates[1][i]);
        ts_i2cTransfer(2, PCF_LOWEST_ADDR + i, NULL, 0,
                g_SwitchStateSampled.switchState.i2cReadData[2], 1,
                i2cDoneCallback, &g_i2cReadStates[2][i]);
        ts_i2cTransfer(3, PCF_LOWEST_ADDR + i, NULL, 0,
                g_SwitchStateSampled.switchState.i2cReadData[3], 1,
                i2cDoneCallback, &g_i2cReadStates[3][i]);
    }
}

void debounceAlgo(uint32_t *sample, uint32_t *state, uint32_t *toggle) {
//  Takes the switch state as uint32_t array of length N_LONGS
//    Uses a 2 bit vertical counter algorithm to debounce each bit (needs 4 ticks)
//    toggle is an array indicating which bits changed
    uint8_t i;
    static uint32_t cnt0[N_LONGS], cnt1[N_LONGS];
    uint32_t delta[N_LONGS];
    for (i = 0; i < N_LONGS; i++) {
        delta[i] = sample[i] ^ state[i];
        cnt1[i] = (cnt1[i] ^ cnt0[i]) & delta[i];
        cnt0[i] = ~(cnt0[i]) & delta[i];
        toggle[i] = delta[i] & ~(cnt0[i] | cnt1[i]);
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
    ustrncpy(outBuffer, "SE:", REPORT_SWITCH_BUF_SIZE); //SE = Switch event
    for (i = 0; i < N_LONGS; i++) {
        if (g_SwitchStateToggled.longValues[i]) {
            tempValue = g_SwitchStateToggled.longValues[i];
            for (j = 0; j <= 31; j++) {
                if (tempValue & 0x00000001) { //We found a bit that changed, report over serial USB
                    // 3rd switch changed to 0, 125th switch changed to 1 "SE:003=0;07D=1;"
                    switchValue = (g_SwitchStateDebounced.longValues[i] >> j)
                            & 0x00000001;
                    charsWritten += usnprintf(&outBuffer[charsWritten],
                    REPORT_SWITCH_BUF_SIZE - charsWritten, "%03x=%01d ",
                            i * 32 + j, switchValue);
                    if (charsWritten >= REPORT_SWITCH_BUF_SIZE - 10) {
                        UARTprintf(
                                "reportSwitchStates(): Too much changed, string buffer overflow!\n");
                        return;
                    }
                }
                tempValue = tempValue >> 1;
                if (tempValue == 0) {    //No more bits are set
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
//                If OFF_ON_RELEASE flag is set:
//                    If input is released:
//                        switch output Off
//                        set Rule to untriggered state
//                Else:
//                    set Rule to untriggered state
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
            pinValue = HWREGBITB(&g_SwitchStateDebounced.charValues[bIndex],
                    pinIndex);
            if (TF(QRF_STATE_TRIG)) {                           //    If it is currently triggered:
                if (currentRule->triggerHoldOffCounter <= 0) {  //      If holdOff time expired:
                    if (TF(QRF_OFF_ON_RELASE)) {                //            If OFF_ON_RELEASE flag is set:
                        if (pinValue != TF(QRF_TRIG_EDGE_POS)){ //                If input released:
                            TF(QRF_STATE_TRIG) = 0;             //                    set Rule to untriggered state
                                                                //                    Switch output Off
                            UARTprintf("%22s: [%d] Outp = Off after release\n",
                                    "processQuickRules()", i);
                            setPCFOutput(currentRule->outputDriverId, 0, 0, 0);
                        }
                    } else {                                    //            Else:
                        TF(QRF_STATE_TRIG) = 0;                 //                set Rule to untriggered state
                    }
                } else {                                        //        Else:
                    currentRule->triggerHoldOffCounter--;       //            decrement holdOff time
                }
            } else {                                            //    If it is not triggered:
                if ( HWREGBITB(&g_SwitchStateToggled.charValues[bIndex],
                        pinIndex) || TF(QRF_LEVEL_TRIG)) {      // Check if pin toggled (skip if level triggered)
                    if (pinValue == TF(QRF_TRIG_EDGE_POS)) {    //    Check if the edge matches
                        TF( QRF_STATE_TRIG ) = 1;               //      Set Rule to triggered state
                        currentRule->triggerHoldOffCounter =
                                currentRule->triggerHoldOffTime;
                        UARTprintf("%22s: [%d] Triggered, Outp. set\n",
                                "processQuickRules()", i);
                        setPCFOutput(currentRule->outputDriverId,
                                currentRule->tPulse, currentRule->pwmHigh,
                                currentRule->pwmLow);
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
        uint16_t tPulse, uint8_t pwmHigh, uint8_t pwmLow,
        bool trigPosEdge, bool outOffOnRelease, bool levelTriggered) {
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
    TF( QRF_OFF_ON_RELASE ) = outOffOnRelease;
    TF( QRF_LEVEL_TRIG ) = levelTriggered;
    TF( QRF_ENABLED ) = 1;
}

void taskDebouncer(void *pvParameters) {
//    Read the state of all switches every 3 ms
//    Then we need to debounce each switch and send `state changed` events
//    uint32_t ticks;
    TickType_t xLastWakeTime;
    uint8_t i;
    UARTprintf("%22s: %s", "taskDebouncer()", "Started!\n");
    for (i = 0; i < MAX_QUICK_RULES; i++) {
        disableQuickRule(i);
    }
    vTaskDelay( 1 );
    i2cStartPCFL8574refresh();
    readSwitchMatrix();
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (ulTaskNotifyTake( pdTRUE, portMAX_DELAY)) {    // Wait for i2c scanner ISR to finish
            // Run debounce algo (14 us)
            debounceAlgo( g_SwitchStateSampled.longValues, g_SwitchStateDebounced.longValues, g_SwitchStateToggled.longValues);
            // Notify Mission pinball over serial port of all changed switches
            reportSwitchStates();
            processQuickRules();
//            ticks = stopTimer();
//            UARTprintf("readSwitchMatrix() %d ticks\n", ticks );
            //Run every 3 ms (333 Hz) --> 12 ms debounce latency
            vTaskDelayUntil(&xLastWakeTime, 3);
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
t_outputBit decodeHwIndex(uint16_t hwIndex) {
// Decode a hwIndex and fill the t_outputBit structure with details
// Meaning of byteIndex:  HW_INDEX_SWM: column,  HW_INDEX_I2Cn: right shited I2C address
    int16_t i2cCh;
    t_outputBit tempResult;
    tempResult.byteIndex = hwIndex / 8;
    tempResult.pinIndex = hwIndex % 8;    // Which bit of the byte is addressed
    tempResult.i2cChannel = -1;
    if (tempResult.byteIndex <= 7) {// byteIndex 0 - 7 are Switch Matrix addresses
        tempResult.hwIndexType = HW_INDEX_SWM;
        return tempResult;
    }
    i2cCh = (tempResult.byteIndex - 8) / 8;    // Only i2c channel 0-3 exists
    if (i2cCh <= 3) {
        tempResult.hwIndexType = HW_INDEX_I2C;
        tempResult.i2cChannel = i2cCh;//I2C address, each channel has address 0x20 - 0x27
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
    UARTprintf("%22s: %s", "taskPCLOutWriter()", "Started!\n");
    uint8_t i, j, lastTickCount = 0;
    uint8_t bcmCycleCounter = 0;    //Which bit to output
//    uint32_t c = 0, ticks;
    TickType_t xLastWakeTime;
    t_PCLOutputByte *outListPtr = g_outWriterList;
//    -------------------------------------------------------
//    Init Data structure for caching the output values
//    -------------------------------------------------------
    for (i = 0; i < OUT_WRITER_LIST_LEN; i++) {
        for (j = 0; j < N_BIT_PWM; j++) {
            outListPtr->bcmBuffer[j] = 0xFF;    //Set all bits by default
        }
        outListPtr->i2cChannel = -1;            //This marks the entry as invalid
        outListPtr++;
    }
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
            return;
        }
        outListPtr++;
    }
//    Error, no more space in outList :(
    UARTprintf("%22s: Error, no more space in g_outWriterList :(\n", "setPclOutput()");
}
