// Handles inputs through PCF I2C extenders
//   * reading them periodically
//   * communication error handling
//   * debouncing
//   * reporting state changes to USB

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_ints.h"
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
#include "utils/ustdlib.h"
#include "sensorlib/i2cm_drv.h"
#include "my_uartstdio.h"
// My stuff
#include "main.h"
#include "myTasks.h"
#include "i2c_in.h"
#include "i2c_out.h"
#include "switch_matrix.h"

//*****************************************************************************
// Global vars.
//*****************************************************************************
tI2CMInstance g_sI2CInst[4];                         //Four TI I2C driver instances for 4 I2C channels

// I2C driver streams input state data into the below arrays
unsigned g_I2CState[4][PCF_MAX_PER_CHANNEL];         //Status of last transmission

t_switchStateConverter g_SwitchStateSampled;         //Read values of last I2C scan
t_switchStateConverter g_SwitchStateDebounced;       //Debounced values (the same after 4 reads)
t_switchStateConverter g_SwitchStateToggled;         //Bits which changed
t_switchStateConverter g_SwitchStateNoDebounce;      //Debouncing-OFF flags

// Stuff for synchronizing TI I2C driver with freeRtos `read inputs` task
bool g_reDiscover;                                      // Flag Rescann all I2C inputs
volatile SemaphoreHandle_t g_pcfReadsInProgress = NULL;// I2C read finished when == 0
TaskHandle_t hPcfInReader = NULL;      //send freeRtos task notification there once i2c scan is done

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

// prints g_I2CState in human readable form
const char *getI2cStateStr(unsigned state){
    state &= 0x0F;
    static const char *i2cStates[] = {
        "OK",
        "ADDR_NACK",
        "DATA_NACK",
        "ARB_LOST",
        "ERROR",
        "BATCH_DONE",
        "BATCH_READY",
        "UNKNOWN",
        "BLOCKED"  // happens with no pullups on the SCL lines
    };
    if (state < I2CM_STATUS_SUCCESS || state > I2CM_STATUS_BLOCKED)
        return "INV_STATE";
    return i2cStates[state];
}

void i2cUnstucker(uint32_t base, uint8_t pin_SCL, uint8_t pin_SDA){
    //--------------------------
    // Unstuck I2C SDA lines
    //--------------------------
    ROM_GPIOPinTypeGPIOOutputOD( base, pin_SCL | pin_SDA );
    ROM_GPIOPinWrite(base, pin_SDA, 0xFF);
    for (unsigned i=0; i<16; i++){
        ROM_GPIOPinWrite(base, pin_SCL, 0x00);
        ROM_SysCtlDelay(100);   // ~ 4 us @ 80 MHz
        ROM_GPIOPinWrite(base, pin_SCL, 0xFF);
        ROM_SysCtlDelay(100);
    }
    GPIOPinTypeGPIOInput(base, pin_SDA | pin_SCL);
    ROM_SysCtlDelay(100);
    unsigned rVal = GPIOPinRead(base, pin_SDA | pin_SCL);
    rVal = ~rVal & (pin_SDA | pin_SCL);
    if(rVal){
        UARTprintf("i2cUnstucker()        : pin stuck low @%x: ", base);
        if(rVal & pin_SCL)
            UARTprintf("SCL ");
        if(rVal & pin_SDA)
            UARTprintf("SDA ");
        UARTprintf("\n");
    }
}

void initMyI2C() {
    unsigned i;
    for (i = 0; i < N_LONGS; i++) {
        g_SwitchStateDebounced.longValues[i] = 0;
        g_SwitchStateSampled.longValues[i] = 0;
        g_SwitchStateToggled.longValues[i] = 0;
    }
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB );   //I2C0
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA );   //I2C1
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOE );   //I2C2
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );   //I2C3

    //--------------------------
    // Unstuck I2C SDA lines
    //--------------------------
    i2cUnstucker(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_3);
    i2cUnstucker(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_7);
    i2cUnstucker(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_5);
    i2cUnstucker(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_1);

    //--------------------------
    //  Set up the i2c masters
    //--------------------------
    // The I2C7 peripheral must be enabled before use.
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C0 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C1 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C2 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C3 );
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C0 );
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C1 );
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C2 );
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C3 );

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
    int8_t retVal;
//    Do a thread safe I2C transfer in background (add command to the i2c queue)
    // UARTprintf("i2c(%x %x %p)\n", channel, ui8Addr, pvCallbackData);
    if (channel > 3)
        return;
    taskENTER_CRITICAL();
    retVal = I2CMCommand(
        &g_sI2CInst[channel],
        ui8Addr,
        pui8WriteData,
        ui16WriteCount,
        ui16WriteCount,
        pui8ReadData,
        ui16ReadCount,
        ui16ReadCount,
        pfnCallback,
        pvCallbackData
    );
    taskEXIT_CRITICAL();
    if(!retVal){
        DISABLE_SOLENOIDS();
        REPORT_ERROR( "ER:0000\n" );
        UARTprintf("%22s: I2CMCommand not added to queue\n", "ts_i2cTransfer()");
        disableAllWriters();
    }
}

void i2cReadDone(void* pvCallbackData, uint_fast8_t ui8Status) {
    // pvCallbackData = pointer to a place in a global array, where the
    // status of the current transaction can be stored
    *(unsigned*)pvCallbackData = ui8Status;
    xSemaphoreTakeFromISR(g_pcfReadsInProgress, NULL);     //Decrement `jobs in progress` counter
    if (xQueueIsQueueEmptyFromISR(g_pcfReadsInProgress)) { //We're done with all jobs
        vTaskNotifyGiveFromISR(hPcfInReader, NULL);
    }
}

// Reads out 1 byte from the address range 0x20 - 0x27 (where PCFL can be) from all 4 I2C channels.
// Interrupt driven and runs in background. Writes to the global data and state arrays
//    Finished when g_readCounter == 4 * MAX_PCLS_PER_CHANNEL
// The I2C status of a read is stored in the 2D array [channel][ADR] g_i2cReadStates
// Only the inputs are read which have not previously reported an I2C error
// e.g., the ones which are actually connected!
void i2cStartPCFL8574refresh() {
    // Make sure `I2C-DONE` cannot be triggered before all jobs are added
    xSemaphoreGive(g_pcfReadsInProgress);
    for (unsigned nPcf = 0; nPcf < PCF_MAX_PER_CHANNEL; nPcf++) {
        for (unsigned nChannel = 0; nChannel <= 3; nChannel++) {
            // unsigned previousState = g_I2CState[nChannel][nPcf];
            // if(previousState == I2CM_STATUS_SUCCESS ||
               // previousState == I2CM_STATUS_UNKNOWN){
                // Do a i2c read, increment reads in progress counter
                xSemaphoreGive(g_pcfReadsInProgress);
                // if(previousState == I2CM_STATUS_UNKNOWN) {
                //     g_I2CState[nChannel][nPcf] = I2CM_STATUS_BLOCKED;
                // }
                ts_i2cTransfer(
                    nChannel,
                    PCF_LOWEST_ADDR + nPcf,
                    NULL,
                    0,
                    &g_SwitchStateSampled.switchState.i2cReadData[nChannel][nPcf],
                    1,
                    i2cReadDone,
                    &g_I2CState[nChannel][nPcf]
                );
            // }
        }
    }
    // Only after this, `I2C-DONE` can be triggered from the callback
    xSemaphoreTake(g_pcfReadsInProgress, 1);
    // In case the I2C callback was fired already, manually check if I2C is done here
    if (uxQueueMessagesWaiting(g_pcfReadsInProgress) == 0) { //done with all reads
        xTaskNotifyGive(hPcfInReader);
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
//        outBuffer[charsWritten + 1] = '\r';
        ts_usbSend( (uint8_t*)outBuffer, charsWritten+1 );
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
                        UARTprintf( "R%02d ", i );
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

const uint8_t zero8 = 0;

void i2cResetState(){
    unsigned nPcf, nChannel;
    UARTprintf("Discover!\n");
    // Reset all read stati
    for (nChannel=0; nChannel<=3; nChannel++) {
        for (nPcf=0; nPcf<PCF_MAX_PER_CHANNEL; nPcf++) {
            g_I2CState[nChannel][nPcf] = I2CM_STATUS_UNKNOWN;
            // ts_i2cTransfer(
            //     nChannel,
            //     PCF_LOWEST_ADDR + nPcf,
            //     &zero8,
            //     1,
            //     NULL,
            //     0,
            //     NULL,
            //     NULL
            // );
        }
    }
}

void taskPcfInReader(void *pvParameters) {
//    Read the state of all switches every 3 ms
//    Then we need to debounce each switch and send `state changed` events
//    uint32_t ticks;
    TickType_t xLastWakeTime;
    unsigned i;
    // hPcfInReader = xTaskGetCurrentTaskHandle();
    UARTprintf("%22s: Started! I2CbufferSize = %d\n", "taskPcfInReader()", NUM_I2CM_COMMANDS);
    for (i=0; i<MAX_QUICK_RULES; i++) {
        disableQuickRule(i);
    }
    // Each of the 4*8 read operations takes a sema. + 1 extra sema for waiting
    // during when the jobs are added
    // I2C-read is done when the queue behind the semaphore `g_pcfReadsInProgress` is empty
    g_pcfReadsInProgress = xSemaphoreCreateCounting(4 * PCF_MAX_PER_CHANNEL + 1, 0);
    initMyI2C();
    i2cResetState();
    vTaskDelay(1);
    // Get the initial state of all switches silently (without reporting Switch Events)
    i2cStartPCFL8574refresh();
    readSwitchMatrix();
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // Wait for i2c scanner ISR to finish
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
            if(g_reDiscover){
                initMyI2C();
                i2cResetState();
                xQueueReset(g_pcfReadsInProgress);
                UARTprintf("I2C reset done!\n");
                g_reDiscover = 0;
            }
            // Run debounce algo (14 us)
            debounceAlgo(g_SwitchStateSampled.longValues, g_SwitchStateDebounced.longValues, g_SwitchStateToggled.longValues, g_SwitchStateNoDebounce.longValues);
            // Notify Mission pinball over serial port of all changed switches
            if(g_reportSwitchEvents){
                reportSwitchStates();
            }
            processQuickRules();
            //Run every 3 ms (333 Hz) --> 12 ms debounce latency
            vTaskDelayUntil(&xLastWakeTime, DEBOUNCER_READ_PERIOD);
            // vTaskDelayUntil(&xLastWakeTime, 3000);
            //Start background I2C scanner (takes ~ 600 us)
            i2cStartPCFL8574refresh();
            //Should take >= 500 us as it happens in parallel with the I2C scan
            readSwitchMatrix();
        }
    }
}
