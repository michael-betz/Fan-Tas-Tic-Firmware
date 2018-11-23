// Functions to _WRITE_ to PCF outputs
//  * Calculate BCM patterns
//  * periodically write them to PCFs

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_types.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "my_uartstdio.h"
#include "myTasks.h"
#include "i2c_out.h"
#include "i2c_in.h"

// Arrays to keep track of timed output pin states and quick-fire rules
t_quickRule g_QuickRuleList[MAX_QUICK_RULES];        //List of Quick fire rules
t_PCLOutputByte g_outWriterList[OUT_WRITER_LIST_LEN];// A list to keep track of the pulse state of all I2C output pins ever written to

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
    if (tempResult.byteIndex <= 7) {
        if(asInput){                          // And hence must be an input !
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

void disableAllWriters(void){
    unsigned i, j;
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
}

void taskPCFOutWriter(void *pvParameters) {
    // Dispatch I2C write commands to PCL GPIO extenders periodically
    // Use binary code modulation for N bit PWM
    UARTprintf("%22s: Started!\n", "taskPCLOutWriter()");
    unsigned i, dt_ms = 0;
    unsigned cycle = 0;    // Which bit to output
    TickType_t xLastWakeTime;
    t_PCLOutputByte *p;    // Output state
    disableAllWriters();
    vTaskDelay(1);
    xLastWakeTime = xTaskGetTickCount();
    while (1) {
        //------------------------------------------------------------------
        // This loop does binary code modulation
        // It executes periodically with increasing delay (d) like this:
        // d=1, d=2, d=4, d=8, d=1, ...
        //------------------------------------------------------------------
        for (i = 0; i < OUT_WRITER_LIST_LEN; i++) {
            p = &g_outWriterList[i];
            if (p->i2cChannel < 0 ||
                p->i2cChannel > 3 ||
                p->i2cAddress < PCF_LOWEST_ADDR ||
                p->i2cAddress > PCF_LOWEST_ADDR + PCF_MAX_PER_CHANNEL - 1){
                continue;
                // Invalid item, ignore.
            }
            uint8_t lastState = g_I2CState[p->i2cChannel][p->i2cAddress - PCF_LOWEST_ADDR];
            if (lastState != I2CM_STATUS_SUCCESS){
                // Last read failed, ignore!
                continue;
            }
            // output the current bcm buffer value over I2C
            ts_i2cTransfer(
                p->i2cChannel,
                p->i2cAddress,
                &p->bcmBuffer[cycle],
                1,
                NULL,
                0,
                NULL,
                NULL
            );
            handleBitRules(p, dt_ms);
        }
        //-----------------------------------------
        // Delay until next bit needs to be output
        //-----------------------------------------
        // cycle, dt_ms: 0, 1 ms; 1, 2 ms; 2, 4 ms; 3, 8 ms; repeat
        dt_ms = (1 << cycle);
        vTaskDelayUntil(&xLastWakeTime, dt_ms);
        cycle++;
        if (cycle >= N_BIT_PWM) {
            cycle = 0;
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
    for (i=0; i<OUT_WRITER_LIST_LEN; i++) {
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
    REPORT_ERROR("ER:0004\n");
}
