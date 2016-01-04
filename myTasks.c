/*
 * tasks.c
 *
 * FreeRTOS tasks running in parallel
 * Here is most of the functionality of the pinball controller
 *
 *  Created on: Dec 22, 2015
 *      Author: michael
 */

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/debug.h"
#include "drivers/pinout.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "sensorlib/i2cm_drv.h"
#include "driverlib/sysctl.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// TivaWare includes
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "utils/cmdline.h"

// USB stuff
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "myTasks.h"
#include "i2cHandlerTask.h"

// Global vars
t_PCLOutputByte g_outWriterList[OUT_WRITER_LIST_LEN];   // A list to keep track of the state of all output pins
t_spiTransferState g_spiState[3];
TaskHandle_t g_customI2cTask = NULL;
SemaphoreHandle_t g_MutexCustomI2C = NULL;
uint16_t g_customI2CnBytesRx;
uint8_t g_customI2CrxBuffer[CUSTOM_I2C_BUF_LEN];
uint8_t g_customI2Cstate;


const uint16_t g_ssi_lut[16] = {                        // Encodes a 4 bit nibble to a 16 bit SPI word
    0b1000100010001000,     //0                         // The SPI modules sends MSB first!
    0b1000100010001100,     //1
    0b1000100011001000,     //2
    0b1000100011001100,     //3
    0b1000110010001000,     //4
    0b1000110010001100,     //5
    0b1000110011001000,     //6
    0b1000110011001100,     //7
    0b1100100010001000,     //8
    0b1100100010001100,     //9
    0b1100100011001000,     //A
    0b1100100011001100,     //B
    0b1100110010001000,     //C
    0b1100110010001100,     //D
    0b1100110011001000,     //E
    0b1100110011001100,     //F
};

//-----------------------
// Command parser
//-----------------------
// This is the table that holds the command names, implementing functions, and brief description.
tCmdLineEntry g_psCmdTable[] = {
        { "?",     Cmd_help, ": Display list of commands" },
        { "*IDN?", Cmd_IDN,  ": Display ID and version info" },
        { "SW?",   Cmd_SW,   ": Return the state of ALL switches (40 bytes)" },
        { "OUT",   Cmd_OUT,  ": OUT <hwIndex> <tPulse> <PWMhigh> <PWMlow>\nOUT   : OUT <hwIndex> <PWMvalue>" },
        { "RUL",   Cmd_RUL,  ": RUL <ID> <IDin> <IDout> <trHoldOff> <tPulse>\n        <pwmOn> <pwmOff> <bPosEdge> <bAutoOff> <bLevelTr>" },
        { "RULE",  Cmd_RULE, ": Enable  a previously disabled rule: RULE <ID>" },
        { "RULD",  Cmd_RULD, ": Disable a previously defined rule:  RULD <ID>" },
        { "LED",   Cmd_LED,  ": LED <channel> <nBytes>\\n<binary blob of nBytes>" },
        { "I2C",   Cmd_I2C,  ": I2C <channel> <I2Caddr> <sendData> <nBytesRx>" },
        { 0, 0, 0 } };

uint16_t strMyStrip(uint8_t *cmdString, uint16_t cmdLen) {
    //Remove \n, \r and make sure there is a \0 at the end
    uint8_t *pos = cmdString;
    uint16_t i;
    for (i = 0; i < cmdLen; i++) {
        if (*pos == '\n' || *pos == '\r' || *pos == '\0') {
            *pos = '\0';
            return (i);
        }
        pos++;
    }
    *pos = '\0';
    return (i);
}

void taskDemoLED(void *pvParameters) {
// Flash the LEDs on the launchpad
    UARTprintf("%22s: %s", "taskDemoLED()", "Started!\n");
    while (1) {
        // Turn on LED 1
        LEDWrite(0x0F, 0x01);
        vTaskDelay(300);
        // Turn on LED 2
        LEDWrite(0x0F, 0x02);
        vTaskDelay(300);
        // Turn on LED 3
        LEDWrite(0x0F, 0x04);
        vTaskDelay(300);
        // Turn on LED 4
        LEDWrite(0x0F, 0x08);
        vTaskDelay(300);
    }
}

void taskUsbCommandParser( void *pvParameters ) {
    // Read data from USB serial and parse it
    UARTprintf("%22s: %s", "taskUsbCommandParser()", "Started!\n");
    static uint8_t charBuffer[CMD_PARSER_BUF_LEN];
//    uint32_t minStackSpace;
    uint32_t nCharsRead=0, tempCharsRead;
    int retVal;
    uint8_t *writePointer = charBuffer;
    while (1) {
        if (ulTaskNotifyTake( pdTRUE, portMAX_DELAY)) {    // Wait for receiving new serial data over USB
            while ( USBBufferDataAvailable(&g_sRxBuffer) ) {
                tempCharsRead = USBBufferRead(&g_sRxBuffer, writePointer, CMD_PARSER_BUF_LEN-nCharsRead-1 );
                writePointer += tempCharsRead-1;           //Points to last character now
                nCharsRead += tempCharsRead;
                if( nCharsRead>=(CMD_PARSER_BUF_LEN-1) || *writePointer=='\n' || *writePointer=='\r' || *writePointer=='\0' ){
                    retVal = CmdLineProcess( (char*)charBuffer, nCharsRead );
                    switch (retVal) {
                     case CMDLINE_BAD_CMD:
                         UARTprintf("[CMDLINE_BAD_CMD] %s\n", charBuffer);
                         break;
                     case CMDLINE_INVALID_ARG:
                         UARTprintf("[CMDLINE_INVALID_ARG] %s\n", charBuffer);
                         break;
                     case CMDLINE_TOO_FEW_ARGS:
                         UARTprintf("[CMDLINE_TOO_FEW_ARGS] %s\n", charBuffer);
                         break;
                     case CMDLINE_TOO_MANY_ARGS:
                         UARTprintf("[CMDLINE_TOO_MANY_ARGS] %s\n", charBuffer);
                         break;
                    }
                    nCharsRead = 0;
                    writePointer = charBuffer;
                } else {
                    writePointer++;                     //Points to first free position in charBuffer now
                }
                ASSERT(nCharsRead < CMD_PARSER_BUF_LEN);// Check for buffer overflow due to too long command
            }
//            minStackSpace = uxTaskGetStackHighWaterMark( NULL );
//            UARTprintf("taskUsbCommandParser(): Min Stack Space = %d words\n", minStackSpace);
        }
    }
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

void handleBitRules(t_PCLOutputByte *outListPtr, uint8_t dt) {
    // Handle the switchover from `Pulsed` state to `unpulsed` state for each output pin
    uint8_t i;
    t_BitModifyRules *bitRules = outListPtr->bitRules;
    for (i = 0; i <= 7; i++) {
        if (bitRules->tPulse > 0) {                                    //Is the entry valid?
            bitRules->tPulse -= dt;                                    //Apply dt timestep
            if (bitRules->tPulse <= 0) {                               //Did the countdown expire?
                setBcm(outListPtr->bcmBuffer, i, bitRules->lowPWM);    //Then apply the pulse_low bcm pattern
            }
        }
        bitRules++;
    }
}

void taskPCLOutWriter(void *pvParameters) {
    // Dispatch I2C write commands to PCL GPIO extenders every 1 ms
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
            } else {
                //A valid item, simply output the current bcm buffer value over I2C
                ts_i2cTransfer(outListPtr->i2cChannel, outListPtr->i2cAddress,
                        &outListPtr->bcmBuffer[bcmCycleCounter], 1, NULL, 0,
                        NULL, NULL);
                handleBitRules(outListPtr, lastTickCount);
            }
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

void setPclOutput(t_outputBit outLocation, int16_t tPulse, uint8_t highPower, uint8_t lowPower) {
// Set the power level and pulse settings of an output pin
//    tPulse    = duration of the pulse [ms]
//    highPower = PWM value during the pulse
//    lowPower  = PWM value after  the pulse
    uint8_t i;
    t_PCLOutputByte *outListPtr = g_outWriterList;
    t_BitModifyRules *bitRules;
    if (outLocation.hwIndexType != HW_INDEX_I2C)
        return;
    for (i = 0; i < OUT_WRITER_LIST_LEN; i++) {
        if (outListPtr->i2cChannel == -1) {
//            Create new entry!
            bitRules = &outListPtr->bitRules[outLocation.pinIndex];
            bitRules->tPulse = tPulse;
            bitRules->lowPWM = lowPower;
            outListPtr->i2cAddress = outLocation.i2cAddress;
            setBcm(outListPtr->bcmBuffer, outLocation.pinIndex, highPower);
            outListPtr->i2cChannel = outLocation.i2cChannel;//Mark the item as valid to the output routine
            return;
        } else if (outListPtr->i2cChannel == outLocation.i2cChannel
                && outListPtr->i2cAddress == outLocation.i2cAddress) {
//            Found the right byte, change it
            bitRules = &outListPtr->bitRules[outLocation.pinIndex];
            bitRules->lowPWM = lowPower;
            setBcm(outListPtr->bcmBuffer, outLocation.pinIndex, highPower);
            bitRules->tPulse = tPulse;
            return;
        }
        outListPtr++;
    }
//    Error, no more space in outList :(
    UARTprintf("%22s: Error, no more space in g_outWriterList :(\n",
            "setPclOutput()");
}

void ts_usbSend(uint8_t *data, uint16_t len) {
//    Do a thread safe USB TX transfer in background (add data to USB send buffer)
    uint32_t freeSpace;
    taskENTER_CRITICAL();
    freeSpace = USBBufferSpaceAvailable(&g_sTxBuffer);
//        UARTprintf( "ts_usbSend(): TX buffer %d bytes free.\n", freeSpace );
    if (freeSpace >= len) {
        USBBufferWrite(&g_sTxBuffer, data, len);
    } else {
        UARTprintf("%22s: Not enough space in USB TX buffer! Need %d have %d\n",
                "ts_usbSend()", len, freeSpace);
    }
    taskEXIT_CRITICAL();
}

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
        tempResult.i2cChannel = i2cCh;//I2C address, each channel has address 0x40 - 0x47
        tempResult.i2cAddress = (tempResult.byteIndex - 8) % 8 + 0x40;
        return tempResult;
    }
    tempResult.hwIndexType = HW_INDEX_INVALID;
    return tempResult;
}

// This function implements the "help" command.  It prints a simple list of the available commands with a brief description.
int Cmd_help(uint16_t nMax, int argc, char *argv[]) {
    tCmdLineEntry *pEntry;
    UARTprintf("\nAvailable commands\n");
    UARTprintf("------------------\n");
    pEntry = &g_psCmdTable[0];    // Point at the beginning of the command table.
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    while (pEntry->pcCmd) {
        UARTprintf("%6s%s\n", pEntry->pcCmd, pEntry->pcHelp);// Print the command name and the brief description.
        pEntry++;                    // Advance to the next entry in the table.
    }
    return (0);                                                // Return success.
}

int Cmd_IDN(uint16_t nMax, int argc, char *argv[]) {
    UARTprintf( VERSION_INFO);
    return (0);
}

int Cmd_SW(uint16_t nMax, int argc, char *argv[]) {
    // Report state of all switches
    static char outBuffer[REPORT_SWITCH_BUF_SIZE];
    uint16_t charsWritten = 3;
    uint8_t i;
    ustrncpy(outBuffer, "SW:", REPORT_SWITCH_BUF_SIZE); //SW = Hex coded switch state
    for (i = 0; i < N_LONGS; i++) {
        charsWritten += usnprintf(&outBuffer[charsWritten],
                REPORT_SWITCH_BUF_SIZE - charsWritten, "%08x ",
                g_SwitchStateDebounced.longValues[i]);
        if (charsWritten >= REPORT_SWITCH_BUF_SIZE - 10) {
            UARTprintf("Cmd_SW(): string buffer overflow!\n");
            return (0);
        }
    }
    outBuffer[charsWritten - 1] = '\n';
    outBuffer[charsWritten] = '\r';
    ts_usbSend((uint8_t*) outBuffer, charsWritten + 1);
    return (0);
}

int Cmd_OUT(uint16_t nMax, int argc, char *argv[]) {
//    OUT <hwIndex> <PWMlow> <tPulse> <PWMhigh>   or OUT <hwIndex> <PWMvalue>
//    OUT 0x0FE 1 1500 15
//    OUT 0x0FE 2
    int32_t hwIndex;
    t_outputBit outLocation;
    uint16_t tPulse, pwmHigh, pwmLow;
    if (argc == 3 || argc == 5) {
        hwIndex = ustrtoul(argv[1], NULL, 0);
        if (argc == 5) {
            tPulse = ustrtoul(argv[3], NULL, 0);
            pwmHigh = ustrtoul(argv[4], NULL, 0);
            pwmLow = ustrtoul(argv[2], NULL, 0);
        } else if (argc == 3) {
            tPulse = 0;
            pwmLow = ustrtoul(argv[2], NULL, 0);
            pwmHigh = pwmLow;
        }
        if (pwmHigh >= (1 << N_BIT_PWM) || pwmLow >= (1 << N_BIT_PWM)) {
            UARTprintf("Cmd_OUT(): PWMvalue must be < %d\n", (1 << N_BIT_PWM));
            return 0;
        }
        outLocation = decodeHwIndex(hwIndex);
        if (outLocation.hwIndexType == HW_INDEX_I2C) {
            UARTprintf(
                    "Cmd_OUT(): i2cCh %d, i2cAdr 0x%02x, bit %d = tp %d, pH %d, pL %d\n",
                    outLocation.i2cChannel, outLocation.i2cAddress,
                    outLocation.pinIndex, tPulse, pwmHigh, pwmLow);
            setPclOutput(outLocation, tPulse, pwmHigh, pwmLow);
            return (0);
        } else if (outLocation.hwIndexType == HW_INDEX_SWM) {
            UARTprintf("Cmd_OUT(): hwIndex=%d is a SM input\n", hwIndex);
            return (0);
        } else if (outLocation.hwIndexType == HW_INDEX_INVALID) {
            UARTprintf("Cmd_OUT(): HW_INDEX_INVALID: %s\n", argv[1]);
            return (0);
        }
    }
    return ( CMDLINE_TOO_FEW_ARGS);
}

int Cmd_RULE(uint16_t nMax, int argc, char *argv[]) {
    //Enable a quickfire rule
    uint8_t id;
    if (argc == 2) {
        id = ustrtoul(argv[1], NULL, 0);
        if (id >= MAX_QUICK_RULES) {
            UARTprintf("%22s: quickRuleId must be < %d\n", "Cmd_RULE()",
                    MAX_QUICK_RULES);
            return 0;
        }
        enableQuickRule(id);
        return (0);
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_RULD(uint16_t nMax, int argc, char *argv[]) {
    //Disable a quickfire rule
    uint8_t id;
    if (argc == 2) {
        id = ustrtoul(argv[1], NULL, 0);
        if (id >= MAX_QUICK_RULES) {
            UARTprintf("%22s: quickRuleId must be < %d\n", "Cmd_RULD()",
                    MAX_QUICK_RULES);
            return 0;
        }
        disableQuickRule(id);
        return (0);
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_RUL(uint16_t nMax, int argc, char *argv[]) {
// Configure and activate a Quick-fire rule:
//  * quickRuleId (0-64)
//  * input switch ID number
//  * driver output ID number
//  * post trigger hold-off time [ms]
//  * pulse duration [ms]
//  * pulse pwm [only for output ID 0-3 which are the pwm channels]
//  * hold pwm  [only for output ID 0-3 which are the pwm channels]
//  * Enable trigger on pos edge?
//  * Enable auto. output off once input releases
//  * Enable level Trigger (no edge check)
//  ------------------
//   Example command:
//  ------------------
//  RUL ID IDin IDout trHoldOff tPulse pwmOn pwmOff bPosEdge bAutoOff bLevelTr
//  RUL 0 0x23 0x100 4 1 15 3 1 0 0
    uint8_t id, pwmHigh, pwmLow;
    uint16_t triggerHoldOffTime, tPulse;
    t_outputBit inputSwitchId, outputDriverId;
    bool trigPosEdge, outOffOnRelease, levelTriggered;
    if (argc == 11) {
        id = ustrtoul(argv[1], NULL, 0);
        if (id >= MAX_QUICK_RULES) {
            UARTprintf("%22s: quickRuleId must be < %d\n", "Cmd_RUL()",
                    MAX_QUICK_RULES);
            return 0;
        }
        inputSwitchId = decodeHwIndex(ustrtoul(argv[2], NULL, 0));
        if (inputSwitchId.hwIndexType == HW_INDEX_INVALID) {
            UARTprintf("%22s: inputSwitchId = %s invalid\n", "Cmd_RUL()",
                    argv[2]);
            return 0;
        }
        outputDriverId = decodeHwIndex(ustrtoul(argv[3], NULL, 0));
        if (outputDriverId.hwIndexType == HW_INDEX_INVALID
                || outputDriverId.hwIndexType == HW_INDEX_SWM) {
            UARTprintf("%22s: outputDriverId = %s invalid\n", "Cmd_RUL()",
                    argv[3]);
            return 0;
        }
        triggerHoldOffTime = ustrtoul(argv[4], NULL, 0);
        tPulse = ustrtoul(argv[5], NULL, 0);
        pwmHigh = ustrtoul(argv[6], NULL, 0);
        pwmLow = ustrtoul(argv[7], NULL, 0);
        if (pwmHigh >= (1 << N_BIT_PWM) || pwmLow >= (1 << N_BIT_PWM)) {
            UARTprintf("%22s: pwmValues must be < %d\n", "Cmd_RUL()",
                    (1 << N_BIT_PWM));
            return 0;
        }
        trigPosEdge = ustrtoul(argv[8], NULL, 0) == 1;
        outOffOnRelease = ustrtoul(argv[9], NULL, 0) == 1;
        levelTriggered = ustrtoul(argv[10], NULL, 0) == 1;
        UARTprintf("%22s: Setting up autofiring rule %d\n", "Cmd_RUL()", id);
        setupQuickRule(id, inputSwitchId, outputDriverId, triggerHoldOffTime,
                tPulse, pwmHigh, pwmLow, trigPosEdge, outOffOnRelease,
                levelTriggered);
        return (0);
    }
    return CMDLINE_TOO_FEW_ARGS;
}

uint8_t g_spiBuffer[3][N_LEDS_MAX*3];   //3 channels * 3 colors --> 9.2 kByte

int Cmd_LED(uint16_t nMax, int argc, char *argv[]) {    //nMax = number of all received characters so far
    //LED 0 128\nxxxxxx
    uint8_t channel, *pWrite, *pRead;
    uint32_t blobSize, blobReceived, tempReceived;
    if( argc==3 ){
        channel = ustrtoul(argv[1], NULL, 0);
        if(  channel <= 2 ){
            blobSize = ustrtoul(argv[2], NULL, 0);
            if( blobSize%3 ){
                UARTprintf("%22s: Invalid number of bytes (%d)\n", "Cmd_LED()", blobSize);
                return 0;
            }
            blobReceived = nMax - ustrlen(argv[0]) - ustrlen(argv[1]) - ustrlen(argv[2]) - 3;
            pWrite = (uint8_t*)&g_spiBuffer[channel];
            pRead = (uint8_t*)argv[3];
            memcpy( pWrite, pRead, blobReceived );
            pWrite += blobReceived;
            while( blobReceived < blobSize ){
                tempReceived = USBBufferRead( &g_sRxBuffer, pWrite, blobSize-blobReceived );
                pWrite += tempReceived;
                blobReceived += tempReceived;
            }
            UARTprintf("Blasting %d bytes of data to LED string on channel %d\n", blobSize, channel);
            spiSend( channel, blobSize );
        }
    } else {
        UARTprintf("%22s: Invalid number of arguments (%d)\n", "Cmd_LED()", argc);
    }
    return( 0 );
}

void spiHwSetup( uint8_t channel, uint32_t ssin_base, uint8_t intNo ){
    ROM_SSIDisable( ssin_base );
    // USer internal 120 MHz clock
    ROM_SSIClockSourceSet( ssin_base, SSI_CLOCK_SYSTEM );
    // SPI at 3.2 MHz, 16 bit words (encoding 4 bit data)
    ROM_SSIConfigSetExpClk( ssin_base, SYSTEM_CLOCK, SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 3200000, 16 );
    // Enable it
    ROM_SSIEnable( ssin_base );
    g_spiState[channel].baseAdr = ssin_base;
    g_spiState[channel].intNo = intNo;
    g_spiState[channel].semaToReleaseWhenFinished = xSemaphoreCreateBinary();
    xSemaphoreGive( g_spiState[channel].semaToReleaseWhenFinished );
}

void spiSetup(){
    // Enable SPI modules
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI1 );
    ROM_SysCtlPeripheralReset(  SYSCTL_PERIPH_SSI1 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI2 );
    ROM_SysCtlPeripheralReset(  SYSCTL_PERIPH_SSI2 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI3 );
    ROM_SysCtlPeripheralReset(  SYSCTL_PERIPH_SSI3 );
    // Select the pinout
    ROM_GPIOPinConfigure( GPIO_PE4_SSI1XDAT0 );
    ROM_GPIOPinConfigure( GPIO_PD1_SSI2XDAT0 );
    ROM_GPIOPinConfigure( GPIO_PQ2_SSI3XDAT0 );
    ROM_GPIOPinTypeSSI(   GPIO_PORTE_BASE, GPIO_PIN_4 );
    ROM_GPIOPinTypeSSI(   GPIO_PORTD_BASE, GPIO_PIN_1 );
    ROM_GPIOPinTypeSSI(   GPIO_PORTQ_BASE, GPIO_PIN_2 );
    // Setup SPI modules
    spiHwSetup( 0, SSI1_BASE, INT_SSI1 );
    spiHwSetup( 1, SSI2_BASE, INT_SSI2 );
    spiHwSetup( 2, SSI3_BASE, INT_SSI3 );
}

void spiISR( uint8_t channel ){         //FIFO got 8 positions, we get notified when it is half full or less
    uint8_t temp, nib, retVal;          //This will come back every 20 us or 2000 ticks
    uint32_t status;//,ticks;           //as long as there is data to be sent
    t_spiTransferState *state = &g_spiState[channel];
//    ticks = stopTimer();
    status = ROM_SSIIntStatus(state->baseAdr, true);
    if( status & SSI_TXFF ){
        temp = *state->currentByte;
        while( state->nBytesLeft > 0 ){
            if( state->doFirstNibbel ){     // Do first nibble
                nib = (temp & 0xF0) >> 4;   // MSB nibble
            } else {
                nib = temp & 0x0F;          // LSB nibble
            }
            retVal = ROM_SSIDataPutNonBlocking( state->baseAdr, g_ssi_lut[nib] );
            if( retVal ){                   //Success
                if( state->doFirstNibbel ){
                    state->doFirstNibbel = false;
                } else {
                    state->doFirstNibbel = true;
                    state->currentByte++;
                    temp = *state->currentByte;
                    state->nBytesLeft--;
                }
            } else {                        //Didn't work. TX FIFO is full!
//                startTimer();
                return;                     //Exit the interrupt and continue sending in next one
            }
        }
//        UARTprintf("%22s: %d Ticks between ISRs\n", "spiISR()", ticks);
        ROM_SSIIntDisable( state->baseAdr, SSI_TXFF );  //Disable FIFO is less than halffull int
        ROM_SSIIntEnable(  state->baseAdr, SSI_TXEOT ); //ENable end of transmission int.
        if( !SSIBusy(state->baseAdr) ){                 //Check if finished already
            ROM_SSIIntDisable( state->baseAdr, SSI_TXEOT );
            ROM_IntDisable( state->intNo );
            xSemaphoreGiveFromISR( state->semaToReleaseWhenFinished, NULL );
            return;
        }
        // Wait for SPI to empty the FIFO and trigger the SSI_TXEOT interrupt
    }
    if ( (state->nBytesLeft==0) && (status&SSI_TXEOT) ) {
        // We are finished sending, now we should keep TX low for > 50 us to latch the LEDs
        // But the SPI hardware does tristate that pin, shall we reconfigure it as gpio and set it low?
        ROM_SSIIntClear( state->baseAdr, SSI_TXEOT );
        ROM_SSIIntDisable( state->baseAdr, SSI_TXEOT );
        ROM_IntDisable( state->intNo );
        xSemaphoreGiveFromISR( state->semaToReleaseWhenFinished, NULL );
        return;
    }
}

void spiSend( uint8_t channel, uint32_t nBytes ){
    t_spiTransferState *state = &g_spiState[channel];
    if( xSemaphoreTake( state->semaToReleaseWhenFinished, 1000 ) ){
        state->currentByte = g_spiBuffer[channel];
        state->doFirstNibbel = true;
        state->nBytesLeft = nBytes;
//      Enable and Trigger SPI TX buffer empty interrupt
        ROM_IntEnable( state->intNo );
        ROM_SSIIntEnable( state->baseAdr, SSI_TXFF );  //Enable FIFO is less than halffull int
        // It will jump to the ISR and start sending data immediately
    } else {
        UARTprintf("%22s: Timeout, could not access sendBuffer %d\n", "spiSend()", channel);
    }
}

uint8_t hexDigitToNibble( uint8_t hexChar ){
    if( hexChar >= '0' && hexChar <= '9' ){
        return( hexChar - '0' );
    } else if ( hexChar >= 'a' && hexChar <= 'f' ){
        return( hexChar - 'a' + 10 );
    } else if ( hexChar >= 'A' && hexChar <= 'F' ){
        return( hexChar - 'A' + 10 );
    }
    return( 0 );
}

void taskI2CCustomReporter(void *pvParameters) {
    // When a custom I2C transaction is finished, report the result to commandline
    static uint8_t outBuffer[CUSTOM_I2C_BUF_LEN*2+5];
    uint8_t *writePointer, *readPointer;
    uint16_t i;
    g_customI2cTask = xTaskGetCurrentTaskHandle();
    g_MutexCustomI2C = xSemaphoreCreateMutex();
    UARTprintf("%22s: %s", "taskI2CustomReporter()", "Started!\n");
    while( 1 ){
        if( ulTaskNotifyTake( pdTRUE, portMAX_DELAY) ) {    // Wait for notification of i2c transaction finished
            writePointer = outBuffer;
            readPointer = g_customI2CrxBuffer;
            // Report result
            writePointer += usprintf( (char*)writePointer, "I2C:" );
            switch( g_customI2Cstate ){
            case I2CM_STATUS_SUCCESS:
                for( i=0; i<g_customI2CnBytesRx; i++ ){
                    writePointer += usprintf( (char*)writePointer, "%02X", *readPointer++ );
                }
            break;
            case I2CM_STATUS_ADDR_NACK:
                writePointer += usprintf( (char*)writePointer, "I2CM_STATUS_ADDR_NACK" );
            break;
            case I2CM_STATUS_DATA_NACK:
                writePointer += usprintf( (char*)writePointer, "I2CM_STATUS_DATA_NACK" );
            break;
            case I2CM_STATUS_ARB_LOST:
                writePointer += usprintf( (char*)writePointer, "I2CM_STATUS_ARB_LOST" );
            break;
            case I2CM_STATUS_ERROR:
                writePointer += usprintf( (char*)writePointer, "I2CM_STATUS_ERROR" );
            break;
            }
            writePointer += usprintf( (char*)writePointer, "\n\r" );
            ts_usbSend( outBuffer, ustrlen((char*)outBuffer) );
            xSemaphoreGive( g_MutexCustomI2C ); // release the Mutex to allow another custom I2C command
        }
    }
}

void cmdI2CCallback(void* pvCallbackData, uint_fast8_t ui8Status){
//   This is called from ISR context, so dont do anything ambitious here
//   Notify taskI2CCustomReporter task to report results
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    g_customI2Cstate = ui8Status;
    vTaskNotifyGiveFromISR( g_customI2cTask, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int Cmd_I2C(uint16_t nMax, int argc, char *argv[]) {
    //I2C <channel> <I2Caddr> <sendData> <nBytesRx>
    static uint8_t txBuffer[CUSTOM_I2C_BUF_LEN];
    uint8_t channel, i2cAddr, *readPointer, *writePointer;
    uint16_t nBytesTx, temp;
    if ( argc == 5 ){
        channel  = ustrtoul(argv[1], NULL, 0);
        i2cAddr  = ustrtoul(argv[2], NULL, 0);
        g_customI2CnBytesRx = ustrtoul(argv[4], NULL, 0);
        nBytesTx = ustrlen( argv[3] )/2;      //argv[3] string contains hex characters [0FFEDEADBEEF]
        writePointer = txBuffer;
        readPointer = (uint8_t*)argv[3];
        if( g_customI2CnBytesRx > CUSTOM_I2C_BUF_LEN ){
            UARTprintf("%22s: Too many bytes to receive: %d, max. %d\n", "Cmd_I2C()", g_customI2CnBytesRx, CUSTOM_I2C_BUF_LEN);
            return 0;
        }
        if( nBytesTx > CUSTOM_I2C_BUF_LEN ){
            UARTprintf("%22s: Too many bytes to send: %d, max. %d\n", "Cmd_I2C()", nBytesTx, CUSTOM_I2C_BUF_LEN);
            return 0;
        }
        for( temp=0; temp<nBytesTx; temp++ ){
            *writePointer  = hexDigitToNibble( *readPointer++ )<<4;
            *writePointer |= hexDigitToNibble( *readPointer++ );
            writePointer++;
        }
        // Take the binary semaphore g_semaCustomI2C (released after reporting result on USB)
        if( xSemaphoreTake( g_MutexCustomI2C, 3000 ) ){
            ts_i2cTransfer( channel, i2cAddr, txBuffer, nBytesTx, g_customI2CrxBuffer, g_customI2CnBytesRx, cmdI2CCallback, NULL );
        } else {
            UARTprintf("%22s: Timeout, could not acquire custom I2C mutex\n", "Cmd_I2C()");
        }
        return 0;
    }
    return CMDLINE_TOO_FEW_ARGS;
}
