/*
 * FreeRTOS tasks running in parallel (`threads`)
 * This file mostly contains Functions for the communicating
 * with the host PC, which includes parsing and executing commands.
 */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
// TivaWare includes
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "utils/ustdlib.h"
#include "utils/cmdline.h"
#include "my_uartstdio.h"
// USB stuff
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "drivers/usb_serial_structs.h"
#include "myTasks.h"
#include "mySpi.h"
#include "quick_rules.h"

//-------------------
// Global vars
//-------------------
bool g_reportSwitchEvents = 0;
uint8_t g_errorBuffer[8];

//-------------------
// cmd line commands
//-------------------
int Cmd_help(int argc, char *argv[]);
int Cmd_IDN(int argc, char *argv[]);
int Cmd_IL(int argc, char *argv[]);
int Cmd_OL(int argc, char *argv[]);
int Cmd_IR(int argc, char *argv[]);
int Cmd_SW(int argc, char *argv[]);
int Cmd_SWE(int argc, char *argv[]);
int Cmd_DEB(int argc, char *argv[]);
int Cmd_SOE(int argc, char *argv[]);
int Cmd_OUT(int argc, char *argv[]);
int Cmd_RUL(int argc, char *argv[]);
int Cmd_RULE(int argc, char *argv[]);
int Cmd_LEC(int argc, char *argv[]);
int Cmd_LED(int argc, char *argv[]);
int Cmd_I2C(int argc, char *argv[]);
int Cmd_HI(int argc, char *argv[]);

// This is the table that holds the command names,
// implementing functions, and brief description.
tCmdLineEntry g_psCmdTable[] = {
        {"?",     Cmd_help, ": Display list of commands"},
        {"*IDN?", Cmd_IDN,  ": Display ID and version info"},
        {"IL",    Cmd_IL,   ": I2C: List status of GPIO expanders"},
        {"IR",    Cmd_IR,   ": I2C: Reset I2C system"},
        {"OL",    Cmd_OL,   ": I2C: List output writers"},
        {"HI",    Cmd_HI,   ": <hwIndex> set all ports of PCF high (input mode)"},
        {"SWE",   Cmd_SWE,  ": <OnOff> En./Dis. reporting of switch events"},
        {"DEB",   Cmd_DEB,  ": <hwIndex> <OnOff> En./Dis. 12 ms debouncing"},
        {"SW?",   Cmd_SW,   ": Return the state of ALL switches (40 bytes)"},
        {"SOE",   Cmd_SOE,  ": <OnOff> En./Dis. 24 V solenoid power (careful!)"},
        {"OUT",   Cmd_OUT,  ": <hwIndex> <PWMlow> [tPulse] [PWMhigh]"},
        {"RUL",   Cmd_RUL,  ": <ID> <IDin> <IDout> <trHoldOff>\n        <tPulse> <pwmOn> <pwmOff> <bPosEdge>"},
        {"RULE",  Cmd_RULE, ": En./Dis a prev. def. rule: RULE <ID> <OnOff>"},
        {"LEC",   Cmd_LEC,  ": <channel> <spiSpeed [Hz]> [frameFmt]"},
        {"LED",   Cmd_LED,  ": <channel> <nBytes>\\n<binary blob of nBytes>"},
        {"I2C",   Cmd_I2C,  ": <channel> <I2Caddr> [hexSendData] <nBytesRx>"},
        {NULL, NULL, NULL}
};

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
    return i;
}

void taskDemoLED(void *pvParameters) {
    // Flash the LEDs on the launchpad
    // Set up the UART which is connected to the virtual debugging COM port
    UARTStdioConfig(0, 115200, SYSTEM_CLOCK);
    UARTprintf("\n\n\n\n"
            "**************************************************\n"
            " Hi, here's the brain of Fan-Tas-Tic Pinball \n"
            "**************************************************\n");
    UARTprintf("%22s: %s", "taskDemoLED()", "Started!\n");
    vTaskDelay(1000);
    UARTprintf("Press any key to enable debug messages ... ");
    globalDebugEnabled = 0; // Disables output to UART in ./utils/uartstdio.c
    vTaskDelete(NULL);
}

int8_t cmdParse( uint8_t *charBuffer, uint16_t nCharsRead ){
    if (nCharsRead == 0){   //This must be a \n, ignore silently
        return( 0 );
    }
    int8_t retVal = CmdLineProcess((char*)charBuffer); //, nCharsRead );
    switch (retVal) {
     case CMDLINE_BAD_CMD:
         REPORT_ERROR( "ER:0006\n" );
         UARTprintf("[CMDLINE_BAD_CMD] %s\n", charBuffer);
         break;
     case CMDLINE_INVALID_ARG:
         REPORT_ERROR( "ER:0007\n" );
         UARTprintf("[CMDLINE_INVALID_ARG] %s\n", charBuffer);
         break;
     case CMDLINE_TOO_FEW_ARGS:
         REPORT_ERROR( "ER:0008\n" );
         UARTprintf("[CMDLINE_TOO_FEW_ARGS] %s\n", charBuffer);
         break;
     case CMDLINE_TOO_MANY_ARGS:
         REPORT_ERROR( "ER:0009\n" );
         UARTprintf("[CMDLINE_TOO_MANY_ARGS] %s\n", charBuffer);
         break;
    }
    return retVal;
}

int16_t eolSearch( uint8_t *charPointer, uint16_t nChars ){
//  Return -1 if no eol has been found, otherwise return offset to eol
    uint16_t i;
    for( i=0; i<nChars; i++ ){          // Find the end of line in received chars
        if( *charPointer=='\n' || *charPointer=='\r' || *charPointer=='\0' ){
            *charPointer = '\0';        //Replace EOL with \0
            return i;
        }
        charPointer++;
    }
    return -1;
}

typedef enum{
    PARS_MODE_ASCII, PARS_MODE_BIN_LED
}t_usbParserMode;

uint32_t g_LEDnBytesToCopy;
int8_t g_LEDChannel;

void taskUsbCommandParser( void *pvParameters ) {
    // Read data from USB serial and parse it
    UARTprintf("%22s: %s", "taskUsbCommandParser()", "Started!\n");
    static uint8_t charBuffer[CMD_PARSER_BUF_LEN];
    uint32_t nCharsRead=0, tempCharsRead=0, remainderSize=0;
    int16_t retVal;
    uint8_t *writePointer = charBuffer;                         // Points to first free place in charBuffer
    uint8_t *readPointer = charBuffer;                          // Points to the next unprocessed character
    uint8_t *spiWritePointer;                                   // Points to first free place in spiBuffer
    t_usbParserMode currentMode = PARS_MODE_ASCII;
    while (1) {
        switch( currentMode ){
        case PARS_MODE_ASCII:
            // -----------------------------------------------------------
            //  Check if there is any data in the buffer to process
            // -----------------------------------------------------------
            if( remainderSize == 0 ){                           // There's no unprocessed data in the buffer
                while( !USBBufferDataAvailable(&g_sRxBuffer) ){
                    ulTaskNotifyTake( pdTRUE, portMAX_DELAY);   // Wait for receiving new serial data over USB
                }                                               // Here we must have new data in any case
                tempCharsRead = USBBufferRead(&g_sRxBuffer, writePointer, CMD_PARSER_BUF_LEN-nCharsRead-1 );
//                UARTwrite( writePointer, tempCharsRead );
                writePointer += tempCharsRead;
                remainderSize += tempCharsRead;                 // How many chars to process
            }
            // -----------------------------------------------------------
            //  Wait for a EOL character, then parse the substring
            // -----------------------------------------------------------
            retVal = eolSearch( readPointer, remainderSize );
            if( retVal > -1 ){                                              //EOL was found in new chars
                readPointer += retVal + 1;                                  //Points to the first remainder char after the found EOL char
                remainderSize -= retVal + 1;                                //Length of the remainder after the CMD
                //Parse cmd from beginning of charBuffer
                if( cmdParse( charBuffer, nCharsRead + retVal ) == PARS_MODE_BIN_LED ){
                    // cmdParse might enable PARS_MODE_BIN_LED
                    // cmdParse calls CMD_LED, which will wait until a previous send on this channel is finished
                    // -----------------------------------------------------------
                    //  Switch to binary input mode
                    // -----------------------------------------------------------
                    currentMode = PARS_MODE_BIN_LED;
                    //Okay we need to copy the remainder from the charBuffer into the SPI buffer
                    nCharsRead = MIN(remainderSize, g_LEDnBytesToCopy);      //How many chars to copy? remainder might contain more commands!
                    memcpy( g_spiBuffer[ g_LEDChannel ], readPointer, nCharsRead );
                    spiWritePointer= &g_spiBuffer[g_LEDChannel][nCharsRead];//Points to next free char in spiBuffer
                    readPointer += nCharsRead;
                    remainderSize -= nCharsRead;                            //This is the remainder after taking the LED data out
                                                                            //Might not be zero if multiple commands are in charBuffer!
                } else {
                    nCharsRead = 0;                                         //We've processed one command, start again with the next!
                }
                // -----------------------------------------------------------
                //  Prepare parsing of the next Ascii cmd
                // -----------------------------------------------------------
                //Okay we need to copy the remainder from the charBuffer into the beginning of the charBuffer
                memcpy( charBuffer, readPointer, remainderSize );       //Remainder is now in the beginning
                // At this point, either all chars are processed (reminderSize=0)
                // Or the remaining chars have been copied to the beginning of the charBuffer
                readPointer = charBuffer;
                writePointer = &charBuffer[remainderSize];              //Point to first free char

            } else {
                // -----------------------------------------------------------
                //  Here we have processed all chars in charBuffer and haven't
                //  Found any EOL. Keep appending new stuff to it.
                //  Also Check how many chars we can still append
                // -----------------------------------------------------------
                nCharsRead += remainderSize;
                readPointer += remainderSize;
                remainderSize = 0;
                if( nCharsRead >= CMD_PARSER_BUF_LEN - 1 ){
                    REPORT_ERROR( "ER:000A\n" );
                    UARTprintf("%22s: %s", "taskUsbCommandParser()", "Command buffer overflow, try a shorter command!\n");
                    USBBufferFlush( &g_sRxBuffer );
                    writePointer = charBuffer;
                    nCharsRead = 0;
                    remainderSize = 0;
                }                                       //We still have space!
            }
            break;

        case PARS_MODE_BIN_LED:
            // -----------------------------------------------------------
            //  simply take g_LEDnBytesToCopy bytes from USB and put them
            //  in the spiBuffer
            // -----------------------------------------------------------
            if( nCharsRead < g_LEDnBytesToCopy ){       //If there was not enough data in the remainder, get more over USB
                ASSERT( remainderSize == 0 );           //Remainder must be empty before we take data from USB
                tempCharsRead = USBBufferRead(&g_sRxBuffer, spiWritePointer, g_LEDnBytesToCopy-nCharsRead );
                spiWritePointer += tempCharsRead;
                nCharsRead += tempCharsRead;
            }
            if( nCharsRead >= g_LEDnBytesToCopy ){      //Are we good already?
                spiSend(g_LEDChannel, g_LEDnBytesToCopy);
//                xSemaphoreGive( g_spiState[g_LEDChannel].semaToReleaseWhenFinished );
                nCharsRead = 0;
                currentMode = PARS_MODE_ASCII;
            }
            break;

        default:
            REPORT_ERROR("ER:000B\n");
            UARTprintf("%22s: currentMode unknown, 0x%02x\n", "taskUsbCommandParser()", currentMode);
            currentMode = PARS_MODE_ASCII;
        }
    }
}

void ts_usbSend(uint8_t *data, uint16_t len) {
//    Do a thread safe USB TX transfer in background (add data to USB send buffer)
    uint32_t freeSpace;
    taskENTER_CRITICAL();
    UARTwrite( (const char*) data, len );   //Echo to debug connection
    freeSpace = USBBufferSpaceAvailable(&g_sTxBuffer);
//        UARTprintf( "ts_usbSend(): TX buffer %d bytes free.\n", freeSpace );
    if (freeSpace >= len) {
        USBBufferWrite(&g_sTxBuffer, data, len);
    } else {
//        UARTprintf("%22s: Not enough space in USB TX buffer! Need %d have %d. <FLUSH>\n",
//                "ts_usbSend()", len, freeSpace);
        USBBufferFlush( &g_sTxBuffer );
    }
    taskEXIT_CRITICAL();
}

// This function implements the "help" command.  It prints a simple list of the available commands with a brief description.
int Cmd_help(int argc, char *argv[]) {
    tCmdLineEntry *pEntry;
    UARTprintf("\n**************************************************\n");
    UARTprintf(  " Available commands   <required>  [optional]\n");
    UARTprintf(  "**************************************************\n");
    pEntry = &g_psCmdTable[0];    // Point at the beginning of the command table.
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    while (pEntry->pcCmd) {
        UARTprintf("%6s%s\n", pEntry->pcCmd, pEntry->pcHelp);// Print the command name and the brief description.
        pEntry++;                    // Advance to the next entry in the table.
#ifdef UART_BUFFERED
        while( UARTTxBytesFree() < 150 ){
            vTaskDelay( 1 );
        }
#endif
    }
    return 0;                                                // Return success.
}

int Cmd_IDN(int argc, char *argv[]) {
    const uint8_t buff[] = VERSION_IDN;
    ts_usbSend( (uint8_t*)buff, VERSION_IDN_LEN );
    UARTprintf( VERSION_INFO );
    UARTprintf( "xPortGetFreeHeapSize(): %d\n", xPortGetFreeHeapSize() );
    return 0;
}

int Cmd_OL(int argc, char *argv[]){
    print_out_writer_list();
    return 0;
}

int Cmd_IL(int argc, char *argv[]){
    print_pcf_state();
    return 0;
}

int Cmd_IR(int argc, char *argv[]){
    UARTprintf("Reseting I2C system ... ");
    g_reDiscover = 1;
    // task might be blocked (no pullups?) ...
    xTaskNotify(hPcfInReader, 0x0F, eSetBits);
    return 0;
}

int Cmd_SW(int argc, char *argv[]) {
    // Report state of all switches
    static char outBuffer[REPORT_SWITCH_BUF_SIZE];
    uint16_t charsWritten = 3;
    uint8_t i;
    ustrncpy(outBuffer, "SW:", REPORT_SWITCH_BUF_SIZE); //SW = Hex coded switch state
    for (i = 0; i < N_LONGS; i++) {
        charsWritten += usnprintf(&outBuffer[charsWritten], REPORT_SWITCH_BUF_SIZE - charsWritten, "%08x", g_SwitchStateDebounced.longValues[i]);
        if (charsWritten >= REPORT_SWITCH_BUF_SIZE - 1) {
            REPORT_ERROR( "ER:000C\n" );
            UARTprintf("Cmd_SW(): string buffer overflow!\n");
            return 0;
        }
    }
    outBuffer[ charsWritten ] = '\n';
    ts_usbSend((uint8_t*) outBuffer, charsWritten+1 );
    return 0;
}

int Cmd_DEB(int argc, char *argv[]) {
    //Enable / Disable the 12 ms debouncing timer for an input
    uint16_t hwIndex;
    uint8_t onOff;
    t_hw_index inputSwitchId;
    if (argc == 3) {
        hwIndex = ustrtoul(argv[1], NULL, 0);
        onOff = ustrtoul(argv[2], NULL, 0);     // 1: Debouncing ON
        inputSwitchId = decodeHwIndex( hwIndex, 1 );
        if (inputSwitchId.channel == C_INVALID) {
            REPORT_ERROR( "ER:000D\n" );
            UARTprintf( "%22s: inputSwitchId = %s invalid\n", "Cmd_DEB()", argv[1] );
            return 0;
        }
        if( onOff ){
            // Reset noDebounce Flag
            HWREGBITB( &g_SwitchStateNoDebounce.charValues[inputSwitchId.byteIndex], inputSwitchId.pinIndex ) = 0;
        } else {
            // Set noDebounce Flag
            HWREGBITB( &g_SwitchStateNoDebounce.charValues[inputSwitchId.byteIndex], inputSwitchId.pinIndex ) = 1;
        }
        return 0;
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_SOE(int argc, char *argv[]){
    uint8_t onOff;
    if (argc == 2) {
        onOff = ustrtoul(argv[1], NULL, 0);
        if( onOff ){
            ENABLE_SOLENOIDS();
            UARTprintf( "Cmd_OUT(): 24 V solenoid power enabled.\n" );
            return 0;
        }else{
            DISABLE_SOLENOIDS();
            UARTprintf( "Cmd_OUT(): 24 V solenoid power disabled.\n" );
            return 0;
        }
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_OUT(int argc, char *argv[]) {
//    OUT <hwIndex> <PWMlow> <tPulse> <PWMhigh>   or OUT <hwIndex> <PWMvalue>
//    OUT 0x0FE 1 1500 15
//    OUT 0x0FE 2
    int32_t hwIndex;
    t_hw_index outLocation;
    uint16_t tPulse, pwmHigh, pwmLow;
    if (argc == 5) {
        tPulse  = ustrtoul(argv[3], NULL, 0);
        pwmHigh = ustrtoul(argv[4], NULL, 0);
        pwmLow  = ustrtoul(argv[2], NULL, 0);
    } else if (argc == 3) {
        tPulse  = 0;
        pwmLow  = ustrtoul(argv[2], NULL, 0);
        pwmHigh = pwmLow;
    } else {
        return( CMDLINE_TOO_FEW_ARGS );
    }
    hwIndex = ustrtoul(argv[1], NULL, 0);
    outLocation = decodeHwIndex(hwIndex, 0);
    switch(outLocation.channel){
    case C_I2C0:
    case C_I2C1:
    case C_I2C2:
    case C_I2C3:
        if (pwmHigh >= (1 << N_BIT_PWM) || pwmLow >= (1 << N_BIT_PWM)) {
            REPORT_ERROR( "ER:000E\n" );
            UARTprintf("Cmd_OUT(): I2C PWMvalue must be < %d\n", (1 << N_BIT_PWM));
            return 0;
        }
        UARTprintf("Cmd_OUT(): i2cCh %d, i2cAdr 0x%02x, bit %d = tp %d, pH %d, pL %d\n", outLocation.channel, outLocation.i2c_addr, outLocation.pinIndex, tPulse, pwmHigh, pwmLow);
        setPCFOutput(&outLocation, tPulse, pwmHigh, pwmLow);
        return 0;

    case C_FAST_PWM:
        if ( pwmHigh > MAX_PWM || pwmLow > MAX_PWM ) {
            REPORT_ERROR( "ER:000F\n" );
            UARTprintf("Cmd_OUT(): HW PWMvalue must be <= %d\n", MAX_PWM);
            return 0;
        }
        UARTprintf("Cmd_OUT(): HW_PWM_CH %d, tp %d, pH %d, pL %d\n", outLocation.pinIndex, tPulse, pwmHigh, pwmLow);
        setPCFOutput(&outLocation, tPulse, pwmHigh, pwmLow);
        return 0;

    case C_SWITCH_MATRIX:
        REPORT_ERROR( "ER:0010\n" );
        UARTprintf("Cmd_OUT(): HW_INDEX_INVALID: %s\n", argv[1]);
        return 0;

    case C_INVALID:
        REPORT_ERROR( "ER:0011\n" );
        UARTprintf("Cmd_OUT(): HW_INDEX_INVALID: %s\n", argv[1]);
        return 0;

    default:
        DISABLE_SOLENOIDS();
        REPORT_ERROR( "ER:0012\n" );
    }
    return 0;
}

int Cmd_SWE(int argc, char *argv[]) {
    //Enable / Disable the reporting of Switch Events
    uint8_t onOff;
    if (argc == 2) {
        onOff = ustrtoul(argv[1], NULL, 0);
        if( onOff ){
            g_reportSwitchEvents = 1;
        } else {
            g_reportSwitchEvents = 0;
        }
        return 0;
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_RULE(int argc, char *argv[]) {
    //Enable / Disable a quickfire rule
    uint8_t id, onOff;
    if (argc == 3) {
        id = ustrtoul(argv[1], NULL, 0);
        onOff = ustrtoul(argv[2], NULL, 0);
        if (id >= MAX_QUICK_RULES) {
            REPORT_ERROR( "ER:0013\n" );
            UARTprintf("%22s: quickRuleId must be < %d\n", "Cmd_RULE()",
                    MAX_QUICK_RULES);
            return 0;
        }
        if( onOff ){
            enableQuickRule(id);
        } else {
            disableQuickRule(id);
        }
        return 0;
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_RUL(int argc, char *argv[]) {
// Configure and activate a Quick-fire rule:
//  * quickRuleId (0 .. MAX_QUICK_RULES - 1)
//  * input switch hwIndex
//  * driver output hwIndex
//  * post trigger hold-off time [ms]
//  * pulse duration [ms]
//  * pulse pwm intensity
//  * hold pwm intensity
//  * Enable trigger on pos edge?
//  ------------------
//   Example command:
//  ------------------
//  RUL ID IDin IDout trHoldOff tPulse pwmOn pwmOff bPosEdge
//  RUL 0 0x23 0x100 4 1 15 3 1
    uint8_t id;
    int16_t pwmHigh, pwmLow;
    uint16_t triggerHoldOffTime, tPulse, hwIndex;
    t_hw_index inputSwitchId, outputDriverId;
    bool trigPosEdge;
    if (argc == 9) {
        id = ustrtoul(argv[1], NULL, 0);
        triggerHoldOffTime = ustrtoul(argv[4], NULL, 0);
        tPulse = ustrtoul(argv[5], NULL, 0);
        pwmHigh = ustrtoul(argv[6], NULL, 0);
        pwmLow = ustrtoul(argv[7], NULL, 0);
        trigPosEdge = ustrtoul(argv[8], NULL, 0) == 1;
        if (id >= MAX_QUICK_RULES) {
            REPORT_ERROR( "ER:0014\n" );
            UARTprintf("%22s: quickRuleId must be < %d\n", "Cmd_RUL()",
                    MAX_QUICK_RULES);
            return 0;
        }
        inputSwitchId = decodeHwIndex( ustrtoul(argv[2], NULL, 0), 1 );
        if ( inputSwitchId.channel == C_INVALID) {
            REPORT_ERROR( "ER:0015\n" );
            UARTprintf( "%22s: inputSwitchId = %s invalid\n", "Cmd_RUL()", argv[2] );
            return 0;
        }
        hwIndex = ustrtoul(argv[3], NULL, 0);
        outputDriverId = decodeHwIndex(hwIndex, 0);
        switch (outputDriverId.channel){
        case C_I2C0:
        case C_I2C1:
        case C_I2C2:
        case C_I2C3:
            if ( pwmHigh >= (1 << N_BIT_PWM) || pwmLow >= (1 << N_BIT_PWM) ) {
                REPORT_ERROR( "ER:0016\n" );
                UARTprintf( "%22s: pwmValues must be < %d\n", "Cmd_RUL()", (1 << N_BIT_PWM) );
                return 0;
            }
            break;
        case C_FAST_PWM:
            if (pwmHigh > MAX_PWM || pwmLow > MAX_PWM) {
                REPORT_ERROR("ER:0017\n");
                UARTprintf("%22s: HW pwmValues must be < %d\n", "Cmd_RUL()", MAX_PWM);
                return 0;
            }
            break;
        case C_INVALID:
        case C_SWITCH_MATRIX:
            REPORT_ERROR( "ER:0018\n" );
            UARTprintf( "%22s: outputDriverId = %s invalid\n", "Cmd_RUL()", argv[3] );
            return 0;
        }
        UARTprintf("%22s: Setting up autofiring rule %d\n", "Cmd_RUL()", id);
        setupQuickRule( id, inputSwitchId, outputDriverId, triggerHoldOffTime,
                tPulse, pwmHigh, pwmLow, trigPosEdge );
        return 0;
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_LEC(int argc, char *argv[]) {   //Here we need re - initialize the SPI module
    //LEC <channel> <spiSpeed [Hz]> <SSI_FRF (opt)>"
    uint8_t channel;
    uint32_t baseAddr, spiSpeed, frameFmt=SSI_FRF_MOTO_MODE_1;
    if( argc==3 || argc==4 ){
        channel = ustrtoul(argv[1], NULL, 0);
        if( channel<=2 ){
            baseAddr = g_spiState[channel].baseAdr;
        } else {
            REPORT_ERROR( "ER:0019\n" );
            UARTprintf("%22s: Invalid LED channel (%d)\n", "Cmd_LEC()", channel);
            return 0;
        }
        spiSpeed = ustrtoul(argv[2], NULL, 0);
        if( argc == 4 ){
            frameFmt = ustrtoul(argv[3], NULL, 0);
        }
        while( HWREG( baseAddr + SSI_O_SR ) & SSI_SR_BSY ){  //Wait for SPI to finish
            vTaskDelay( 10 );
        }
        UARTprintf("%22s: Setting SPI to %d Hz, frameFmt = 0x%02x\n", "Cmd_LEC()", spiSpeed, frameFmt);
        ROM_SSIDisable( baseAddr );
        // SPI at 3.2 MHz, 16 bit SPI words (encoding 4 bit data each)
        ROM_SSIConfigSetExpClk( baseAddr, SYSTEM_CLOCK, frameFmt, SSI_MODE_MASTER, spiSpeed, 16 );
        ROM_SSIEnable( baseAddr );
    } else {
        return( CMDLINE_TOO_FEW_ARGS );
    }

    return 0;
}


int Cmd_LED(int argc, char *argv[]) {   //Here we need to switch the serialCommandParser to Binary mode
    //LED 0 128\nxxxxxx
    uint8_t channel;
    uint32_t blobSize;
    if( argc==3 ){
        channel = ustrtoul(argv[1], NULL, 0);
        if( channel <= 2 ){
            blobSize = ustrtoul(argv[2], NULL, 0);
            if( blobSize%3 || blobSize>N_LEDS_MAX*3 ){
                REPORT_ERROR( "ER:001A\n" );
                UARTprintf("%22s: Invalid number of bytes (%d)\n", "Cmd_LED()", blobSize);
                return 0;
            }
//            UARTprintf("Blasting %d bytes of data to LED string on channel %d\n", blobSize, channel);
            //-------------------------
            // Lock the SPI channel
            //-------------------------
            t_spiTransferState *state = &g_spiState[channel];
            if( xSemaphoreTake( state->semaToReleaseWhenFinished, 1000 ) ){
                g_LEDnBytesToCopy = blobSize;   //We got the Lock, start copy process
                g_LEDChannel = channel;
                return PARS_MODE_BIN_LED;
            } else {
                REPORT_ERROR( "ER:001B\n" );
                UARTprintf("%22s: Timeout, could not access sendBuffer %d\n", "spiSend()", channel);
            }
        } else {
            REPORT_ERROR( "ER:001C\n" );
            UARTprintf("%22s: Invalid LED channel (%d)\n", "Cmd_LED()", channel);
        }
    } else {
        REPORT_ERROR( "ER:001D\n" );
        UARTprintf("%22s: Invalid number of arguments (%d)\n", "Cmd_LED()", argc);
    }
    return( 0 );
}

static uint8_t hexToNibble(char hexChar)
{
    if(hexChar >= '0' && hexChar <= '9') {
        return(hexChar - '0');
    } else if (hexChar >= 'a' && hexChar <= 'f'){
        return(hexChar - 'a' + 10);
    } else if (hexChar >= 'A' && hexChar <= 'F'){
        return(hexChar - 'A' + 10);
    }
    return(0);
}

static char nibbleToHex(unsigned nibble)
{
    return "0123456789ABCDEF"[nibble & 0x0F];
}

// Takes a string, returns a buff. Make sure to free it after use.
uint8_t *hexToBuff(char *str, unsigned *nWritten)
{
    uint8_t *p, *buff;
    unsigned nBytesTx = ustrlen(str);      //argv[3] string contains hex characters [0FFEDEADBEEF]
    if (nBytesTx % 2) {
        UARTprintf("%22s: hex string must be even length\n", "hexToBuff()");
        *nWritten = 0;
        return NULL;
    }
    nBytesTx /= 2;
    buff = pvPortMalloc(nBytesTx);
    if (!buff) {
        UARTprintf("%22s: pvPortMalloc() failed\n", "hexToBuff()");
        *nWritten = 0;
        return NULL;
    }
    p = buff;
    for(unsigned i=0; i<nBytesTx; i++) {
        *p  = hexToNibble(*str++) << 4;
        *p |= hexToNibble(*str++);
        p++;
    }
    *nWritten = nBytesTx;
    return buff;
}

// Takes a buffer, writes hex string to dest
// returns pointer to \0 of \0 terminated string.
char *buffToHex(uint8_t *buf, unsigned len, char *dest)
{
    for (unsigned i=0; i<len; i++) {
        *dest++ = nibbleToHex(*buf >> 4);
        *dest++ = nibbleToHex(*buf++);
    }
    *dest = '\0';
    return dest;
}

// Fills up a t_i2cCustom after sanity checking and puts it on the queue
int Cmd_I2C(int argc, char *argv[]) {
    //I2C <channel> <I2Caddr> [<sendData>] <nBytesRx>
    t_i2cCustom i2c;
    if (argc == 5 || argc == 4) {
        i2c.channel = ustrtoul(argv[1], NULL, 0);
        if (i2c.channel > 3) {
            REPORT_ERROR("ER:001E\n");
            UARTprintf("%22s: I2C Channel must be <= 4\n", "Cmd_I2C()");
            return 0;
        }
        i2c.i2c_addr = ustrtoul(argv[2], NULL, 0);
        if (i2c.i2c_addr > 127) {
            REPORT_ERROR("ER:001E\n");
            UARTprintf("%22s: I2Caddr must be <= 127\n", "Cmd_I2C()");
            return 0;
        }
        i2c.readBuff = NULL;
        if (argc == 4) {
            i2c.nWrite = 0;
            i2c.writeBuff = NULL;
            i2c.nRead = ustrtoul(argv[3], NULL, 0);
        } else {
            i2c.writeBuff = hexToBuff(argv[3], (unsigned*)&i2c.nWrite);
            if (!i2c.writeBuff) return 0;
            i2c.nRead = ustrtoul(argv[4], NULL, 0);
        }
        if (i2c.nRead > 0) {
            i2c.readBuff = pvPortMalloc(i2c.nRead);
            if (!i2c.readBuff) {
                UARTprintf("%22s: read buffer not allocated :(\n", "Cmd_I2C()");
                vPortFree(i2c.writeBuff); i2c.writeBuff = NULL;
                vPortFree(i2c.readBuff); i2c.readBuff = NULL;
                return 0;
            }
        }
        // UARTprintf(
        //     "CH: %x, addr: %x, nRead: %x, nWrite: %x [",
        //     i2c.channel,
        //     i2c.i2c_addr,
        //     i2c.nRead,
        //     i2c.nWrite
        // );
        // for (unsigned i=0; i<i2c.nWrite; i++) {
        //     UARTprintf("%02x ", i2c.writeBuff[i]);
        // }
        // UARTprintf("]\n");
        if(!xQueueSendToBack(g_i2c_queue, &i2c, 100 / portTICK_PERIOD_MS)) {
            UARTprintf("%22s: I2C queue full!\n", "Cmd_I2C()");
            vPortFree(i2c.writeBuff); i2c.writeBuff = NULL;
            vPortFree(i2c.readBuff); i2c.readBuff = NULL;
        }
        return 0;
    }
    return CMDLINE_TOO_FEW_ARGS;
}

int Cmd_HI(int argc, char *argv[]) {
    // Writes a 0xFF to the relevant PCF to be used as input
    // HI 0x123
    t_i2cCustom i2c;
    if(argc != 2) return CMDLINE_TOO_FEW_ARGS;
    t_hw_index i = decodeHwIndex(ustrtoul(argv[1], NULL, 0), 1);
    if (i.channel > 3) {
        // REPORT_ERROR("ER:001E\n");
        UARTprintf("%22s: Not a I2C channel: %x\n", "Cmd_HI()", i.channel);
        return 0;
    }
    if (i.i2c_addr > 0x27) {
        // REPORT_ERROR("ER:001E\n");
        UARTprintf("%22s: Invalid i2c_addr: %x\n", "Cmd_HI()", i.i2c_addr);
        return 0;
    }
    i2c.channel = i.channel;
    i2c.i2c_addr = i.i2c_addr;
    i2c.nWrite = 1;
    i2c.nRead = 0;
    i2c.readBuff = NULL;
    i2c.writeBuff = pvPortMalloc(1);
    if (!i2c.writeBuff) {
        UARTprintf("%22s: pvPortMalloc() failed\n", "Cmd_HI()");
        return 0;
    }
    *i2c.writeBuff = 0xFF;
    UARTprintf("%22s: CH: %x, ADDR: %x gets high\n", "Cmd_HI()", i2c.channel, i2c.i2c_addr);
    if(!xQueueSendToBack(g_i2c_queue, &i2c, 100 / portTICK_PERIOD_MS)) {
        UARTprintf("%22s: I2C queue full!\n", "Cmd_HI()");
        vPortFree(i2c.writeBuff); i2c.writeBuff = NULL;
    }
    return 0;
}
