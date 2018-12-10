#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
// TivaWare includes
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "utils/ustdlib.h"
#include "my_uartstdio.h"
// freeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
// My stuff
#include "myTasks.h"
#include "switch_matrix.h"
#include "bit_rules.h"
#include "main.h"

bool g_reDiscover = 0;
TaskHandle_t hPcfInReader = NULL;
// To queue up custom i2c transactions
QueueHandle_t g_i2c_queue = NULL;
// to keep track of the read input states
t_switchStateConverter g_SwitchStateSampled;    //Read values of last I2C scan
t_switchStateConverter g_SwitchStateNoDebounce; //Debouncing-OFF flags
t_switchStateConverter g_SwitchStateDebounced;  //Debounced values (the same after 4 reads)
static t_switchStateConverter g_SwitchStateToggled;    //Bits which changed
// to keep track of pulsed ouputs
static t_PCLOutputByte g_outWriterList[OUT_WRITER_LIST_LEN];
// to keep track of quick-fire rules configurations
static t_quickRule g_QuickRuleList[MAX_QUICK_RULES];

t_hw_index decodeHwIndex(uint16_t hwIndex, bool asInput) {
    unsigned i2cCh;
    t_hw_index tempResult;
    tempResult.channel = C_INVALID;
    //-----------------------------------------
    // Check for HW. PWM outputs (60 - 63)
    //-----------------------------------------
    if( (!asInput) && (hwIndex>=60) && (hwIndex<=63) ){
        tempResult.pinIndex = hwIndex - 60;
        tempResult.i2c_addr = 0;
        tempResult.channel = C_FAST_PWM;
        return tempResult;
    }
    tempResult.byteIndex = hwIndex / 8;
    tempResult.pinIndex = hwIndex % 8;          // Which bit of the byte is addressed
    //-----------------------------------------
    // Check for a Switch Matrix Input (0 - 7)
    //-----------------------------------------
    if (tempResult.byteIndex <= 7) {
        if(asInput)
            tempResult.channel = C_SWITCH_MATRIX;
        return tempResult;
    }
    //-----------------------------------------
    // Check for I2C channel
    //-----------------------------------------
    i2cCh = (tempResult.byteIndex - 8) / 8;     // Only i2c channel 0-3 exists
    if (i2cCh <= 3) {
        tempResult.channel = i2cCh;
        tempResult.i2c_addr = (tempResult.byteIndex - 8) % 8 + PCF_LOWEST_ADDR;
    }
    return tempResult;
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
    // --> as during the context switch it copies the whole thing on the heap :p
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
                //We found a bit that changed, report over serial USB
                if ( tempValue & 0x00000001 ) {
                    // 3rd switch changed to 0, 125th switch changed to 1 "SE:003=0;07D=1;"
                    switchValue = HWREGBITW(&g_SwitchStateDebounced.longValues[i], j);
                    charsWritten += usnprintf(
                        &outBuffer[charsWritten],
                        REPORT_SWITCH_BUF_SIZE-charsWritten,
                        "%03x=%01d ",
                        i * 32 +j,
                        switchValue
                    );
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
                        setPCFOutput( &(currentRule->outputDriverId),
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

void setupQuickRule(uint8_t id, t_hw_index inputSwitchId,
        t_hw_index outputDriverId, uint16_t triggerHoldOffTime,
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

static void fillBitRule(t_hw_index *pin, t_PCLOutputByte *w, int16_t tPulse, uint16_t highPower, uint16_t lowPower){
    // Fill the output pulse state `b`
    t_BitModifyRules *b = &(w->bitRules[pin->pinIndex]);
    b->tPulse = tPulse;
    b->lowPWM = lowPower;
    if (pin->channel == C_FAST_PWM) {
        setPwm(pin->pinIndex, highPower);
        return;
    } else if (w->pcf) {
        set_bcm(w->pcf->bcm_buffer, pin->pinIndex, highPower);
        w->pcf->flags = FPCF_WENABLED; // Intentionally clearing the read flag
        return;
    }
    UARTprintf("fillBitRule(): !!! trouble !!!\n");
}

void setPCFOutput(t_hw_index *pin, int16_t tPulse, uint16_t highPower, uint16_t lowPower) {
    // Will add or update a job in the output BCM list
    t_PCLOutputByte *w = g_outWriterList;
    if (pin->channel < C_I2C0 ||
        pin->channel > C_FAST_PWM)
        return;
    for (unsigned i=0; i<OUT_WRITER_LIST_LEN; i++) {
        if (w->channel == C_INVALID) {
            // Create a new entry!
            w->pcf = get_pcf(pin);  // returns NULL when not a I2C channel
            fillBitRule(pin, w, tPulse, highPower, lowPower);
            // Mark the item as valid to the output routine
            w->channel = pin->channel;
            // UARTprintf("%22s: wrote to g_outWriterList[%d] (new)\n", "setPclOutput()", i);
            return;
        } else if (w->channel == pin->channel) {
            if (pin->channel == C_FAST_PWM ||
                (w->pcf && w->pcf->i2c_addr == pin->i2c_addr)) {
                // Found the right byte, change it
                fillBitRule(pin, w, tPulse, highPower, lowPower);
                // UARTprintf("%22s: wrote to g_outWriterList[%d]\n", "setPclOutput()", i);
                return;
            }
        }
        w++;
    }
    // Error, no more space in outList :(
    UARTprintf("%22s: Error, no more space in g_outWriterList :(\n", "setPclOutput()");
    REPORT_ERROR("ER:0004\n");
}

void print_out_writer_list()
{
    UARTprintf(" N: [CH,I2C] PWM0 PWM1 ...\n");
    t_PCLOutputByte *w = g_outWriterList;
    for (unsigned i=0; i<OUT_WRITER_LIST_LEN; i++) {
        if (w->channel == C_INVALID) continue;
        UARTprintf("%2x: [%x", i, w->channel);
        if (w->pcf) {
            UARTprintf(",%2x]", w->pcf->i2c_addr);
        } else {
            UARTprintf("]   ");
        }
        for (unsigned p=0; p<=7; p++)
            if(w->pcf)
                UARTprintf("  %3x", get_bcm(w->pcf->bcm_buffer, p));
            else
                UARTprintf("  %3x", w->bitRules[p].lowPWM);
        UARTprintf("\n");
        w++;
    }
}

static void handleBitRules(unsigned dt) {
    // Handle the switchover from `Pulsed` state to `unpulsed` state for each output pin
    t_PCLOutputByte *w = g_outWriterList;
    for (unsigned i=0; i<OUT_WRITER_LIST_LEN; i++) {
        if (w->channel == C_INVALID){
            w++;
            continue;
        }
        t_BitModifyRules *bitRules = w->bitRules;
        for (unsigned j = 0; j <= 7; j++) {
            // Is the entry valid?
            if (bitRules->tPulse > 0) {
                bitRules->tPulse -= dt;
                // Did the countdown expire?
                if (bitRules->tPulse <= 0) {
                    if (w->channel == C_FAST_PWM){
                        // Apply low HW pwm
                        setPwm(j, bitRules->lowPWM);
                    } else if (w->pcf) {
                        // Apply the I2C pulse_low bcm pattern
                        set_bcm(w->pcf->bcm_buffer, j, bitRules->lowPWM );
                    }
                }
            }
            bitRules++;
        }
        w++;
    }
}

// TODO also handle reads (reply to usb) and multi-byte transfers
static void handle_i2c_custom()
{
    t_i2cCustom i2c;
    // Do up to 3 transactions per PCF cycle
    for (unsigned i=0; i<3; i++){
        if (!xQueueReceive(g_i2c_queue, &i2c, 0)) break;
        if (i2c.nWrite == 1) {
            i2c_send_yield(i2c.channel, i2c.i2c_addr, *i2c.writeBuff);
        }
        vPortFree(i2c.readBuff);  i2c.readBuff  = NULL;
        vPortFree(i2c.writeBuff); i2c.writeBuff = NULL;
    }
}

// Called every 1 ms (hopefully) after new states have been read
static void process_IO()
{
    unsigned i;
    // Run debounce algo (14 us)
    debounceAlgo(
        g_SwitchStateSampled.longValues,
        g_SwitchStateDebounced.longValues,
        g_SwitchStateToggled.longValues,
        g_SwitchStateNoDebounce.longValues
    );
    // Notify Mission pinball over serial port of all changed switches
    if (g_reportSwitchEvents) reportSwitchStates();
    handleBitRules(DEBOUNCER_READ_PERIOD);
    processQuickRules();
    handle_i2c_custom();
    if (g_reDiscover) {
        g_reDiscover = 0;
        for (i=0; i<MAX_QUICK_RULES; i++) disableQuickRule(i);
        for (i=0; i<OUT_WRITER_LIST_LEN; i++) g_outWriterList[i].channel = C_INVALID;
        init_i2c_system(true);
        UARTprintf("done\n");
    }
}

void task_pcf_io(void *pvParameters)
{
//    Read the state of all switches every 1 ms
//    Then we need to debounce each switch and send `state changed` events
    TickType_t xLastWakeTime;
    unsigned i;
    // hPcfInReader = xTaskGetCurrentTaskHandle();
    UARTprintf("%22s: Started! Cycle time = %d ms\n", "task_pcf_io()", DEBOUNCER_READ_PERIOD);
    if (!g_i2c_queue) g_i2c_queue = xQueueCreate(32, sizeof(t_i2cCustom));
    for (i=0; i<MAX_QUICK_RULES; i++) disableQuickRule(i);
    for (i=0; i<OUT_WRITER_LIST_LEN; i++) g_outWriterList[i].channel = C_INVALID;
    vTaskDelay(1);
    init_i2c_system(true);
    // Get the initial state of all switches silently (without reporting Switch Events)
    trigger_i2c_cycle();
    readSwitchMatrix();
    xLastWakeTime = xTaskGetTickCount();
    i = 0;
    while (1) {
        // Wait for 4 x I2C ISR notification (bit 0 - 4 of notification value)
        wait_for_noti_bits(0x0F);
        process_IO();
        // unsigned cycles = stopTimer(); UARTprintf("%d cycles, %d us\n", cycles, cycles * 1000ll * 1000 / SYSTEM_CLOCK);
        //Run every 1 ms --> 4 ms debounce latency
        vTaskDelayUntil(&xLastWakeTime, DEBOUNCER_READ_PERIOD / portTICK_PERIOD_MS);
        // vTaskDelayUntil(&xLastWakeTime, 3000);
        //Start background I2C scanner (takes ~ 400 us)
        // startTimer();
        trigger_i2c_cycle();
        //Should take >= 500 us as it happens in parallel with the I2C scan
        readSwitchMatrix();  // 32965 cycles, 412 us
        i++;
    }
}
