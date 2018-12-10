// Quick fire rules logic:
// Fire solenoid X when there is a rising / falling edge on switch Y

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "my_uartstdio.h"
#include "io_manager.h"
#include "quick_rules.h"

// to keep track of quick-fire rules configurations
static t_quickRule g_QuickRuleList[MAX_QUICK_RULES];

// Naughty but convenient way of addressing a single flag-bit
#define TF(f) HWREGBITB(&currentRule->triggerFlags, f)

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
