#ifndef SWITCH_MATRIX_H_
#define SWITCH_MATRIX_H_

// Define Port pins to the Switch Matrix column driver (shift register IC)
#define SM_COL_DAT GPIO_PIN_0
#define SM_COL_CLK GPIO_PIN_1

//resetSMcol() + 8 * advanceSMcol() should take ~ 500 us
// At 80 MHz We should spent 40000 instructions
// We got 320 + 160 + 60 = 530 useful instr.
// We got 6 + 16 + 32 = 54 delay calls which do n*3 instructions
// So each delay call should do ... delay units:
// (40000 - 530)/54/3 = 244
// 200 --> Switch matrix has ~ 8 us to settle
#define SM_COL_DELAY_CNT 60

void readSwitchMatrix();

#endif

