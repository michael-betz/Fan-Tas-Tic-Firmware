// SOmething triggers the I2Ctransaction periodically every 1 ms
// it goes through several states:
// 1. go through all PCF slots
//  * ignore disabled ones
//  * read state of the ones which are inputs
//  * count iterations, every 2^N ms, write bcm buffer to relevant PCFs
// 2. Check if a custom i2c transaction shall be carried out, do that
// 3. Repeat

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_pwm.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"

#include "FreeRTOS.h"
#include "task.h"

#include "my_uartstdio.h"
#include "main.h"
#include "i2c_inout.h"
#include "switch_matrix.h"
#include "bit_rules.h"

// Four TI I2C driver instances for 4 I2C channels
t_i2cChannelState g_sI2CInst[4];

// current bcmBuffer index for all PCF outputs,
// set by trigger_i2c_cycle()
static unsigned g_bcmIndex = 0;

void i2CIntHandler0(void) {
    i2c_isr(&g_sI2CInst[0]);
}

void i2CIntHandler1(void) {
    i2c_isr(&g_sI2CInst[1]);
}

void i2CIntHandler2(void) {
    i2c_isr(&g_sI2CInst[2]);
}

void i2CIntHandler3(void) {
    i2c_isr(&g_sI2CInst[3]);
}

static void i2cUnstucker(uint32_t base, uint8_t pin_SCL, uint8_t pin_SDA){
    //--------------------------
    // Unstuck I2C SDA lines
    //--------------------------
    ROM_GPIOPinTypeGPIOOutputOD( base, pin_SCL | pin_SDA );
    ROM_GPIOPinWrite(base, pin_SDA, 0xFF);
    for (unsigned i=0; i<16; i++){
        ROM_GPIOPinWrite(base, pin_SCL, 0x00);
        ROM_SysCtlDelay(25);   // ~ 4 us @ 80 MHz
        ROM_GPIOPinWrite(base, pin_SCL, 0xFF);
        ROM_SysCtlDelay(25);
    }
    GPIOPinTypeGPIOInput(base, pin_SDA | pin_SCL);
    ROM_SysCtlDelay(25);
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

t_pcf_state *get_pcf(t_hw_index *pin)
{
    if (!pin) return NULL;
    unsigned c = pin->channel;
    unsigned a = pin->i2c_addr - PCF_LOWEST_ADDR;
    if (c > C_I2C3 || a >= PCF_MAX_PER_CHANNEL) return NULL;
    return &(g_sI2CInst[c].pcf_state[a]);
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

static void my_pcf_init(t_pcf_state *pcf)
{
    pcf->flags = 0;//FPCF_RENABLED;
    pcf->last_mcs = 0;
    pcf->err_cnt = 0;
    uint8_t *bcm = pcf->bcm_buffer;
    for(unsigned i=0; i<N_BIT_PWM; i++)
        *bcm++ = 0;
}

static void i2c_state_init(t_i2cChannelState *state, uint32_t base_addr, uint_fast8_t int_addr)
{
    state->base_addr = base_addr;
    state->int_addr = int_addr;
    state->i2c_state = I2C_IDLE;
    state->currentPcf = 0;
    t_pcf_state *pcf = state->pcf_state;
    for (unsigned i=0; i<PCF_MAX_PER_CHANNEL; i++) {
        my_pcf_init(pcf);
        pcf->i2c_addr = PCF_LOWEST_ADDR + i;
        pcf++;
    }
    IntPendClear(int_addr);
    IntEnable(int_addr);
    ROM_I2CMasterIntEnableEx(base_addr, I2C_MASTER_INT_DATA);
}

// Send byte over i2c and block (no interrupts)
// b = base_addr, addr = left shifted i2c address
void i2c_send(uint32_t b, uint8_t addr, uint8_t data)
{
    while (HWREG(b + I2C_O_MCS) & I2C_MCS_BUSY);
    HWREG(b + I2C_O_MSA) = (addr << 1) | 0;
    HWREG(b + I2C_O_MDR) = data;
    HWREG(b + I2C_O_MCS) = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
}

// Wait for all bits to be set in notification value (and clear them)
void wait_for_noti_bits(uint32_t bits)
{
    uint32_t noti_new, noti_ored = 0;
    do {
        xTaskNotifyWait(0, 0xFFFFFFFF, &noti_new, portMAX_DELAY); //1000 / portTICK_PERIOD_MS
        noti_ored |= noti_new;
    } while ((bits & noti_ored) != bits);
}

// Yield from the task until i2c transaction complete
void i2c_send_yield(uint8_t channel, uint8_t addr, uint8_t data)
{
    if (channel > 3) return;
    t_i2cChannelState *c = &g_sI2CInst[channel];
    if (c->i2c_state != I2C_IDLE)
        return;
    c->i2c_state = I2C_CUSTOM;
    i2c_send(c->base_addr, addr, data);
    wait_for_noti_bits(1 << channel);
}

// b = base_addr
// Returns true if a read or write has been started
static bool setup_pcf_rw(unsigned b, t_pcf_state *pcf)
{
    if (pcf->flags & FPCF_WENABLED){
        // UARTprintf(" W%2x ", pcf->i2c_addr);
        HWREG(b + I2C_O_MSA) = (pcf->i2c_addr<<1) | 0;
        HWREG(b + I2C_O_MDR) = pcf->bcm_buffer[g_bcmIndex];
        HWREG(b + I2C_O_MCS) = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
        return true;
    }
    if (pcf->flags & FPCF_RENABLED){
        // START condition followed by RECEIVE and STOP condition
        // (master remains in Idle state).
        // UARTprintf(" R%2x ", pcf->i2c_addr);
        HWREG(b + I2C_O_MSA) = (pcf->i2c_addr<<1) | 1;
        HWREG(b + I2C_O_MCS) = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
        return true;
    }
    return false;
}

// Get i2c channel from base_addr
static unsigned _get_ch(unsigned b)
{
    return (b - I2C0_BASE) >> 12;
}

// Each I2C ISR has a channel number (0-3)
// When the ISR is done, the pcfInReader task is notified
// the bit corresponding to its CH# is set in the task noti. value
// When all 4 bits are set, the task continues
static void _isr_notify(t_i2cChannelState *state, BaseType_t *hpw)
{
    state->i2c_state = I2C_IDLE;
    xTaskNotifyFromISR(
        hPcfInReader,
        1 << _get_ch(state->base_addr),
        eSetBits,
        hpw
    );
}


void i2c_isr(t_i2cChannelState *state)
{
    // true if we should return to a higher prio task
    BaseType_t hpw = pdFALSE;
    unsigned b = state->base_addr;
    unsigned mcs = HWREG(b + I2C_O_MCS);
    t_pcf_state *pcf;
    ledOut(1);
    ROM_I2CMasterIntClear(b);
    // When the BUSY bit is set, the other I2C status bits are not valid.
    if (mcs & I2C_MCS_BUSY){
        ledOut(0);
        return;
    }

    // UARTprintf(" !%x:%x,%x! ", _get_ch(state->base_addr), state->i2c_state, mcs);

    switch(state->i2c_state){
        case I2C_START:
            // Start a single byte read
            state->currentPcf = 0;
            pcf = &(state->pcf_state[state->currentPcf]);
            state->i2c_state = I2C_PCF;
            // Setup first read or write
            while(!setup_pcf_rw(b, pcf)){
                state->currentPcf++;
                if (state->currentPcf >= PCF_MAX_PER_CHANNEL){
                    // Nothing to do, notify PCF_Reader
                    _isr_notify(state, &hpw);
                    break;
                }
                pcf++;
            }
            break;

        case I2C_PCF:
            // Evaluate result of the single byte read or write
            pcf = &(state->pcf_state[state->currentPcf]);
            pcf->last_mcs = mcs;
            if (mcs & I2C_MCS_ERROR) {
                pcf->err_cnt++;
                // Disable PCF when there's too many errors
                if(pcf->err_cnt > PCF_ERR_CNT_DISABLE){
                    pcf->flags = 0;
                    pcf->err_cnt = 0;
                }
            } else if (pcf->value_target && (pcf->flags & FPCF_RENABLED)){
                // pcf->value = HWREG(b + I2C_O_MDR);
                *(pcf->value_target) = HWREG(b + I2C_O_MDR);
            }
            // Setup next read or write
            do {
                state->currentPcf++;
                if (state->currentPcf >= PCF_MAX_PER_CHANNEL){
                    _isr_notify(state, &hpw);
                    break;
                }
            } while (!setup_pcf_rw(b, ++pcf));
            break;

        case I2C_CUSTOM:
            _isr_notify(state, &hpw);
            break;

        case I2C_IDLE:
            UARTprintf(" I ");
            break;

        default:
            UARTprintf(" ?%x? ", state->i2c_state);
            state->i2c_state = I2C_IDLE;
    }
    ledOut(2);
    portYIELD_FROM_ISR(hpw);
    ledOut(0);
}

// Resets GPIO and I2C hardware
// isr_init = false: also send 0x00 to all PCFs
// isr_init = true:  also init data structures and interrupts
void init_i2c_system(bool isr_init) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);   //I2C0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);   //I2C1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);   //I2C2
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);   //I2C3

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
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C0);
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C1);
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C2);
    ROM_SysCtlPeripheralReset( SYSCTL_PERIPH_I2C3);

    // Configure the pin muxing for I2C0 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);            //I2C0
    ROM_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);            //I2C1
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    ROM_GPIOPinConfigure(GPIO_PE4_I2C2SCL);            //I2C2
    ROM_GPIOPinConfigure(GPIO_PE5_I2C2SDA);
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);            //I2C3
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); //I2C0
    ROM_GPIOPinTypeI2C(   GPIO_PORTB_BASE, GPIO_PIN_3);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); //I2C1
    ROM_GPIOPinTypeI2C(   GPIO_PORTA_BASE, GPIO_PIN_7);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4); //I2C2
    ROM_GPIOPinTypeI2C(   GPIO_PORTE_BASE, GPIO_PIN_5);
    ROM_GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0); //I2C3
    ROM_GPIOPinTypeI2C(   GPIO_PORTD_BASE, GPIO_PIN_1);

    // Initialize the I2C master module for 400 kHz
    ROM_I2CMasterInitExpClk(I2C0_BASE, SYSTEM_CLOCK, true);
    ROM_I2CMasterInitExpClk(I2C1_BASE, SYSTEM_CLOCK, true);
    ROM_I2CMasterInitExpClk(I2C2_BASE, SYSTEM_CLOCK, true);
    ROM_I2CMasterInitExpClk(I2C3_BASE, SYSTEM_CLOCK, true);

    if (isr_init){
        // Initialize I2C0 peripheral driver.
        i2c_state_init(&g_sI2CInst[0], I2C0_BASE, INT_I2C0);
        i2c_state_init(&g_sI2CInst[1], I2C1_BASE, INT_I2C1);
        i2c_state_init(&g_sI2CInst[2], I2C2_BASE, INT_I2C2);
        i2c_state_init(&g_sI2CInst[3], I2C3_BASE, INT_I2C3);

        // Set destination pointers
        for (unsigned c=0; c<4; c++){
            for (unsigned p=0; p<PCF_MAX_PER_CHANNEL; p++){
                // Ready for some sick pointer acrobatics?
                g_sI2CInst[c].pcf_state[p].value_target =
                &g_SwitchStateSampled.switchState.i2cReadData[c][p];
            }
        }
        // Only enable PCF
        g_sI2CInst[0].pcf_state[0].flags = FPCF_RENABLED;
        g_sI2CInst[0].pcf_state[1].flags = FPCF_RENABLED;
    } else {
        for (unsigned p=0; p<PCF_MAX_PER_CHANNEL; p++){
            i2c_send(I2C0_BASE, 0x20+p, 0x00);
            i2c_send(I2C1_BASE, 0x20+p, 0x00);
            i2c_send(I2C2_BASE, 0x20+p, 0x00);
            i2c_send(I2C3_BASE, 0x20+p, 0x00);
        }
    }
}

// Shall be called every 1 ms
void trigger_i2c_cycle()
{
    static unsigned nCalls = 0;
    UARTprintf("T");
    nCalls++;
    // Cycle through the bcm_buffer in a binary way
    if (nCalls >= (1<<g_bcmIndex)) {
        nCalls = 0;
        g_bcmIndex++;
        if(g_bcmIndex >= N_BIT_PWM){
            g_bcmIndex = 0;
        }
    }
    // UARTprintf("%x ", g_bcmIndex);
    // The whole i2c read sequence takes < 0.5 us
    for (unsigned i=0; i<=3; i++){
        t_i2cChannelState *s = &g_sI2CInst[i];
        s->i2c_state = I2C_START;
        IntTrigger(s->int_addr);
    }
}

void print_pcf_state()
{
    UARTprintf("  R/W[I2C_ADDR]: VAL (ERR_CNT)\n");
    for(unsigned pcf=0; pcf<=7; pcf++){
        for(unsigned ch=0; ch<=3; ch++){
            t_i2cChannelState *chp = &(g_sI2CInst[ch]);
            t_pcf_state *pcfp = &(chp->pcf_state[pcf]);
            if (pcfp->flags & FPCF_WENABLED){
                UARTprintf(
                    "  W[%2x]:    (%5x)   ",
                    pcfp->i2c_addr,
                    pcfp->err_cnt
                );
            } else if (pcfp->flags & FPCF_RENABLED) {
                UARTprintf(
                    "  R[%2x]: %2x (%2x,%5x)",
                    pcfp->i2c_addr,
                    // pcfp->value,
                    *pcfp->value_target,
                    pcfp->last_mcs,
                    pcfp->err_cnt
                );
            } else {
                UARTprintf(
                    "   [%2x]:              ",
                    pcfp->i2c_addr
                );
            }
        }
        UARTprintf("\n");
    }
    UARTprintf("\n");
}
