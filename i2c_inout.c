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

static void my_pcf_init(t_pcfState *pcf)
{
    pcf->flags = FPCF_RENABLED;
    pcf->last_err_mcs = 0;
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
    for(unsigned i=0; i<PCF_MAX_PER_CHANNEL; i++){
        t_pcfState *pcf = &state->pcf_state[i];
        my_pcf_init(pcf);
        pcf->i2c_addr = PCF_LOWEST_ADDR + i;
        if (i==1) {
            pcf->flags = FPCF_WENABLED;
        }
    }

    // Initialize the I2C master module.
    ROM_I2CMasterInitExpClk(base_addr, SYSTEM_CLOCK, true);

    // Enable the I2C interrupt.
    ROM_IntEnable(int_addr);
    ROM_I2CMasterIntEnableEx(base_addr, I2C_MASTER_INT_DATA);
}

// b = base_addr
// Returns false on error
static bool setup_pcf_rw(unsigned b, t_pcfState *pcf)
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

void i2c_isr(t_i2cChannelState *state)
{
    unsigned b = state->base_addr;
    unsigned mcs = HWREG(b + I2C_O_MCS);
    t_pcfState *pcf;
    ROM_I2CMasterIntClear(b);

    // When the BUSY bit is set, the other status bits are not valid.
    if (mcs & I2C_MCS_BUSY)
        return;

    // UARTprintf(" !%x,%x! ", state->i2c_state, mcs);

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
                    state->i2c_state = I2C_IDLE;
                    break;
                }
                pcf++;
            }
            break;

        case I2C_PCF:
            // Evaluate result of the single byte read or write
            pcf = &(state->pcf_state[state->currentPcf]);
            pcf->last_value = HWREG(b + I2C_O_MDR);
            if (mcs & I2C_MCS_ERROR){
                // Latch the error
                pcf->last_err_mcs = mcs;
                pcf->err_cnt++;
            }
            // Setup next read or write
            do {
                state->currentPcf++;
                if (state->currentPcf >= PCF_MAX_PER_CHANNEL){
                    state->i2c_state = I2C_IDLE;
                    break;
                }
            } while (!setup_pcf_rw(b, ++pcf));
            break;

        case I2C_IDLE:
            UARTprintf(" I ", state->i2c_state);
            break;

        default:
            UARTprintf(" ?%x? ", state->i2c_state);
            state->i2c_state = I2C_IDLE;
    }
}

void init_i2c_system() {
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
    i2c_state_init(&g_sI2CInst[0], I2C0_BASE, INT_I2C0);
    i2c_state_init(&g_sI2CInst[1], I2C1_BASE, INT_I2C1);
    i2c_state_init(&g_sI2CInst[2], I2C2_BASE, INT_I2C2);
    i2c_state_init(&g_sI2CInst[3], I2C3_BASE, INT_I2C3);
}

// Shall be called every 1 ms
void trigger_i2c_cycle()
{
    static unsigned nCalls = 0;
    nCalls++;
    if (nCalls >= (1<<g_bcmIndex)) {
        nCalls = 0;
        g_bcmIndex++;
        if(g_bcmIndex >= N_BIT_PWM){
            g_bcmIndex = 0;
        }
    }
    // UARTprintf("%x ", g_bcmIndex);
    // The whole sequence takes < 0.5 us
    for (unsigned i=0; i<=3; i++){
        t_i2cChannelState *s = &g_sI2CInst[i];
        s->i2c_state = I2C_START;
        IntTrigger(s->int_addr);
        break;
    }
}

void print_pcf_state()
{
    UARTprintf("\n");
    for(unsigned pcf=0; pcf<=7; pcf++){
        for(unsigned ch=0; ch<=3; ch++){
            t_i2cChannelState *chp = &(g_sI2CInst[ch]);
            t_pcfState *pcfp = &(chp->pcf_state[pcf]);
            UARTprintf(
                "[%2x,%x]: %2x %2x %5x  ",
                pcfp->i2c_addr,
                pcfp->flags,
                pcfp->last_value,
                pcfp->last_err_mcs,
                pcfp->err_cnt
            );
        }
        UARTprintf("\n");
    }
    UARTprintf("\n");
}
