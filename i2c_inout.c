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
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_nvic.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "my_uartstdio.h"
#include "utils/ustdlib.h"
#include "myTasks.h"
#include "i2c_inout.h"

// Four TI I2C driver instances for 4 I2C channels
t_i2cChannelState g_sI2CInst[4];

// current bcmBuffer index for all PCF outputs,
// set by trigger_i2c_cycle()
static unsigned g_bcmIndex = 0;
// Counts how many PCM cycles have been triggered
static unsigned g_i2c_cycle = 0;

void i2CIntHandler0(void) {i2c_isr(0);}
void i2CIntHandler1(void) {i2c_isr(1);}
void i2CIntHandler2(void) {i2c_isr(2);}
void i2CIntHandler3(void) {i2c_isr(3);}

// Get i2c channel from base_addr
static unsigned _get_ch(unsigned b)
{
    return (b - I2C0_BASE) >> 12;
}

static void i2cUnstucker(uint32_t base, uint8_t pin_SCL, uint8_t pin_SDA) {
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
        REPORT_ERROR("ER:0025\n");
        UARTprintf("i2cUnstucker()        : pin stuck low @%x: ", base);
        if(rVal & pin_SCL)
            UARTprintf("SCL ");
        if(rVal & pin_SDA)
            UARTprintf("SDA ");
        UARTprintf("\n");
    }
}

// Return a PCF instance specific for pin
t_pcf_state *get_pcf(t_hw_index *pin)
{
    if (!pin) return NULL;
    unsigned c = pin->channel;
    unsigned a = pin->i2c_addr - PCF_LOWEST_ADDR;
    if (c > C_I2C3 || a >= PCF_MAX_PER_CHANNEL) return NULL;
    return &(g_sI2CInst[c].pcf_state[a]);
}

// Pre calculate and SET the values which need to be output on a pin to get
// Binary Code Modulation (BCM) with a certain power level.
void set_bcm(uint8_t *bcmBuffer, uint8_t pin, uint8_t pwmValue) {
    if (pin > 7) return;
    taskENTER_CRITICAL();
    for (unsigned j=0; j < N_BIT_PWM; j++) {
        HWREGBITB(bcmBuffer, pin) = HWREGBITB(&pwmValue, j);
        bcmBuffer++;
    }
    taskEXIT_CRITICAL();
}

// Return PWM value of certain pin from bcmbuffer
uint8_t get_bcm(uint8_t *bcmBuffer, uint8_t pin) {
    uint8_t temp = 0;
    if (pin > 7) return 0;
    for (unsigned j=0; j<N_BIT_PWM; j++) {
        HWREGBITB(&temp, j) = HWREGBITB(bcmBuffer, pin);
        bcmBuffer++;
    }
    return temp;
}

static void my_pcf_init(t_pcf_state *pcf)
{
    pcf->flags = FPCF_RENABLED;
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
    // Timeout cycles for a stuck low SCL line (times 16)
    HWREG(base_addr + I2C_O_MCLKOCNT) = 32 >> 4;
    IntPendClear(int_addr);
    IntEnable(int_addr);
    ROM_I2CMasterIntEnableEx(
        base_addr,
        I2C_MASTER_INT_DATA | I2C_MASTER_INT_NACK | I2C_MASTER_INT_TIMEOUT
    );
}

// Send byte over i2c and block (no interrupts)
// b = base_addr, addr = 7 bit i2c address
static void stupid_i2c_send(uint32_t b, uint8_t addr, uint8_t data)
{
    unsigned i=0;
    while (HWREG(b + I2C_O_MCS) & I2C_MCS_BUSY) {
        if (i++ > 0x0000FFFF) {
            REPORT_ERROR("ER:0024\n");
            UARTprintf("stupid_i2c_send(): timeout on ch %x! SDA pullups installed?\n", _get_ch(b));
            return;
        }
    }
    HWREG(b + I2C_O_MSA) = (addr << 1) | 0;
    HWREG(b + I2C_O_MDR) = data;
    HWREG(b + I2C_O_MCS) = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
}

// Send, then receive tx/rx_len bytes over i2c and yield until interrupt
// b = base_addr, addr = 7 bit i2c address
// can only be called by process_IO(),
// must only be called by process_IO() at the right time
void i2c_tx_rx_n(uint32_t b, t_i2cCustom *i2c)
{
    if ((i2c == NULL) || ((i2c->nWrite == 0) && (i2c->nRead == 0))) return;
    unsigned tempFlags, mcs;
    uint8_t *tx_data = i2c->writeBuff;
    uint8_t *rx_data = i2c->readBuff;
    if (i2c->nWrite && (tx_data == NULL)) return;
    if (i2c->nRead && (rx_data == NULL)) return;
    i2c->flags = 0;
    while (HWREG(b + I2C_O_MCS) & I2C_MCS_BUSY);

    // send out i2c->nWrite bytes
    if (i2c->nWrite) {
        HWREG(b + I2C_O_MSA) = (i2c->i2c_addr << 1) | 0;
        for (unsigned i = 0; i < i2c->nWrite; i++) {
            tempFlags = I2C_MCS_RUN;
            // send START on first iteration
            if (i == 0)
                tempFlags |= I2C_MCS_START;
            if (i >= i2c->nWrite - 1 && i2c->nRead == 0)
                tempFlags |= I2C_MCS_STOP;
            HWREG(b + I2C_O_MDR) = *tx_data++;
            HWREG(b + I2C_O_MCS) = tempFlags;
            xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
            ledOut(2);
            ledOut(0);
            mcs = HWREG(b + I2C_O_MCS);
            // Store NACK error in flags
            i2c->flags |= (mcs >> I2C_MCS_ADRACK) & 0x03;
        }
        // still in TRANSMIT state when we leave
    }

    // receive i2c->nRead bytes
    if (i2c->nRead) {
        HWREG(b + I2C_O_MSA) = (i2c->i2c_addr << 1) | 1;
        for (unsigned i = 0; i < i2c->nRead; i++) {
            tempFlags = I2C_MCS_RUN;
            // send repeated START on first iteration
            if (i == 0)
                tempFlags |= I2C_MCS_START;
            if (i >= i2c->nRead - 1)
                tempFlags |= I2C_MCS_STOP;
            else
                tempFlags |= I2C_MCS_ACK;
            HWREG(b + I2C_O_MCS) = tempFlags;
            xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
            ledOut(2);
            ledOut(0);
            *rx_data++ = HWREG(b + I2C_O_MDR);
            mcs = HWREG(b + I2C_O_MCS);
            i2c->flags |= ((mcs >> I2C_MCS_ADRACK) << 2) & 0x0C;
        }
    }
}

// carry out a custom i2c transaction and yield until done
// can only be called by process_IO(),
// which makes sure the timing is right. It gets sneaked in after all
// PCFs are read / written.
void handle_i2c_custom()
{
    char *hexStr=NULL, *chr=NULL;
    t_i2cCustom i2c;
    // Do up to one transactions per PCF cycle
    if (!xQueueReceive(g_i2c_queue, &i2c, 0))
        return;
    if (i2c.channel > 3)
        goto handle_i2c_custom_finally;
    t_i2cChannelState *c = &g_sI2CInst[i2c.channel];
    if (c->i2c_state != I2C_IDLE) {
        UARTprintf("handle_i2c_custom(): Error! I2C not in IDLE state\n");
        REPORT_ERROR("ER:0021\n");
        goto handle_i2c_custom_finally;
    }
    c->i2c_state = I2C_CUSTOM;
    i2c_tx_rx_n(c->base_addr, &i2c);
    c->i2c_state = I2C_IDLE;

    hexStr = pvPortMalloc(2 * i2c.nRead + 24);
    if (!hexStr) {
        UARTprintf("handle_i2c_custom(): Could not allocate hexStr buffer!\n");
        goto handle_i2c_custom_finally;
    }
    int temp = usprintf(hexStr, "I2: %x, %02x", i2c.channel, i2c.flags);
    chr = &hexStr[temp];
    if (i2c.nRead) {
        hexStr = pvPortMalloc(2 * i2c.nRead + 24);
        if (!hexStr) {
            UARTprintf("handle_i2c_custom(): Could not allocate hexStr buffer!\n");
            REPORT_ERROR("ER:0022\n");
            goto handle_i2c_custom_finally;
        }
        int temp = usprintf(hexStr, "I2: %x, %02x", i2c.channel, i2c.flags);
        chr = &hexStr[temp];

        *chr++ = ',';
        *chr++ = ' ';
        chr = buffToHex(i2c.readBuff, i2c.nRead, chr);
        temp += 2 + 2 * i2c.nRead;

        *chr++ = '\n';
        *chr++ = '\0';
        temp += 1;
        ts_usbSend((uint8_t *)hexStr, temp);
    }

handle_i2c_custom_finally:
    vPortFree(i2c.readBuff);  i2c.readBuff  = NULL;
    vPortFree(i2c.writeBuff); i2c.writeBuff = NULL;
    vPortFree(hexStr); hexStr = NULL;
}

// b = base_addr
// Returns true if a read or write has been started
static bool setup_pcf_rw(unsigned b, t_pcf_state *pcf)
{
    if (pcf->flags & (FPCF_WENABLED | FPCF_RENABLED)) {
        // If bus is busy or I2C master is already busy,
        // don't start a new transaction
        if(HWREG(b + I2C_O_MCS) & (I2C_MCS_BUSBSY | I2C_MCS_BUSY)) {
            UARTprintf("setup_pcf_rw(): busy error\n");
            REPORT_ERROR("ER:0023\n");
            return false;
        }
    }
    if (pcf->flags & FPCF_WENABLED) {
        // UARTprintf(" W%2x ", pcf->i2c_addr);
        HWREG(b + I2C_O_MSA) = (pcf->i2c_addr << 1) | 0;
        HWREG(b + I2C_O_MDR) = pcf->bcm_buffer[g_bcmIndex];
        HWREG(b + I2C_O_MCS) = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
        return true;
    }
    if (pcf->flags & FPCF_RENABLED) {
        // START condition followed by RECEIVE and STOP condition
        // (master remains in Idle state).
        // UARTprintf(" R%2x ", pcf->i2c_addr);
        HWREG(b + I2C_O_MSA) = (pcf->i2c_addr << 1) | 1;
        HWREG(b + I2C_O_MCS) = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
        return true;
    }
    return false;
}

// Each I2C ISR has a channel number (0-3)
// When the ISR is done, the pcfInReader task is notified
// the bit corresponding to its CH# is set in the task noti. value
// When all 4 bits are set, the task continues
static void _isr_notify(unsigned flags, BaseType_t *hpw)
{
    // indicates Which channels are done
    static volatile unsigned isrFlags=0;
    isrFlags |= flags;
    if (isrFlags == 0x0F) {
        isrFlags = 0;
        xTaskNotifyFromISR(hPcfInReader, 0, eNoAction, hpw);
    }
}

void i2c_isr(unsigned channel)
{
    // # Interrupt sources
    //   * Master transaction completed
    //   * Master arbitration lost
    //   * Master transaction error
    //   * Master bus timeout
    t_i2cChannelState *state = &g_sI2CInst[channel];
    BaseType_t hpw = pdFALSE;  // true if returning to a higher prio task
    unsigned b = state->base_addr;
    unsigned mcs = HWREG(b + I2C_O_MCS);  // read clears status bits!!!
    t_pcf_state *pcf;
    ledOut(1);
    ROM_I2CMasterIntClear(b);

    if (mcs & I2C_MCS_BUSY) {
        // When the BUSY bit is set, the other I2C status bits are not valid.
        // Actually this gets triggered on ADR_ACK error!
        // UARTprintf("<%x>", mcs);
        ledOut(0);
        return;
    }

    switch(state->i2c_state){
        case I2C_START:
            // Start a single byte read
            ledOut(2);
            state->currentPcf = 0;
            pcf = state->pcf_state;
            // Setup first read or write
            while(!setup_pcf_rw(b, pcf)) {
                state->currentPcf++;
                if (state->currentPcf >= PCF_MAX_PER_CHANNEL){
                    // Nothing to do, notify PCF_Reader
                    state->i2c_state = I2C_IDLE;
                    _isr_notify((1 << channel), &hpw);
                    break;
                }
                pcf++;
            }
            state->i2c_state = I2C_PCF;
            break;

        case I2C_PCF:
            // Evaluate result of the single byte read or write
            ledOut(3);
            pcf = &(state->pcf_state[state->currentPcf]);
            pcf->last_mcs = mcs;
            // Check error bits and increment error counter
            // Errata I2C#07: DATACK bit is not cleared on read!
            if (mcs & (I2C_MCS_ADRACK | I2C_MCS_ARBLST | I2C_MCS_CLKTO)) {
                pcf->err_cnt++;
            } else if (pcf->value_target && (pcf->flags & FPCF_RENABLED)) {
                // No error, take read data value and store it
                *(pcf->value_target) = HWREG(b + I2C_O_MDR);
            }
            // Setup next read or write
            do {
                state->currentPcf++;
                if (state->currentPcf >= PCF_MAX_PER_CHANNEL){
                    state->i2c_state = I2C_IDLE;
                    _isr_notify((1 << channel), &hpw);
                    break;
                }
            } while (!setup_pcf_rw(b, ++pcf));
            break;

        case I2C_CUSTOM:
            // 1 byte has been transmitted / received
            // UARTprintf("<C>");
            _isr_notify(0x0F, &hpw);
            break;

        case I2C_IDLE:
            UARTprintf("<I>");
            break;

        default:
            UARTprintf("<?%x?>", state->i2c_state);
            state->i2c_state = I2C_IDLE;
    }
    portYIELD_FROM_ISR(hpw);
    ledOut(0);
}

static void init_i2c_gpio(unsigned b, unsigned sda, unsigned scl)
{
    // see Table 10-3, p. 657
    // in http://www.ti.com/lit/ds/symlink/tm4c123gh6pm.pdf
    HWREG(b + GPIO_O_AFSEL) |= scl | sda;  // Alternate function select
    HWREG(b + GPIO_O_ODR) |= sda;          // Open drain mode
    HWREG(b + GPIO_O_DEN) |= scl | sda;    // Digital enable
    HWREG(b + GPIO_O_DR8R) |= scl | sda;   // 8 mA drive strength
    HWREG(b + GPIO_O_PUR) |= scl | sda;    // Weak pullup enabled
}

// Resets GPIO and I2C hardware
// isr_init = false: send 0x00 to all PCFs
// isr_init = true:  init data structures and interrupts
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
    init_i2c_gpio(GPIO_PORTB_BASE, (1 << 3), (1 << 2));  // I2C0
    init_i2c_gpio(GPIO_PORTA_BASE, (1 << 7), (1 << 6));  // I2C1
    init_i2c_gpio(GPIO_PORTE_BASE, (1 << 5), (1 << 4));  // I2C2
    init_i2c_gpio(GPIO_PORTD_BASE, (1 << 1), (1 << 0));  // I2C3

    // Initialize the I2C master module for 400 kHz
    ROM_I2CMasterInitExpClk(I2C0_BASE, SYSTEM_CLOCK, true);
    ROM_I2CMasterInitExpClk(I2C1_BASE, SYSTEM_CLOCK, true);
    ROM_I2CMasterInitExpClk(I2C2_BASE, SYSTEM_CLOCK, true);
    ROM_I2CMasterInitExpClk(I2C3_BASE, SYSTEM_CLOCK, true);

    if (isr_init){
        g_i2c_cycle = 0;
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
    } else {
        // interrupts are not initialized yet, use blocking I2C
        for (unsigned p=0; p<PCF_MAX_PER_CHANNEL; p++) {
            stupid_i2c_send(I2C0_BASE, 0x20+p, 0x00);
            stupid_i2c_send(I2C1_BASE, 0x20+p, 0x00);
            stupid_i2c_send(I2C2_BASE, 0x20+p, 0x00);
            stupid_i2c_send(I2C3_BASE, 0x20+p, 0x00);
        }
    }
}

// Start one I2C read / write cycle, which will run completely within the ISR
void trigger_i2c_cycle()
{
    static unsigned bcm_ticks=0;
    g_i2c_cycle++;
    bcm_ticks++;
    // Cycle through the bcm_buffer in a binary way
    if (bcm_ticks >= (1 << g_bcmIndex)) {
        bcm_ticks = 0;
        if(g_bcmIndex >= N_BIT_PWM - 1) {
            g_bcmIndex = 0;
        } else {
            g_bcmIndex++;
        }
    }
    // Trigger a new i2c sequence (takes < 0.5 us until completion)
    t_i2cChannelState *s = g_sI2CInst;
    for (unsigned i=0; i<=3; i++) {
        s->i2c_state = I2C_START;

        // Have a look at the error counts every 5 seconds
        if ((g_i2c_cycle % PCF_ERR_CHECK_CYCLE) == 0) {
            t_pcf_state *pcf = s->pcf_state;
            for (unsigned p=0; p<PCF_MAX_PER_CHANNEL; p++) {
                // If too many writes failed in the 5 s window, shutdown 24 V!
                if (pcf->flags & FPCF_WENABLED) {
                    if (pcf->err_cnt > PCF_ERR_CNT_DISABLE) {
                        // Panic shutdown!!!!
                        DISABLE_SOLENOIDS();
                        ROM_GPIOPinWrite(GPIO_PORTF_BASE, 0x0E, 1 << 1);  // red LED
                        REPORT_ERROR("ER:0100\n");
                        UARTprintf("trigger_i2c_cycle(): too many write errors!\ni2c_addr = %02x. Fatal! Rebooting!", pcf->i2c_addr);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);

                        // Software reset
                        HWREG(NVIC_APINT) = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                        return;  // We should never get here
                    } else {
                        pcf->err_cnt = 0;
                    }
                // If too many reads failed within the first 5 s, give up on
                // reading these PCFs. There's probably nothing connected there.
                } else if ((pcf->flags & FPCF_RENABLED) && (g_i2c_cycle == PCF_ERR_CHECK_CYCLE)) {
                    if (pcf->err_cnt > PCF_ERR_CNT_DISABLE)
                        pcf->flags = 0;
                }

                pcf++;
            }
        }

        IntTrigger(s->int_addr);
        s++;
    }
}

void print_pcf_state()
{
    UARTprintf("Syntax: R/W[HW_INDEX]: VAL (ERR_CNT)\n");
    UARTprintf("----------------------------------------------------------------------------------\n");
    UARTprintf("        CHANNEL_0          CHANNEL_1          CHANNEL_2          CHANNEL_3\n");
    UARTprintf("----------------------------------------------------------------------------------\n");
    for(unsigned pcf=0; pcf<=7; pcf++){
        UARTprintf("I2C_%2x  ", 0x20 + pcf);
        for(unsigned ch=0; ch<=3; ch++){
            unsigned hw_index = 0x40 + ch * 0x40 + pcf * 8;
            t_i2cChannelState *chp = &(g_sI2CInst[ch]);
            t_pcf_state *pcfp = &(chp->pcf_state[pcf]);
            if (pcfp->flags & FPCF_WENABLED){
                UARTprintf(
                    "W[%3x]:    (%4x)  ",
                    hw_index,
                    pcfp->err_cnt
                );
            } else if (pcfp->flags & FPCF_RENABLED) {
                UARTprintf(
                    "R[%3x]: %2x (%4x)  ",
                    hw_index,
                    *pcfp->value_target,
                    pcfp->err_cnt
                );
            } else {
                UARTprintf(
                    " [%3x]:            ",
                    hw_index
                );
            }
        }
        UARTprintf("\n");
    }
    UARTprintf("\n");
}
