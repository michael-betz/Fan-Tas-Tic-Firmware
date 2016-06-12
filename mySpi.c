/*
 * mySpi.c
 * DMA based WS2811 driver
 *
 *  Created on: Dec 22, 2015
 *      Author: michael
 *
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "inc/hw_udma.h"
//*****************************************************************************
// FreeRTOS includes
//*****************************************************************************
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//*****************************************************************************
// TivaWare includes
//*****************************************************************************
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "utils/i2cm_drv.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "utils/cmdline.h"
//*****************************************************************************
// USB stuff
//*****************************************************************************
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "drivers/usb_serial_structs.h"
#include "myTasks.h"
#include "i2cHandlerTask.h"
#include "mySpi.h"

//*****************************************************************************
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
unsigned char ucControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ucControlTable, 1024)
unsigned char ucControlTable[1024];
#else
unsigned char ucControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
// Global vars
//*****************************************************************************
t_spiTransferState g_spiState[3];
uint8_t g_spiBuffer[3][N_LEDS_MAX*3];   //3 channels * 3 colors --> 9.2 kByte


// Non inverted
#define SPI_LOW_VALUE 0x0000
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

// Inverted
//#define SPI_LOW_VALUE 0xFFFF
//const uint16_t g_ssi_lut[16] = {                        // Encodes a 4 bit nibble to a 16 bit SPI word
//    0b0111011101110111,     //0
//    0b0111011101110011,     //1
//    0b0111011100110111,     //2
//    0b0111011100110011,     //3
//    0b0111001101110111,     //4
//    0b0111001101110011,     //5
//    0b0111001100110111,     //6
//    0b0111001100110011,     //7
//    0b0011011101110111,     //8
//    0b0011011101110011,     //9
//    0b0011011100110111,     //A
//    0b0011011100110011,     //B
//    0b0011001101110111,     //C
//    0b0011001101110011,     //D
//    0b0011001100110111,     //E
//    0b0011001100110011,     //F
//};

void spiHwSetup( uint8_t channel, uint32_t ssin_base, uint8_t intNo, uint32_t dmaChannel ){
    ROM_SSIDisable( ssin_base );
    // USer internal 80 MHz clock
    ROM_SSIClockSourceSet( ssin_base, SSI_CLOCK_SYSTEM );
    // SPI at 3.2 MHz, 16 bit SPI words (encoding 4 bit data each)
    ROM_SSIConfigSetExpClk( ssin_base, SYSTEM_CLOCK, SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 3200000, 16 );
    // Enable it
    ROM_SSIEnable( ssin_base );
    // Enable DMA
    ROM_SSIDMAEnable( ssin_base, SSI_DMA_TX );
    // Enable SSI int (will only be triggered by uDMA module)
    ROM_IntEnable( intNo );
    // Assign peripheral to DMA channel
    ROM_uDMAChannelAssign( dmaChannel );
    // Reset attributes
    ROM_uDMAChannelAttributeDisable( dmaChannel,
                                  UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK );
    ROM_uDMAChannelAttributeEnable( dmaChannel, UDMA_ATTR_USEBURST );   //Never use single, always Burst
    // Transfer 16 bit sized items, in 8 item bursts from mem. to SPI buff.
    // Triggers when SPI FIFO is half empty (fixed at 4)
    ROM_uDMAChannelControlSet(  dmaChannel | UDMA_PRI_SELECT,
                                  UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_4 );
    // Init control data structures
    g_spiState[channel].baseAdr = ssin_base;
    g_spiState[channel].dmaChannel = dmaChannel;
    g_spiState[channel].intNo = intNo;
    g_spiState[channel].semaToReleaseWhenFinished = xSemaphoreCreateBinary();
    xSemaphoreGive( g_spiState[channel].semaToReleaseWhenFinished );
}

void spiSetup(){
    //---------------------------------------
    // DMA stuff
    //---------------------------------------
    // Enable the uDMA controller at the system level
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    // Enable the uDMA controller error interrupt. (on bus error)
    ROM_IntEnable(INT_UDMAERR);
    // Enable the uDMA controller.
    ROM_uDMAEnable();
    // Point at the control table to use for channel control structures.
    ROM_uDMAControlBaseSet(ucControlTable);
    //---------------------------------------
    // SPI module stuff
    //---------------------------------------
    // Enable SPI modules
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI1 );
    ROM_SysCtlPeripheralReset(  SYSCTL_PERIPH_SSI1 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI2 );
    ROM_SysCtlPeripheralReset(  SYSCTL_PERIPH_SSI2 );
    ROM_SysCtlPeripheralEnable( SYSCTL_PERIPH_SSI3 );
    ROM_SysCtlPeripheralReset(  SYSCTL_PERIPH_SSI3 );
    // Select the pinout
    ROM_GPIOPinConfigure( GPIO_PF1_SSI1TX );     //SSI1
    ROM_GPIOPinConfigure( GPIO_PB7_SSI2TX );     //SSI2
    ROM_GPIOPinConfigure( GPIO_PD3_SSI3TX );     //SSI3

//   Try open drain config
//    ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_DIR_MODE_HW);
//    ROM_GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_DIR_MODE_HW);
//    ROM_GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_DIR_MODE_HW);
//    ROM_GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD );
//    ROM_GPIOPadConfigSet( GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD );
//    ROM_GPIOPadConfigSet( GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_OD );

//   Default push pull config
    ROM_GPIOPinTypeSSI(   GPIO_PORTF_BASE, GPIO_PIN_1 );
    ROM_GPIOPinTypeSSI(   GPIO_PORTB_BASE, GPIO_PIN_7 );
    ROM_GPIOPinTypeSSI(   GPIO_PORTD_BASE, GPIO_PIN_3 );

//    ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_DIR_MODE_HW);
//    ROM_GPIOPadConfigSet( GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD );

    //---------------------------------------
    // Tivaware stuff
    //---------------------------------------
    // Setup SPI modules
    spiHwSetup( 0, SSI1_BASE, INT_SSI1, UDMA_CH11_SSI1TX );
    spiHwSetup( 1, SSI2_BASE, INT_SSI2, UDMA_CH13_SSI2TX );
    spiHwSetup( 2, SSI3_BASE, INT_SSI3, UDMA_CH15_SSI3TX );
}

uint8_t *fillPiongBuffer( uint8_t *srcPointer, uint16_t *destPointer, int32_t *nBytesLeft ){
    // Encodes a data byte to 2 x 16 bit words to be sent over SPI to WS2811
    // Fills a complete Ping/Pong buffer (destPointer) and returns incremented
    // srcPointer. Decrements nBytesLeft
    // 3594 ticks without compiler optimizations
    uint8_t i;
    if( *nBytesLeft <= 0 ){
        return srcPointer;
    }
    // Copy always one full Ping/Pong buffer
    // (Could contain some dirt from the last transmission)
    for( i=0; i<SPI_DMA_BUFFER_SIZE/2; i++ ){
        *destPointer++ = g_ssi_lut[ (*srcPointer)>>4 ];     //Encode hi nibble to 16 bit SPI word
        *destPointer++ = g_ssi_lut[ (*srcPointer)& 0x0F ];  //Encode lo nibble to 16 bit SPI word
        srcPointer++;
        (*nBytesLeft)--;
    }
    return srcPointer;
}

void spiISR( uint8_t channel ){
    //Is called by uDMA interrupt after one block has been transferred! (takes 640 us)
    //It takes < 50 us to refresh a buffer
    uint32_t temp;
    uint16_t *bufferToRefill;
    t_spiTransferState *state = &g_spiState[channel];
    temp = ROM_SSIIntStatus(state->baseAdr, 1);
    ROM_SSIIntClear(state->baseAdr, temp);
    // Make sure the previous DMA transfer has finished. Else something fishy is going on
    ASSERT( !ROM_uDMAChannelIsEnabled(state->dmaChannel) );
    switch( state->state ){
    case SPI_SEND_PING:
        //---------------------------------------------
        //Send PING buffer, recharge PONG buffer
        //---------------------------------------------
        // Basic = One shot mode. Define source buffer and dest. register and number of data items
        ROM_uDMAChannelTransferSet( state->dmaChannel | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC, state->pingBuffer,
                                       (void *)(state->baseAdr + SSI_O_DR),
                                       SPI_DMA_BUFFER_SIZE );
        ROM_uDMAChannelEnable( state->dmaChannel );
        // Used to have a stack overflow here, and it took me ages to find ! What a PITA !
        state->state = SPI_SEND_PONG;       // Setup next state
        bufferToRefill = state->pongBuffer; // Setup next state
        break;

    case SPI_SEND_PONG:
        //---------------------------------------------
        //Send PONG buffer, recharge PING buffer
        //---------------------------------------------
//        ASSERT( HWREG( state->baseAdr+SSI_O_SR) & SSI_SR_BSY );     //SPI is still transmitting (otherwise buffer underflow)
        ROM_uDMAChannelTransferSet( state->dmaChannel | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC, state->pongBuffer,
                                       (void *)(state->baseAdr + SSI_O_DR),
                                       SPI_DMA_BUFFER_SIZE );
        ROM_uDMAChannelEnable( state->dmaChannel );
//        ROM_SysCtlDelay( 1000 );
        state->state = SPI_SEND_PING;
        bufferToRefill = state->pingBuffer;
        break;

    case SPI_SEND_ZERO:
        //---------------------------------------------
        //Keep line LOW for 50 us to latch LEDs
        //---------------------------------------------
        state->pingBuffer[0] = SPI_LOW_VALUE;
        ROM_uDMAChannelControlSet(  state->dmaChannel | UDMA_PRI_SELECT,
                                      UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_NONE | UDMA_ARB_4 );
        ROM_uDMAChannelTransferSet( state->dmaChannel | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC, state->pingBuffer,
                                       (void *)(state->baseAdr + SSI_O_DR),
                                       11 );    //10 * 16 * 1 / 3.2MHz = 50 us
        ROM_uDMAChannelEnable( state->dmaChannel );
//        ROM_SysCtlDelay( 1000 );
        state->state = SPI_IDLE;
        return;

    case SPI_IDLE:
        xSemaphoreGiveFromISR( state->semaToReleaseWhenFinished, NULL );
        return;

    default:
        UARTprintf("%22s: WTF! unknow state. Ch %d\n", "spiISR()", channel);
        ASSERT(0);
    }
    // Check if a buffer recharge is neccesary
    if( state->nLEDBytesLeft <= 0 ){
        state->state = SPI_SEND_ZERO;    //Send 50 us of LOW to latch LEDs

    } else {
        // Otherwise refill the buffer
        state->currentLEDByte = fillPiongBuffer( state->currentLEDByte, bufferToRefill, &state->nLEDBytesLeft );
    }
}

void spiSend( uint8_t channel, int32_t nBytes ){
    uint8_t *srcPointer;//, i;
    t_spiTransferState *state = &g_spiState[channel];
    ASSERT( !ROM_uDMAChannelIsEnabled(state->dmaChannel) );
    ROM_uDMAChannelControlSet(  state->dmaChannel | UDMA_PRI_SELECT,
                                      UDMA_SIZE_16 | UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_4 );
    //Need to init the PING buffer first
    srcPointer = g_spiBuffer[channel];
//    for( i=0; i<SPI_DMA_BUFFER_SIZE; i++ ){
//        state->pingBuffer[i] = SPI_LOW_VALUE;
//    }
    srcPointer = fillPiongBuffer( srcPointer, state->pingBuffer, &nBytes );
    state->currentLEDByte = srcPointer;
    state->nLEDBytesLeft = nBytes;
    //Start transmission of PING buffer in the ISR
    state->state = SPI_SEND_PING;
    // Artificially Trigger SSI interrupt here
    IntTrigger( state->intNo );
    // As soon as the first DMA is finished, the ISR will take over control
}
