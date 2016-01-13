/*
 * mySpi.h
 *
 *  Created on: Dec 22, 2015
 *      Author: michael
 */

#ifndef FAN_TAS_TIC_CONTROLLER_MYSPI_H_
#define FAN_TAS_TIC_CONTROLLER_MYSPI_H_

//*****************************************************************************
// Defines
//*****************************************************************************
#define N_LEDS_MAX 1024         //Max number of WS2811 LEDs per channel (limited by RAM)
#define SPI_DMA_BUFFER_SIZE 128 //Number of 16 bit SPI words which will be precomputed
                                // 200 = 1 ms worth of SPI data

//*****************************************************************************
// Custom types
//*****************************************************************************

typedef enum{
    SPI_IDLE, SPI_SEND_PING, SPI_SEND_PONG
}t_spiDmaState;

typedef struct {
    uint32_t baseAdr;
    uint32_t dmaChannel;
    uint8_t intNo;              //Hardware interrupt number
    uint8_t *currentLEDByte;    //points into g_spiBuffer
    int32_t nLEDBytesLeft;      //refers to data from g_spiBuffer
    // DMA stuff
    t_spiDmaState state;
    uint16_t pingBuffer[SPI_DMA_BUFFER_SIZE];   //1 ms worth of SPI data
    uint16_t pongBuffer[SPI_DMA_BUFFER_SIZE];   //1 ms worth of SPI data
    // Freertos stuff
    SemaphoreHandle_t semaToReleaseWhenFinished;
}t_spiTransferState;

//*****************************************************************************
// Global vars
//*****************************************************************************
extern const uint16_t g_ssi_lut[16];
extern uint8_t g_spiBuffer[3][N_LEDS_MAX*3];//3 channels * 3 colors --> 9.2 kByte
extern t_spiTransferState g_spiState[3];

//*****************************************************************************
// Function / Task declaations
//*****************************************************************************
void spiSend( uint8_t channel, int32_t nBytes );
void spiSetup();
void spiISR( uint8_t channel );

#endif /* FAN_TAS_TIC_CONTROLLER_MYSPI_H_ */
