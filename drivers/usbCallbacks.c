/*
 * usbCallbacks.c
 *
 *  Created on: Dec 22, 2015
 *      Author: michael
 */

#include <stdint.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "utils/uartstdio.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usb_serial_structs.h"
#include "usbCallbacks.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "myTasks.h"


//*****************************************************************************
// Handles CDC driver notifications related to control and setup of the device.
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
// \return The return value is event-specific.
//*****************************************************************************
uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData) {
    switch(ui32Event) {									// We are connected to a host and communication is now possible.
        case USB_EVENT_CONNECTED:						// Flush our buffers.
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);
            break;
        case USB_EVENT_DISCONNECTED:					// The host has disconnected.
        case USBD_CDC_EVENT_GET_LINE_CODING:			// Return the current serial communication parameters.
        case USBD_CDC_EVENT_SET_LINE_CODING:			// Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:		// Set the current serial communication parameters.
        case USBD_CDC_EVENT_SEND_BREAK:					// Send a break condition on the serial line.
        case USBD_CDC_EVENT_CLEAR_BREAK:				// Clear the break condition on the serial line.
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;
        default:   // We don't expect to receive any other events.  Ignore any that show but hang in a debug build.
#ifdef DEBUG
        	UARTprintf("ControlHandler( %d ) ... Unexpected ... hang()\n", ui32Event );
            while(1);
#else
            break;
#endif
    }
    return(0);
}

//*****************************************************************************
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
// \return The return value is event-specific.
//*****************************************************************************
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData ){
    switch(ui32Event) {					// Which event have we been sent?
        case USB_EVENT_TX_COMPLETE:		// Since we are using the USBBuffer, we don't need to do anything here.
            break;
        default:   // We don't expect to receive any other events.  Ignore any that show but hang in a debug build.
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }
    return(0);
}

//*****************************************************************************
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
// \param ui32CBData is the client-supplied callback data value for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
// \return The return value is event-specific.
//*****************************************************************************
uint32_t RxHandler( void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData ) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    switch(ui32Event){					// Which event are we being sent?
        case USB_EVENT_RX_AVAILABLE:	// A new packet has been received.  Notify and wake up parser task
        	 vTaskNotifyGiveFromISR( hUSBCommandParser, &xHigherPriorityTaskWoken );
        	 portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		break;
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the command parser is IDLE or 1 if it is
        // in the process of parsing something.
        case USB_EVENT_DATA_REMAINING:
//        	UARTprintf(" USB_EVENT_DATA_REMAINING ");
        	return( USBBufferDataAvailable(&g_sRxBuffer) );
        case USB_EVENT_REQUEST_BUFFER:			// We are being asked to provide a buffer into which the next packet
//        	UARTprintf(" USB_EVENT_REQUEST_BUFFER ");
            return(0);

        default:		// We don't expect to receive any other events.  Ignore any that show but hang in a debug build.
#ifdef DEBUG
            while(1);
#else
            break;
#endif
    }
    return(0);
}
