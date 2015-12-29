/*
 * usbCallbacks.h
 *
 *  Created on: Dec 22, 2015
 *      Author: michael
 */

#ifndef USBDRIVER_USBCALLBACKS_H_
#define USBDRIVER_USBCALLBACKS_H_

uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData);
uint32_t TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData );
uint32_t RxHandler( void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData );



#endif /* USBDRIVER_USBCALLBACKS_H_ */
