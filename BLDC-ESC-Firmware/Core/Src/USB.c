/*
 * USB.c
 *
 *  Created on: Jun 29, 2024
 *      Author: Leon
 */
#include "main.h"
#include "usb.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"


void transString(uint16_t byte){
	char txBuf[8];
	sprintf(txBuf,"%u\r\n",byte);
	CDC_Transmit_FS((uint8_t*)txBuf, strlen(txBuf));
}


