/*
 * suart.h
 *
 *  Created on: 11 Aug 2020
 *      Author: Steve Chang
 */
#ifndef _SUART_H_
#define _SUART_H_

#include <stdio.h>
#include "bool.h"

typedef enum { B1200, B2400, B4800, B9600, B19200, B38400, B115200 } BaudRate;

void SoftUart0_Init(BaudRate br, uint16_t buffer_size);
void SoftUart0_Task(void *p);

void SoftUart0_Write(uint8_t *data, size_t len);
int SoftUart0_Read(uint8_t *data, size_t len);
int SoftUart0_Poll(void);
int SoftUart0_Poll2(uint16_t timeout_ms);

#endif
