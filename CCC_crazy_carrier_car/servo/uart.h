#ifndef	_UART_H
#define	_UART_H

#include <stdint.h>
extern void Uart_Init(uint32_t baudRate);
extern int16_t Uart_Read(uint8_t *buf , uint16_t len, uint32_t timeout);
extern void Uart_Send(uint8_t *buf , uint16_t len);

#endif
