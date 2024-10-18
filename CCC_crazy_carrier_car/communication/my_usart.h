#ifndef __MY_USART_H__
#define __MY_USART_H__

#include "main.h"
#include "stm32f4xx_hal_usart.h"
#include "usart.h"
#include "string.h"
#include "motor.h"
#include "uart_screen.h"
#include "tim.h"
#include "my_timer.h"

#define FIFO_SIZE 128

typedef struct {
	uint16_t buffer[FIFO_SIZE];
	__IO uint8_t ptrWrite;
	__IO uint8_t ptrRead;
}FIFO_t;

void UART_handle_function_1();
void UART_handle_function_2();
void UART_handle_function_3();
void UART_receive_process_1();
void UART_receive_process_2();
void UART_receive_process_3();

void usart_SendCmd_u1(__IO uint8_t *cmd, uint8_t len);
void usart_SendByte_u1(uint16_t data);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void UART_send_data_u2(char* data);

// 定义uchar
typedef unsigned char uchar;

extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;

// void fifo_initQueue(void);
// void fifo_enQueue(uint16_t data);
// uint16_t fifo_deQueue(void);
// bool fifo_isEmpty(void);
// uint16_t fifo_queueLength(void);

#endif
