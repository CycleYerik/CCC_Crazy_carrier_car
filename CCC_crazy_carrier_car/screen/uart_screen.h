#ifndef __UART_SCREEN_H__
#define __UART_SCREEN_H__

#include "usart.h"
#include "main.h"
#include <stdio.h>

void send_data_origin(int a,int ch);
void itoa(int num,char str[] );
void send_screen_data(char *data);

#endif

