/*
 * uart.c
 * UART接口
 * 日期: 2020.7.9
 * 作者: 
 */

#include "stm32f4xx.h"
#include "uart.h"

UART_HandleTypeDef UART_InitStructure;

/*---------------
使用USE_USART1_ 宏定义
配置USART1，端口映射(TX)PA9/(RX)PA10
USART1作为舵机串口
------------------*/
#ifdef USE_USART1_
void Uart_Init(uint32_t baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//UART2 GPIO 配置
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pin = GPIO_PIN_9;//PA9 UART1_TX
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_10;//PA10 UART10_RX
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//UART 数据格式配置
  UART_InitStructure.Instance = USART1;
  UART_InitStructure.Init.BaudRate = baudRate;
  UART_InitStructure.Init.WordLength = UART_WORDLENGTH_8B;
  UART_InitStructure.Init.StopBits = UART_STOPBITS_1;
  UART_InitStructure.Init.Parity = UART_PARITY_NONE;
  UART_InitStructure.Init.Mode = UART_MODE_TX_RX;
  UART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART_InitStructure.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UART_InitStructure);
}

#endif

/*---------------
使用USE_USART2_宏定义
配置USART2，端口映射(TX)PA2/(RX)PA3
USART2作为舵机串口
------------------*/
#ifdef USE_USART2_
void Uart_Init(uint32_t baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//UART2 GPIO 配置
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pin = GPIO_PIN_5;//PA2 UART2_TX
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_6;//PA3 UART2_RX
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//UART 数据格式配置
  UART_InitStructure.Instance = USART2;
  UART_InitStructure.Init.BaudRate = baudRate;
  UART_InitStructure.Init.WordLength = UART_WORDLENGTH_8B;
  UART_InitStructure.Init.StopBits = UART_STOPBITS_1;
  UART_InitStructure.Init.Parity = UART_PARITY_NONE;
  UART_InitStructure.Init.Mode = UART_MODE_TX_RX;
  UART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UART_InitStructure.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&UART_InitStructure);
}

#endif

void Uart_Send(uint8_t *buf , uint16_t len)
{
	HAL_UART_Transmit(&UART_InitStructure, buf, len, HAL_MAX_DELAY);
}

int16_t Uart_Read(uint8_t *buf , uint16_t len, uint32_t timeout)
{
	if(HAL_UART_Receive(&UART_InitStructure, buf, len, timeout)==HAL_OK){
		return len;
	}else{
		return 0;
	}
}
