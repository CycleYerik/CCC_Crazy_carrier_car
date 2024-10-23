/*
 * uart.c
 * UART�ӿ�
 * ����: 2020.7.9
 * ����: 
 */

#include "stm32f4xx.h"
#include "uart.h"

UART_HandleTypeDef UART_InitStructure;

/*---------------
ʹ��USE_USART1_ �궨��
����USART1���˿�ӳ��(TX)PA9/(RX)PA10
USART1��Ϊ�������
------------------*/
#ifdef USE_USART1_
void Uart_Init(uint32_t baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//UART2 GPIO ����
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pin = GPIO_PIN_9;//PA9 UART1_TX
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_10;//PA10 UART10_RX
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//UART ���ݸ�ʽ����
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
ʹ��USE_USART2_�궨��
����USART2���˿�ӳ��(TX)PA2/(RX)PA3
USART2��Ϊ�������
------------------*/
#ifdef USE_USART2_
void Uart_Init(uint32_t baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//UART2 GPIO ����
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pin = GPIO_PIN_5;//PA2 UART2_TX
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin = GPIO_PIN_6;//PA3 UART2_RX
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//UART ���ݸ�ʽ����
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
