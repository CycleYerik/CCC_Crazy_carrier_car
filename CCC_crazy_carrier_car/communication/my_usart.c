#include "my_usart.h"

uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128]; // usart2,3接收缓冲区
uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1; // 暂存usart2,3接收到的数据
uchar rxflag_u2,rxflag_u3,rxflag_u1; 
int velocity = 30;
extern float acceleration;
extern float x_velocity, y_velocity;
extern float x_move_position , y_move_position; // x、y轴每次移动距离
extern float position_move_velocity ; // 单次位置移动速度

int tim2_count = 0;
int get_motor_real_vel_ok = 0;

__IO bool rxFrameFlag = FALSE;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;
extern float pos , Motor_Cur_Pos_1  , Motor_Cur_Pos_2 , Motor_Cur_Pos_3, Motor_Cur_Pos_4;
extern float vel , Motor_Vel_1 , Motor_Vel_2 , Motor_Vel_3 , Motor_Vel_4 ;

extern float x_error, y_error;

/**
	* @brief   USART1中断函数
	* @param   无
	* @retval  无
	*/
// void USART1_IRQHandler(void)
// {
// 	__IO uint16_t i = 0;

// /**********************************************************
// ***	串口接收中断
// **********************************************************/
// 	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
// 	{
// 		// 未完成一帧数据接收，数据进入缓冲队列
// 		fifo_enQueue((uint8_t)USART1->DR);

// 		// 清除串口接收中断
// 		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
// 	}

// /**********************************************************
// ***	串口空闲中断
// **********************************************************/
// 	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
// 	{
// 		// 先读SR再读DR，清除IDLE中断
// 		USART1->SR; USART1->DR;

// 		// 提取一帧数据命令
// 		rxCount = fifo_queueLength(); for(i=0; i < rxCount; i++) { rxCmd[i] = fifo_deQueue(); }

// 		// 一帧数据接收完成，置位帧标志位
// 		rxFrameFlag = TRUE;
// 	}
// }


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) // 10ms一次
    {
        // tim2_count++; // 用定时器中断实现延时10ms的效果
        // if(tim2_count == 1)
        // {
        //     Emm_V5_Read_Sys_Params(1, S_VEL); 
        //     UART_handle_function_1();
        // }
        // else if(tim2_count == 2)
        // {
        //     Emm_V5_Read_Sys_Params(2, S_VEL);
        //     UART_handle_function_1();
        // }
        // else if(tim2_count == 3)
        // {
        //     Emm_V5_Read_Sys_Params(3, S_VEL);
        //     UART_handle_function_1();
        // }
        // else if(tim2_count == 4)
        // {
        //     Emm_V5_Read_Sys_Params(4, S_VEL);
        //     UART_handle_function_1();
        // }
        // else if (tim2_count >= 5)
        // {   
        //     UART_handle_function_1();
        //     tim2_count = 0;
        // }
    }
    if(htim -> Instance == TIM3) // 1000ms
    {
        // UART_handle_function_2();
        // move_all_direction_pid(acceleration, x_velocity, y_velocity);
        // move_all_direction(acceleration, x_velocity, y_velocity);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
        // move_all_direction_position(acceleration, position_move_velocity, 0, 5); // 不能直接用，里面有HAL_Delay,阻塞导致失败
        
        
    }
}

void UART_handle_function_1()
{
    if(rxflag_u1 != 0)
    {
        int temp = rxflag_u1;
        HAL_Delay(1); // 确保接受完了全部数据
        if(temp == rxflag_u1) 
        {
            UART_receive_process_1(); 
        }
    }
}
void UART_handle_function_2()
{
    if(rxflag_u2 != 0)
    {
        int temp = rxflag_u2;
        HAL_Delay(1); // 确保接受完了全部数据
        if(temp == rxflag_u2) 
        {
            UART_receive_process_2(); 
        }
    }
}

void UART_handle_function_3()
{
    if(rxflag_u3 != 0)
    {
        int temp = rxflag_u3;
        HAL_Delay(1); // 确保接受完了全部数据
        if(temp == rxflag_u3) 
        {
            UART_receive_process_3(); 
        }
    }
}

/// @brief 串口中断回调函数
/// @param huart 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(rxflag_u1 >= 128)
    {
        rxflag_u1 = 0;
    }
    if(rxflag_u2 >= 50)
    {
        rxflag_u2 = 0;
    }
    if(rxflag_u3 >= 50)
    {
        rxflag_u3 = 0;
    }
    rxdata_u1[rxflag_u1++] = received_rxdata_u1;
    rxdata_u2[rxflag_u2++] = received_rxdata_u2;
    rxdata_u3[rxflag_u3++] = received_rxdata_u3;
    if(huart->Instance == USART2)
    {
        HAL_UART_Receive_IT(huart, &received_rxdata_u2, 1); // 每次处理一个字符
    }
    else if(huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(huart, &received_rxdata_u3, 1); // 每次处理一个字符
    }
    else if(huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(huart, &received_rxdata_u1, 1); // 每次处理一个字符
    }
}

/// @brief 处理接收到的电机数据
void UART_receive_process_1()
{
    if (rxflag_u1 > 0)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"hello,summer", strlen("hello,summer"), 50);
        // 将PB0引脚电平翻转
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        if (rxdata_u1[0] == 0x01 && rxdata_u1[1] == 0x35) // && rxCount == 6
        {
            // 拼接成uint16_t类型数据
            vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));

            // 实时转速
            Motor_Vel_1 = vel;

            // 符号
            if (rxdata_u1[2])
            {
                Motor_Vel_1 = -Motor_Vel_1;
            }
            // 将rxdata_u1的数据显示在串口屏上
            // printf("t1.txt=\"m1%s\"\xff\xff\xff", rxdata_u1);
        }
        else if (rxdata_u1[0] == 0x02 && rxdata_u1[1] == 0x35) 
        {
            vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));
            Motor_Vel_2 = vel;

            if (rxdata_u1[2])
            {
                Motor_Vel_2 = -Motor_Vel_2;
            }
            // printf("t1.txt=\"m2%s\"\xff\xff\xff", rxdata_u1);
        }
        else if (rxdata_u1[0] == 3 && rxdata_u1[1] == 0x35) 
        {
            vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));
            Motor_Vel_3 = vel;

            if (rxdata_u1[2])
            {
                Motor_Vel_3 = -Motor_Vel_3;
            }
            // printf("t1.txt=\"m3%s\"\xff\xff\xff", rxdata_u1);
        }
        else if (rxdata_u1[0] == 4 && rxdata_u1[1] == 0x35) 
        {
            vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));
            Motor_Vel_4 = vel;

            if (rxdata_u1[2])
            {
                Motor_Vel_4 = -Motor_Vel_4;
            }
            // printf("t1.txt=\"m4%s\"\xff\xff\xff", rxdata_u1);
        }



        if (rxdata_u1[0] == 1 && rxdata_u1[1] == 0x36 ) //&& rxCount == 8
        {
            // 拼接成uint32_t类型
            pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
                             ((uint32_t)rxdata_u1[4] << 16) |
                             ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));

            // 转换成角度
            Motor_Cur_Pos_1 = (float)pos * 360.0f / 65536.0f;

            // 符号
            if (rxdata_u1[2])
            {
                Motor_Cur_Pos_1 = -Motor_Cur_Pos_1;
            }
        }
        else if (rxdata_u1[0] == 2 && rxdata_u1[1] == 0x36) 
        {
            pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
                             ((uint32_t)rxdata_u1[4] << 16) |
                             ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));
            Motor_Cur_Pos_2 = (float)pos * 360.0f / 65536.0f;

            if (rxdata_u1[2])
            {
                Motor_Cur_Pos_2 = -Motor_Cur_Pos_2;
            }
        }
        else if (rxdata_u1[0] == 3 && rxdata_u1[1] == 0x36) 
        {
            pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
                             ((uint32_t)rxdata_u1[4] << 16) |
                             ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));
            Motor_Cur_Pos_3 = (float)pos * 360.0f / 65536.0f;

            if (rxdata_u1[2])
            {
                Motor_Cur_Pos_3 = -Motor_Cur_Pos_3;
            }
        }
        else if (rxdata_u1[0] == 4 && rxdata_u1[1] == 0x36) 
        {
            pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
                             ((uint32_t)rxdata_u1[4] << 16) |
                             ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));
            Motor_Cur_Pos_4 = (float)pos * 360.0f / 65536.0f;

            if (rxdata_u1[2])
            {
                Motor_Cur_Pos_4 = -Motor_Cur_Pos_4;
            }
        }
        rxflag_u1 = 0;
        memset(rxdata_u1, 0, 128);
    }


    
}

/// @brief 处理树莓派数据
void UART_receive_process_3()
{
    if (rxflag_u3 > 0)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"hello,summer", strlen("hello,summer"), 50);
        // 将PB0引脚电平翻转
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
        // printf("t0.txt=\"%s\"\xff\xff\xff", rxdata_u3);

        //! 处理思路 接受到数据为x、y的偏差值，采用速度控制进行微调，一旦达到目标位置，树莓派发送到位信号，立刻停止电机
        //! 离散式PID控制
        x_error = (float)rxdata_u3[0];
        y_error = (float)rxdata_u3[1];

        rxflag_u3 = 0;
        memset(rxdata_u3, 0, 50);
    }
}

/// @brief 处理串口屏的数据
void UART_receive_process_2()
{
    if (rxflag_u2 > 0)
    {
        // if(
        // rxflag > 50)
        // {
        //     HAL_UART_Transmit(&huart3, (uint8_t*)"too long\r\n", 10, 50);
        // }
        // else
        // {
        //     HAL_UART_Transmit(&huart3, (uint8_t*)"success", strlen("success"), 50);
        //     HAL_UART_Transmit(&huart3, (uint8_t*)rxdata, rxflag, 50);
        // }

        // 处理接收到的数据

        // 全向
        if (rxdata_u2[0] == 0x38)
        {
            // Forward_move_velocity(velocity, acceleration);
            move_all_direction(acceleration, x_velocity, y_velocity);

        }
        // 前进
        else if (rxdata_u2[0] == 0x21)
        {
            // Forward_move_velocity(velocity, acceleration);
            move_all_direction_position(acceleration, position_move_velocity, 0, y_move_position);
            
        }
        // 后退
        else if (rxdata_u2[0] == 0x12)
        {
            // Backward_move_velocity(velocity, acceleration);
            move_all_direction_position(acceleration, position_move_velocity, 0, -y_move_position);

        }
        // 停止
        else if (rxdata_u2[0] == 0x88)
        {
            stop();
        }

        // 左走
        else if (rxdata_u2[0] == 0x13)
        {
            // move_left_velocity(velocity, acceleration);
            move_all_direction_position(acceleration, position_move_velocity, -x_move_position, 0);
        }
        // 右走
        else if (rxdata_u2[0] == 0x14)
        {
            // move_right_velocity(velocity, acceleration);
            move_all_direction_position(acceleration, position_move_velocity, x_move_position, 0);
        }
        // 左旋
        else if (rxdata_u2[0] == 0x15)
        {
            spin_left(velocity, acceleration, 90);
        }
        // 右旋
        else if (rxdata_u2[0] == 0x16)
        {
            spin_right(velocity, acceleration, 90);
        }
        else if (rxdata_u2[0] == 0x66)
        {
            if (velocity < 1000)
            {
                velocity += 10;
            }
            y_velocity += 10;
        }
        else if (rxdata_u2[0] == 0x99)
        {
            if (velocity > 10)
            {
                velocity -= 10;
            }
            y_velocity -= 10;
        }
        else if (rxdata_u2[0] == 0x78)
        {
            x_velocity += 10;
            x_move_position += 0.1;
        }
        else if (rxdata_u2[0] == 0x87)
        {
            x_velocity -= 10;
            x_move_position -= 0.1;
        }

        // 清空接收缓冲区
        rxflag_u2 = 0;
        memset(rxdata_u2, 0, 50);
    }
}

void UART_send_data_u2(char* data)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)data, strlen(data), 50);
}




void usart_SendCmd_u1(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0;
	
	for(i=0; i < len; i++) { usart_SendByte_u1(cmd[i]); }
}

void usart_SendByte_u1(uint16_t data)
{
	__IO uint16_t t0 = 0;
	
	USART1->DR = (data & (uint16_t)0x01FF);

	while(!(USART1->SR & USART_FLAG_TXE))
	{
		++t0; if(t0 > 8000)	{	return; }
	}
}

// void initQueue(void)
// {
// 	rxFIFO.ptrRead  = 0;
// 	rxFIFO.ptrWrite = 0;
// }

// /**
// 	* @brief   入队
// 	* @param   无
// 	* @retval  无
// 	*/
// void fifo_enQueue(uint16_t data)
// {
// 	rxFIFO.buffer[rxFIFO.ptrWrite] = data;
	
// 	++rxFIFO.ptrWrite;
	
// 	if(rxFIFO.ptrWrite >= FIFO_SIZE)
// 	{
// 		rxFIFO.ptrWrite = 0;
// 	}
// }

// /**
// 	* @brief   出队
// 	* @param   无
// 	* @retval  无
// 	*/
// uint16_t fifo_deQueue(void)
// {
// 	uint16_t element = 0;

// 	element = rxFIFO.buffer[rxFIFO.ptrRead];

// 	++rxFIFO.ptrRead;

// 	if(rxFIFO.ptrRead >= FIFO_SIZE)
// 	{
// 		rxFIFO.ptrRead = 0;
// 	}

// 	return element;
// }

// /**
// 	* @brief   判断空队列
// 	* @param   无
// 	* @retval  无
// 	*/
// bool fifo_isEmpty(void)
// {
// 	if(rxFIFO.ptrRead == rxFIFO.ptrWrite)
// 	{
// 		return true;
// 	}

// 	return false;
// }

// /**
// 	* @brief   计算队列长度
// 	* @param   无
// 	* @retval  无
// 	*/
// uint16_t fifo_queueLength(void)
// {
// 	if(rxFIFO.ptrRead <= rxFIFO.ptrWrite)
// 	{
// 		return (rxFIFO.ptrWrite - rxFIFO.ptrRead);
// 	}
// 	else
// 	{
// 		return (FIFO_SIZE - rxFIFO.ptrRead + rxFIFO.ptrWrite);
// 	}
// }