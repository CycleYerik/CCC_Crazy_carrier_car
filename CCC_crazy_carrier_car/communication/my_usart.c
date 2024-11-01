#include "my_usart.h"
/// usart1,2,3接收缓冲区
uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50],rxdata_u5[50]; // usart1,2,3,4接收缓冲区
uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1,received_rxdata_u4,received_rxdata_u5; // 暂存usart1,2,3接收到的数据
uchar rxflag_u2,rxflag_u3,rxflag_u1,rxflag_u4,rxflag_u5; // usart1,2,3接收到的数据的标志位

extern float gyro_z;


/// @brief 串口屏速度控制时用到的速度变量
int velocity = 30;
extern float acceleration;
extern float x_velocity, y_velocity;
extern float x_move_position , y_move_position; // x、y轴每次移动距离
extern float position_move_velocity ; // 单次位置移动速度
extern int is_motor_start_move;

int get_motor_real_vel_ok = 0;

__IO bool rxFrameFlag = FALSE;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;
extern float pos , Motor_Cur_Pos_1  , Motor_Cur_Pos_2 , Motor_Cur_Pos_3, Motor_Cur_Pos_4;
extern float vel , Motor_Vel_1 , Motor_Vel_2 , Motor_Vel_3 , Motor_Vel_4 ;

extern float error_x,error_y,error_x_sum,error_y_sum,error_last_x,error_last_y,error_pre_x,error_pre_y;

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




void UART_handle_function_1(void)
{
    if(rxflag_u1 != 0)
    {
        int temp = rxflag_u1;
        // HAL_Delay(1); // 如果是在main中使用可以加入延时，在定时器中断中调用则不能加
        if(temp == rxflag_u1) 
        {
            UART_receive_process_1(); 
        }
    }
}
void UART_handle_function_2(void)
{
    if(rxflag_u2 != 0)
    {
        int temp = rxflag_u2;
        // HAL_Delay(1); // 如果是在main中使用可以加入延时，在定时器中断中调用则不能加
        if(temp == rxflag_u2) 
        {
            UART_receive_process_2(); 
        }
    }
}

void UART_handle_function_3(void)
{
    if(rxflag_u3 != 0)
    {
        int temp = rxflag_u3;
        // HAL_Delay(1); // 如果是在main中使用可以加入延时，在定时器中断中调用则不能加
        if(temp == rxflag_u3) 
        {
            UART_receive_process_3(); 
        }
    }
}

void UART_handle_function_4(void)
{
    if(rxflag_u4 != 0)
    {
        int temp = rxflag_u4;
        // HAL_Delay(1); // 如果是在main中使用可以加入延时，在定时器中断中调用则不能加
        if(temp == rxflag_u4) 
        {
            UART_receive_process_4(); 
        }
    }
}

void UART_handle_function_5(void)
{
    if(rxflag_u5 != 0)
    {
        int temp = rxflag_u5;
        // HAL_Delay(1); // 如果是在main中使用可以加入延时，在定时器中断中调用则不能加
        if(temp == rxflag_u5) 
        {
            // UART_receive_process_5(); 
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
    if(rxflag_u4 >= 50)
    {
        rxflag_u4 = 0;
    }
    if(rxflag_u5 >= 50)
    {
        rxflag_u5 = 0;
    }
    rxdata_u1[rxflag_u1++] = received_rxdata_u1;
    rxdata_u2[rxflag_u2++] = received_rxdata_u2;
    rxdata_u3[rxflag_u3++] = received_rxdata_u3;
    rxdata_u4[rxflag_u4++] = received_rxdata_u4;
    rxdata_u5[rxflag_u5++] = received_rxdata_u5;
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
    else if(huart->Instance == UART4)
    {
        HAL_UART_Receive_IT(huart, &received_rxdata_u4, 1); // 每次处理一个字符
    }
    else if(huart->Instance == UART5)
    {
        HAL_UART_Receive_IT(huart, &received_rxdata_u5, 1); // 每次处理一个字符
    }
}

/// @brief 处理接收到的电机数据
void UART_receive_process_1(void)
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
void UART_receive_process_3(void)
{
    if (rxflag_u3 > 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)rxdata_u3, strlen(rxdata_u3), 50);
        // 将PB0引脚电平翻转
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        // printf("t0.txt=\"%s\"\xff\xff\xff", rxdata_u3);


        // error_x = (float)rxdata_u3[2];
        // error_y = (float)rxdata_u3[3];
        // x_move_position = error_x;
        // y_move_position = error_y;

        //! 数据包：01(符号位+) 16(x轴数字位) 02(符号位-)05(y轴数字位) 即该数据包表示 error_x = 16, error_y = -5
        error_x = (int) rxdata_u3[0];
        if(rxdata_u3[1] == 0x01)
        {
            error_x = error_x;
        }
        else if(rxdata_u3[1] == 0x02)
        {
            error_x = -error_x;
        }
        error_y = (int) rxdata_u3[2];
        if(rxdata_u3[3] == 0x01)
        {
            error_y = error_y;
        }
        else if(rxdata_u3[3] == 0x02)
        {
            error_y = -error_y;
        }

        // 位置误差累加
        error_x_sum += error_x;
        error_y_sum += error_y;


        // 用位置式PID计算出了x_move_position和y_move_position
        // position_pid(); 


        // 不用位置式PID则取消注释
        x_move_position = error_x;
        y_move_position = error_y;


        // PID 参数更新
        error_last_x = error_x;
        error_last_y = error_y;
        error_pre_x = error_last_x;
        error_pre_y = error_last_y;
        

        
        // 发送的数据为（十六进制）010205 则为右前，x=2，y=5 030508 则为左前，x=-5，y=8 
        float move_c = 0.1; // 移动系数
        float move_once = 0.5;

        
        // if(rxdata_u3[0] == 0x01 )//右前
        // {
        //     // x_move_position = move_c * (float)rxdata_u3[1];
        //     // y_move_position = move_c * (float)rxdata_u3[2];
        //     x_move_position = move_once;
        //     y_move_position = move_once;

        // }
        // else if(rxdata_u3[0] == 0x02)//左前
        // {
        //     // x_move_position = - move_c * (float)rxdata_u3[1] ;
        //     // y_move_position =  move_c * (float)rxdata_u3[2];
        //     x_move_position = -move_once;
        //     y_move_position = move_once;
        // }
        // else if(rxdata_u3[0] == 0x03)//左后
        // {
        //     // x_move_position = - move_c * (float)rxdata_u3[1];
        //     // y_move_position = - move_c * (float)rxdata_u3[2];
        //     x_move_position = -move_once;
        //     y_move_position = -move_once;
        // }
        // else if(rxdata_u3[0] == 0x04)//右后
        // {
        //     // x_move_position = move_c *  (float)rxdata_u3[1];
        //     // y_move_position = - move_c * (float)rxdata_u3[2];
        //     x_move_position = move_once;
        //     y_move_position = -move_once;
        // }
        
        
        if(is_motor_start_move == 0)
        {
            is_motor_start_move = 1;
        }
        rxflag_u3 = 0;
        memset(rxdata_u3, 0, 50);
    }
}

/// @brief 处理精密舵机的数据
void UART_receive_process_2(void)
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

        // // 全向
        // if (rxdata_u2[0] == 0x38)
        // {
        //     // Forward_move_velocity(velocity, acceleration);
        //     move_all_direction(acceleration, x_velocity, y_velocity);

        // }
        // // 前进
        // else if (rxdata_u2[0] == 0x21)
        // {
        //     // Forward_move_velocity(velocity, acceleration);
        //     move_all_direction_position(acceleration, position_move_velocity, 0, y_move_position);
            
        // }
        // // 后退
        // else if (rxdata_u2[0] == 0x12)
        // {
        //     // Backward_move_velocity(velocity, acceleration);
        //     move_all_direction_position(acceleration, position_move_velocity, 0, -y_move_position);

        // }
        // // 停止
        // else if (rxdata_u2[0] == 0x88)
        // {
        //     stop();
        // }

        // // 左走
        // else if (rxdata_u2[0] == 0x13)
        // {
        //     // move_left_velocity(velocity, acceleration);
        //     move_all_direction_position(acceleration, position_move_velocity, -x_move_position, 0);
        // }
        // // 右走
        // else if (rxdata_u2[0] == 0x14)
        // {
        //     // move_right_velocity(velocity, acceleration);
        //     move_all_direction_position(acceleration, position_move_velocity, x_move_position, 0);
        // }
        // // 左旋
        // else if (rxdata_u2[0] == 0x15)
        // {
        //     spin_left(velocity, acceleration, 90);
        // }
        // // 右旋
        // else if (rxdata_u2[0] == 0x16)
        // {
        //     spin_right(velocity, acceleration, 90);
        // }
        // else if (rxdata_u2[0] == 0x66)
        // {
        //     if (velocity < 1000)
        //     {
        //         velocity += 10;
        //     }
        //     y_velocity += 10;
        // }
        // else if (rxdata_u2[0] == 0x99)
        // {
        //     if (velocity > 10)
        //     {
        //         velocity -= 10;
        //     }
        //     y_velocity -= 10;
        // }
        // else if (rxdata_u2[0] == 0x78)
        // {
        //     x_velocity += 10;
        //     // x_move_position += 0.1;
        // }
        // else if (rxdata_u2[0] == 0x87)
        // {
        //     x_velocity -= 10;
        //     // x_move_position -= 0.1;
        // }

        // 清空接收缓冲区
        rxflag_u2 = 0;
        memset(rxdata_u2, 0, 50);
    }
}

/// @brief 处理陀螺仪数据
void UART_receive_process_4(void) 
{
    if (rxflag_u4 > 0)
    {
        // rxdata_u4[7]<<8 | rxdata_u4[6] )* 180 / 32768
        gyro_z = (float)((rxdata_u4[7] << 8 | rxdata_u4[6]) * 180 ); // 32768
        //! 校验位待加入
        rxflag_u4 = 0;
        memset(rxdata_u4, 0, 50);
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
