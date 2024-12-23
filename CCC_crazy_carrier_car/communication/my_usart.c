#include "my_usart.h"
/// usart1,2,3接收缓冲区
uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50],rxdata_u5[50]; // usart1,2,3,4接收缓冲区
uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1,received_rxdata_u4,received_rxdata_u5; // 暂存usart1,2,3接收到的数据
uchar rxflag_u2,rxflag_u3,rxflag_u1,rxflag_u4,rxflag_u5; // usart1,2,3接收到的数据的标志位

extern float gyro_z;


/// @brief 串口屏速度控制时用到的速度变量
int velocity = 30;
extern float acceleration;
extern float x_move_position , y_move_position; // x、y轴每次移动距离
extern float volatile spin_which_direction;
extern float position_move_velocity ; // 单次位置移动速度
extern int is_motor_start_move;
extern int is_slight_move,motor_state,is_slight_spin;

int get_motor_real_vel_ok = 0;


extern int volatile get_plate,is_start_get_plate;
extern int is_get_qrcode_target;
extern volatile int target_colour[6];

volatile int test_slight_move = 1;

float x_err_1 = 0;
float x_err_2 = 0;
float x_err_3 = 0;
float y_err_1 = 0;
float y_err_2 = 0;
float y_err_3 = 0;


#define Kp_slight_move 0.6  // 0.5
#define Ki_slight_move 0.05
#define Kd_slight_move 0.01





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
    rxdata_u5[rxflag_u5++] = received_rxdata_u5;  //! 顺序？
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
        // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        // if (rxdata_u1[0] == 0x01 && rxdata_u1[1] == 0x35) // && rxCount == 6
        // {
        //     // 拼接成uint16_t类型数据
        //     vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
        //                      ((uint16_t)rxdata_u1[4] << 0));

        //     // 实时转速
        //     Motor_Vel_1 = vel;

        //     // 符号
        //     if (rxdata_u1[2])
        //     {
        //         Motor_Vel_1 = -Motor_Vel_1;
        //     }
        //     // 将rxdata_u1的数据显示在串口屏上
        //     // printf("t1.txt=\"m1%s\"\xff\xff\xff", rxdata_u1);
        // }
        // else if (rxdata_u1[0] == 0x02 && rxdata_u1[1] == 0x35) 
        // {
        //     vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
        //                      ((uint16_t)rxdata_u1[4] << 0));
        //     Motor_Vel_2 = vel;

        //     if (rxdata_u1[2])
        //     {
        //         Motor_Vel_2 = -Motor_Vel_2;
        //     }
        //     // printf("t1.txt=\"m2%s\"\xff\xff\xff", rxdata_u1);
        // }
        // else if (rxdata_u1[0] == 3 && rxdata_u1[1] == 0x35) 
        // {
        //     vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
        //                      ((uint16_t)rxdata_u1[4] << 0));
        //     Motor_Vel_3 = vel;

        //     if (rxdata_u1[2])
        //     {
        //         Motor_Vel_3 = -Motor_Vel_3;
        //     }
        //     // printf("t1.txt=\"m3%s\"\xff\xff\xff", rxdata_u1);
        // }
        // else if (rxdata_u1[0] == 4 && rxdata_u1[1] == 0x35) 
        // {
        //     vel = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
        //                      ((uint16_t)rxdata_u1[4] << 0));
        //     Motor_Vel_4 = vel;

        //     if (rxdata_u1[2])
        //     {
        //         Motor_Vel_4 = -Motor_Vel_4;
        //     }
        //     // printf("t1.txt=\"m4%s\"\xff\xff\xff", rxdata_u1);
        // }



        // if (rxdata_u1[0] == 1 && rxdata_u1[1] == 0x36 ) //&& rxCount == 8
        // {
        //     // 拼接成uint32_t类型
        //     pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
        //                      ((uint32_t)rxdata_u1[4] << 16) |
        //                      ((uint32_t)rxdata_u1[5] << 8) |
        //                      ((uint32_t)rxdata_u1[6] << 0));

        //     // 转换成角度
        //     Motor_Cur_Pos_1 = (float)pos * 360.0f / 65536.0f;

        //     // 符号
        //     if (rxdata_u1[2])
        //     {
        //         Motor_Cur_Pos_1 = -Motor_Cur_Pos_1;
        //     }
        // }
        // else if (rxdata_u1[0] == 2 && rxdata_u1[1] == 0x36) 
        // {
        //     pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
        //                      ((uint32_t)rxdata_u1[4] << 16) |
        //                      ((uint32_t)rxdata_u1[5] << 8) |
        //                      ((uint32_t)rxdata_u1[6] << 0));
        //     Motor_Cur_Pos_2 = (float)pos * 360.0f / 65536.0f;

        //     if (rxdata_u1[2])
        //     {
        //         Motor_Cur_Pos_2 = -Motor_Cur_Pos_2;
        //     }
        // }
        // else if (rxdata_u1[0] == 3 && rxdata_u1[1] == 0x36) 
        // {
        //     pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
        //                      ((uint32_t)rxdata_u1[4] << 16) |
        //                      ((uint32_t)rxdata_u1[5] << 8) |
        //                      ((uint32_t)rxdata_u1[6] << 0));
        //     Motor_Cur_Pos_3 = (float)pos * 360.0f / 65536.0f;

        //     if (rxdata_u1[2])
        //     {
        //         Motor_Cur_Pos_3 = -Motor_Cur_Pos_3;
        //     }
        // }
        // else if (rxdata_u1[0] == 4 && rxdata_u1[1] == 0x36) 
        // {
        //     pos = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
        //                      ((uint32_t)rxdata_u1[4] << 16) |
        //                      ((uint32_t)rxdata_u1[5] << 8) |
        //                      ((uint32_t)rxdata_u1[6] << 0));
        //     Motor_Cur_Pos_4 = (float)pos * 360.0f / 65536.0f;

        //     if (rxdata_u1[2])
        //     {
        //         Motor_Cur_Pos_4 = -Motor_Cur_Pos_4;
        //     }
        // }
        rxflag_u1 = 0;
        memset(rxdata_u1, 0, 128);
    }


    
}

/// @brief 处理树莓派数据
void UART_receive_process_3(void)
{
    if (rxflag_u3 > 0)
    {
        // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);



        is_motor_start_move = 1;


        // 得到任务
        if (is_get_qrcode_target < 2) //! 此处还可以加入更复杂的校验位，避免误操作
        {
            if ((int)rxdata_u3[0] == 9 && (int)rxdata_u3[1] == 9)
            {
                for (int i = 2; i < 8; i++)
                {
                    if ((int)rxdata_u3[i] == 1 || (int)rxdata_u3[i] == 2 || (int)rxdata_u3[i] == 3)
                    {
                        target_colour[i-2] = (int)rxdata_u3[i];
                    }
                }
                is_get_qrcode_target++;
            }
        }

        // 在从转盘抓取
        if (is_start_get_plate == 1) 
        {
            if (rxdata_u3[0] == 0x01) //! 此处可以加入更复杂的校验位
            {
                get_plate = 1;
            }
            if (rxdata_u3[0] == 0x02) // 绿
            {
                get_plate = 2;
            }
            if (rxdata_u3[0] == 0x03) // 蓝
            {
                get_plate = 3;
            }
        }



        // 进行直线微调
        if(is_slight_spin == 1)
        {
            if(rxdata_u3[0] == 0x01)
            {
                if((float)rxdata_u3[1] > 10)
                {
                    spin_which_direction = 10;
                }
                else if((float)rxdata_u3[1] < 0.5)
                {
                    spin_which_direction = 0.5;
                }
                else{
                    spin_which_direction =(float)rxdata_u3[1];
                }
                
            }
            else if(rxdata_u3[0] == 0x02)
            {
                if((float)rxdata_u3[1] >10)
                {
                    spin_which_direction = -10;
                }
                else if((float)rxdata_u3[1] < 0.5)
                {
                    spin_which_direction = -0.5;
                }
                else{
                    spin_which_direction = -(float)rxdata_u3[1];
                }
            }
            
        }

        // 直线微调停止
        if( rxdata_u3[0] == 0x27 )
        {
            spin_which_direction = 0;
        }

        // 直线微调结束
        if( rxdata_u3[0] == 0x28 )
        {
            is_slight_spin = 0;
        }

        // if(rxdata_u3[0] == 0x99 && rxdata_u3[1] == 0x99 && rxdata_u3[2] == 0x99 && rxdata_u3[3] == 0x99)
        // {
        //     is_slight_move = 1;
        // }

        
        // 进行位置微调
        if(is_slight_move == 1)  
        {
            //将原始的偏差数据转换为实际的移动方向

            //将十六进制的数据转换为十进制
            if(rxdata_u3[0] == 0x01)
            {
                x_move_position = (float) rxdata_u3[1];
                // x_move_position = 10;
                // HAL_UART_Transmit(&huart3, (uint8_t*)"right front", strlen("right front"), 50);

            }
            else if(rxdata_u3[0] == 0x02)
            {
                x_move_position = - (float) rxdata_u3[1];
                // x_move_position = -10;
                // HAL_UART_Transmit(&huart3, (uint8_t*)"left front", strlen("left front"), 50);
            }
            if(rxdata_u3[2] == 0x01)
            {
                y_move_position = (float) rxdata_u3[3];
                // y_move_position = 10;
            }
            else if(rxdata_u3[2] == 0x02)
            {
                y_move_position = - (float) rxdata_u3[3];
                // y_move_position = -10;
            }
            x_move_position *= 0.1;  //! magic number
            y_move_position *= 0.1;

            // 对位置微调进行PID控制
            x_err_1 = x_move_position;
            y_err_1 = y_move_position;
            x_err_2 = x_err_1;
            y_err_2 = y_err_1;
            x_err_3 = x_err_2;
            y_err_3 = y_err_2;

            x_move_position = Kp_slight_move * (x_err_1) + Ki_slight_move * (x_err_1+x_err_2 + x_err_3) + Kd_slight_move * (x_err_3+x_err_1 - 2*x_err_2)/2;
            y_move_position = Kp_slight_move * (y_err_1) + Ki_slight_move * (y_err_1 + x_err_2 + x_err_3) + Kd_slight_move * (y_err_3+y_err_1 - 2*y_err_2)/2    ;





            if(x_move_position > 5)
            {
                x_move_position = 5;
            }
            if(y_move_position > 5)
            {
                y_move_position = 5;
            }
            if(x_move_position < -5)
            {
                x_move_position = -5;
            }
            if(y_move_position < -5)
            {
                y_move_position = -5;
            }
            if(x_move_position < 0.5 && x_move_position >0)
            {
                x_move_position = 0.5;
            }
            if(y_move_position < 0.5 && y_move_position >0)
            {
                y_move_position = 0.5;
            }
            if(x_move_position > -0.5 && x_move_position <0)
            {
                x_move_position = -0.5;
            }
            if(y_move_position > -0.5 && y_move_position <0)
            {
                y_move_position = -0.5;
            }


            
            
        }



        if( rxdata_u3[0] == 0x17 ) // !停止移动
        {
            // is_slight_move = 0;
            // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50);
            x_move_position = 0;
            y_move_position = 0;


 
        }

        if(rxdata_u3[0] == 0x18 )  //结束微调
        {
            // motor_state = 0;
            is_slight_move = 0;
            test_slight_move = 0;
            x_move_position = 0;
            y_move_position = 0;

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
        //! 校验位待加入
        
        //寻找帧头
        int i = 0;
        for(i = 0; i < rxflag_u4; i++)
        {
            if(rxdata_u4[i] == 0x55 && rxdata_u4[i+1] == 0x53)
            {
                break;
            }
        }
        //如果找到帧头
        if(i < rxflag_u4)
        {
            //提取数据
            uint16_t temp = (uint16_t)((rxdata_u4[i+7] << 8 )| rxdata_u4[i+6]);
            gyro_z = (float)(temp * 180.0/32768.0); // / 32768.0
        }
        // printf("t0.txt=\"%.3f\"\xff\xff\xff", gyro_z);
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
