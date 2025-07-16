#include "my_usart.h"

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!  






/// usart1,2,3接收缓冲区
extern int is_raspi_get_massage;
uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50],rxdata_u5[50]; // usart1,2,3,4接收缓冲区
uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1,received_rxdata_u4,received_rxdata_u5; // 暂存usart1,2,3接收到的数据
uchar rxflag_u2,rxflag_u3,rxflag_u1,rxflag_u4,rxflag_u5; // usart1,2,3接收到的数据的标志位


//串口屏打印数组
extern volatile char print_screen_buffer_xy[][4];
extern volatile char print_screen_buffer_thetar[][4]; // 串口屏打印缓冲区
extern volatile int print_screen_buffer_index_xy; // 串口屏打印缓冲区索引
extern volatile int print_screen_buffer_index_thetar; // 串口屏打印缓冲区索引
extern int is_show_origin_xy_data ;
extern int is_show_origin_theta_data ;

extern float volatile gyro_z;

extern int servo_adjust_status;

/// @brief 串口屏速度控制时用到的速度变量
int velocity = 30;
extern float acceleration;
extern float x_move_position , y_move_position; // x、y轴每次移动距离
extern float volatile spin_which_direction;
extern float position_move_velocity ; // 单次位置移动速度
extern int is_motor_start_move;
extern int is_slight_move,motor_state,is_slight_spin;

extern int is_servo_adjust;
extern int is_adjust_plate_servo;
extern int is_get_empty ,start_judge_empty;
extern volatile int x_camera_error , y_camera_error ;
extern volatile int  r_servo_now ; // 机械臂伸缩舵机的位置
extern volatile int  theta_servo_now ; // 机械臂中板旋转舵机的位置

extern volatile int x_plate_error , y_plate_error;

extern int seeking_for_circle;
extern int is_find_circle;

int get_motor_real_vel_ok = 0;


extern int volatile get_plate,is_start_get_plate;
extern int is_get_qrcode_target;
extern volatile int target_colour[6];
extern volatile int material_place[3];

volatile int test_slight_move = 1;

int x_err_1 = 0;
int x_err_2 = 0;
int x_err_3 = 0;
int y_err_1 = 0;
int y_err_2 = 0;
int y_err_3 = 0;

float pos_motor_1 = 0, pos_motor_2 = 0, pos_motor_3 = 0, pos_motor_4 = 0;
float angle_motor_1 = 0, angle_motor_2 = 0, angle_motor_3 = 0, angle_motor_4 = 0;
float vel_1 = 0, vel_2 = 0, vel_3 = 0, vel_4 = 0;
float motor_vel_1 = 0, motor_vel_2 = 0, motor_vel_3 = 0, motor_vel_4 = 0;

extern int is_start_judge_move_before_slight_adjust; // 是否开始判断在微调前是否需要移动
extern int is_move_before_slight_adjust ; // 在微调前是否需要移动
extern int x_move_before_slight_move ;

extern int is_slight_spin_and_move;

int line_spin_error_1 = 0, line_spin_error_2 = 0,line_spin_error_3 = 0; 

extern float Kp_line_spin,Kd_line_spin,Ki_line_spin;

int temp_spin_which_direction = 0;

extern const float Kp_slight_move,Ki_slight_move,Kd_slight_move;


extern int is_get_massage;
extern int test_raspi_communication_start,test_raspi_communication_status;

extern int is_adjust_plate_with_put;
extern int volatile start_check_plate_back_state;
extern int x_plate_error_with_put,y_plate_error_with_put;
extern int is_plate_with_put_ok_1,is_plate_with_put_ok_2,
is_plate_with_put_ok_3; 

extern int is_plate_first_move;
extern int is_plate_move_adjust;
extern int is_third_preput;

extern int is_get_material_from_temp_area;
extern int test_is_uart_message_lost,uart_data;

extern int is_put_material_in_plate;

extern const float y_plate_k,x_plate_k;
extern const float xy_move_k;

extern const float spin_limit_max ;
extern const float spin_limit_min ; // 旋转的最小值
extern const float move_limit_max; // 移动的最大值
extern const float move_limit_min ; // 移动的最小值

extern int is_put_plate;

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
    UART_receive_process_3(); 
    //! 
    // if(rxflag_u3 != 0)
    // {
    //     int temp = rxflag_u3;
    //     // HAL_Delay(1); // 如果是在main中使用可以加入延时，在定时器中断中调用则不能加
    //     if(temp == rxflag_u3) 
    //     {
    //         UART_receive_process_3(); 
    //     }
    // }
}

void UART_handle_function_4(void)
{
    // if(rxflag_u4 != 0)
    // {
        // int temp = rxflag_u4;
        // HAL_Delay(1); // 如果是在main中使用可以加入延时，在定时器中断中调用则不能加
        // if(temp == rxflag_u4) 
        // {
        //     UART_receive_process_4(); 
        // }
        UART_receive_process_4(); 
    // }
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
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
    // if(rxflag_u1 >= 128)
    // {
    //     rxflag_u1 = 0;
    // }
    // if(rxflag_u2 >= 50)
    // {
    //     rxflag_u2 = 0;
    // }
    // if(rxflag_u3 >= 50)
    // {
    //     rxflag_u3 = 0;
    // }
    // if(rxflag_u4 >= 50)
    // {
    //     rxflag_u4 = 0;
    // }
    // if(rxflag_u5 >= 50)
    // {
    //     rxflag_u5 = 0;
    // }
    // rxdata_u1[rxflag_u1++] = received_rxdata_u1;
    // rxdata_u2[rxflag_u2++] = received_rxdata_u2;
    // rxdata_u3[rxflag_u3++] = received_rxdata_u3;
    // rxdata_u4[rxflag_u4++] = received_rxdata_u4;
    // rxdata_u5[rxflag_u5++] = received_rxdata_u5;  //! 顺序？
    // if(huart->Instance == USART2)
    // {
    //     HAL_UART_Receive_IT(huart, &received_rxdata_u2, 1); // 每次处理一个字符
    // }
    // else if(huart->Instance == USART3)
    // {
    //     HAL_UART_Receive_IT(huart, &received_rxdata_u3, 1); // 每次处理一个字符
    // }
    // else if(huart->Instance == USART1)
    // {
    //     HAL_UART_Receive_IT(huart, &received_rxdata_u1, 1); // 每次处理一个字符
    // }
    // else if(huart->Instance == UART4)
    // {
    //     HAL_UART_Receive_IT(huart, &received_rxdata_u4, 1); // 每次处理一个字符
    // }
    // else if(huart->Instance == UART5)
    // {
    //     HAL_UART_Receive_IT(huart, &received_rxdata_u5, 1); // 每次处理一个字符
    // }
// }

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
        if(rxdata_u1[0] == 1 && rxdata_u1[1] == 0x35)
        {
            vel_1 = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));
            motor_vel_1 = vel_1;
            if(rxdata_u1[2])
            {
                motor_vel_1 = -motor_vel_1;
            }
        }
        if(rxdata_u1[0] == 2 && rxdata_u1[1] == 0x35)
        {
            vel_2 = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));
            motor_vel_2 = vel_2;
            if(rxdata_u1[2])
            {
                motor_vel_2 = -motor_vel_2;
            }
        }
        if(rxdata_u1[0] == 3 && rxdata_u1[1] == 0x35)
        {
            vel_3 = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));
            motor_vel_3 = vel_3;
            if(rxdata_u1[2])
            {
                motor_vel_3 = -motor_vel_3;
            }
        }
        if(rxdata_u1[0] == 4 && rxdata_u1[1] == 0x35)
        {
            vel_4 = (uint16_t)(((uint16_t)rxdata_u1[3] << 8) |
                             ((uint16_t)rxdata_u1[4] << 0));
            motor_vel_4 = vel_4;
            if(rxdata_u1[2])
            {
                motor_vel_4 = -motor_vel_4;
            }
        }

        if(rxdata_u1[0] == 1 && rxdata_u1[1] == 0x36)
        {
            pos_motor_1 = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
            ((uint32_t)rxdata_u1[4] << 16) |
            ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));
            angle_motor_1 = (float)pos_motor_1 * 360.0f / 65536.0f;
            if(rxdata_u1[2])
            {
                angle_motor_1 = -angle_motor_1;
            }
        }
        if(rxdata_u1[0] == 2 && rxdata_u1[1] == 0x36)
        {
            pos_motor_2 = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
            ((uint32_t)rxdata_u1[4] << 16) |
            ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));
            angle_motor_2 = (float)pos_motor_2 * 360.0f / 65536.0f;
            if(rxdata_u1[2])
            {
                angle_motor_2 = -angle_motor_2;
            }
        }
        if(rxdata_u1[0] == 3 && rxdata_u1[1] == 0x36)
        {
            pos_motor_3 = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
            ((uint32_t)rxdata_u1[4] << 16) |
            ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));
            angle_motor_3 = (float)pos_motor_3 * 360.0f / 65536.0f;
            if(rxdata_u1[2])
            {
                angle_motor_3 = -angle_motor_3;
            }
        }
        if(rxdata_u1[0] == 4 && rxdata_u1[1] == 0x36)
        {
            pos_motor_4 = (uint32_t)(((uint32_t)rxdata_u1[3] << 24) |
            ((uint32_t)rxdata_u1[4] << 16) |
            ((uint32_t)rxdata_u1[5] << 8) |
                             ((uint32_t)rxdata_u1[6] << 0));
            angle_motor_4 = (float)pos_motor_4 * 360.0f / 65536.0f;
            if(rxdata_u1[2])
            {
                angle_motor_4 = -angle_motor_4;
            }
        }
        rxflag_u1 = 0;
        memset(rxdata_u1, 0, 128);
    }
}

/// @brief 处理树莓派数据
void UART_receive_process_3(void)
{
    //!树莓派对应的引脚（USB端口在前，风扇在后）25.7.3
    //! 灰色：左列由后往前第三个
    //! 蓝色：右列由后往前第四个
    //! 橙色：右列由前往后第六个
    
    // if (rxflag_u3 > 0)
    if(1) //! 由于使用了空闲中断，故不需要原先中断的判断
    {
        // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        // HAL_UART_Transmit(&huart3, (uint8_t*)rxdata_u3, rxflag_u3, 50);  //! 阻塞式，会造成串口阻塞

        //? 通信测试
        if(test_is_uart_message_lost == 1 )
        {
            uart_data = rxdata_u3[1];
        }
        
        //? 返回车转盘后再判断是否空抓
        if(start_check_plate_back_state == 1)
        {
            if(rxdata_u3[0] == '?' && rxdata_u3[1] == 0x03)
            {
                start_check_plate_back_state = 0;
            }
        }

        //? 转盘放置，接收到直接放（用于往转盘上放置）
        if(is_put_plate == 0)
        {
            if(rxdata_u3[0] == '?' && rxdata_u3[1] == 0x77)
            {
                is_put_plate = 1;
            }
        }

        //? 在暂存区定位随机放置的物料并抓取
        if(is_get_material_from_temp_area == 1)
        {
            if(rxdata_u3[0] == '?' && rxdata_u3[1] == 0x44)
            {
                is_slight_spin_and_move = 0;
            }   
        }
        // 等待树莓派返回识别结果
        if(is_get_material_from_temp_area == 2)
        {
            if (rxdata_u3[0] == '%' )
            {
                for (int i = 1; i < 4; i++)
                {
                    if ((int)rxdata_u3[i] == 1 || (int)rxdata_u3[i] == 2 || (int)rxdata_u3[i] == 3)
                    {
                        material_place[i-1] = (int)rxdata_u3[i];
                    }
                }
                is_get_material_from_temp_area = 3;
            }
        }

        //? 通信单独测试
        if(test_raspi_communication_start == 1)
        {
            if(rxdata_u3[0] == '?' && rxdata_u3[1] == 0x27)
            {
                // HAL_UART_Transmit(&huart3, (uint8_t*)"I get ///", strlen("I get ///"), 50);
                test_raspi_communication_status = 1;
            }
            if(rxdata_u3[0] == '#')
            {
                // HAL_UART_Transmit(&huart3, (uint8_t*)"I get ###", strlen("I get ###"), 50);
            }
            if(rxdata_u3[0] == '!')
            {
                // HAL_UART_Transmit(&huart3, (uint8_t*)"I get !!!", strlen("I get !!!"), 50);
            }
            if(rxdata_u3[0] == '*')
            {
                // HAL_UART_Transmit(&huart3, (uint8_t*)"I get ***", strlen("I get ***"), 50);
            }
            if(rxdata_u3[0] == '%')
            {
                // HAL_UART_Transmit(&huart3, (uint8_t*)"I get %%", strlen("I get %%"), 50);
            }
            if(rxdata_u3[0] == '@')
            {
                // HAL_UART_Transmit(&huart3, (uint8_t*)"I get @@", strlen("I get @@"), 50);
            }
        }

        //? 将物料放置在转盘的带环的位置
        if(is_plate_first_move == 2)
        {
            if(rxdata_u3[0] == '?' && rxdata_u3[1] == 0x61)
            {
                is_plate_first_move = 1;
            }
        }
        if(is_third_preput == 0)
        {
            if( rxdata_u3[0] == '?' && rxdata_u3[1] == 0x62)
            {
                is_third_preput = 1;
            }
        }

        //? 将物料放置在转盘上
        if(is_adjust_plate_with_put == 1 && rxdata_u3[6] == '!')
        {
            if(rxdata_u3[0] == 0x01)
            {
                x_plate_error_with_put = (int) (rxdata_u3[1] <<8 | rxdata_u3[2]);
            }
            else if(rxdata_u3[0] == 0x02)
            {
                x_plate_error_with_put = - (int) (rxdata_u3[1] <<8 | rxdata_u3[2]);
            }
            if(rxdata_u3[3] == 0x01)
            {
                y_plate_error_with_put = (int) (rxdata_u3[4] <<8 | rxdata_u3[5]);
            }
            else if(rxdata_u3[3] == 0x02)
            {
                y_plate_error_with_put = - (int) (rxdata_u3[4] <<8 | rxdata_u3[5]);
            }

            if(rxdata_u3[1] == 0x10 && rxdata_u3[0] == '?') //调整结束
            {
                is_adjust_plate_with_put = 0;
            }

        }

        if(is_adjust_plate_with_put == 2)
        {
            if (rxdata_u3[1] == 0x07 && rxdata_u3[0] == '?')
            {
                is_plate_with_put_ok_1 = 1;
                rxdata_u3[1] = 0x00;
            }
            if (rxdata_u3[1] == 0x08 && rxdata_u3[0] == '?') // 绿
            {
                is_plate_with_put_ok_2 = 1;
                rxdata_u3[1] = 0x00;
            }
            if (rxdata_u3[1] == 0x09 && rxdata_u3[0] == '?') // 蓝
            {
                is_plate_with_put_ok_3 = 1;
                rxdata_u3[1] = 0x00;
            }
        }


        is_motor_start_move = 1;

        // 测试用
        if( rxdata_u3[1] == 0x88 && rxdata_u3[0] == '?') 
        {
            is_get_massage = 1;
        }


        // 得到任务
        if (is_get_qrcode_target < 2) //! 此处还可以加入更复杂的校验位，避免误操作
        {
            if (rxdata_u3[0] == '*' )
            {
                for (int i = 1; i < 7; i++)
                {
                    if ((int)rxdata_u3[i] == 1 || (int)rxdata_u3[i] == 2 || (int)rxdata_u3[i] == 3)
                    {
                        target_colour[i-1] = (int)rxdata_u3[i];
                    }
                }
                is_get_qrcode_target++;
            }
        }

        //转盘调整机械臂位置实现抓取

        if(is_adjust_plate_servo == 1 && rxdata_u3[6] == '!')
        {
            if(rxdata_u3[0] == 0x01)
            {
                x_plate_error = (int) (rxdata_u3[1] <<8 | rxdata_u3[2]);
                x_plate_error *= x_plate_k;
                

            }
            else if(rxdata_u3[0] == 0x02)
            {
                x_plate_error = - (int) (rxdata_u3[1] <<8 | rxdata_u3[2]);
                x_plate_error *= x_plate_k;

            }
            if(rxdata_u3[3] == 0x01)
            {
                y_plate_error = (int) (rxdata_u3[4] <<8 | rxdata_u3[5]);
                y_plate_error *= y_plate_k;
                

            }
            else if(rxdata_u3[3] == 0x02)
            {
                y_plate_error = - (int) (rxdata_u3[4] <<8 | rxdata_u3[5]);
                y_plate_error *= y_plate_k;
            }
            is_adjust_plate_servo = 0;
        }
        if(start_judge_empty == 1)
        {
            if(rxdata_u3[1] == 0x03 && rxdata_u3[0] == '?')
            {
                is_get_empty = 1;
            }
        }

        //?在从转盘抓取（用于给出是否该从转盘上抓取）
        if (is_start_get_plate == 1) 
        {
            if (rxdata_u3[1] == 0x07 && rxdata_u3[0] == '?') //! 此处可以加入更复杂的校验位
            {
                get_plate = 1;
                rxdata_u3[1] = 0x00;
            }
            if (rxdata_u3[1] == 0x08 && rxdata_u3[0] == '?') // 绿
            {
                get_plate = 2;
                rxdata_u3[1] = 0x00;
            }
            if (rxdata_u3[1] == 0x09 && rxdata_u3[0] == '?') // 蓝
            {
                get_plate = 3;
                rxdata_u3[1] = 0x00;
            }
        }

        //? 未修改数据发送
        // if(is_start_judge_move_before_slight_adjust == 1)
        // {
        //     if(rxdata_u3[0] == 0x66)
        //     {
        //         is_move_before_slight_adjust = 1;
        //         if(rxdata_u3[1]== 0x01)
        //         {
        //             x_move_before_slight_move = (int )rxdata_u3[2];
        //         }
        //         else if(rxdata_u3[1] == 0x02)
        //         {
        //             x_move_before_slight_move = - (int )rxdata_u3[2];
        //         }
        //         is_start_judge_move_before_slight_adjust = 0;
        //     }
        // }
        // //! 新版直线和圆一起调
        // if(is_slight_spin_and_move == 1 && rxdata_u3[8] == '!')
        // {
        //     if(rxdata_u3[0] == 0x01)
        //     {
        //         line_spin_error_1 = (int)rxdata_u3[1];
        //     }
        //     else if(rxdata_u3[0] == 0x02)
        //     {
        //         line_spin_error_1 = -(int)rxdata_u3[1];
        //     }
        //     if(rxdata_u3[2] == 0x01)
        //     {
        //         x_err_1 = (int)( rxdata_u3[3]<<8 | rxdata_u3[4]);
        //     }
        //     else if(rxdata_u3[2] == 0x02)
        //     {
        //         x_err_1 = - (int) ( rxdata_u3[3]<<8 | rxdata_u3[4]);
        //     }
        //     if(rxdata_u3[5] == 0x01)
        //     {
        //         y_err_1 = (int) ( rxdata_u3[6]<<8 | rxdata_u3[7]);
        //     }
        //     else if(rxdata_u3[5] == 0x02)
        //     {
        //         y_err_1 = - (int)  ( rxdata_u3[6]<<8 | rxdata_u3[7]);
        //     }
        //     x_err_1 *= xy_move_k;  //TODO magic number
        //     y_err_1 *= xy_move_k;
            
        // }

        //! 旧的直线和圆一起调 
        if(is_slight_spin_and_move == 1 && rxdata_u3[8] == '!')
        {
            if(rxdata_u3[0] == 0x01)
            {
                if(is_show_origin_xy_data == 1)
                {
                    print_screen_buffer_xy[print_screen_buffer_index_xy][0] = (int)rxdata_u3[1];
                }
                

                temp_spin_which_direction = Kp_line_spin * (float)rxdata_u3[1] + Ki_line_spin * ((float)rxdata_u3[1] +line_spin_error_1 + line_spin_error_2) + Kd_line_spin * (-2*line_spin_error_1 +(float)rxdata_u3[1]+ line_spin_error_2);
                if(temp_spin_which_direction > spin_limit_max)
                {
                    spin_which_direction = spin_limit_max;
                }
                else if(temp_spin_which_direction < spin_limit_min)
                {
                    spin_which_direction = spin_limit_min;
                }
                else
                {
                    spin_which_direction = temp_spin_which_direction;
                }

                if((int)rxdata_u3[1] == 0)
                {
                    spin_which_direction = 0; // 如果是0则不转
                }
                line_spin_error_2 = line_spin_error_1;
                line_spin_error_1 = (float)rxdata_u3[1];
                
            }
            else if(rxdata_u3[0] == 0x02)
            {
                if(is_show_origin_xy_data == 1)
                {
                    print_screen_buffer_xy[print_screen_buffer_index_xy][0] = -(int)rxdata_u3[1];
                }


                temp_spin_which_direction = Kp_line_spin * (-(float)rxdata_u3[1]) + Ki_line_spin * ((-(float)rxdata_u3[1]) +line_spin_error_1 + line_spin_error_2) + Kd_line_spin * (-2*line_spin_error_1 +(-(float)rxdata_u3[1])+ line_spin_error_2);
                if(temp_spin_which_direction < -spin_limit_max)
                {
                    spin_which_direction = -spin_limit_max;
                }
                else if(temp_spin_which_direction > -spin_limit_min)
                {
                    spin_which_direction = -spin_limit_min;
                }
                else
                {
                    spin_which_direction = temp_spin_which_direction;
                }

                if((int)rxdata_u3[1] == 0)
                {
                    spin_which_direction = 0; // 如果是0则不转
                }
                line_spin_error_2 = line_spin_error_1;
                line_spin_error_1 = -(float)rxdata_u3[1];
                
            }
            if(rxdata_u3[2] == 0x01)
            {
                x_move_position = (float)( rxdata_u3[3]<<8 | rxdata_u3[4]);
            }
            else if(rxdata_u3[2] == 0x02)
            {
                x_move_position = - (float) ( rxdata_u3[3]<<8 | rxdata_u3[4]);
            }
            if(rxdata_u3[5] == 0x01)
            {
                y_move_position = (float) ( rxdata_u3[6]<<8 | rxdata_u3[7]);
            }
            else if(rxdata_u3[5] == 0x02)
            {
                y_move_position = - (float)  ( rxdata_u3[6]<<8 | rxdata_u3[7]);
            }
            if(is_show_origin_xy_data == 1)
            {
                print_screen_buffer_xy[print_screen_buffer_index_xy][1] = (int)x_move_position;
                print_screen_buffer_xy[print_screen_buffer_index_xy][2] = (int)y_move_position;
            }
            x_move_position *= xy_move_k;  //TODO magic number
            y_move_position *= xy_move_k;
            x_err_3 = x_err_2;
            y_err_3 = y_err_2;
            x_err_2 = x_err_1;
            y_err_2 = y_err_1;
            x_err_1 = x_move_position;
            y_err_1 = y_move_position;
            
            x_move_position = Kp_slight_move * (x_err_1) + Ki_slight_move * (x_err_1+x_err_2 + x_err_3) + Kd_slight_move * (x_err_3+x_err_1 - 2*x_err_2)/2.0;
            y_move_position = Kp_slight_move * (y_err_1) + Ki_slight_move * (y_err_1 + y_err_2 + y_err_3) + Kd_slight_move * (y_err_3+y_err_1 - 2*y_err_2)/2.0 ;
            if(x_move_position > move_limit_max)
            {
                x_move_position = move_limit_max;
            }
            if(y_move_position > move_limit_max)
            {
                y_move_position = move_limit_max;
            }
            if(x_move_position < -move_limit_max)
            {
                x_move_position = -move_limit_max;
            }
            if(y_move_position < -move_limit_max)
            {
                y_move_position = -move_limit_max;
            }
            if(x_move_position < move_limit_min && x_move_position >0)
            {
                x_move_position = move_limit_min;
            }
            if(y_move_position < move_limit_min && y_move_position >0)
            {
                y_move_position = move_limit_min;
            }
            if(x_move_position > -move_limit_min && x_move_position <0)
            {
                x_move_position = -move_limit_min;
            }
            if(y_move_position > -move_limit_min && y_move_position <0)
            {
                y_move_position = -move_limit_min;
            }


            if(print_screen_buffer_index_xy < max_data_length - 1)
            {
                print_screen_buffer_index_xy++;
            }
            else
            {
                print_screen_buffer_index_xy = 0;
            }
        }

        if(rxdata_u3[1] == 0x44 && is_slight_spin_and_move == 1 && rxdata_u3[0] == '?')
        {
            is_slight_spin_and_move = 0;
            x_move_position = 0;
            y_move_position = 0;
            spin_which_direction = 0;          
        }

        // 进行直线微调(废弃)
        if(is_slight_spin == 1)
        {
            if(rxdata_u3[0] == 0x01)
            {
                // if((float)rxdata_u3[1] > 10)
                // {
                //     spin_which_direction = 10;
                // }
                // else if((float)rxdata_u3[1] < 0.5)
                // {
                //     spin_which_direction = 0.5;
                // }
                // else{
                //     spin_which_direction =(float)rxdata_u3[1];
                // }
                temp_spin_which_direction = Kp_line_spin * (float)rxdata_u3[1] + Ki_line_spin * ((float)rxdata_u3[1] +line_spin_error_1 + line_spin_error_2) + Kd_line_spin * (-2*line_spin_error_1 +(float)rxdata_u3[1]+ line_spin_error_2);
                if(temp_spin_which_direction > 5)
                {
                    spin_which_direction = 5;
                }
                else if(temp_spin_which_direction< 1)
                {
                    spin_which_direction = 1;
                }
                else
                {
                    spin_which_direction = temp_spin_which_direction;
                }
                line_spin_error_1 = (float)rxdata_u3[1];
                line_spin_error_2 = line_spin_error_1;
                
            }
            else if(rxdata_u3[0] == 0x02)
            {
                // if((float)rxdata_u3[1] >10)
                // {
                //     spin_which_direction = -10;
                // }
                // else if((float)rxdata_u3[1] < 0.5)
                // {
                //     spin_which_direction = -0.5;
                // }
                // else{
                //     spin_which_direction = -(float)rxdata_u3[1];
                // }
                temp_spin_which_direction = Kp_line_spin * (-(float)rxdata_u3[1]) + Ki_line_spin * ((-(float)rxdata_u3[1]) +line_spin_error_1 + line_spin_error_2) + Kd_line_spin * (-2*line_spin_error_1 +(-(float)rxdata_u3[1])+ line_spin_error_2);
                if(temp_spin_which_direction < -5)
                {
                    spin_which_direction = -5;
                }
                else if(temp_spin_which_direction > -1)
                {
                    spin_which_direction = -1;
                }
                else
                {
                    spin_which_direction = temp_spin_which_direction;
                }
                line_spin_error_1 = -(float)rxdata_u3[1];
                line_spin_error_2 = line_spin_error_1;
            }
            
        }

        // 直线微调停止
        if( rxdata_u3[1] == 0x27 && rxdata_u3[0] == '?')
        {
            spin_which_direction = 0;
            is_slight_spin = 0; //! 未知的
        }

        // 直线微调结束
        if( rxdata_u3[1] == 0x28 && rxdata_u3[0] == '?')
        {
            is_slight_spin = 0;
        }

        // if(rxdata_u3[0] == 0x99 && rxdata_u3[1] == 0x99 && rxdata_u3[2] == 0x99 && rxdata_u3[3] == 0x99)
        // {
        //     is_slight_move = 1;
        // }

        
        // 进行位置微调（废弃）
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
            x_move_position *= 0.4;  //TODO magic number 原先是0.1
            y_move_position *= 0.4;

            // 对位置微调进行PID控制
            x_err_1 = x_move_position;
            y_err_1 = y_move_position;
            x_err_2 = x_err_1;
            y_err_2 = y_err_1;
            x_err_3 = x_err_2;
            y_err_3 = y_err_2;

            x_move_position = Kp_slight_move * (x_err_1) + Ki_slight_move * (x_err_1+x_err_2 + x_err_3) + Kd_slight_move * (x_err_3+x_err_1 - 2*x_err_2)/2.0;
            y_move_position = Kp_slight_move * (y_err_1) + Ki_slight_move * (y_err_1 + x_err_2 + x_err_3) + Kd_slight_move * (y_err_3+y_err_1 - 2*y_err_2)/2.0 ;





            if(x_move_position > 10)
            {
                x_move_position = 10;
            }
            if(y_move_position > 10)
            {
                y_move_position = 10;
            }
            if(x_move_position < -10)
            {
                x_move_position = -10;
            }
            if(y_move_position < -10)
            {
                y_move_position = -10;
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

        if( rxdata_u3[0] == 0x17 ) // !停止移动（废弃）
        {
            // is_slight_move = 0;
            // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50);
            x_move_position = 0;
            y_move_position = 0;
        }
        if(rxdata_u3[0] == 0x18 )  //结束微调（废弃）
        {
            // motor_state = 0;
            is_slight_move = 0;
            test_slight_move = 0;
            x_move_position = 0;
            y_move_position = 0;

        }



        //! 进行视觉闭环微调
        // if(rxdata_u3[0] == 0x37 )
        // {
        //     is_servo_adjust = 1;
        // }

        // if(rxdata_u3[0] == 0x38 )
        // {
        //     is_servo_adjust = 0;
        //     x_camera_error = 0;
        //     y_camera_error = 0;
        // }
        // 结束
        
        if(rxdata_u3[1] == 0x39 && servo_adjust_status == 1 && rxdata_u3[0] == '?')
        {
            is_servo_adjust = 0;
            x_camera_error = 0;
            y_camera_error = 0;
        }
        if(rxdata_u3[1] == 0x40 && servo_adjust_status == 2  && rxdata_u3[0] == '?')
        {
            is_servo_adjust = 0;
            x_camera_error = 0;
            y_camera_error = 0;
        }
        if(rxdata_u3[1] == 0x41 && servo_adjust_status == 3 && rxdata_u3[0] == '?')
        {
            is_servo_adjust = 0;
            x_camera_error = 0;
            y_camera_error = 0;
        }

        if(is_servo_adjust == 1 && rxdata_u3[6] == '!')
        {
            // x_camera_error = 0;
            // y_camera_error = 0;
            if(rxdata_u3[0] == 0x01)
            {
                x_camera_error = (int) (rxdata_u3[1] << 8 | rxdata_u3[2]);
                is_find_circle = 1;
                // x_move_position = 10;
                // HAL_UART_Transmit(&huart3, (uint8_t*)"right front", strlen("right front"), 50);
            }
            else if(rxdata_u3[0] == 0x02)
            {
                x_camera_error = - (int) (rxdata_u3[1] << 8 | rxdata_u3[2]);
                is_find_circle = 1;
                // if(x_camera_error == -10)
                // {
                //     x_camera_error = -11;
                // }
                // if(x_camera_error == -8)
                // {
                //     x_camera_error = -9;
                // }
                // x_move_position = -10;
                // HAL_UART_Transmit(&huart3, (uint8_t*)"left front", strlen("left front"), 50);
            }
            if(rxdata_u3[3] == 0x01)
            {
                y_camera_error = (int) (rxdata_u3[4] << 8 | rxdata_u3[5]);
                is_find_circle = 1;
                // y_move_position = 10;
            }
            else if(rxdata_u3[3] == 0x02)
            {
                y_camera_error = - (int) (rxdata_u3[4] << 8 | rxdata_u3[5]);
                is_find_circle = 1;
                // y_move_position = -10;
            }
            if(is_show_origin_xy_data == 1)
            {
                print_screen_buffer_xy[print_screen_buffer_index_xy][1] = x_camera_error;
                print_screen_buffer_xy[print_screen_buffer_index_xy][2] = y_camera_error;
                if(print_screen_buffer_index_xy < max_data_length - 1)
                {
                    print_screen_buffer_index_xy++;
                }
                else
                {
                    print_screen_buffer_index_xy = 0;
                }
            }
        }

        if(is_put_material_in_plate == 0)
        {
            if(rxdata_u3[0] == '?' && rxdata_u3[1] == 0x33)
            {
                is_put_material_in_plate = 1;
            }
        }



        // HAL_UART_Transmit(&huart3, (uint8_t*)rxdata_u3, rxflag_u3, 50);
        rxflag_u3 = 0;
        memset(rxdata_u3, 0, 50); //
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
    // if (rxflag_u4 > 0)
    // {
    //     //! 校验位待加入
        
    //     //寻找帧头
    //     int i = 0;
    //     for(i = 0; i < rxflag_u4; i++)
    //     {
    //         if(rxdata_u4[i] == 0x55 && rxdata_u4[i+1] == 0x53)
    //         {
    //             break;
    //         }
    //     }
    //     //如果找到帧头
    //     if(i < rxflag_u4)
    //     {
    //         //提取数据
    //         uint16_t temp = (uint16_t)((rxdata_u4[i+7] << 8 )| rxdata_u4[i+6]);
    //         gyro_z = (float)(temp * 180.0/32768.0); // / 32768.0
    //         if(gyro_z > 180)
    //         {
    //             gyro_z = gyro_z - 360;
    //         }
    //     }
    //     // printf("t0.txt=\"%.3f\"\xff\xff\xff", gyro_z);
    //     rxflag_u4 = 0;
        // memset(rxdata_u4, 0, 50);
    // }
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
