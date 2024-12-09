#include "my_timer.h"

int tim3_count = 0;
int tim2_count = 0;

int is_slight_spin = 0; //! 是否进行微调旋转,1为进行微调，在到位后开始微调，将此置为1
int is_slight_move = 0; //! 是否进行微调,1为进行微调，在到位后开始微调，将此置为1
int motor_state = 0; // 电机状态，0为默认情况，1为微调情况

/// @brief 电机是否可以开始移动（当已经接受了移动指令后，到电机移动到位前，该标志位为1）
int is_motor_start_move = 0; 

/// @brief 电机正在移动时为1，用来在定时器中计算电机移动时间
int is_motor_moving = 0; 

extern float volatile spin_which_direction;

/// @brief 电机发送指令到了第几步
int send_motor_message_flag = 0; 
int send_motor_message_flag_stop = 0;

/// @brief 电机移动时间计数（乘上定时器的时间间隔即为电机移动时间）
int motor_moving_timecount = 0;

/// @brief 是否计算了电机移动时间（用于在刚开始移动前计算电机移动时间）
int calculate_motor_move_time = 0; 

/// @brief 定时器3的时间间隔（10ms）
int tim3_period = 10; 

extern float x_move_time, y_move_time,all_move_time;
extern float volatile x_move_position, y_move_position;
extern float position_move_velocity;
extern float acceleration;

extern uint8_t rxdata_u3[50];
extern uint8_t received_rxdata_u3;
extern uint8_t rxflag_u3,rxflag_u4;

/// @brief 中断回调函数，所有的定时器中断都在这里处理
/// @param htim
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) // 10ms一次 用来进行电机控制
    {
        tim2_count++;
        if (rxflag_u3 != 0)
        {
            UART_handle_function_3(); // 处理串口3接收到的数据
        }

        if (tim2_count >= 20) //! magic number
        {
            if (rxflag_u4 != 0)
            {
                UART_handle_function_4(); // 处理串口4接收到的数据
            }
            tim2_count = 0;
        }
        // if(is_motor_start_move == 0)
        // {
        //     is_motor_start_move = 1;
        // }
    }
    if (htim->Instance == TIM3) // 10ms
    {
        tim3_count++;
        if(is_slight_spin == 1)
        {
            if (is_motor_start_move == 1)
            {
                if(send_motor_message_flag < 6)
                {
                    send_motor_message_flag++;
                }
                if(send_motor_message_flag == 1)
                {
                    spin_all_direction_tim(acceleration,spin_which_direction,1);
                }
                if(send_motor_message_flag == 2)
                {
                    spin_all_direction_tim(acceleration,spin_which_direction,2);
                }
                if(send_motor_message_flag == 3)
                {
                    spin_all_direction_tim(acceleration,spin_which_direction,3);
                }
                if(send_motor_message_flag == 4)
                {
                    spin_all_direction_tim(acceleration,spin_which_direction,4);
                }
                if(send_motor_message_flag == 5)
                {
                    spin_all_direction_tim(acceleration,spin_which_direction,5);
                    is_motor_moving = 1;
                }
            }
            if(is_motor_moving == 1)
            {
                is_motor_start_move = 0;
                is_motor_moving = 0;
                send_motor_message_flag = 0;
            }
        }
        if(is_slight_move == 1 && motor_state == 1) // 电机在微调
        {
            if (is_motor_start_move == 1) // 电机可以移动了
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // 板载LED闪烁
                if (calculate_motor_move_time == 0)                 // 开始移动后计算电机移动时间（只计算一次），随后在这段时间结束前不再接受新的移动指令
                {
                    calculate_motor_move_time = 1;
                    // x_move_time = x_move_position / (float)wheel_circumference / (position_move_velocity /60.0) *1000; // x轴移动时间,ms
                    // y_move_time = y_move_position / (float)wheel_circumference / (position_move_velocity /60.0) *1000; // y轴移动时间,ms
                    // all_move_time = x_move_time > y_move_time ? x_move_time : y_move_time; // 两轴移动时间取最大值
                    all_move_time = 100;
                }

                if (send_motor_message_flag < 6) // 进行电机移动指令的计数
                {
                    send_motor_message_flag++; // 定时器中断10ms一次，所以每10ms发送一次电机移动指令，以下相当于HAL_Delay(10)的作用
                }
                if (send_motor_message_flag == 1)
                {
                    // move_all_direction_position_tim(acceleration,position_move_velocity,x_move_position,y_move_position,1);
                    move_all_direction_tim(acceleration, x_move_position, y_move_position, 1);
                }
                if (send_motor_message_flag == 2)
                {
                    // move_all_direction_position_tim(acceleration,position_move_velocity,x_move_position,y_move_position,2);
                    move_all_direction_tim(acceleration, x_move_position, y_move_position, 2);
                }
                if (send_motor_message_flag == 3)
                {
                    // move_all_direction_position_tim(acceleration,position_move_velocity,x_move_position,y_move_position,3);
                    move_all_direction_tim(acceleration, x_move_position, y_move_position, 3);
                }
                if (send_motor_message_flag == 4)
                {
                    // move_all_direction_position_tim(acceleration,position_move_velocity,x_move_position,y_move_position,4);
                    move_all_direction_tim(acceleration, x_move_position, y_move_position, 4);
                }
                if (send_motor_message_flag == 5)
                {
                    // move_all_direction_position_tim(acceleration,position_move_velocity,x_move_position,y_move_position,5);
                    move_all_direction_tim(acceleration, x_move_position, y_move_position, 5);
                }
                if (send_motor_message_flag == 6)
                {
                    // move_all_direction_position_tim(acceleration,position_move_velocity,x_move_position,y_move_position,6);
                    is_motor_moving = 1;
                }
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
            }
            if (is_motor_moving == 1)
            {
                motor_moving_timecount++; // 电机移动时间计数,达到指定时间后则下一条电机移动的指令可以发送
                // if(motor_moving_timecount * tim3_period >= all_move_time) // 电机移动时间到达
                if (1) // 电机移动时间到达motor_moving_timecount * tim3_period >= all_move_time
                {
                    motor_moving_timecount = 0;
                    is_motor_start_move = 0;
                    calculate_motor_move_time = 0;
                    is_motor_moving = 0;
                    send_motor_message_flag = 0;
                }
            }
        }
        

        if(tim3_count > 100000)
        {
            tim3_count = 0;
            HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
        }
    }
}