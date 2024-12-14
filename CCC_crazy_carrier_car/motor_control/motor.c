#include "motor.h"



// 以下数据暂时都不用
// // 速度控制PID
// float pos = 0.0f, Motor_Cur_Pos_1 = 0.0f, Motor_Cur_Pos_2 = 0.0f, Motor_Cur_Pos_3 = 0.0f, Motor_Cur_Pos_4 = 0.0f; // 电机当前实际位置
// float Motor_target_vel_1 = 10, Motor_target_vel_2 = 10, Motor_target_vel_3 = 10, Motor_target_vel_4 = 10; // 电机目标速度(默认为10r/min)
// float vel = 0.0f, Motor_Vel_1 = 0.0f, Motor_Vel_2 = 0.0f, Motor_Vel_3 = 0.0f, Motor_Vel_4 = 0.0f; // 电机实际速度
// float vel1_error= 0,vel1_error_last = 0,vel1_error_long_last = 0; // 电机1速度误差
// float vel2_error= 0,vel2_error_last = 0,vel2_error_long_last = 0; // 电机2速度误差
// float vel3_error= 0,vel3_error_last = 0,vel3_error_long_last = 0; // 电机3速度误差
// float vel4_error= 0,vel4_error_last = 0,vel4_error_long_last = 0; // 电机4速度误差
// float KP_vel = 2, KI_vel = 0.1, KD_vel = 0; // 速度控制PID参数
// // 位置控制PID
// float KP_y = 1, KI_y = 0.1, KD_y = 0.1; // y轴PID参数
// float KP_x = 1, KI_x = 0.1, KD_x = 0.1; // x轴PID参数
// float error_x = 0, error_last_x = 0, error_pre_x = 0; // x轴PID误差
// float error_y = 0, error_last_y = 0, error_pre_y = 0; // y轴PID误差
// float error_x_sum = 0, error_y_sum = 0; // x、y轴误差和
// float x_bias_limit = 1, y_bias_limit = 1; // x、y偏差限制,单位cm,待根据视觉情况调整

/// 所有运动情况下的加速度
float acceleration = 2;  //一直用的50


/// @brief x、y轴移动速度（根据树莓派发送的偏差值进行调整）
float volatile x_move_position = 0, y_move_position = 0; 

/// @brief 顺逆时针旋转速度（0为不旋转）
float volatile spin_which_direction = 0; 

/// @brief 单次位置移动速度（树莓派视觉联调时的移动速度）
float position_move_velocity = 30; 

/// @brief 旋转速度（树莓派视觉联调时的旋转速度）
float spin_move_velocity = 8; 


// 暂时不用
float x_move_time=0; // x轴移动时间,ms
float y_move_time = 0; // y轴移动时间,ms
float all_move_time = 0; // 视觉联调时总移动时间(根据所需的移动时间取最大值)




// /// @brief （废弃）根据树莓派发送的x、y偏差值进行PID位置控制
// /// @param x_bias 圆心到视野中心位置的x偏差，以视野中心为原点，向右为正
// /// @param y_bias 圆心到视野中心位置的y偏差，以视野中心为原点，向上为正
// /// @return 
// int PID_motor_control(float x_bias, float y_bias)
// {
//     if(x_bias < x_bias_limit && y_bias < y_bias_limit)
//     {
//         stop();
//         return 1;
//     }
//     else
//     {
//         //! 加入位置控制
//         // error_x = x_bias;
//         // error_y = y_bias;

//         // float P_x = KP_x * error_x;
//         // float I_x = KI_x * (error_x + error_last_x);
//         // float D_x = KD_x * (error_x - error_last_x);

//         // float P_y = KP_y * error_y;
//         // float I_y = KI_y * (error_y + error_last_y);
//         // float D_y = KD_y * (error_y - error_last_y);

//         // float output_x = P_x + I_x + D_x;
//         // float output_y = P_y + I_y + D_y;

//         // move_all_direction(2, output_x, output_y);
        
        

//         return 0;
//     }
// }

/// @brief 根据移动的距离计算需要的脉冲数
/// @param distance 
/// @return 
uint32_t get_clk(float distance)
{
    return (uint32_t)(distance / wheel_circumference * pulse_per_circle);
}

/// @brief 根据移动的距离和速度计算需要的时间（ms）
int get_distance_time(float distance, float velocity)
{
    return (int)(1000* distance /(float)(speed_ratio * velocity ));
}
// 速度/60 *周长 = 每秒的距离

/// @brief （仅在定时器中使用）实现电机位置控制（参考my_timer.c中的用法）
/// @param acc 
/// @param velocity 
/// @param x_move_length 
/// @param y_move_length 
/// @param times_count 发送指令的次数
void move_all_direction_position_tim(uint8_t acc,uint16_t velocity, float x_move_length,float y_move_length,int times_count)
{
    if(x_move_length >= 0 && y_move_length >= 0)
    {
        float delta_xy = x_move_length - y_move_length;
        switch(times_count)
        {
            case 1:
                Emm_V5_Pos_Control(1,0,velocity, acc,  get_clk(x_move_length + y_move_length),0,1);
                break;
            case 2:
                Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(x_move_length + y_move_length), 0, 1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if (x_move_length >= 0 && y_move_length < 0)
    {
        float delta_xy = x_move_length + y_move_length;
        switch(times_count)
        {
            case 1:
                Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(x_move_length - y_move_length), 0,1);
                break;
            case 2:
                Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(x_move_length - y_move_length), 0,1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(1,0,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if (x_move_length < 0 && y_move_length >= 0)
    {
        float delta_xy = y_move_length + x_move_length;
        switch(times_count)
        {
            case 1:
                Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(y_move_length - x_move_length), 0,1);
                break;
            case 2:
                Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(y_move_length - x_move_length), 0,1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(1,0,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if (x_move_length < 0 && y_move_length < 0)
    {
        float delta_xy = y_move_length - x_move_length;
        switch(times_count)
        {
            case 1:
                Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-x_move_length - y_move_length), 0,1);
                break;
            case 2:
                Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-x_move_length - y_move_length), 0,1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {
                    Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(delta_xy), 0,1);
                }
                else
                {
                    Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(-delta_xy), 0,1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }   
                break;
    
        }
    }
    
    
}

void stop_tim(int times_count)
{
    switch(times_count)
    {
        case 1:
            Emm_V5_Stop_Now(1,1);
            break;
        case 2:
            Emm_V5_Stop_Now(2,1);
            break;
        case 3:
            Emm_V5_Stop_Now(3,1);
            break;
        case 4:
            Emm_V5_Stop_Now(4,1);
            break;
    }
}

void spin_all_direction_tim(uint8_t acc, float spin_direction, int times_count)
{
    if(spin_direction >0) // 左转
    {
        switch(times_count)
        {
            case 1:
                Emm_V5_Vel_Control(1,1,spin_direction, acceleration, 1);
                break;
            case 2:
                Emm_V5_Vel_Control(2,1,spin_direction, acceleration, 1);
                break;
            case 3:
                Emm_V5_Vel_Control(3,1,spin_direction, acceleration, 1);
                break;
            case 4:
                Emm_V5_Vel_Control(4,1,spin_direction, acceleration, 1);
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if(spin_direction < 0)
    {
        switch(times_count)
        {
            case 1:
                Emm_V5_Vel_Control(1,0,-spin_direction, acceleration, 1);
                break;
            case 2:
                Emm_V5_Vel_Control(2,0,-spin_direction, acceleration, 1);
                break;
            case 3:
                Emm_V5_Vel_Control(3,0,-spin_direction, acceleration, 1);
                break;
            case 4:
                Emm_V5_Vel_Control(4,0,-spin_direction, acceleration, 1);
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if(spin_direction == 0)
    {
        switch(times_count)
        {
            case 1:
                Emm_V5_Stop_Now(1,1);
                break;
            case 2:
                Emm_V5_Stop_Now(2,1);
                break;
            case 3:
                Emm_V5_Stop_Now(3,1);
                break;
            case 4:
                Emm_V5_Stop_Now(4,1);
                break;
        }
    }
}

void move_all_direction_tim(uint8_t acc, float x_vel,float y_vel,int times_count)
{
    if(x_vel >= 0 && y_vel >= 0)
    {
        float delta_xy = x_vel - y_vel;
        switch(times_count)
        {
            case 1:
                Emm_V5_Vel_Control(1,0,x_vel + y_vel, acc, 1);
                break;
            case 2:
                Emm_V5_Vel_Control(4,1,x_vel + y_vel, acc,  1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Vel_Control(2,0,delta_xy, acc,1);
                }
                else
                {
                    Emm_V5_Vel_Control(2,1,-delta_xy, acc,1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {

					Emm_V5_Vel_Control(3,1,delta_xy, acc,1);
                }
                else
                {
                    Emm_V5_Vel_Control(3,0,-delta_xy, acc, 1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if (x_vel >= 0 && y_vel < 0)
    {
        float delta_xy = x_vel + y_vel;
        switch(times_count)
        {
            case 1:
                Emm_V5_Vel_Control(2,0,x_vel - y_vel, acc, 1);
                break;
            case 2:
                Emm_V5_Vel_Control(3,1,x_vel - y_vel, acc, 1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Vel_Control(1,0,delta_xy, acc, 1);
                }
                else
                {
                    Emm_V5_Vel_Control(1,1,-delta_xy, acc, 1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {
                    Emm_V5_Vel_Control(4,1,delta_xy, acc,1);
                }
                else
                {
                    Emm_V5_Vel_Control(4,0,delta_xy, acc, 1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if (x_vel < 0 && y_vel >= 0)
    {
        float delta_xy = y_vel + x_vel;
        switch(times_count)
        {
            case 1:
                Emm_V5_Vel_Control(2,1,y_vel- x_vel, acc, 1);
                break;
            case 2:
                Emm_V5_Vel_Control(3,0,y_vel- x_vel, acc, 1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Vel_Control(1,0,delta_xy, acc,1);
                }
                else
                {
                    Emm_V5_Vel_Control(1,1,-delta_xy, acc, 1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {
                    Emm_V5_Vel_Control(4,1,delta_xy, acc,1);
                }
                else
                {
                    Emm_V5_Vel_Control(4,0,-delta_xy, acc, 1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }
                break;
        }
    }
    else if (x_vel< 0 && y_vel < 0)
    {
        float delta_xy = y_vel - x_vel;
        switch(times_count)
        {
            case 1:
                Emm_V5_Vel_Control(1,1,-x_vel - y_vel, acc,1);
                break;
            case 2:
                Emm_V5_Vel_Control(4,0,-x_vel - y_vel, acc, 1);
                break;
            case 3:
                if(delta_xy >= 0)
                {
                    Emm_V5_Vel_Control(2,1,delta_xy, acc, 1);
                }
                else
                {
                    Emm_V5_Vel_Control(2,0,-delta_xy, acc, 1);
                }
                break;
            case 4:
                if(delta_xy >= 0)
                {
                    Emm_V5_Vel_Control(3,0,delta_xy, acc, 1);
                }
                else
                {
                    Emm_V5_Vel_Control(3,1,-delta_xy, acc, 1);
                }
                break;
            case 5:
                {
                    Emm_V5_Synchronous_motion(0);
                }   
                break;
    
        }
    }
}



/// @brief 全向位置移动,需自己给Delay延时，速度1对应0.47cm/s，delaytime(ms) = length /(0.47cm/s * velocity) *1000 
/// @param acc 
/// @param velocity 
/// @param x_move_length 
/// @param y_move_length 
void move_all_direction_position(uint8_t acc,uint16_t velocity, float x_move_length,float y_move_length)
{
    if(x_move_length >= 0 && y_move_length >= 0)
    {
        float delta_xy = x_move_length - y_move_length;
        Emm_V5_Pos_Control(1,0,velocity, acc,  get_clk(x_move_length + y_move_length),0,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(x_move_length + y_move_length), 0, 1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length >= 0 && y_move_length < 0)
    {
        float delta_xy = x_move_length + y_move_length;
        Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(x_move_length - y_move_length), 0,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(x_move_length - y_move_length), 0,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(1,0,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length < 0 && y_move_length >= 0)
    {
        float delta_xy = y_move_length + x_move_length;
        Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(y_move_length - x_move_length), 0,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(y_move_length - x_move_length), 0,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(1,0,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length < 0 && y_move_length < 0)
    {
        float delta_xy = y_move_length - x_move_length;
        Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-x_move_length - y_move_length), 0,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-x_move_length - y_move_length), 0,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(delta_xy), 0,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(-delta_xy), 0,1);
            HAL_Delay(10);
        }
    }
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
    // uint32_t timex =  get_distance_time(abs(x_move_length), velocity);
    // uint32_t timey = get_distance_time(abs(y_move_length), velocity);
    // HAL_Delay(timex > timey ? timex : timey);

}

// /// @brief （未实现）全向速度移动，自带PID控制,调用PID_vel_Control函数
// /// @param acc 
// /// @param x_move_velocity 
// /// @param y_move_velocity 
// void move_all_direction_pid(uint8_t acc,float x_move_velocity,float y_move_velocity)
// {
    
    
//     if(x_move_velocity >= 0 && y_move_velocity >= 0)
//     {
//         float delta_vx_vy = x_move_velocity - y_move_velocity;
//         // Emm_V5_Vel_Control(2, 1, (uint8_t)(x_move_velocity + y_move_velocity ), acc, 1);
//         // HAL_Delay(10);
//         // Emm_V5_Vel_Control(3, 0, (uint8_t)(x_move_velocity + y_move_velocity ), acc, 1);
//         // HAL_Delay(10);

//         PID_vel_Control(2,acc,-(x_move_velocity + y_move_velocity));
//         PID_vel_Control(3,acc,(x_move_velocity + y_move_velocity));

//         if(delta_vx_vy > 0)
//         {
//             // Emm_V5_Vel_Control(1, 1, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(4, 0, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);
//             PID_vel_Control(1,acc,-delta_vx_vy);
//             PID_vel_Control(4,acc,delta_vx_vy);

//         }
//         else
//         {
//             // Emm_V5_Vel_Control(1, 0, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(4, 1, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);
//             PID_vel_Control(1,acc,delta_vx_vy);
//             PID_vel_Control(4,acc,-delta_vx_vy);
//         }
//     }
//     else if (x_move_velocity > 0 && y_move_velocity < 0)
//     {
//         float delta_vx_vy = x_move_velocity + y_move_velocity;
//         // Emm_V5_Vel_Control(1, 1, (uint8_t)(x_move_velocity - y_move_velocity ), acc, 1);
//         // HAL_Delay(10);
//         // Emm_V5_Vel_Control(4, 0, (uint8_t)(x_move_velocity - y_move_velocity ), acc, 1);
//         // HAL_Delay(10);

//         PID_vel_Control(1,acc,-(x_move_velocity - y_move_velocity));

//         PID_vel_Control(4,acc,(x_move_velocity - y_move_velocity));
        

//         if(delta_vx_vy > 0)
//         {
//             // Emm_V5_Vel_Control(2, 1, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(3, 0, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);

//             PID_vel_Control(2,acc,-delta_vx_vy);
//             PID_vel_Control(3,acc,delta_vx_vy);
//         }
//         else
//         {
//             // Emm_V5_Vel_Control(2, 0, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(3, 1, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);

//             PID_vel_Control(2,acc,-delta_vx_vy);
//             PID_vel_Control(3,acc,delta_vx_vy);

//         }
//     }
//     else if (x_move_velocity < 0 && y_move_velocity > 0)
//     {
//         float delta_vx_vy = y_move_velocity + x_move_velocity;
//         // Emm_V5_Vel_Control(1, 0, (uint8_t)(y_move_velocity - x_move_velocity ), acc, 1);
//         // HAL_Delay(10);
//         // Emm_V5_Vel_Control(4, 1, (uint8_t)(y_move_velocity - x_move_velocity ), acc, 1);
//         // HAL_Delay(10);

//         PID_vel_Control(1,acc,(y_move_velocity - x_move_velocity));
//         PID_vel_Control(4,acc,-(y_move_velocity - x_move_velocity));

//         if(delta_vx_vy > 0)
//         {
//             // Emm_V5_Vel_Control(2, 1, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(3, 0, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);

//             PID_vel_Control(2,acc,-delta_vx_vy);
//             PID_vel_Control(3,acc,delta_vx_vy);
//         }
//         else
//         {
//             // Emm_V5_Vel_Control(2, 0, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(3, 1, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);

//             PID_vel_Control(2,acc,-delta_vx_vy);
//             PID_vel_Control(3,acc,delta_vx_vy);
//         }
//     }
//     else if (x_move_velocity < 0 && y_move_velocity < 0)
//     {
//         float delta_vx_vy = y_move_velocity - x_move_velocity;
//         // Emm_V5_Vel_Control(2, 0, (uint8_t)(-x_move_velocity - y_move_velocity ), acc, 1);
//         // HAL_Delay(10);
//         // Emm_V5_Vel_Control(3, 1, (uint8_t)(-x_move_velocity - y_move_velocity ), acc, 1);
//         // HAL_Delay(10);

//         PID_vel_Control(2,acc,-(x_move_velocity + y_move_velocity));
//         PID_vel_Control(3,acc,(x_move_velocity + y_move_velocity));


//         if(delta_vx_vy > 0)
//         {
//             // Emm_V5_Vel_Control(1, 0, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(4, 1, (uint8_t)delta_vx_vy, acc, 1);
//             // HAL_Delay(10);

//             PID_vel_Control(1,acc,delta_vx_vy);
//             PID_vel_Control(4,acc,-delta_vx_vy);
//         }
//         else
//         {
//             // Emm_V5_Vel_Control(1, 1, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);
//             // Emm_V5_Vel_Control(4, 0, (uint8_t)(-delta_vx_vy), acc, 1);
//             // HAL_Delay(10);

//             PID_vel_Control(1,acc,delta_vx_vy);
//             PID_vel_Control(4,acc,-delta_vx_vy);
//         }
//     }
//     Emm_V5_Synchronous_motion(0);
//     HAL_Delay(10);

// }


/// @brief 全向速度移动，不含PID,调用Emm_V5_Vel_Control函数
/// @param acc 
/// @param x_move_velocity 
/// @param y_move_velocity 
void move_all_direction(uint8_t acc,float x_move_velocity,float y_move_velocity)
{
    int delay_time = 10;
    
    if(x_move_velocity >= 0 && y_move_velocity >= 0)
    {
        float delta_vx_vy = x_move_velocity - y_move_velocity;
        Emm_V5_Vel_Control(2, 1, (uint8_t)(x_move_velocity + y_move_velocity ), acc, 1);
        HAL_Delay(delay_time);
        Emm_V5_Vel_Control(3, 0, (uint8_t)(x_move_velocity + y_move_velocity ), acc, 1);
        HAL_Delay(delay_time);

        

        if(delta_vx_vy > 0)
        {
            Emm_V5_Vel_Control(1, 0, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(4, 1, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);
            // PID_vel_Control(1,acc,-delta_vx_vy);
            // PID_vel_Control(4,acc,delta_vx_vy);

        }
        else
        {
            Emm_V5_Vel_Control(1, 1, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(4, 0, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);
            // PID_vel_Control(1,acc,delta_vx_vy);
            // PID_vel_Control(4,acc,-delta_vx_vy);
        }
    }
    else if (x_move_velocity > 0 && y_move_velocity < 0)
    {
        float delta_vx_vy = x_move_velocity + y_move_velocity;
        Emm_V5_Vel_Control(1, 1, (uint8_t)(x_move_velocity - y_move_velocity ), acc, 1);
        HAL_Delay(delay_time);
        Emm_V5_Vel_Control(4, 0, (uint8_t)(x_move_velocity - y_move_velocity ), acc, 1);
        HAL_Delay(delay_time);

        // PID_vel_Control(1,acc,-(x_move_velocity - y_move_velocity));

        // PID_vel_Control(4,acc,(x_move_velocity - y_move_velocity));
        

        if(delta_vx_vy > 0)
        {
            Emm_V5_Vel_Control(2, 1, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(3, 0, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);

            // PID_vel_Control(2,acc,-delta_vx_vy);
            // PID_vel_Control(3,acc,delta_vx_vy);
        }
        else
        {
            Emm_V5_Vel_Control(2, 0, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(3, 1, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);

            // PID_vel_Control(2,acc,-delta_vx_vy);
            // PID_vel_Control(3,acc,delta_vx_vy);

        }
    }
    else if (x_move_velocity < 0 && y_move_velocity > 0)
    {
        float delta_vx_vy = y_move_velocity + x_move_velocity;
        Emm_V5_Vel_Control(1, 0, (uint8_t)(y_move_velocity - x_move_velocity ), acc, 1);
        HAL_Delay(delay_time);
        Emm_V5_Vel_Control(4, 1, (uint8_t)(y_move_velocity - x_move_velocity ), acc, 1);
        HAL_Delay(delay_time);

        // PID_vel_Control(1,acc,(y_move_velocity - x_move_velocity));
        // PID_vel_Control(4,acc,-(y_move_velocity - x_move_velocity));

        if(delta_vx_vy > 0)
        {
            Emm_V5_Vel_Control(2, 1, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(3, 0, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);

            // PID_vel_Control(2,acc,-delta_vx_vy);
            // PID_vel_Control(3,acc,delta_vx_vy);
        }
        else
        {
            Emm_V5_Vel_Control(2, 0, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(3, 1, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);

            // PID_vel_Control(2,acc,-delta_vx_vy);
            // PID_vel_Control(3,acc,delta_vx_vy);
        }
    }
    else if (x_move_velocity < 0 && y_move_velocity < 0)
    {
        float delta_vx_vy = y_move_velocity - x_move_velocity;
        Emm_V5_Vel_Control(2, 0, (uint8_t)(-x_move_velocity - y_move_velocity ), acc, 1);
        HAL_Delay(delay_time);
        Emm_V5_Vel_Control(3, 1, (uint8_t)(-x_move_velocity - y_move_velocity ), acc, 1);
        HAL_Delay(delay_time);

        // PID_vel_Control(2,acc,-(x_move_velocity + y_move_velocity));
        // PID_vel_Control(3,acc,(x_move_velocity + y_move_velocity));


        if(delta_vx_vy > 0)
        {
            Emm_V5_Vel_Control(1, 0, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(4, 1, (uint8_t)delta_vx_vy, acc, 1);
            HAL_Delay(delay_time);

            // PID_vel_Control(1,acc,delta_vx_vy);
            // PID_vel_Control(4,acc,-delta_vx_vy);
        }
        else
        {
            Emm_V5_Vel_Control(1, 1, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);
            Emm_V5_Vel_Control(4, 0, (uint8_t)(-delta_vx_vy), acc, 1);
            HAL_Delay(delay_time);

            // PID_vel_Control(1,acc,delta_vx_vy);
            // PID_vel_Control(4,acc,-delta_vx_vy);
        }
    }
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(delay_time);

}


// /// @brief 加入PID的速度控制，自带延时，在该函数中调用Emm_V5_Vel_Control函数
// /// @param addr 电机地址
// /// @param acc 加速度
// /// @param target_vel 该电机目标速度
// void PID_vel_Control(uint8_t addr,uint8_t acc, float target_vel)
// {
//     if(addr == 1)
//     {
//         // 用PID控制速度
//         Motor_target_vel_1 = target_vel;


//         vel1_error = target_vel - Motor_Vel_1;
//         vel1_error_last = vel1_error;
//         vel1_error_long_last = vel1_error_last;
//         float P = KP_vel * (vel1_error);
//         float I = KI_vel * (vel1_error+ vel1_error_last+ vel1_error_long_last);
//         float D = KD_vel * (vel1_error - 2 * vel1_error_last + vel1_error_long_last);
//         float output =Motor_target_vel_1+ P + I + D; 
        
//         if(output >= 0)
//         {
//             Emm_V5_Vel_Control(addr, 0, (uint8_t)output, acc, 1); // 电机1正转(顺时针)
//         }
//         else
//         {
//             Emm_V5_Vel_Control(addr, 1, (uint8_t)(-output), acc, 1); // 电机1反转(逆时针)
//         }
//     }
//     else if(addr == 2)
//     {
//         // 用PID控制速度
//         Motor_target_vel_2 = target_vel;
//         vel2_error = target_vel - Motor_Vel_2;
//         vel2_error_last = vel2_error;
//         vel2_error_long_last = vel2_error_last;
//         float P = KP_vel * (vel2_error );
//         float I = KI_vel * (vel2_error+ vel2_error_last+ vel2_error_long_last);
//         float D = KD_vel * (vel2_error - 2 * vel2_error_last + vel2_error_long_last);
//         float output = Motor_target_vel_2 +P + I + D;
//         if(output >= 0)
//         {
//             Emm_V5_Vel_Control(addr, 0, (uint8_t)output, acc, 1); // 电机2正转(顺时针)
//         }
//         else
//         {
//             Emm_V5_Vel_Control(addr, 1, (uint8_t)(-output), acc, 1); // 电机2反转(逆时针)
//         }
//     }
//     else if(addr == 3)
//     {
//         // 用PID控制速度
//         Motor_target_vel_3 = target_vel;
//         vel3_error = target_vel - Motor_Vel_3;
//         vel3_error_last = vel3_error;
//         vel3_error_long_last = vel3_error_last;
//         float P = KP_vel * (vel3_error );
//         float I = KI_vel * (vel3_error+ vel3_error_last+ vel3_error_long_last);
//         float D = KD_vel * (vel3_error - 2 * vel3_error_last + vel3_error_long_last);
//         float output = Motor_target_vel_3 + P + I + D;
//         if(output >= 0)
//         {
//             Emm_V5_Vel_Control(addr, 0, (uint8_t)output, acc, 1); // 电机3正转(顺时针)
//         }
//         else
//         {
//             Emm_V5_Vel_Control(addr, 1, (uint8_t)(-output), acc, 1); // 电机3反转(逆时针)
//         }
//     }
//     else if(addr == 4)
//     {
//         // 用PID控制速度
//         Motor_target_vel_4 = target_vel;
//         vel4_error = target_vel - Motor_Vel_4;
//         vel4_error_last = vel4_error;
//         vel4_error_long_last = vel4_error_last;
//         float P = KP_vel * (vel4_error);
//         float I = KI_vel * (vel4_error+ vel4_error_last+ vel4_error_long_last);
//         float D = KD_vel * (vel4_error - 2 * vel4_error_last + vel4_error_long_last);
//         float output = Motor_target_vel_4 + P + I + D;
//         if(output >= 0)
//         {
//             Emm_V5_Vel_Control(addr, 0, (uint8_t)output, acc, 1); // 电机4正转(顺时针)
//         }
//         else
//         {
//             Emm_V5_Vel_Control(addr, 1, (uint8_t)(-output), acc, 1); // 电机4反转（逆时针）
//         }
//     }
//     HAL_Delay(10);
// }

/// @brief 以速度模式前进
/// @param vel 
/// @param acc 
void Forward_move_velocity(uint16_t vel,uint8_t acc)
{
    Emm_V5_Vel_Control(1, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(2, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(3, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(4, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0); // 多机同步运动
    HAL_Delay(10);
}

/// @brief 整车前进
/// @param vel 
/// @param acc 
/// @param distance //单位cm 
void Forward_move( uint16_t vel,uint8_t acc, uint32_t distance)
{
    uint32_t clk = distance / wheel_circumference * pulse_per_circle;
    Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 1, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 1, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 以速度模式后退
/// @param vel 
/// @param acc 
void Backward_move_velocity(uint16_t vel,uint8_t acc)
{
    Emm_V5_Vel_Control(1, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(2, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(3, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(4, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 整车后退
/// @param vel 
/// @param acc 
/// @param distance //单位cm
void Backward_move( uint16_t vel,uint8_t acc, uint32_t distance)
{
    uint32_t clk = distance / wheel_circumference * pulse_per_circle;
    Emm_V5_Pos_Control(1, 1, vel, acc,clk, 0,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 0, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 1, vel, acc,clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 0, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 
/// @param vel 
/// @param acc 
void move_right_velocity(uint16_t vel,uint8_t acc)
{
    Emm_V5_Vel_Control(1, 0, vel, acc, 1);
    HAL_Delay(2);
    Emm_V5_Vel_Control(2, 0, vel, acc, 1);
    HAL_Delay(2);
    Emm_V5_Vel_Control(3, 1, vel, acc, 1);
    HAL_Delay(2);
    Emm_V5_Vel_Control(4, 1, vel, acc, 1);
    HAL_Delay(2);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 
/// @param vel 
/// @param acc 
/// @param distance //单位cm
void move_right(uint16_t vel,uint8_t acc, uint32_t distance)
{
    uint32_t clk = distance / wheel_circumference * pulse_per_circle;
    Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 0, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 1, vel, acc,clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 1, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 
/// @param vel 
/// @param acc 
void move_left_velocity(uint16_t vel,uint8_t acc)
{
    Emm_V5_Vel_Control(1, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(2, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(3, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(4, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}


/// @brief 
/// @param vel 
/// @param acc 
/// @param distance //单位cm
void move_left(uint16_t vel,uint8_t acc, uint32_t distance)
{
    uint32_t clk = distance / wheel_circumference * pulse_per_circle;
    Emm_V5_Pos_Control(1, 1, vel, acc,clk, 0,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 1, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 0, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 以速度模式左转
/// @param vel 
/// @param acc 
void spin_left_velocity(uint16_t vel,uint8_t acc)
{
    Emm_V5_Vel_Control(1, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(2, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(3, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(4, 1, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}


/// @brief 左转
/// @param vel 
/// @param acc 
/// @param angle //单位度 
void spin_left(uint16_t vel,uint8_t acc, uint32_t angle)
{
    uint32_t clk = (uint32_t)((float)angle / 360 * spin_radius * 2 * pi / wheel_circumference * pulse_per_circle);
    Emm_V5_Pos_Control(1, 1, vel, acc,clk, 0,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 1, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 1, vel, acc,clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 1, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 以速度模式右转
/// @param vel 
/// @param acc 
void spin_right_velocity(uint16_t vel,uint8_t acc)
{
    Emm_V5_Vel_Control(1, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(2, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(3, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(4, 0, vel, acc, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

/// @brief 右转
/// @param vel 
/// @param acc 
/// @param angle //单位度 
void spin_right(uint16_t vel,uint8_t acc, uint32_t angle)
{
    uint32_t clk = (uint32_t)((float)angle / 360 * spin_radius * 2 * pi / wheel_circumference * pulse_per_circle);
    Emm_V5_Pos_Control(1, 0, vel, acc,clk, 0,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 0, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 0, vel, acc,clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 0, vel, acc, clk, 0, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}


/// @brief 整车立即停车
void stop(void)
{
    Emm_V5_Stop_Now((uint8_t)1, 0);
    HAL_Delay(10);
    Emm_V5_Stop_Now((uint8_t)2,0);
    HAL_Delay(10);
    Emm_V5_Stop_Now((uint8_t)3,0);
    HAL_Delay(10);
    Emm_V5_Stop_Now((uint8_t)4, 0);
    HAL_Delay(10);
}


/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW(逆时针)
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t raF, uint8_t snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 5);
}

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 4);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, i);
}

