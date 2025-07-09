#include "motor.h"
#include <math.h>
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!     步进电机参数
//!     步进电机速度0-5000
//!     步进电机加速度0-255
//!     5cm前后对应188像素
//!     5cm左右对应像素181

#define pi 3.1415926
// #define wheel_circumference 31.4 //轮子周长
// #define pulse_per_circle 3200 //每圈脉冲数
// #define spin_radius_180 21.51 //自转半径，数据不准确 // 21.495 90 
// #define spin_radius_90 21.495 //自转半径，数据不准确 // 21.495 90
// #define spin_radius 21.495 //自转半径，数据不准确 // 21.495 90
// #define speed_ratio 0.52 //速度比例系数，0.47 cm/s 对应速度为1 （此处修改为0.52cm/s 对应速度为1）
const float wheel_circumference = 31.4; //轮子周长
const float pulse_per_circle = 3200; //每圈脉冲数
const float spin_radius_180 = 21.51; //自转半径，数据不准确 // 21.495 90 
const float spin_radius_90 = 21.495; //自转半径，数据不准确 // 21.495 90
const float spin_radius = 21.495; //自转半径，数据不准确 // 21.495 90
const float speed_ratio = 0.52; //速度比例系数，0.47 cm/s 对应速度为1 （此处修改为0.52cm/s 对应速度为1）


/// 所有运动情况下的加速度
extern float acceleration;
extern float acceleration_spin;
extern float acceleration_adjust;

extern float adjust_spin_scale; // 旋转和移动的比例
extern float adjust_move_scale;



/// @brief x、y轴移动速度（根据树莓派发送的偏差值进行调整）
float volatile x_move_position = 0, y_move_position = 0; 

/// @brief 顺逆时针旋转速度（0为不旋转）
float volatile spin_which_direction = 0; 

extern int motor_vel_adjust_with_spin ;


volatile int motor_vel_target_1 = 0, motor_vel_target_2 = 0, motor_vel_target_3 = 0, motor_vel_target_4 = 0;

extern int motor_pos_move_mode;

float min_motor_velocity = 1.0f; // 可根据需要调整

void slight_spin_plate_line(void)
{

}

/// @brief 差速移动给定角度和半径
/// @param left_or_right 
/// @param vel 
/// @param R_spin 
/// @param angles 
void move_leftright_front_with_spin_position(int left_or_right, float vel  , float R_spin, float angles)
{
    float L = 24;
    float r = 5;
    float VL = 0, VR = 0;
    int VL_motor = 0,VR_motor = 0;
    uint32_t clk_L = 0, clk_R = 0;


    if(left_or_right == 1) //右
    {
        VL = vel * (R_spin + L/2) / R_spin;
        VR = vel * (R_spin - L/2) / R_spin;
        float L_distance = angles  / 360.0 * 2 * pi * (R_spin + L/2);
        float R_distance = angles  / 360.0 * 2 * pi * (R_spin - L/2);
        if(L_distance < 0) L_distance = -L_distance;
        if(R_distance < 0) R_distance = -R_distance;
        clk_L = get_clk(L_distance);
        clk_R = get_clk(R_distance);
    }
    else if(left_or_right == 0) //左
    {
        VL = vel * (R_spin - L/2) / R_spin;
        VR = vel * (R_spin + L/2) / R_spin;
        float L_distance = angles  / 360.0 * 2 * pi * (R_spin - L/2);
        float R_distance = angles  / 360.0 * 2 * pi * (R_spin + L/2);
        if(L_distance < 0) L_distance = -L_distance;
        if(R_distance < 0) R_distance = -R_distance;
        clk_L = get_clk(L_distance);
        clk_R = get_clk(R_distance);
    }
    VL_motor = (VL );
    VR_motor = (VR );
    
    // if(VL_motor > 200) VL_motor = 200;
    // if(VR_motor > 200) VR_motor = 200;
    // if(VL_motor < -200) VL_motor = -200;
    // if(VR_motor < -200) VR_motor = -200;

    
        if(VL_motor >= 0)
        {
            Emm_V5_Pos_Control(1,0,VL_motor,acceleration,clk_L,motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,0,VL_motor,acceleration,clk_L,motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(1,1,-VL_motor,acceleration,clk_L,motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,1,-VL_motor,acceleration,clk_L,motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        if(VR_motor >= 0)
        {
            Emm_V5_Pos_Control(2,1,VR_motor,acceleration,clk_R,motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,1,VR_motor,acceleration,clk_R,motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(2,0,-VR_motor,acceleration,clk_R,motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,0,-VR_motor,acceleration,clk_R,motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        Emm_V5_Synchronous_motion(0);
}


/// @brief 差速移动
/// @param left_or_right  1为右，0为左
/// @param vel 车总体移动速度
/// @param R_spin  旋转半径
void move_leftright_front_with_spin(int left_or_right, float vel  , float R_spin)
{
    float L = 24;
    float r = 5;
    float VL = 0, VR = 0;
    int VL_motor = 0,VR_motor = 0;

    if(left_or_right == 1) //右
    {
        VL = vel * (R_spin + L/2) / R_spin;
        VR = vel * (R_spin - L/2) / R_spin;
    }
    else if(left_or_right == 0) //左
    {
        VL = vel * (R_spin - L/2) / R_spin;
        VR = vel * (R_spin + L/2) / R_spin;
    }
    VL_motor = (int)(VL / 5);
    VR_motor = (int)(VR / 5);
    
    if(VL_motor > 200) VL_motor = 200;
    if(VR_motor > 200) VR_motor = 200;
    if(VL_motor < -200) VL_motor = -200;
    if(VR_motor < -200) VR_motor = -200;

    
        if(VL_motor >= 0)
        {
            Emm_V5_Vel_Control(1,0,VL_motor,acceleration,1);
            HAL_Delay(10);
            Emm_V5_Vel_Control(3,0,VL_motor,acceleration,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Vel_Control(1,1,-VL_motor,acceleration,1);
            HAL_Delay(10);
            Emm_V5_Vel_Control(3,1,-VL_motor,acceleration,1);
            HAL_Delay(10);
        }
        if(VR_motor >= 0)
        {
            Emm_V5_Vel_Control(2,1,VR_motor,acceleration,1);
            HAL_Delay(10);
            Emm_V5_Vel_Control(4,1,VR_motor,acceleration,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Vel_Control(2,0,-VR_motor,acceleration,1);
            HAL_Delay(10);
            Emm_V5_Vel_Control(4,0,-VR_motor,acceleration,1);
            HAL_Delay(10);
        }
        Emm_V5_Synchronous_motion(0);

}

/// @brief 直线和圆一起调
/// @param  调参的思路：x/y_move_position乘了一个偏差量，最后直线的参数也乘了一个偏差量，通过修改这两个偏差量使得直线和圆的调整比例合适，同时也控制各自的数值不会过大
void slight_spin_and_move(void)
{
    float vel_target_1_f = adjust_spin_scale *spin_which_direction + adjust_move_scale * (-x_move_position - y_move_position);
    float vel_target_2_f = adjust_spin_scale *spin_which_direction + adjust_move_scale * (-x_move_position + y_move_position);
    float vel_target_3_f = adjust_spin_scale *spin_which_direction + adjust_move_scale * (x_move_position - y_move_position);
    float vel_target_4_f = adjust_spin_scale *spin_which_direction + adjust_move_scale * (x_move_position + y_move_position);

    // 限幅
    if(vel_target_1_f > motor_vel_adjust_with_spin) vel_target_1_f = motor_vel_adjust_with_spin;
    if(vel_target_2_f > motor_vel_adjust_with_spin) vel_target_2_f = motor_vel_adjust_with_spin;
    if(vel_target_3_f > motor_vel_adjust_with_spin) vel_target_3_f = motor_vel_adjust_with_spin;
    if(vel_target_4_f > motor_vel_adjust_with_spin) vel_target_4_f = motor_vel_adjust_with_spin;
    if(vel_target_1_f < -motor_vel_adjust_with_spin) vel_target_1_f = -motor_vel_adjust_with_spin;
    if(vel_target_2_f < -motor_vel_adjust_with_spin) vel_target_2_f = -motor_vel_adjust_with_spin;
    if(vel_target_3_f < -motor_vel_adjust_with_spin) vel_target_3_f = -motor_vel_adjust_with_spin;
    if(vel_target_4_f < -motor_vel_adjust_with_spin) vel_target_4_f = -motor_vel_adjust_with_spin;

    // 保证最小速度为min_motor_velocity
    if(vel_target_1_f > 0 && vel_target_1_f < min_motor_velocity) vel_target_1_f = min_motor_velocity;
    if(vel_target_1_f < 0 && vel_target_1_f > -min_motor_velocity) vel_target_1_f = -min_motor_velocity;
    if(vel_target_2_f > 0 && vel_target_2_f < min_motor_velocity) vel_target_2_f = min_motor_velocity;
    if(vel_target_2_f < 0 && vel_target_2_f > -min_motor_velocity) vel_target_2_f = -min_motor_velocity;
    if(vel_target_3_f > 0 && vel_target_3_f < min_motor_velocity) vel_target_3_f = min_motor_velocity;
    if(vel_target_3_f < 0 && vel_target_3_f > -min_motor_velocity) vel_target_3_f = -min_motor_velocity;
    if(vel_target_4_f > 0 && vel_target_4_f < min_motor_velocity) vel_target_4_f = min_motor_velocity;
    if(vel_target_4_f < 0 && vel_target_4_f > -min_motor_velocity) vel_target_4_f = -min_motor_velocity;

    // 最后转为int
    motor_vel_target_1 = (int)vel_target_1_f;
    motor_vel_target_2 = (int)vel_target_2_f;
    motor_vel_target_3 = (int)vel_target_3_f;
    motor_vel_target_4 = (int)vel_target_4_f;

    // // 保证最小速度为min_motor_velocity
    // if(motor_vel_target_1 > 0 && motor_vel_target_1 < min_motor_velocity) motor_vel_target_1 = (int)min_motor_velocity;
    // if(motor_vel_target_1 < 0 && motor_vel_target_1 > -min_motor_velocity) motor_vel_target_1 = (int)-min_motor_velocity;
    // if(motor_vel_target_2 > 0 && motor_vel_target_2 < min_motor_velocity) motor_vel_target_2 = (int)min_motor_velocity;
    // if(motor_vel_target_2 < 0 && motor_vel_target_2 > -min_motor_velocity) motor_vel_target_2 = (int)-min_motor_velocity;
    // if(motor_vel_target_3 > 0 && motor_vel_target_3 < min_motor_velocity) motor_vel_target_3 = (int)min_motor_velocity;
    // if(motor_vel_target_3 < 0 && motor_vel_target_3 > -min_motor_velocity) motor_vel_target_3 = (int)-min_motor_velocity;
    // if(motor_vel_target_4 > 0 && motor_vel_target_4 < min_motor_velocity) motor_vel_target_4 = (int)min_motor_velocity;
    // if(motor_vel_target_4 < 0 && motor_vel_target_4 > -min_motor_velocity) motor_vel_target_4 = (int)-min_motor_velocity;

    



        if(motor_vel_target_1 >= 0)
        {
            Emm_V5_Vel_Control(1,1,motor_vel_target_1,acceleration_adjust,1);
        }
        else
        {
            Emm_V5_Vel_Control(1,0,-motor_vel_target_1,acceleration_adjust,1);
        }
        HAL_Delay(10);
        if(motor_vel_target_2 >= 0)
        {
            Emm_V5_Vel_Control(2,1,motor_vel_target_2,acceleration_adjust,1);
        }
        else
        {
            Emm_V5_Vel_Control(2,0,-motor_vel_target_2,acceleration_adjust,1);
        }
        HAL_Delay(10);
        if(motor_vel_target_3 >= 0)
        {
            Emm_V5_Vel_Control(3,1,motor_vel_target_3,acceleration_adjust,1);
        }
        else
        {
            Emm_V5_Vel_Control(3,0,-motor_vel_target_3,acceleration_adjust,1);
        }
        HAL_Delay(10);
        if(motor_vel_target_4 >= 0)
        {
            Emm_V5_Vel_Control(4,1,motor_vel_target_4,acceleration_adjust,1);
        }
        else
        {
            Emm_V5_Vel_Control(4,0,-motor_vel_target_4,acceleration_adjust,1);
        }
        HAL_Delay(10);
        Emm_V5_Synchronous_motion(0);
        HAL_Delay(10);
}

/// @brief 根据移动的距离计算需要的脉冲数
/// @param distance 
/// @return 
uint32_t get_clk(float distance)
{
    return (uint32_t)(distance / wheel_circumference * pulse_per_circle);
}

float get_angle(float distance)
{
    return (float)distance / wheel_circumference * 360.0;
}

/// @brief 根据移动的距离和速度计算需要的时间（ms）
int get_distance_time(float distance, float velocity)
{
    float temp = (1000* distance /speed_ratio / velocity );
    return (int)(temp);
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

/// @brief 废弃
/// @param acc 
/// @param spin_direction 
/// @param times_count 
void spin_all_direction_tim(uint8_t acc, float spin_direction, int times_count)
{
    if(spin_direction >=0) // 左转
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

/// @brief 废弃
/// @param acc 
/// @param x_vel 
/// @param y_vel 
/// @param times_count 
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



/// @brief 横向位置*1.02  全向位置移动,需自己给Delay,直接使用move_all_direction_position_delay延时函数即可，速度1对应0.47cm/s，delaytime(ms) = 0.05*v *(256-acc)+ x/v （在加减速过程中，t2-t1 = （256-acc）*50us， vt2 = vt1 + 1
/// @param acc 
/// @param velocity 
/// @param x_move_length 面部朝向为车头前进方向，x轴正方向为右手方向
/// @param y_move_length y轴正方向为车头前进方向
void move_all_direction_position(uint8_t acc,uint16_t velocity, float x_move_length,float y_move_length)
{
    if(x_move_length >= 0 && y_move_length >= 0)
    {
        float delta_xy = x_move_length - y_move_length;
        Emm_V5_Pos_Control(1,0,velocity, acc,  get_clk(x_move_length + y_move_length),motor_pos_move_mode,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(x_move_length + y_move_length), motor_pos_move_mode, 1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length >= 0 && y_move_length < 0)
    {
        float delta_xy = x_move_length + y_move_length;
        Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(x_move_length - y_move_length), motor_pos_move_mode,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(x_move_length - y_move_length), motor_pos_move_mode,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(1,0,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length < 0 && y_move_length >= 0)
    {
        float delta_xy = y_move_length + x_move_length;
        Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(y_move_length - x_move_length), motor_pos_move_mode,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(y_move_length - x_move_length), motor_pos_move_mode,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(1,0,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,1,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length < 0 && y_move_length < 0)
    {
        float delta_xy = y_move_length - x_move_length;
        Emm_V5_Pos_Control(1,1,velocity, acc, get_clk(-x_move_length - y_move_length), motor_pos_move_mode,1);
        HAL_Delay(10);
        Emm_V5_Pos_Control(4,0,velocity, acc, get_clk(-x_move_length - y_move_length), motor_pos_move_mode,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            Emm_V5_Pos_Control(2,1,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,0,velocity, acc, get_clk(delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
        else
        {
            Emm_V5_Pos_Control(2,0,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
            Emm_V5_Pos_Control(3,1,velocity, acc, get_clk(-delta_xy), motor_pos_move_mode,1);
            HAL_Delay(10);
        }
    }
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
    // if(x_move_length < 0)
    // {
    //     x_move_length = -x_move_length;
    // }
    // if(y_move_length < 0)
    // {
    //     y_move_length = -y_move_length;
    // }
    // int timex =  get_distance_time(x_move_length, (int)velocity)+0.05*(int)velocity*(256-acc);
    // int timey = get_distance_time(y_move_length, (int)velocity)+0.05*(int)velocity*(256-acc);
    // HAL_Delay(timex > timey ? timex : timey);// 1068+630

    // char temp_print[30];
    // sprintf(temp_print,"t0.txt=\"%d,%d\"\xff\xff\xff",timex,timey);
    // HAL_UART_Transmit(&huart5, (uint8_t *)temp_print, strlen(temp_print), 1000);
    
    // move_all_direction_position_delay(acc,velocity,x_move_length,y_move_length);

}

/// @brief 配合move_all_direction_position使用，计算时间并延时
/// @param acc 
/// @param velocity 
/// @param x_move_length 
/// @param y_move_length 
void move_all_direction_position_delay(uint8_t acc,uint16_t velocity, float x_move_length,float y_move_length)
{
    if(x_move_length < 0)
    {
        x_move_length = -x_move_length;
    }
    if(y_move_length < 0)
    {
        y_move_length = -y_move_length;
    }
    int timex =  get_distance_time(x_move_length, (int)velocity)+0.05*(int)velocity*(256-acc);
    int timey = get_distance_time(y_move_length, (int)velocity)+0.05*(int)velocity*(256-acc);
    // HAL_Delay(timex > timey ? timex : timey);

    //TODO 原计划是计算达不到匀速时的时间，但实际测试时间过长，遂舍去
    // if(x_move_length < (float)((float)velocity * (float)velocity * speed_ratio  * (256-(int)acc) *20000.0))
    // {
    //     timex = 1000*sqrt(x_move_length *(256-acc) /20.0 / speed_ratio);
    // }
    // if(y_move_length < (float)((float)velocity * (float)velocity * speed_ratio  * (256-(int)acc) *20000.0))
    // {
    //     timey = 1000*sqrt(y_move_length *(256-acc) /20.0 / speed_ratio);
    // }

    // char temp_print[30];
    // sprintf(temp_print,"t0.txt=\"%d,%d\"\xff\xff\xff",timex,timey);
    // HAL_UART_Transmit(&huart5, (uint8_t *)temp_print, strlen(temp_print), 1000);

    int delay_time = timex > timey ? timex : timey;
    HAL_Delay(delay_time);
    
}


void move_all_direction_position_y42(uint16_t acc_start,uint16_t acc_stop, float vel,float x_move_length,float y_move_length)
{
    if(x_move_length >= 0 && y_move_length >= 0)
    {
        float delta_xy = x_move_length - y_move_length;
        ZDT_X42_V2_Traj_Position_Control(1,0,acc_start,acc_stop,vel,  get_angle(x_move_length + y_move_length),0,1);
        HAL_Delay(10);
        ZDT_X42_V2_Traj_Position_Control(4,1,acc_start,acc_stop,vel,  get_angle(x_move_length + y_move_length),0,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            ZDT_X42_V2_Traj_Position_Control(2,0,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(3,1,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
        }
        else
        {
            ZDT_X42_V2_Traj_Position_Control(2,1,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(3,0,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length >= 0 && y_move_length < 0)
    {
        float delta_xy = x_move_length + y_move_length;
        ZDT_X42_V2_Traj_Position_Control(2,0,acc_start,acc_stop,vel,  get_angle(x_move_length - y_move_length),0,1);
        HAL_Delay(10);
        ZDT_X42_V2_Traj_Position_Control(3,1,acc_start,acc_stop,vel,  get_angle(x_move_length - y_move_length),0,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            ZDT_X42_V2_Traj_Position_Control(1,0,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(4,1,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
        }
        else
        {
            ZDT_X42_V2_Traj_Position_Control(1,1,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(4,0,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length < 0 && y_move_length >= 0)
    {
        float delta_xy = y_move_length + x_move_length;
        ZDT_X42_V2_Traj_Position_Control(2,1,acc_start,acc_stop,vel,  get_angle(x_move_length - y_move_length),0,1);
        HAL_Delay(10);
        ZDT_X42_V2_Traj_Position_Control(3,0,acc_start,acc_stop,vel,  get_angle(x_move_length - y_move_length),0,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            ZDT_X42_V2_Traj_Position_Control(1,0,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(4,1,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
        }
        else
        {
            ZDT_X42_V2_Traj_Position_Control(1,1,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(4,0,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
        }
    }
    else if (x_move_length < 0 && y_move_length < 0)
    {
        float delta_xy = y_move_length - x_move_length;
        ZDT_X42_V2_Traj_Position_Control(1,1,acc_start,acc_stop,vel,  get_angle(-x_move_length - y_move_length),0,1);
        HAL_Delay(10);
        ZDT_X42_V2_Traj_Position_Control(4,0,acc_start,acc_stop,vel,  get_angle(-x_move_length - y_move_length),0,1);
        HAL_Delay(10);
        if(delta_xy >= 0)
        {
            ZDT_X42_V2_Traj_Position_Control(2,1,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(3,0,acc_start,acc_stop,vel,  get_angle(delta_xy),0,1);
            HAL_Delay(10);
        }
        else
        {
            ZDT_X42_V2_Traj_Position_Control(2,0,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
            ZDT_X42_V2_Traj_Position_Control(3,1,acc_start,acc_stop,vel,  get_angle(-delta_xy),0,1);
            HAL_Delay(10);
        }
    }
    ZDT_X42_V2_Synchronous_motion(0);
    HAL_Delay(10);
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

void Forward_move_with_yaw_adjust(uint16_t vel_left,uint16_t vel_right,uint8_t acc)
{
    
    Emm_V5_Vel_Control(1, 0, vel_left, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(2, 1, vel_right, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(3, 0, vel_left, acc, 1);
    HAL_Delay(10);
    Emm_V5_Vel_Control(4, 1, vel_right, acc, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}


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

void spin_right_180(uint16_t vel,uint8_t acc)
{
    uint32_t clk = (uint32_t)((float)180.8 / 360 * spin_radius_180 * 2 * pi / wheel_circumference * pulse_per_circle);
    Emm_V5_Pos_Control(1, 0, vel, acc,clk, motor_pos_move_mode,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 0, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 0, vel, acc,clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 0, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

void spin_right_180_y42(float vel,uint16_t acc_start, uint16_t acc_stop)
{
    float clk = 180;
    ZDT_X42_V2_Traj_Position_Control(1,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(2,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(3,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(4,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Synchronous_motion(0);
    HAL_Delay(10);
}

void spin_left_180(uint16_t vel,uint8_t acc)
{
    uint32_t clk = (uint32_t)((float)180.8 / 360 * spin_radius_180 * 2 * pi / wheel_circumference * pulse_per_circle);
    Emm_V5_Pos_Control(1, 1, vel, acc,clk, motor_pos_move_mode,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 1, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 1, vel, acc,clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 1, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

void spin_left_180_y42(float vel,uint16_t acc_start, uint16_t acc_stop)
{
    float clk = 180;
    ZDT_X42_V2_Traj_Position_Control(1,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(2,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(3,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(4,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Synchronous_motion(0);
    HAL_Delay(10);
}

void spin_right_90(uint16_t vel,uint8_t acc)
{
    uint32_t clk = (uint32_t)((float)90.7 / 360 * spin_radius_90 * 2 * pi / wheel_circumference * pulse_per_circle);
    Emm_V5_Pos_Control(1, 0, vel, acc,clk, motor_pos_move_mode,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 0, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 0, vel, acc,clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 0, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

void spin_right_90_y42(float vel, uint16_t acc_start,uint16_t acc_stop)
{
    float clk = 90;
    ZDT_X42_V2_Traj_Position_Control(1,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(2,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(3,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(4,0,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Synchronous_motion(0);
    HAL_Delay(10);
}

void spin_left_90(uint16_t vel,uint8_t acc)
{
    uint32_t clk = (uint32_t)((float)90.27 / 360 * spin_radius_90 * 2 * pi / wheel_circumference * pulse_per_circle);
    Emm_V5_Pos_Control(1, 1, vel, acc,clk, motor_pos_move_mode,1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(2, 1, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(3, 1, vel, acc,clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Pos_Control(4, 1, vel, acc, clk, motor_pos_move_mode, 1);
    HAL_Delay(10);
    Emm_V5_Synchronous_motion(0);
    HAL_Delay(10);
}

void spin_left_90_y42(float vel, uint16_t acc_start,uint16_t acc_stop)
{
    float clk = 90;
    ZDT_X42_V2_Traj_Position_Control(1,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(2,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(3,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Traj_Position_Control(4,1,acc_start,acc_stop,vel,clk,0,1);
    HAL_Delay(10);
    ZDT_X42_V2_Synchronous_motion(0);
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

void stop_y42(void)
{
    ZDT_X42_V2_Stop_Now((uint8_t)1, (uint8_t)1);
    HAL_Delay(10);
    ZDT_X42_V2_Stop_Now((uint8_t)2,(uint8_t)1);
    HAL_Delay(10);
    ZDT_X42_V2_Stop_Now((uint8_t)3,(uint8_t)1);
    HAL_Delay(10);
    ZDT_X42_V2_Stop_Now((uint8_t)4, (uint8_t)1);
    HAL_Delay(10);
    ZDT_X42_V2_Synchronous_motion(0);
    HAL_Delay(10);
}


/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，(此处有问题）1为顺时针，0为逆时针
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
  * @param    dir ：方向        ，(此处有问题）1为顺时针，0为逆时针
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

/// @brief 修改电机ID
/// @param addr 目标电机的ID
/// @param modify_to_addr 修改之后的ID
void set_motor_ID(uint8_t addr,uint8_t modify_to_addr)
{
    uint8_t cmd[16] = {0};

    cmd[0] = addr;
    cmd[1] = 0xAE;
    cmd[2] = 0x4B;
    cmd[3] = 0x01;
    cmd[4] = modify_to_addr;
    cmd[5] = 0x6B;
    usart_SendCmd_u1(cmd, 6);
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

/**
  * @brief    梯形曲线位置模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    acc     ：加速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    dec     ：减速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[32] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(acc >> 8);       // 加速加速度(RPM/s)高8位字节
  cmd[4]  =  (uint8_t)(acc >> 0);       // 加速加速度(RPM/s)低8位字节  
  cmd[5]  =  (uint8_t)(dec >> 8);       // 减速加速度(RPM/s)高8位字节
  cmd[6]  =  (uint8_t)(dec >> 0);       // 减速加速度(RPM/s)低8位字节  
  cmd[7]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[8]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[9]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[10] =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[11] =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[12] =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[13] =  raf;                       // 相位位置/绝对位置标志
  cmd[14] =  snF;                       // 多机同步运动标志
  cmd[15] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 16);
}



/**
  * @brief    直通限速位置模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t addr, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0; uint32_t pos = 0;

  // 将速度和位置放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFB;                      // 功能码
  cmd[2]  =  dir;                       // 符号（方向）
  cmd[3]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节 
  cmd[5]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
  cmd[6]  =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
  cmd[7]  =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
  cmd[8]  =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
  cmd[9]  =  raf;                       // 相位位置/绝对位置标志
  cmd[10] =  snF;                       // 多机同步运动标志
  cmd[11] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 12);
}

/**
  * @brief    速度模式
  * @param    addr  	：电机地址
  * @param    dir     ：方向         ，0为CW，其余值为CCW
  * @param    v_ramp  ：斜率(RPM/s)  ，范围0 - 65535RPM/s
  * @param    velocity：速度(RPM)    ，范围0.0 - 4000.0RPM
  * @param    snF     ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF)
{
  uint8_t cmd[16] = {0}; uint16_t vel = 0;

  // 将速度放大10倍发送过去
  vel = (uint16_t)ABS(velocity * 10.0f);

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 符号（方向）
  cmd[3] =  (uint8_t)(v_ramp >> 8);     // 速度斜率(RPM/s)高8位字节
  cmd[4] =  (uint8_t)(v_ramp >> 0);     // 速度斜率(RPM/s)低8位字节
  cmd[5] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[6] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[7] =  snF;                        // 多机同步运动标志
  cmd[8] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd_u1(cmd, 9);
}


/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF)
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
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Synchronous_motion(uint8_t addr)
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