#include "my_servo.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;
extern int theta_servo_value[],r_servo_value[];

extern void print_to_screen(int t_num,char *pData);
/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

// 树莓派 Tx Rx

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!      普通舵机调试的注意事项
//!      夹爪PD13  载物盘PD15 机械臂PD12
//!      普通舵机转动120°对应给定数据为44.44
//!      普通舵机参数范围25-125  对应0-270°


//? 夹爪舵机参数
const int open_claw_avoid_collide_position = 108; //从物料的侧面过的张开角度 //TODO 待修改
const int open_claw_180_position = 104;  
const int open_claw_position = 86; 
const int open_claw_bigger_position = 92; 
const int close_claw_position = 73; 
const int close_bit_position = 78; //略微夹紧

//? 载物盘舵机参数 
const int state_spin_position_1 = 25;  //TODO 待修改
const int state_spin_position_2 = 69;//TODO 待修改
const int state_spin_position_3 = 111; //TODO 待修改

const int state_spin_without_claw_position_1 = 25;   //TODO 待修改
const int state_spin_without_claw_position_2 = 69;//TODO 待修改
const int state_spin_without_claw_position_3 = 111;//TODO 待修改



//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!     飞特舵机调试的注意事项
//!     飞特舵机参数范围0-4095
//!     飞特舵机速度范围0-3400
//!     飞特舵机加速度范围0-255
//!     对于机械臂伸缩，2000步进对应12.4cm，0.0062cm/pulse

const int feet_acc = 210; //TODO 待修改
const int feet_acc_claw_up_down = 0;
const int feet_acc_put_down_ground_slightly = 240;
const int feet_acc_claw_spin = 0;
const int feet_acc_claw_spin_slight = 100;

//? 初赛物料升降参数（一号舵机）  
const int servo_1_add_num = -12; //! 应对位置变化的调整量

//TODO 待修改（全部重新测）
//! 测量新值后务必注意有没有负值
const int put_claw_down_pile_position = 1878+servo_1_add_num; //码垛位置 
const int put_claw_down_state_position = 777 +servo_1_add_num; //!从车的载物盘上 与放在地上相差2230
const int put_claw_down_position = 1701+servo_1_add_num;  // 从转盘上取物料  
const int put_claw_down_ground_position = 3010+servo_1_add_num; // 放在地上 3144
const int put_claw_up_top_position =280+servo_1_add_num; // 最高点  
const int put_claw_up_position =1667+servo_1_add_num; //  看粗调移动底盘的位置
const int put_claw_down_near_ground_position = 2770+servo_1_add_num; //!细调放置的位置
const int put_claw_down_near_plate_position = 1600+servo_1_add_num; //转盘放置细调的位置


//? 机械臂前端旋转参数（二号舵机）
const int claw_spin_position_front = 3340 ; // 2号精密舵机回到前方
const int claw_spin_position_state = 1600; // 2号精密舵机回到载物盘
const int claw_spin_without_claw_position_state = 1600; //与上面一样


//? 中板整体旋转参数（三号舵机）
//! 测量新值后务必注意有没有负值
// 左：-1083 右： +953
const int middle_arm = 1935;  // 舵机3在不进行动作时的默认位置
const int theta_left_position_limit = middle_arm-1083;
const int theta_right_position_limit = middle_arm+953 ;
const int theta_right_position_rlimit = middle_arm + 550; 
const int theta_left_position_rlimit = middle_arm - 550; 
const int r_front_position_limit = 3431;
const int r_back_position_limit = 1261;
const int r_back_position_rlimit = 2568; // 当theta超过rlimit，r的限制值不能小于这个值


//? 机械臂整体伸缩参数（四号舵机）
//TODO 待测量（全部）
const int shrink_arm = 2973;  //机械臂运动到从车上载物盘抓取物料
const int stretch_arm = 1860; // 机械臂默认伸长位置
const int shrink_arm_all = 1860;


//? 左中右三个动作对应的各自舵机参数
//TODO 待测量（全部）
const int left_2 = 3328; 
const int left_3 = middle_arm-830;  
const int left_3_pileup = left_3;
const int left_4 =  3010; 

const int right_2 = 3328; 
const int right_3 = middle_arm+840; 
const int right_3_pileup = right_3;
const int right_4 =  2930; 

const int middle_2 = 3328;
const int middle_3 = middle_arm;
const int middle_3_pileup = middle_arm;
const int middle_4 =  stretch_arm;  //TODO 这里可以考虑用和其不一样的值



volatile int x_plate_error = 0, y_plate_error = 0; // 载物盘中心和色环中心的偏差
volatile int x_camera_error = 0, y_camera_error = 0; // 物料中心和色环中心的偏差
volatile int x_camera_error_plate = 0, y_camera_error_plate = 0; // 载物盘带有色环时，中心和色环中心的偏差

//! 初始化，需要和上面保持一致
volatile int  r_servo_now =  stretch_arm; // 机械臂伸缩舵机的位置
volatile int  theta_servo_now = middle_arm; // 机械臂中板旋转舵机的位置


volatile int x_error_last = 0, y_error_last = 0; // 上一次的偏差
volatile int x_error_long_last = 0, y_error_long_last = 0; // 上上次的偏差

extern const float pixel_to_distance_r,pixel_to_distance_theta; 
extern const float Kp_theta,Kd_theta,Ki_theta;
extern const float Kp_r,Kd_r,Ki_r;

extern float error_decrease_gain;
extern const float Kp_decrease_gain_big;
extern const float Kp_decrease_gain_middle; 
extern const float Kp_decrease_gain_small ;

extern const float middle_limit;
extern const float small_limit;


char temp_function_char[50];
long UART_count = 0;


/// @brief 机械臂移动的限位函数
/// @param d_theta_move_position 
/// @param d_r_move_position 
void servo_move_with_limit(int d_theta_move_position,int d_r_move_position)
{
    int target_theta = theta_servo_now + d_theta_move_position;
    int target_r = r_servo_now + d_r_move_position;

    // 1. 判断目标theta是否超出rlimit区间
    int theta_out_rlimit = (target_theta < theta_left_position_rlimit) || (target_theta > theta_right_position_rlimit);
    // 2. 判断目标r是否大于r_back_position_rlimit
    int r_valid = (target_r > r_back_position_rlimit);

    // 如果超出范围
    if (theta_out_rlimit) 
    {
        // 只有r大于rlimit时才允许theta超出
        if (r_valid) 
        {
            // 允许移动到目标位置，但是r要大于r_back_position_rlimit,小于r_front_position_limit
            if (target_r > r_front_position_limit)
            {
                target_r = r_front_position_limit;
                print_to_screen(1,"r_front");  
            } 
        } 
        else 
        {
            // 不允许再退了
            target_r = r_back_position_rlimit;  
            print_to_screen(1,"r_back");                 
        }

        // 相当于优先满足theta角度
        if (target_theta < theta_left_position_limit)
        {
            target_theta = theta_left_position_limit;
            print_to_screen(2,"theta_left");  
        }
        else if (target_theta > theta_right_position_limit)
        {
            target_theta = theta_right_position_limit;
            print_to_screen(2,"theta_right");  
        }

        

    } 
    else 
    {
        // theta在rlimit区间内，r可以任意
        // 但r不能超出物理极限
        if (target_r < r_back_position_limit) 
        {
            target_r = r_back_position_limit;
        } 
        else if (target_r > r_front_position_limit) 
        {
            target_r = r_front_position_limit;
        }
    }
    feetech_servo_move(4, target_r, 4000, feet_acc);
    r_servo_now = target_r;
    HAL_Delay(10); 
    feetech_servo_move(3, target_theta, 2000, feet_acc);
    theta_servo_now = target_theta;
}

/// @brief 单次的调整
/// @param x_plate_error_in 
/// @param y_plate_error_in 
void adjust_plate(int x_plate_error_in,int y_plate_error_in)
{
    servo_move_with_limit(x_plate_error_in,y_plate_error_in);
            
            
    // 以下为原先版本
            // if(theta_servo_now+x_plate_error_in < theta_right_position_limit && theta_servo_now+x_plate_error_in > theta_left_position_limit)
            // {
            //     theta_servo_now += x_plate_error_in;
            //     feetech_servo_move(3,theta_servo_now,2000,feet_acc);
            //     x_plate_error_in = 0;
            // }
            // else if (theta_servo_now+x_plate_error_in > theta_right_position_limit)
            // {
            //     theta_servo_now = theta_right_position_limit;
            //     feetech_servo_move(3,theta_servo_now,2000,feet_acc);
            //     x_plate_error_in = 0;
            // }
            // else if (theta_servo_now+x_plate_error_in < theta_left_position_limit)
            // {
            //     theta_servo_now = theta_left_position_limit;
            //     feetech_servo_move(3,theta_servo_now,2000,feet_acc);
            //     x_plate_error_in = 0;
            // }
            // HAL_Delay(50); //TODO 可能会影响速度
            // if(r_servo_now+y_plate_error_in < r_front_position_limit && r_servo_now+y_plate_error_in > r_back_position_limit)
            // {
            //     r_servo_now += y_plate_error_in;
            //     feetech_servo_move(4,r_servo_now,4000,feet_acc); //TODO 飞特加速度未测试
            //     y_plate_error_in = 0;
            // }
            // else if (r_servo_now+y_plate_error_in > r_front_position_limit)
            // {
            //     r_servo_now = r_front_position_limit;
            //     feetech_servo_move(4,r_servo_now,4000,feet_acc);
            //     y_plate_error_in = 0;
            // }
            // else if (r_servo_now+y_plate_error_in < r_back_position_limit)
            // {
            //     r_servo_now = r_back_position_limit;
            //     feetech_servo_move(4,r_servo_now,4000,feet_acc);
            //     y_plate_error_in = 0;
            // }
            // HAL_Delay(50);
            // if(theta_servo_now+x_plate_error_in < theta_right_position_limit && theta_servo_now+x_plate_error_in > theta_left_position_limit)
            // {
            //     theta_servo_now += x_plate_error_in;
            //     feetech_servo_move(3,theta_servo_now,4000,feet_acc);
            //     x_plate_error_in = 0;
            // }
            // else if (theta_servo_now+x_plate_error_in > theta_right_position_limit)
            // {
            //     theta_servo_now = theta_right_position_limit;
            //     feetech_servo_move(3,theta_servo_now,4000,feet_acc);
            //     x_plate_error_in = 0;
            // }
            // else if (theta_servo_now+x_plate_error_in < theta_left_position_limit)
            // {
            //     theta_servo_now = theta_left_position_limit;
            //     feetech_servo_move(3,theta_servo_now,4000,feet_acc);
            //     x_plate_error_in = 0;
            // }
}

//? 机械臂控制中舍弃的部分
// if(x_error > theta_big_limit) // 应该向右转动
    // {
    //     theta_adjust_values = theta_pulse_servo_big;
    // }
    // else if(x_error < theta_big_limit && x_error > theta_mid_limit)
    // {
    //     theta_adjust_values = theta_pulse_servo_mid;
    // }
    // else if(x_error < theta_mid_limit && x_error > theta_small_limit)
    // {
    //     theta_adjust_values = theta_pulse_servo_small;
    // }
    // else if(x_error >0 && x_error < theta_small_limit)
    // {
    //     theta_adjust_values = 1;
    //     is_theta_ok = 1;
    // }
    // else if(x_error < -theta_big_limit) // 应该向左转动
    // {
    //     theta_adjust_values = -theta_pulse_servo_big;
    // }
    // else if(x_error > -theta_big_limit && x_error < -theta_mid_limit)
    // {
    //     theta_adjust_values = -theta_pulse_servo_mid;
    // }
    // else if(x_error > -theta_mid_limit && x_error < -theta_small_limit)
    // {
    //     theta_adjust_values = -theta_pulse_servo_small;
    // }
    // else if(x_error < 0 && x_error > -theta_small_limit)
    // {
    //     theta_adjust_values = -1;
    //     is_theta_ok = 1;
    // }
    // if(y_error >r_big_limit) // 应该向前伸长
    // {
    //     r_adjust_values = r_pulse_servo_big;
    // }
    // else if(y_error < r_big_limit && y_error > r_mid_limit)
    // {
    //     r_adjust_values = r_pulse_servo_mid;
    // }
    // else if(y_error < r_mid_limit && y_error > r_small_limit)
    // {
    //     r_adjust_values = r_pulse_servo_small;
    // }
    // else if(y_error > 0 && y_error < r_small_limit)
    // {
    //     r_adjust_values = 1;
    //     is_r_ok = 1;
    // }
    // else if(y_error < -r_big_limit) // 应该向后缩短
    // {
    //     r_adjust_values = -r_pulse_servo_big;
    // }
    // else if(y_error > -r_big_limit && y_error < -r_mid_limit)
    // {
    //     r_adjust_values = -r_pulse_servo_mid;
    // }
    // else if(y_error > -r_mid_limit && y_error < -r_small_limit)
    // {
    //     r_adjust_values = -r_pulse_servo_small;
    // }
    // else if(y_error < 0 && y_error >- r_small_limit)
    // {
    //     r_adjust_values = -1;
    //     is_r_ok = 1;
    // }
    // theta_adjust_values = Kp_theta * (x_error-x_error_last) + Ki_theta * (x_error ) + Kd_theta * (x_error_long_last + x_error - 2*x_error_last);
    // r_adjust_values = Kp_r * (y_error - y_error_last) + Ki_r * (y_error) + Kd_r * ( y_error_long_last + y_error - 2*y_error_last);
    // theta_adjust_values = theta_adjust_values * pixel_to_distance_theta;
    // r_adjust_values = r_adjust_values * pixel_to_distance_r;

/// @brief 根据视觉进行机械臂末端姿态校正
/// @param 调参思路： 调整pixel_to_distance
int adjust_position_with_camera(int x_error, int y_error,int is_min_1 )
{   
    if(x_error == 0 && y_error == 0)
    {
        return -1;
    }
    float r_adjust_values = 0, theta_adjust_values = 0;
    int is_theta_ok = 0, is_r_ok = 0;
    int x_origin = x_error;
    int y_origin = y_error;
    int max_servo_movement = 100;

    if(x_error > 300 )
    {
        x_error = 300;
    }
    if(y_error > 300)
    {
        y_error = 300;
    }
    if(x_error < -300)
    {
        x_error = -300;
    }
    if(y_error < -300)
    {
        y_error = -300;
    }
    x_camera_error = 0; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    y_camera_error = 0; //TODO 这里需要取消注释（如果是）
    

    //TODO 加入x、yerror的PID
    theta_adjust_values = Kp_theta * x_error + Ki_theta * (x_error + x_error_last + x_error_long_last) + Kd_theta * (x_error - x_error_last);
    r_adjust_values = Kp_r * y_error + Ki_r * (y_error + y_error_last + y_error_long_last) + Kd_r * ( y_error - y_error_last);
    theta_adjust_values = theta_adjust_values * pixel_to_distance_theta;
    r_adjust_values = r_adjust_values * pixel_to_distance_r;
    x_error_long_last = x_error_last;
    x_error_last = x_error;
    y_error_long_last = y_error_last;
    y_error_last = y_error;
    if(theta_adjust_values > max_servo_movement)
    {
        theta_adjust_values = max_servo_movement;
    }
    if(r_adjust_values > max_servo_movement)
    {
        r_adjust_values = max_servo_movement;
    }
    if(theta_adjust_values < -max_servo_movement)
    {
        theta_adjust_values = -max_servo_movement;
    }
    if(r_adjust_values < -max_servo_movement)
    {
        r_adjust_values = -max_servo_movement;
    }

    
    if(is_min_1 == 1) //是否需要最小值限制
    {
        if(theta_adjust_values <1 && theta_adjust_values >0)
        {
            theta_adjust_values = 1;
        }
        if(theta_adjust_values >-1 && theta_adjust_values <0)
        {
            theta_adjust_values = -1;
        }
        if(r_adjust_values <1 && r_adjust_values >0)
        {
            r_adjust_values = 1;
        }
        if(r_adjust_values >-1 && r_adjust_values <0)
        {
            r_adjust_values = -1;
        }
    }
    // sprintf(temp_function_char, "  No:%d,  x:%d,y:%d  theta:%.2f,r:%.2f    ", UART_count,x_origin, y_origin,theta_adjust_values,r_adjust_values);
    // HAL_UART_Transmit(&huart3, (uint8_t*)temp_function_char, strlen(temp_function_char), 50); //发给树莓派，开始校正
    // HAL_Delay(50);
    UART_count++;
    if(UART_count > 1000000)
    {
        UART_count = 0;
    }
    theta_adjust_values = (int)theta_adjust_values;
    r_adjust_values = (int)r_adjust_values;
    // x_camera_error = 0;
    // y_camera_error = 0;
    //! 以下为机械臂位置限制
    // if((theta_servo_now + theta_adjust_values < theta_right_position_rlimit ) && (theta_servo_now + theta_adjust_values > theta_left_position_rlimit))
    // {
    //     if((r_servo_now + r_adjust_values > r_back_position_limit ) && (r_servo_now + r_adjust_values < r_front_position_limit))
    //     {
    //         feetech_servo_move(4,(r_servo_now + r_adjust_values),4000,100);
    //         r_servo_now += r_adjust_values;
    //         HAL_Delay(10);
    //         feetech_servo_move(3,(theta_servo_now + theta_adjust_values),4000,100);
    //         theta_servo_now += theta_adjust_values;
    //     }
    // }
    // else if(((theta_servo_now + theta_adjust_values > theta_left_position_limit ) && (theta_servo_now + theta_adjust_values < theta_left_position_rlimit)) || ((theta_servo_now + theta_adjust_values > theta_right_position_rlimit ) && (theta_servo_now + theta_adjust_values < theta_right_position_limit)) )
    // {
    //     if((r_servo_now + r_adjust_values > r_back_position_rlimit ) && (r_servo_now + r_adjust_values < r_front_position_limit))
    //     {
    //         feetech_servo_move(4,(r_servo_now + r_adjust_values),4000,100);
    //         r_servo_now += r_adjust_values;
    //         HAL_Delay(10);
    //         feetech_servo_move(3,(r_servo_now + r_adjust_values),4000,100);
    //         theta_servo_now += theta_adjust_values;
    //     }
    // }


    // sprintf(temp_function_char,"    theta:%.2f, r:%.2f    ",theta_adjust_values,r_adjust_values);
    // HAL_UART_Transmit(&huart3, (uint8_t*)temp_function_char, strlen(temp_function_char), 50);
    //! 原先版本，限位不够
    // if((theta_servo_now + theta_adjust_values < theta_right_position_limit) && (theta_servo_now + theta_adjust_values > theta_left_position_limit) && (r_servo_now + r_adjust_values < r_front_position_limit) && (r_servo_now + r_adjust_values > r_back_position_limit))
    // {
    //     feetech_servo_move(4,r_servo_now + r_adjust_values,4000,180);
    //     r_servo_now += r_adjust_values;
    //     HAL_Delay(10);
    //     feetech_servo_move(3,theta_servo_now + theta_adjust_values,4000,180);
    //     theta_servo_now += theta_adjust_values;
    // }
    // else
    // {
    //     // HAL_UART_Transmit(&huart3, (uint8_t*)"limit", strlen("limit"), 50);
    //     //TODO 加入通知的功能
    // }


    //! 加入了限位后的
    servo_move_with_limit(theta_adjust_values,r_adjust_values);


    if(is_r_ok == 1 && is_theta_ok == 1)
    {
        return 1;
    }

    else
    {
        return 0;
    }
}


void adjust_position_with_camera_new(int x_error, int y_error, int dt)
{
    static float old_input_x_error = 0;
    static float old_input_y_error = 0;
    static int error_max = 50;

    float r_adjust_values = 0, theta_adjust_values = 0;

    float Kp_gain = 0;

    float max_servo_movement_single = 700;

    if(x_error == 0 && y_error == 0)
    {
        return;
    }

    if(x_error == old_input_x_error && y_error == old_input_y_error) //如果没有新的值传进来，对旧的值进行衰减
    {
        x_error *= error_decrease_gain;
    
        y_error *= error_decrease_gain;
    }

    if(x_error > error_max )
    {
        x_error = error_max;
    }
    if(y_error > error_max)
    {
        y_error = error_max;
    }
    if(x_error < -error_max)
    {
        x_error = -error_max;
    }
    if(y_error < -error_max)
    {
        y_error = -error_max;
    }

    if(abs(x_error) > middle_limit)
    {
        Kp_gain = Kp_decrease_gain_big;
    }
    if(abs(x_error) < middle_limit && abs(x_error) > small_limit)
    {
        Kp_gain = Kp_decrease_gain_middle;
    }
    if(abs(x_error) < small_limit )
    {
        Kp_gain = Kp_decrease_gain_small;
    }

    theta_adjust_values = Kp_theta * x_error * Kp_gain+ Ki_theta * (x_error + x_error_last + x_error_long_last) + Kd_theta * (x_error - x_error_last)/dt/1000;
    r_adjust_values = Kp_r * y_error * Kp_gain+ Ki_r * (y_error + y_error_last + y_error_long_last) + Kd_r * ( y_error - y_error_last)/dt/1000;
    theta_adjust_values = theta_adjust_values * pixel_to_distance_theta;
    r_adjust_values = r_adjust_values * pixel_to_distance_r;
    x_error_long_last = x_error_last;
    x_error_last = x_error;
    y_error_long_last = y_error_last;
    y_error_last = y_error;

    //输出限幅
    if(theta_adjust_values > max_servo_movement_single)
    {
        theta_adjust_values = max_servo_movement_single;
    }
    if(r_adjust_values > max_servo_movement_single)
    {
        r_adjust_values = max_servo_movement_single;
    }
    if(theta_adjust_values < -max_servo_movement_single)
    {
        theta_adjust_values = -max_servo_movement_single;
    }
    if(r_adjust_values < -max_servo_movement_single)
    {
        r_adjust_values = -max_servo_movement_single;
    }

    theta_adjust_values = (int)theta_adjust_values;
    r_adjust_values = (int)r_adjust_values;

    if((theta_servo_now + theta_adjust_values < theta_right_position_limit) && (theta_servo_now + theta_adjust_values > theta_left_position_limit) && (r_servo_now + r_adjust_values < r_front_position_limit) && (r_servo_now + r_adjust_values > r_back_position_limit))
    {
        feetech_servo_move(4,r_servo_now + r_adjust_values,4000,180);
        r_servo_now += r_adjust_values;
        HAL_Delay(10);
        feetech_servo_move(3,theta_servo_now + theta_adjust_values,4000,180);
        theta_servo_now += theta_adjust_values;
        HAL_Delay(10);
    }

}

/// @brief avoid 动作  无视觉辅助的从圆环上抓取物料
/// @param position 
/// @param is_default_position 1则为在默认的位置抓取，0则为根据先前记录的位置抓取，默认为 1
void get_and_load_openloop_avoid(int position,int is_default_position)
{
    state_spin_without_claw(position);
    open_claw_bigger();
    switch(position)
    {
        case 1:
            if( is_default_position == 1)
            {
                feetech_servo_move(3,right_3,2000,feet_acc);
                feetech_servo_move(4,right_4,4095,feet_acc);
                r_servo_now = right_4;
                theta_servo_now = right_3;
            }
            else
            {
                feetech_servo_move(3,theta_servo_value[1],2000,feet_acc);
                feetech_servo_move(4,r_servo_value[1],4095,feet_acc);
                theta_servo_now = theta_servo_value[1];
                r_servo_now = r_servo_value[1];
            }
            break;
        case 2:
            if( is_default_position == 1)
            {
                feetech_servo_move(3,middle_3,2000,feet_acc);
                feetech_servo_move(4,middle_4,4095,feet_acc);
                r_servo_now = middle_4;
                theta_servo_now = middle_3;
            }
            else
            {
                feetech_servo_move(3,theta_servo_value[2],2000,feet_acc);
                feetech_servo_move(4,r_servo_value[2],4095,feet_acc);
                r_servo_now = r_servo_value[2];
                theta_servo_now = theta_servo_value[2];
            }
            break;
        case 3:
            if(is_default_position == 1)
            {
                feetech_servo_move(3,left_3,2000,feet_acc);
                feetech_servo_move(4,left_4,4095,feet_acc);
                r_servo_now = left_4;
                theta_servo_now = left_3;
            }
            else
            {
                feetech_servo_move(3,theta_servo_value[3],2000,feet_acc);
                feetech_servo_move(4,r_servo_value[3],4095,feet_acc);
                r_servo_now = r_servo_value[3];
                theta_servo_now = r_servo_value[3];
            }

            break;
    }
    HAL_Delay(300);
    put_claw_down_ground();
    HAL_Delay(800);
    close_claw();
    HAL_Delay(600);
    put_claw_up_top();
    arm_shrink();
    HAL_Delay(600); //300
    claw_spin_state_without_claw();
    // HAL_Delay(700); //TODO 直接撇进去，以下带？为新增的
    HAL_Delay(700); //? 
    put_claw_down_state(); //?
    HAL_Delay(300);  //?
    open_claw_bigger();
    HAL_Delay(200);
    // put_claw_up_top();
    // HAL_Delay(400); //?
    state_spin_without_claw_avoid_collide(position);
    HAL_Delay(400);
    claw_spin_front();
    open_claw_avoid_collide();
    HAL_Delay(600);
    // arm_stretch();
    open_claw_180();

}



/// @brief 省赛决赛版本，功能为从地上抓取物料（待新版本优化动作速度）
/// @param position 
/// @param state_position 
void get_and_load_openloop_with_temp_put(int position,int state_position)
{
    state_spin_without_claw(state_position);
    open_claw_bigger();
    switch(position)
    {
        case 1:
            feetech_servo_move(3,right_3_pileup,2000,feet_acc);
            feetech_servo_move(4,right_4,4095,feet_acc);
            r_servo_now = right_4;
            theta_servo_now = right_3_pileup;
            break;
        case 2:
            feetech_servo_move(3,middle_3_pileup,2000,feet_acc);
            feetech_servo_move(4,middle_4,4095,feet_acc);
            r_servo_now = middle_4;
            theta_servo_now = middle_3_pileup;
            break;
        case 3:
            feetech_servo_move(3,left_3_pileup,2000,feet_acc);
            feetech_servo_move(4,left_4,4095,feet_acc);
            r_servo_now = left_4;
            theta_servo_now = left_3_pileup;
            break;
    }
    HAL_Delay(500);
    put_claw_down_ground();
    HAL_Delay(800);
    close_claw();
    HAL_Delay(600);
    put_claw_up_top();
    arm_shrink();
    HAL_Delay(600); //300
    claw_spin_state_without_claw();
    // HAL_Delay(700); //TODO 直接撇进去，以下带？为新增的
    HAL_Delay(700); //? 
    put_claw_down_state(); //?
    HAL_Delay(300);  //?
    open_claw_bigger();
    HAL_Delay(200);
    // put_claw_up_top();
    // HAL_Delay(400); //?
    state_spin_without_claw_avoid_collide(position);
    HAL_Delay(400);
    claw_spin_front();
    open_claw_avoid_collide();
    HAL_Delay(600);
    // arm_stretch();
    open_claw();


}



/// @brief 无视觉辅助的从圆环上抓取物料
/// @param position 
/// @param is_default_position 1则为在默认的位置抓取，0则为根据先前记录的位置抓取，默认为 1
void get_and_load_openloop(int position,int is_default_position)
{
    state_spin_without_claw(position);
    open_claw_bigger();
    switch(position)
    {
        case 1:
            if( is_default_position == 1)
            {
                feetech_servo_move(3,right_3,2000,feet_acc);
                feetech_servo_move(4,right_4,4095,feet_acc);
                r_servo_now = right_4;
                theta_servo_now = right_3;
            }
            else
            {
                feetech_servo_move(3,theta_servo_value[1],2000,feet_acc);
                feetech_servo_move(4,r_servo_value[1],4095,feet_acc);
                theta_servo_now = theta_servo_value[1];
                r_servo_now = r_servo_value[1];
            }
            break;
        case 2:
            if( is_default_position == 1)
            {
                feetech_servo_move(3,middle_3,2000,feet_acc);
                feetech_servo_move(4,middle_4,4095,feet_acc);
                r_servo_now = middle_4;
                theta_servo_now = middle_3;
            }
            else
            {
                feetech_servo_move(3,theta_servo_value[2],2000,feet_acc);
                feetech_servo_move(4,r_servo_value[2],4095,feet_acc);
                r_servo_now = r_servo_value[2];
                theta_servo_now = theta_servo_value[2];
            }
            break;
        case 3:
            if(is_default_position == 1)
            {
                feetech_servo_move(3,left_3,2000,feet_acc);
                feetech_servo_move(4,left_4,4095,feet_acc);
                r_servo_now = left_4;
                theta_servo_now = left_3;
            }
            else
            {
                feetech_servo_move(3,theta_servo_value[3],2000,feet_acc);
                feetech_servo_move(4,r_servo_value[3],4095,feet_acc);
                r_servo_now = r_servo_value[3];
                theta_servo_now = r_servo_value[3];
            }
            break;
    }
    HAL_Delay(300);
    put_claw_down_ground();
    HAL_Delay(500);
    close_claw();
    HAL_Delay(300);
    put_claw_up_top();
    arm_shrink();
    HAL_Delay(300); //300
    claw_spin_state_without_claw();
    // HAL_Delay(700); //TODO 直接撇进去，以下带？为新增的
    HAL_Delay(500); //? 
    put_claw_down_state(); //?
    HAL_Delay(300);  //?
    open_claw();
    HAL_Delay(200);
    put_claw_up_top();
    HAL_Delay(300); //?
    claw_spin_front();


}


/// @brief 原省赛决赛版本，在转盘上放置(新优化了动作和速度)
/// @param position 
void get_and_pre_put_spin_plate_avoid_collide(int position, const material_order* order)
{
    state_spin_without_claw_avoid_collide(position);
    open_claw_avoid_collide();
    put_claw_up_top();
    // HAL_Delay(500); //TODO 可能会撞到物料
    int temp_r_servo_position_plate = r_servo_now;
    int temp_theta_servo_position_plate = theta_servo_now;
    arm_shrink(); //TODO 待区分
    HAL_Delay(500);
    claw_spin_state_without_claw();
    HAL_Delay(400);
    put_claw_down_state();
    state_spin_without_claw(position);
    HAL_Delay(300); //400
    close_claw();
    HAL_Delay(600);
    put_claw_up_top();
    HAL_Delay(500); //200
    claw_spin_front();
    feetech_servo_move(4,temp_r_servo_position_plate,4000,feet_acc);
    feetech_servo_move(3,temp_theta_servo_position_plate,4000,feet_acc);
    r_servo_now = temp_r_servo_position_plate;
    theta_servo_now = temp_theta_servo_position_plate;
    HAL_Delay(400);
    put_claw_down();
    HAL_Delay(500);
}

/// @brief 在转盘上放置
/// @param position 
void get_and_pre_put_spin_plate(int position)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    // HAL_Delay(500); //TODO 可能会撞到物料
    int temp_r_servo_position_plate = r_servo_now;
    int temp_theta_servo_position_plate = theta_servo_now;
    arm_shrink(); //TODO 待区分
    HAL_Delay(500);
    claw_spin_state_without_claw();
    // feetech_servo_move(3,middle_3,2000,feet_acc);    
    HAL_Delay(600);
    put_claw_down_state();
    HAL_Delay(600); //? 根据物料高度进行抓取
    close_claw();
    HAL_Delay(400);//? 根据物料高度进行调整
    put_claw_up_top();
    HAL_Delay(200); 
    claw_spin_front_slight(); //? 调整加速度
    feetech_servo_move(4,temp_r_servo_position_plate,4000,feet_acc);
    feetech_servo_move(3,temp_theta_servo_position_plate,4000,feet_acc);
    r_servo_now = temp_r_servo_position_plate;
    theta_servo_now = temp_theta_servo_position_plate;
    HAL_Delay(200);
    put_claw_down_slightly();
    HAL_Delay(400);
    open_claw();
    HAL_Delay(300);
}

/// @brief 在转盘上放置（新版本）
/// @param position 
void get_and_put_spin_plate(int position)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    // HAL_Delay(500); //TODO 可能会撞到物料
    int temp_r_servo_position_plate = r_servo_now;
    int temp_theta_servo_position_plate = theta_servo_now;
    arm_shrink(); //TODO 待区分
    HAL_Delay(500);
    claw_spin_state();
    HAL_Delay(400);
    put_claw_down_state();
    HAL_Delay(400); 
    close_claw();
    HAL_Delay(300);
    put_claw_up_top();
    HAL_Delay(300); 
    claw_spin_front(); //TODO 是否可能撞到
    feetech_servo_move(4,temp_r_servo_position_plate,4000,feet_acc);
    feetech_servo_move(3,temp_theta_servo_position_plate,4000,feet_acc);
    r_servo_now = temp_r_servo_position_plate;
    theta_servo_now = temp_theta_servo_position_plate;
    HAL_Delay(200);
    put_claw_down();
    HAL_Delay(400);
    open_claw_180();
    // if(is_pile_up != 1)
    // {
    // HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    // }
}


/// @brief 通用的函数，适用于从物料上面无法通过的情况
/// @param position 
/// @param near_ground_position 
/// @param state_position 
/// @param pile_up_position 
/// @param is_pile_up 
void get_and_pre_put_avoid(int position,int is_pile_up)
{
    state_spin_without_claw_avoid_collide(position);
    put_claw_down_state();
    HAL_Delay(600); //TODO 可能会撞到物料
    open_claw_avoid_collide();
    arm_shrink(); //TODO 待区分
    HAL_Delay(300);
    claw_spin_state();
    if(position == 1) 
    {
        if(is_pile_up == 1)
        {
            feetech_servo_move(3,right_3_pileup,2000,feet_acc);
            theta_servo_now = right_3_pileup;
        }
        else
        {
            feetech_servo_move(3,right_3,2000,feet_acc);
            theta_servo_now = right_3;
        }
    }
    else if(position == 2)
    {
        if(is_pile_up == 1)
        {
            feetech_servo_move(3,middle_3_pileup,2000,feet_acc);
            theta_servo_now = middle_3_pileup;
        }
        else
        {
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
    }
    else if(position == 3)
    {
        if(is_pile_up == 1)
        {
            feetech_servo_move(3,left_3_pileup,2000,feet_acc);
            theta_servo_now = left_3_pileup;
        }
        else
        {
            feetech_servo_move(3,left_3,2000,feet_acc);
            theta_servo_now = left_3;
        }
    }
    HAL_Delay(500);
    state_spin(position);
    HAL_Delay(200); //400
    close_claw();
    HAL_Delay(700);
    put_claw_up_top();
    // if(is_pile_up == 1)  // TODO 还可能需要加入update与等待时间
    // {
    //     HAL_Delay(1000);
    // }
    HAL_Delay(500); //! 不规则物料所用的识别
    claw_spin_front(); //TODO 是否可能撞到
    if(position == 1) 
    {
        
        feetech_servo_move(4,right_4,4000,feet_acc);
        r_servo_now = right_4;
    }
    else if(position == 2)
    {
        
        feetech_servo_move(4,middle_4,4000,feet_acc);
        r_servo_now = middle_4;
    }
    else if(position == 3)
    {
        
        feetech_servo_move(4,left_4,4000,feet_acc);
        r_servo_now = left_4;
    }
    HAL_Delay(400);
    if(is_pile_up == 1)
    {
        feetech_servo_move(1,put_claw_down_pile_position,4095,feet_acc);
        HAL_Delay(500);
    }
    else
    {
        feetech_servo_move(1,put_claw_down_near_ground_position,4095,feet_acc);
        HAL_Delay(900);
    }
}


/// @brief 不夹物料的放置
void get_and_pre_put_void(int position,int is_pile_up, const material_order* order)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    if(is_pile_up == 1)
    {
        HAL_Delay(600);
    }
    HAL_Delay(500); //TODO 可能会撞到物料 需要根据物料来调整
    if(position == order->right) 
    {
        feetech_servo_move(3,right_3,2000,feet_acc);
        theta_servo_now = right_3;
    }
    else if(position == order->middle)
    {
        if(is_pile_up == 1)
        {
            feetech_servo_move(3,middle_3_pileup,2000,feet_acc);    
            theta_servo_now = middle_3_pileup; 
        }
        else
        {
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
    }
    else if(position == order->left)
    {
        feetech_servo_move(3,left_3,2000,feet_acc);
        theta_servo_now = left_3;
        
    }
    if(position == order->right) 
    {
        feetech_servo_move(4,right_4,4000,feet_acc);
        r_servo_now = right_4;
    }
    else if(position == order->middle)
    {
        feetech_servo_move(4,middle_4,4000,feet_acc);
        r_servo_now = middle_4;
    }
    else if(position == order->left)
    {
        feetech_servo_move(4,left_4,4000,feet_acc);
        r_servo_now = left_4;
    }
    HAL_Delay(200);
    if(is_pile_up == 1)
    {
        HAL_Delay(800);
        put_claw_down_pile();
        HAL_Delay(500);
    }
    else
    {
        put_claw_down_near_ground();
        HAL_Delay(900);
    }
    if(is_pile_up != 1)
    {
    // HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    }
}


/// @brief 根据物料放置到大致的位置，然后开始闭环调整
void get_and_pre_put(int position,int is_pile_up, const material_order* order)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    // HAL_Delay(500); //TODO 可能会撞到物料
    arm_shrink(); //TODO 待区分
    HAL_Delay(500);
    claw_spin_state();
    if(is_pile_up == 1)
    {
        if(position == order->right) 
        {
            feetech_servo_move(3,right_3_pileup,2000,feet_acc);
            theta_servo_now = right_3_pileup;
        }
        else if(position == order->middle)
        {
            feetech_servo_move(3,middle_3_pileup,2000,feet_acc);    
            theta_servo_now = middle_3_pileup;
        }
        else if(position == order->left)
        {
            feetech_servo_move(3,left_3_pileup,2000,feet_acc);
            theta_servo_now = left_3_pileup;
        }
    }
    else
    {
        if(position == order->right) 
        {
            feetech_servo_move(3,right_3,2000,feet_acc);
            theta_servo_now = right_3;
        }
        else if(position == order->middle)
        {
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
        else if(position == order->left)
        {
            feetech_servo_move(3,left_3,2000,feet_acc);
            theta_servo_now = left_3;
        }
    }
    HAL_Delay(500);
    put_claw_down_state();
    HAL_Delay(150); //400
    close_claw();
    HAL_Delay(200);
    put_claw_up_top();
    HAL_Delay(200); //200
    claw_spin_front(); //TODO 是否可能撞到
    if(position == order->right) 
    {
        feetech_servo_move(4,right_4,4000,feet_acc);
        r_servo_now = right_4;
    }
    else if(position == order->middle)
    {
        feetech_servo_move(4,middle_4,4000,feet_acc);
        r_servo_now = middle_4;
    }
    else if(position == order->left)
    {
        feetech_servo_move(4,left_4,4000,feet_acc);
        r_servo_now = left_4;
    }
    HAL_Delay(200);
    if(is_pile_up == 1)
    {
        HAL_Delay(200);
        put_claw_down_pile();
        HAL_Delay(500);
    }
    else
    {
        put_claw_down_near_ground();
        HAL_Delay(500);
    }
    if(is_pile_up != 1)
    {
    // HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    }
}


/// @brief 根据物料放置到大致的位置，然后开始闭环调整
void get_and_pre_put_with_state_find_position(int position,int is_pile_up, const material_order* order)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    HAL_Delay(500); 
    // open_claw_avoid_collide();
    arm_shrink(); 
    // state_spin_without_claw_avoid_collide(target_colour[i]);
    HAL_Delay(600);
    claw_spin_state();
    if(is_pile_up == 1)
    {
        if(position == order->right) 
        {
            feetech_servo_move(3,right_3,2000,feet_acc);
            theta_servo_now = right_3;
        }
        else if(position == order->middle)
        {
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
        else if(position == order->left)
        {
            feetech_servo_move(3,left_3,2000,feet_acc);
            theta_servo_now = left_3;
        }
    }
    else{
        if(position == order->right) 
        {
            feetech_servo_move(3,right_3,2000,feet_acc);
            theta_servo_now = right_3;
        }
        else if(position == order->middle)
        {
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
        else if(position == order->left)
        {
            feetech_servo_move(3,left_3,2000,feet_acc);
            theta_servo_now = left_3;
        }
    }
    HAL_Delay(500);
    put_claw_down_state();
    HAL_Delay(500); //400
    close_claw();
    HAL_Delay(300);
    put_claw_up_top();
    HAL_UART_Transmit(&huart3, (uint8_t*)"update", strlen("update"), 50); //发给树莓派，开始校正
    HAL_Delay(800); //200
    claw_spin_front(); //TODO 是否可能撞到
    if(position == order->right) 
    {
        feetech_servo_move(4,right_4,4000,feet_acc);
        r_servo_now = right_4;
    }
    else if(position == order->middle)
    {
        feetech_servo_move(4,middle_4,4000,feet_acc);
        r_servo_now = middle_4;
    }
    else if(position == order->left)
    {
        feetech_servo_move(4,left_4,4000,feet_acc);
        r_servo_now = left_4;
    }
    HAL_Delay(200);
    if(is_pile_up == 1)
    {
        HAL_Delay(500);
        put_claw_down_pile();
        HAL_Delay(500);
    }
    else
    {
        put_claw_down_near_ground();
        HAL_Delay(900);
    }
    if(is_pile_up != 1)
    {
    // HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    }
}








void put_claw_down_pile(void)
{
    feetech_servo_move(1,put_claw_down_pile_position,4095,feet_acc);
}

/// @brief 精密舵机串口初始化
void my_servo_init()
{
    Uart_Init(115200);
}

void open_claw_avoid_collide(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, open_claw_avoid_collide_position);
}

void open_claw_180(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, open_claw_180_position);
}

/// @brief 夹爪打开
/// @param  
void open_claw(void)
{

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, open_claw_position);
}

void open_claw_bigger(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, open_claw_bigger_position);
}

void close_bit(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, close_bit_position);
}

/// @brief 夹爪关闭
/// @param  
void close_claw(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, close_claw_position);
}


/// @brief 机械臂运动到抓物料
/// @param  
void arm_stretch(void)
{
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arm_stretch_position);
    if( ReadPos(4) >1500)
    {
        feetech_servo_move(4,stretch_arm,4095,feet_acc);
        r_servo_now = stretch_arm;
    }
}


/// @brief 机械臂运动到放物料
/// @param  
void arm_shrink(void)
{
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arm_shrink_position);
    // if( ReadPos(4) >1500)
    // {
        feetech_servo_move(4,shrink_arm,4095,feet_acc);
        r_servo_now = shrink_arm;
    // }
}

/// @brief 忘了干什么的
/// @param  
void arm_shrink_all(void)
{
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arm_shrink_all_position);
    if(ReadPos(4) < shrink_arm_all && ReadPos(4) >1500)
    {
        feetech_servo_move(4,shrink_arm_all,4095,feet_acc);
        r_servo_now = shrink_arm_all;
    }
    
}

/// @brief 载物盘旋转到对应的位置（适用于机械臂伸长的放置）
/// @param state_position 
void state_spin(int state_position)
{
    if(state_position == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_position_1);
    }
    else if(state_position == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_position_2);
    }
    else if(state_position == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_position_3);
    }
}

/// @brief 载物盘旋转到对应的位置(适用于机械臂不动的放置)
void state_spin_without_claw(int state_position)
{
    if(state_position == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_without_claw_position_1);
    }
    else if(state_position == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_without_claw_position_2);
    }
    else if(state_position == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_without_claw_position_3);
    }
}

void state_spin_without_claw_avoid_collide(int state_position)
{
    if(state_position == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_without_claw_position_1+20);
    }
    else if(state_position == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_without_claw_position_2+20);
    }
    else if(state_position == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, state_spin_without_claw_position_2+20);
    }
}

void state_spin_angles(int angle)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, angle);
}


void put_claw_down_ground(void)
{
    feetech_servo_move(1,put_claw_down_ground_position,4095,feet_acc_claw_up_down);
}

void put_claw_down_ground_slightly(void)
{
    feetech_servo_move(1,put_claw_down_ground_position,4095,feet_acc_put_down_ground_slightly);
}

void put_claw_down_near_ground(void)
{
    feetech_servo_move(1,put_claw_down_near_ground_position,4095,feet_acc_claw_up_down);
}

void put_claw_down_near_plate(void)
{
    feetech_servo_move(1,put_claw_down_near_plate_position,4095,feet_acc_claw_up_down);
}

/// @brief 夹爪下降到车的载物盘高度
/// @param  
void put_claw_down_state(void)
{
    feetech_servo_move(1,put_claw_down_state_position,4095,feet_acc_claw_up_down);
}

/// @brief 绳驱转盘转动，夹爪下降,抓取转盘上的物品
/// @param  
void put_claw_down(void)
{
    feetech_servo_move(1,put_claw_down_position,4095,feet_acc_claw_up_down);
}

/// @brief 放到转盘上缓速版本
/// @param  
void put_claw_down_slightly(void)
{
    feetech_servo_move(1,put_claw_down_position,4095,feet_acc_put_down_ground_slightly);
}


/// @brief 绳驱转盘转动，夹爪上升
/// @param  
void put_claw_up(void)
{
    feetech_servo_move(1,put_claw_up_position,4095,feet_acc_claw_up_down);
}


void put_claw_up_top(void)
{
    feetech_servo_move(1,put_claw_up_top_position,4095,feet_acc_claw_up_down);
}

/// @brief 夹爪旋转到朝向前方
/// @param  
void claw_spin_front(void)
{
    if(ReadPos(2) > claw_spin_position_state - 800)
    {
        feetech_servo_move(2,claw_spin_position_front,4000,feet_acc_claw_spin);
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"claw_tooooo_back", strlen("claw_tooooo_back"), 50);
    }
}

/// @brief 夹爪缓慢旋转到朝向前方
/// @param  
void claw_spin_front_slight(void)
{
    if(ReadPos(2) > claw_spin_position_state - 800)
    {
        feetech_servo_move(2,claw_spin_position_front,4000,feet_acc_claw_spin_slight);
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"claw_tooooo_back", strlen("claw_tooooo_back"), 50);
    }
}

/// @brief 夹爪旋转到朝向载物台
/// @param
void claw_spin_state(void)
{
    int now_position = ReadPos(2);
    if(now_position > claw_spin_position_state)
    {
        feetech_servo_move(2,claw_spin_position_state,4000,feet_acc_claw_spin);
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"claw_tooooo_right", strlen("claw_tooooo_right"), 50);
    }
}

void claw_spin_state_without_claw(void)
{
    feetech_servo_move(2,claw_spin_without_claw_position_state,4000,feet_acc_claw_spin);
}

/// @brief 中板旋转,1为中间位置
/// @param position 
void whole_arm_spin(int status)
{
    if(status == 1)
    {
        feetech_servo_move(3,middle_arm,1000,feet_acc);
        theta_servo_now = middle_arm;
    }
    
}


/// @brief (待完善）精密舵机的移动
/// @param servo_ID 
/// @param Position 
/// @param Speed 
/// @param ACC 
void feetech_servo_move(uint8_t servo_ID,int16_t Position,uint16_t Speed,uint8_t ACC)
{
    WritePosEx(servo_ID,Position,Speed,ACC);
    // int position_abs = (Position- ReadPos(servo_ID))? (Position- ReadPos(servo_ID)): (ReadPos(servo_ID)-Position);

    // 此处的延时计算还有问题，先不使用
    // int time = (position_abs / Speed)*1000 + (Speed / (ACC*100))*1000;
    // HAL_Delay(time); //[(P1-P0)/V]*1000+[V/(A*100)]*1000

}


/// @brief 普通舵机转动
/// @param servo_ID 
/// @param angle 
void servo_move(int servo_ID, int angle)
{
    int position = (int)(50 + (200* (float)angle / 270.0));
    if(servo_ID == 1)
    {
        // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, position);
    }
    else if(servo_ID == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, position);
    }
    else if(servo_ID == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, position);
    }
    else if(servo_ID == 4)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, position);
    }
}

