#include "my_servo.h"


extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;
extern int theta_servo_value[],r_servo_value[];
/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

// 夹爪PD13  载物盘PD15 机械臂PD12

// 普通舵机参数范围25-125  270
int open_claw_avoid_collide_position = 108;
int open_claw_180_position = 104; //87 为正常抓放的位置
int open_claw_position = 86; //87 为正常抓放的位置
int open_claw_bigger_position = 92;
int close_claw_position = 74; //74
int close_bit_position = 78;
int arm_stretch_position = 39; // 弃用
int arm_shrink_position = 75;  // 弃用
int arm_shrink_all_position = 75; // 弃用
int state_spin_position_1 = 25;  //120度对应44.44
int state_spin_position_2 = 69;
int state_spin_position_3 = 111; 

int state_spin_without_claw_position_1 = 25;  
int state_spin_without_claw_position_2 = 69;
int state_spin_without_claw_position_3 = 111;


// 精密舵机参数范围0-4095
int feet_acc = 180; //180
int feet_acc_claw_up_down = 240;




//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! 精密舵机2000对应12.4cm， 0.0062cm/pulse
//? 以下为正常物料参数  
int put_claw_down_pile_position = 1878; 
int put_claw_down_state_position = 892 ; //从车的载物盘上 982
int put_claw_down_position = 1883;  // 从转盘上取物料  
int put_claw_down_ground_position = 3156; // 放在地上 3144
int put_claw_up_top_position =280; // 最高点  
int put_claw_up_position =1667; //  看粗调移动底盘的位置
int put_claw_down_near_ground_position = 3028; //!细调放置的位置
int put_claw_down_near_plate_position = 1590; //转盘放置细调的位置
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// int put_claw_down_pile_position = 1878; 
// int put_claw_down_state_position = 777 ; //从车的载物盘上 982
// int put_claw_down_position = 1701;  // 从转盘上取物料  
// int put_claw_down_ground_position = 3010; // 放在地上 3144
// int put_claw_up_top_position =280; // 最高点  
// int put_claw_up_position =1667; //  看粗调移动底盘的位置
// int put_claw_down_near_ground_position = 2770; //!细调放置的位置
// int put_claw_down_near_plate_position = 1600; //转盘放置细调的位置

//? 以下为通用参数
int claw_spin_position_front = 3329 ; // 2号精密舵机回到前方
int claw_spin_position_state = 1590; // 2号精密舵机回到载物盘//TODO 待测量
int claw_spin_without_claw_position_state = 1590; //与上面一样

// //? 异形三棱柱
// int put_claw_down_pile_position = 2012; 
// int put_claw_down_state_position = 973 ; //从车的载物盘上 982
// int put_claw_down_position = 1942;  // 从转盘上取物料  
// int put_claw_down_ground_position = 3250; // 放在地上 3144
// // int put_claw_down_ground_position = 3250; //! 夹取会滑的物料放在地上 
// int put_claw_up_top_position =290; // 最高点  
// int put_claw_up_position =1725; //  看粗调移动底盘的位置
// int put_claw_down_near_ground_position = 3073; //!细调放置的位置
// // int put_claw_down_near_ground_position = 2900; //夹取会滑的物料时的位置
// int put_claw_down_near_plate_position = 1837; //转盘放置细调的位置



//? 三棱柱
// int put_claw_down_pile_position = 1896; 
// int put_claw_down_state_position = 680 ; //从车的载物盘上 982
// int put_claw_down_position = 1800;  // 从转盘上取物料  
// // int put_claw_down_ground_position = 3144; // 放在地上 3144
// int put_claw_down_ground_position = 3000; //! 夹取会滑的物料放在地上 
// int put_claw_up_top_position =376; // 最高点  
// int put_claw_up_position =1725; //  看粗调移动底盘的位置
// // int put_claw_down_near_ground_position = 3003; //!细调放置的位置
// int put_claw_down_near_ground_position = 2800; //夹取会滑的物料时的位置
// int put_claw_down_near_plate_position = 1652; //转盘放置细调的位置


//? 圆台凸 加长高跟鞋爪子3.14
// int put_claw_down_pile_position = 2080; //
// int put_claw_down_state_position = 900 ; //从车的载物盘上  737
// int put_claw_down_position = 1964;  // 从转盘上取物料  1625
// int put_claw_down_ground_position = 3267; // 放在地上 2899
// int put_claw_up_top_position =536; // 最高点  360
// int put_claw_up_position =1845; //  看粗调移动底盘的位置
// int put_claw_down_near_ground_position = 3138; //细调放置的位置
// int put_claw_down_near_plate_position = 1772; //转盘放置细调的位置

//? 圆台凹 细细20cm 
// int put_claw_down_pile_position = 2040; //
// int put_claw_down_state_position = 840; //从车的载物盘上  737
// int put_claw_down_position = 1840;  // 从转盘上取物料  1625
// int put_claw_down_ground_position = 3129; // 放在地上 2899
// int put_claw_up_top_position =376; // 最高点  360
// int put_claw_up_position =1725; //  看粗调移动底盘的位置
// int put_claw_down_near_ground_position = 3024; //细调放置的位置
// int put_claw_down_near_plate_position = 1652; //转盘放置细调的位置

// //? 以下为圆台烟囱物料参数
// int put_claw_down_pile_position = 2250; //1819
// int put_claw_down_state_position = 1080 ; //从车的载物盘上  737
// int put_claw_down_position = 2100;  // 从转盘上取物料  1625
// int put_claw_down_ground_position = 3476; // 放在地上 2899
// int put_claw_up_top_position =536; // 最高点  360
// int put_claw_up_position =1845; //  看粗调移动底盘的位置
// int put_claw_down_near_ground_position = 3265; //细调放置的位置
// int put_claw_down_near_plate_position = 1772; //转盘放置细调的位置

// ? 以下为圆台凸物料参数
// int put_claw_down_pile_position = 2250; //1819
// int put_claw_down_state_position = 1080 ; //从车的载物盘上  737
// int put_claw_down_position = 2070;  // 从转盘上取物料  1625
// int put_claw_down_ground_position = 3476; // 放在地上 2899
// int put_claw_up_top_position =536; // 最高点  360
// int put_claw_up_position =1845; //  看粗调移动底盘的位置
// int put_claw_down_near_ground_position = 3265; //细调放置的位置
// int put_claw_down_near_plate_position = 1772; //转盘放置细调的位置

//? 物料9第三版爪子
// int put_claw_down_pile_position = 2250; //1819
// int put_claw_down_state_position = 1150 ; //从车的载物盘上  737
// int put_claw_down_position = 2155;  // 从转盘上取物料  1625
// int put_claw_down_ground_position = 3470; // 放在地上 2899
// int put_claw_up_top_position =536; // 最高点  360
// int put_claw_up_position =1845; //  看粗调移动底盘的位置
// int put_claw_down_near_ground_position = 3265; //细调放置的位置
// int put_claw_down_near_plate_position = 1772; //转盘放置细调的位置

// 机械臂位置限制 
// 2280左 3400右
// 2000后 4050前
//TODO 待测量
int theta_left_position_limit = 1800;
int theta_right_position_limit = 3800;
int theta_right_position_rlimit = 3300;
int theta_left_position_rlimit = 2440;
int r_front_position_limit = 4090;
int r_back_position_limit = 1950;
int r_back_position_rlimit = 3200; // 当theta超过rlimit，r的限制值不能小于这个值


int shrink_arm = 3800; 
int stretch_arm_longgest = 2704;
int stretch_camera = 2704;
int shrink_arm_all = 2704;

int init_plate = 2632;// 初始抓转盘


int left_2 = 3929; 
int left_3 = 2070;  
int left_3_pileup = 2050;
int left_4 =  3800; 

int right_2 = 3929; 
int right_3 = 3780;  
int right_3_pileup = 3760;
int right_4 =  3810; 

int middle_2 = 3929;
int middle_3 = 2940;
int middle_3_pileup = 2905;
int middle_4 =  2704;  //TODO 务必和shrink一样

int right_arm = 2935;
int left_arm = 2935;
int middle_arm = 2895;  //2865 用了很久
volatile int x_plate_error = 0, y_plate_error = 0; // 载物盘中心和色环中心的偏差
volatile int x_camera_error = 0, y_camera_error = 0; // 物料中心和色环中心的偏差
volatile int x_camera_error_plate = 0, y_camera_error_plate = 0; // 载物盘带有色环时，中心和色环中心的偏差
volatile int  r_servo_now =  2704; // 机械臂伸缩舵机的位置
volatile int  theta_servo_now = 2823; // 机械臂中板旋转舵机的位置



volatile int x_error_last = 0, y_error_last = 0; // 上一次的偏差
volatile int x_error_long_last = 0, y_error_long_last = 0; // 上上次的偏差


// 机械臂位置阈值（废弃）
int r_big_limit = 45;
int r_mid_limit = 10;
int r_small_limit = 2;
int theta_big_limit = 45;
int theta_mid_limit = 10;
int theta_small_limit = 2;

int r_pulse_servo_big = 45; // 精密舵机伸缩位置步进（大）
int r_pulse_servo_mid = 18; // 精密舵机伸缩位置步进（中）
int r_pulse_servo_small = 10; // 精密舵机伸缩位置步进（小）

int theta_pulse_servo_big = 35; // 精密舵机旋转位置步进（大）
int theta_pulse_servo_mid = 15; // 精密舵机旋转位置步进（中）   
int theta_pulse_servo_small = 5; // 精密舵机旋转位置步进（小）

int previous_theta_error = 0,pre_previous_theta_error = 0;
int previous_r_error = 0,pre_previous_r_error = 0;

char temp_function_char[50];
long UART_count = 0;

/// @brief 单次的调整
/// @param x_plate_error_in 
/// @param y_plate_error_in 
void adjust_plate(int x_plate_error_in,int y_plate_error_in)
{
            if(r_servo_now+y_plate_error_in < r_front_position_limit && r_servo_now+y_plate_error_in > r_back_position_limit)
            {
                r_servo_now += y_plate_error_in;
                feetech_servo_move(4,r_servo_now,4000,feet_acc);
                y_plate_error_in = 0;
            }
            else if (r_servo_now+y_plate_error_in > r_front_position_limit)
            {
                r_servo_now = r_front_position_limit;
                feetech_servo_move(4,r_servo_now,4000,feet_acc);
                y_plate_error_in = 0;
            }
            else if (r_servo_now+y_plate_error_in < r_back_position_limit)
            {
                r_servo_now = r_back_position_limit;
                feetech_servo_move(4,r_servo_now,4000,feet_acc);
                y_plate_error_in = 0;
            }
            if(theta_servo_now+x_plate_error_in < theta_right_position_limit && theta_servo_now+x_plate_error_in > theta_left_position_limit)
            {
                theta_servo_now += x_plate_error_in;
                feetech_servo_move(3,theta_servo_now,4000,feet_acc);
                x_plate_error_in = 0;
            }
            else if (theta_servo_now+x_plate_error_in > theta_right_position_limit)
            {
                theta_servo_now = theta_right_position_limit;
                feetech_servo_move(3,theta_servo_now,4000,feet_acc);
                x_plate_error_in = 0;
            }
            else if (theta_servo_now+x_plate_error_in < theta_left_position_limit)
            {
                theta_servo_now = theta_left_position_limit;
                feetech_servo_move(3,theta_servo_now,4000,feet_acc);
                x_plate_error_in = 0;
            }
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
    //TODO 如果看不到，则xy传进来设置一个特殊值，然后开始转动
    float r_adjust_values = 0, theta_adjust_values = 0;
    int is_theta_ok = 0, is_r_ok = 0;
    int x_origin = x_error;
    int y_origin = y_error;
    int max_servo_movement = 100;
    if(x_error == 0 && y_error == 0)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"  ALL zero  ", strlen("  ALL zero  "), 50);
        return 0;
    }
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
    x_camera_error = 0;
    y_camera_error = 0;
    

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
    if((theta_servo_now + theta_adjust_values < theta_right_position_limit) && (theta_servo_now + theta_adjust_values > theta_left_position_limit) && (r_servo_now + r_adjust_values < r_front_position_limit) && (r_servo_now + r_adjust_values > r_back_position_limit))
    {
        feetech_servo_move(4,r_servo_now + r_adjust_values,4000,100);
        r_servo_now += r_adjust_values;
        HAL_Delay(10);
        feetech_servo_move(3,theta_servo_now + theta_adjust_values,4000,100);
        theta_servo_now += theta_adjust_values;
    }
    else
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"limit", strlen("limit"), 50);
    }

    if(is_r_ok == 1 && is_theta_ok == 1)
    {
        return 1;
    }

    else
    {
        return 0;
    }
}

void get_and_load_ground(int position)
{
    // 伸长并打开夹爪
    if(position != 0)
    {
        state_spin(position);
    }
    else
    {
        state_spin(3);
    }

    // 放下夹爪
    put_claw_down_ground();
    HAL_Delay(1200);

    // 抓取
    close_claw();
    HAL_Delay(500);
    arm_shrink();

    // 拉起夹爪
    put_claw_up_top();
    HAL_Delay(800);
    claw_spin_state();
    HAL_Delay(1200);

    // 放下
    open_claw();
    HAL_Delay(800);

    put_claw_up_top();
    HAL_Delay(300);
    claw_spin_front();
    HAL_Delay(1000);
    // arm_shrink_all();
}

/// @brief 抓取并放置在载物盘上,开始时夹爪应提前到位，结束时夹爪在最高位，朝前
/// @param  
void get_and_load(int position)
{
    // 伸长并打开夹爪
    state_spin(position);
    open_claw_180();
    // arm_shrink(); 

    // 放下夹爪
    put_claw_down();
    HAL_Delay(1000);

    // 抓取
    close_claw();
    HAL_Delay(500);
    arm_shrink();

    // 拉起夹爪
    put_claw_up_top();
    HAL_Delay(700);
    claw_spin_state();
    HAL_Delay(1100);

    // 放下
    open_claw();
    HAL_Delay(300);

    put_claw_up_top();
    // HAL_Delay(300);
    claw_spin_front();
    arm_stretch();
    HAL_Delay(500);
}

/// @brief 从载物盘上取物料,结束时为夹爪非最高位，朝前
/// @param position 载物盘编号1-3
void get_from_state(int position)
{
    state_spin(position);
    whole_arm_spin(1);
    arm_shrink(); // 机械臂收回至载物盘上
    open_claw();
    put_claw_up_top();
    HAL_Delay(1000);
    claw_spin_state();
    HAL_Delay(1000);
    put_claw_down_state();
    HAL_Delay(800);
    close_claw();
    put_claw_up_top();
    HAL_Delay(500);
    claw_spin_front();
    arm_stretch();
}

/// @brief 将物料放在正前方地上，和get_from_state前后联合使用
/// @param  
void put_from_state(void)
{
    put_claw_down_ground();
    HAL_Delay(2000);
    open_claw();
    HAL_Delay(1000);
    put_claw_up();
    HAL_Delay(800); 
    //!
    // arm_shrink_all();  
}

void put_from_state_pileup(void)
{
    put_claw_down_pile();
    HAL_Delay(2000);
    open_claw();
    HAL_Delay(1000);
    put_claw_up();
    HAL_Delay(800); 
    //!
    // arm_shrink_all();  
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

/// @brief 抓国赛的圆圆头物料
/// @param position 
/// @param near_ground_position 
/// @param state_position_tall 
/// @param state_position_short 
void get_and_load_openloop_v3(int position,int near_ground_position,int state_position_tall,int state_position_short)
{
    state_spin_without_claw(position);
    switch(position)
    {
        case 1:
            feetech_servo_move(3,right_3,2000,feet_acc);
            feetech_servo_move(4,right_4,4095,feet_acc);
            r_servo_now = right_4;
            theta_servo_now = right_3;
            break;
        case 2:
            feetech_servo_move(3,middle_3,2000,feet_acc);
            feetech_servo_move(4,middle_4,4095,feet_acc);
            r_servo_now = middle_4;
            theta_servo_now = middle_3;
            break;
        case 3:
            feetech_servo_move(3,left_3,2000,feet_acc);
            feetech_servo_move(4,left_4,4095,feet_acc);
            r_servo_now = left_4;
            theta_servo_now = left_3;
            break;
    }
    HAL_Delay(300);
    feetech_servo_move(1,near_ground_position,4095,feet_acc_claw_up_down); //放到地上
    HAL_Delay(650);
    close_claw();
    HAL_Delay(400);
    put_claw_up_top();
    arm_shrink();
    HAL_Delay(600); //300
    claw_spin_state_without_claw();
    feetech_servo_move(1,state_position_short,4095,feet_acc_claw_up_down);
    HAL_Delay(700);
    // open_claw_180();
    close_bit();
    feetech_servo_move(1,state_position_tall,4095,feet_acc_claw_up_down);
    HAL_Delay(800);
    open_claw_180();
    state_spin_without_claw_avoid_collide(position);
    HAL_Delay(500);
    put_claw_up_top();
    HAL_Delay(500);
    claw_spin_front();
}

/// @brief 抓经典物料的底部
/// @param position 
/// @param ground_position 
/// @param state_position 
void get_and_load_openloop_v2(int position,int ground_position,int state_position)
{
    state_spin_without_claw(position);
    switch(position)
    {
        case 1:
            feetech_servo_move(3,right_3,2000,feet_acc);
            feetech_servo_move(4,right_4,4095,feet_acc);
            r_servo_now = right_4;
            theta_servo_now = right_3;
            break;
        case 2:
            feetech_servo_move(3,middle_3,2000,feet_acc);
            feetech_servo_move(4,middle_4,4095,feet_acc);
            r_servo_now = middle_4;
            theta_servo_now = middle_3;
            break;
        case 3:
            feetech_servo_move(3,left_3,2000,feet_acc);
            feetech_servo_move(4,left_4,4095,feet_acc);
            r_servo_now = left_4;
            theta_servo_now = left_3;
            break;
    }
    HAL_Delay(300);
    feetech_servo_move(1,ground_position,4095,feet_acc_claw_up_down); //放到地上
    HAL_Delay(650);
    close_claw();
    HAL_Delay(400);
    feetech_servo_move(1,state_position,4095,feet_acc_claw_up_down);
    arm_shrink();
    HAL_Delay(600); //300
    claw_spin_state_without_claw();
    HAL_Delay(700);
    // open_claw_180();
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 108);
    HAL_Delay(500);
    state_spin_without_claw_avoid_collide(position);
    put_claw_up_top();
    claw_spin_front();
}

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
    HAL_Delay(800);
    close_claw();
    HAL_Delay(600);
    put_claw_up_top();
    arm_shrink();
    HAL_Delay(300); //300
    claw_spin_state_without_claw();
    // HAL_Delay(700); //TODO 直接撇进去，以下带？为新增的
    HAL_Delay(600); //? 
    put_claw_down_state(); //?
    HAL_Delay(300);  //?
    open_claw();
    HAL_Delay(300);
    put_claw_up_top();
    HAL_Delay(400); //?
    claw_spin_front();


}

/// @brief 废弃
/// @param position 
void get_and_load_different_position(int position)
{
    state_spin(position);
    whole_arm_spin(1);
    arm_shrink(); // 机械臂收回至载物盘上
    open_claw();
    put_claw_up_top();
    HAL_Delay(1000);
    if(position == 1) //红
    {
        feetech_servo_move(2,right_2,4095,feet_acc);
        feetech_servo_move(3,right_3,1000,feet_acc);
        feetech_servo_move(4,right_4,4095,feet_acc);
    }
    else if(position == 2)
    {
        feetech_servo_move(4,middle_4,4095,feet_acc); //! magic number
        // HAL_Delay(1000);
        feetech_servo_move(2,middle_2,1000,feet_acc);
        feetech_servo_move(3,middle_3,2000,feet_acc);
        
    }
    else if(position == 3)
    {
        feetech_servo_move(2,left_2,4095,feet_acc);
        feetech_servo_move(3,left_3,1000,feet_acc);
        feetech_servo_move(4,left_4,4095,feet_acc);
    }
    put_claw_down_ground();
    HAL_Delay(1200);
    
    // arm_shrink();
    close_claw();
    HAL_Delay(500);
    arm_shrink();
    put_claw_up_top();
    HAL_Delay(800);
    claw_spin_state();
    HAL_Delay(900);
    open_claw();
    HAL_Delay(500);
    put_claw_up_top();
    claw_spin_front();
    HAL_Delay(500);
}

void get_and_pre_put_spin_plate_avoid_collide(int position)
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
    HAL_Delay(600);
    put_claw_down_state();
    state_spin_without_claw(position);
    HAL_Delay(700); //400
    close_claw();
    HAL_Delay(600);
    put_claw_up_top();
    HAL_Delay(500); //200
    feetech_servo_move(2,claw_spin_position_front,3500,feet_acc);
    feetech_servo_move(4,temp_r_servo_position_plate,4000,feet_acc);
    feetech_servo_move(3,temp_theta_servo_position_plate,4000,feet_acc);
    r_servo_now = temp_r_servo_position_plate;
    theta_servo_now = temp_theta_servo_position_plate;
    HAL_Delay(400);
    put_claw_down();
    HAL_Delay(300);
}

/// @brief 
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
    claw_spin_state();
    // feetech_servo_move(3,middle_3,2000,feet_acc);    
    HAL_Delay(700);
    put_claw_down_state();
    HAL_Delay(700); //400
    close_claw();
    HAL_Delay(400);
    put_claw_up_top();
    HAL_Delay(500); //200
    claw_spin_front(); //TODO 是否可能撞到
    feetech_servo_move(4,temp_r_servo_position_plate,4000,feet_acc);
    feetech_servo_move(3,temp_theta_servo_position_plate,4000,feet_acc);
    r_servo_now = temp_r_servo_position_plate;
    theta_servo_now = temp_theta_servo_position_plate;
    HAL_Delay(200);
    put_claw_down_near_plate();
    HAL_Delay(300);
    // if(is_pile_up != 1)
    // {
    // HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    // }
}

/// @brief 用于国赛圆圆头
/// @param position 
/// @param near_ground_position 
/// @param state_position_tall 
/// @param state_position_short 
/// @param pile_up_position 
/// @param is_pile_up 
void get_and_pre_put_v3(int position,int near_ground_position,int state_position_tall,int state_position_short, int pile_up_position,int is_pile_up)
{
    state_spin_without_claw_avoid_collide(position);
    open_claw_180();
    put_claw_up_top();
    // HAL_Delay(500); //TODO 可能会撞到物料
    arm_shrink(); //TODO 待区分
    HAL_Delay(300);
    claw_spin_state();
    if(is_pile_up == 1)
    {
        if(position == 1) 
        {
            feetech_servo_move(3,right_3_pileup,2000,feet_acc);
            theta_servo_now = right_3;
        }
        else if(position == 2)
        {
            feetech_servo_move(3,middle_3_pileup,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
        else if(position == 3)
        {
            feetech_servo_move(3,left_3_pileup,2000,feet_acc);
            theta_servo_now = left_3;
        }
    }
    else
    {
        if(position == 1) 
        {
            feetech_servo_move(3,right_3,2000,feet_acc);
            theta_servo_now = right_3;
        }
        else if(position == 2)
        {
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
        else if(position == 3)
        {
            feetech_servo_move(3,left_3,2000,feet_acc);
            theta_servo_now = left_3;
        }
    }
    HAL_Delay(700);
    state_spin(position);
    feetech_servo_move(1,state_position_tall,4095,feet_acc_claw_up_down); //根据物料的高度来调整下到转盘的高度
    HAL_Delay(700); //400
    close_bit();
    HAL_Delay(500);
    feetech_servo_move(1,state_position_short,4095,feet_acc_claw_up_down); 
    HAL_Delay(500);
    close_claw();
    HAL_Delay(500);
    put_claw_up_top();
    HAL_Delay(500); //200
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
    HAL_Delay(200);
    if(is_pile_up == 1)
    {
        feetech_servo_move(1,pile_up_position,4095,feet_acc);
        HAL_Delay(500);
    }
    else
    {
        feetech_servo_move(1,near_ground_position,4095,feet_acc);
        HAL_Delay(900);
    }
    if(is_pile_up != 1)
    {
    HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    }
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
            feetech_servo_move(3,right_3_pileup,2000,feet_acc);
            theta_servo_now = right_3_pileup;
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
            feetech_servo_move(3,middle_3_pileup,2000,feet_acc);
            theta_servo_now = middle_3_pileup;
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
            feetech_servo_move(3,left_3_pileup,2000,feet_acc);
            theta_servo_now = left_3_pileup;
        }
    }
    HAL_Delay(500);
    state_spin(position);
    HAL_Delay(400); //400
    close_claw();
    HAL_Delay(700);
    put_claw_up_top();
    // if(is_pile_up == 1)  // TODO 还可能需要加入update与等待时间
    // {
    //     HAL_Delay(1000);
    // }
    HAL_Delay(500); //! 不规则物料所用的识别
    // claw_spin_front(); //TODO 是否可能撞到
    feetech_servo_move(2,claw_spin_position_front,2000,feet_acc);
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
    HAL_Delay(1000);
    if(is_pile_up == 1)
    {
        feetech_servo_move(1,put_claw_down_pile_position,4095,feet_acc);
        HAL_Delay(500);
    }
    else
    {
        feetech_servo_move(1,put_claw_down_near_ground_position,4095,feet_acc);
        HAL_Delay(500);
    }
}

void get_and_pre_put_avoid_triangular(int position,int is_pile_up)
{
    state_spin_without_claw_avoid_collide(position);
    open_claw_avoid_collide();
    put_claw_up_top();
    HAL_Delay(500); //TODO 可能会撞到物料
    arm_shrink(); //TODO 待区分
    HAL_Delay(300);
    claw_spin_state();
    if(position == 1) 
    {
        feetech_servo_move(3,right_3,2000,feet_acc);
        theta_servo_now = right_3;
    }
    else if(position == 2)
    {
        feetech_servo_move(3,middle_3,2000,feet_acc);    
        theta_servo_now = middle_3; 
    }
    else if(position == 3)
    {
        feetech_servo_move(3,left_3,2000,feet_acc);
        theta_servo_now = left_3;
        
    }
    HAL_Delay(600);
    state_spin(position);
    put_claw_down_state();
    HAL_Delay(700); //400
    close_claw();
    HAL_Delay(700);
    put_claw_up_top();
    HAL_UART_Transmit(&huart3, (uint8_t*)"update", strlen("update"), 50); //! 不规则物料所用的识别
    HAL_Delay(800); //! 不规则物料所用的识别
    // HAL_Delay(500); 
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
    if(is_pile_up != 1)
    {
    // HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    }
}

/// @brief 抓经典物料的底部
/// @param position 
/// @param near_ground_position 
/// @param state_position 
/// @param pile_up_position 
/// @param is_pile_up 
void get_and_pre_put_v2(int position,int near_ground_position,int state_position,int pile_up_position,int is_pile_up)
{
    state_spin_without_claw_avoid_collide(position);
    open_claw_180();
    put_claw_up_top();
    // HAL_Delay(500); //TODO 可能会撞到物料
    arm_shrink(); //TODO 待区分
    HAL_Delay(300);
    claw_spin_state();
    if(position == 1) 
    {
        feetech_servo_move(3,right_3,2000,feet_acc);
        theta_servo_now = right_3;
    }
    else if(position == 2)
    {
        feetech_servo_move(3,middle_3,2000,feet_acc);    
        theta_servo_now = middle_3; 
    }
    else if(position == 3)
    {
        feetech_servo_move(3,left_3,2000,feet_acc);
        theta_servo_now = left_3;
        
    }
    HAL_Delay(700);
    state_spin(position);
    feetech_servo_move(1,state_position,4095,feet_acc_claw_up_down); //根据物料的高度来调整下到转盘的高度
    HAL_Delay(700); //400
    close_claw();
    HAL_Delay(500);
    put_claw_up_top();
    HAL_Delay(500); //200
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
    HAL_Delay(200);
    if(is_pile_up == 1)
    {
        feetech_servo_move(1,pile_up_position,4095,feet_acc);
        HAL_Delay(500);
    }
    else
    {
        feetech_servo_move(1,near_ground_position,4095,feet_acc);
        HAL_Delay(900);
    }
    if(is_pile_up != 1)
    {
    HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    }
}




/// @brief 从地上抓物料
/// @param position 
void pre_put_to_get_ground_material(int position)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    HAL_Delay(500); //TODO 可能会撞到物料
    // HAL_Delay(500); //TODO 可能会撞到物料
    // arm_shrink(); //TODO 待区分
    // HAL_Delay(300);
    // claw_spin_state();
    if(position == 1) 
    {
        feetech_servo_move(3,right_3,2000,feet_acc);
        theta_servo_now = right_3;
    }
    else if(position == 2)
    {
        
        
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        
    }
    else if(position == 3)
    {
        feetech_servo_move(3,left_3,2000,feet_acc);
        theta_servo_now = left_3;
        
    }
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
    HAL_Delay(200);
    
    
    put_claw_down_ground();
    HAL_Delay(900);
    
    
}

/// @brief 从地上抓取物料，动作由close开始
/// @param  
void get_material_from_ground(int state_position)
{
    close_claw();
    HAL_Delay(500);
    put_claw_up_top();
    arm_shrink();
    state_spin_without_claw(state_position);
    HAL_Delay(500);
    claw_spin_state_without_claw();
    HAL_Delay(600);
    put_claw_down_state();
    HAL_Delay(600);
    open_claw_bigger();
    HAL_Delay(300);
    put_claw_up_top();
    HAL_Delay(500);
    claw_spin_front();
    HAL_Delay(700);
}

void get_material_from_ground_void(int state_position)
{
    close_claw();
    HAL_Delay(500);
    put_claw_up_top();
    arm_shrink();
    state_spin_without_claw(state_position);
    HAL_Delay(700);
    claw_spin_state_without_claw();
    HAL_Delay(600);
    put_claw_down_state();
    HAL_Delay(600);
    open_claw_bigger();
    HAL_Delay(300);
    put_claw_up_top();
    HAL_Delay(400);
    state_spin_without_claw_avoid_collide(state_position);
    HAL_Delay(800);
    open_claw_avoid_collide();
    claw_spin_front();
    HAL_Delay(700);
}



/// @brief 不夹物料的放置
void get_and_pre_put_void(int position,int is_pile_up)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    if(is_pile_up == 1)
    {
        HAL_Delay(600);
    }
    HAL_Delay(800); //TODO 可能会撞到物料 需要根据物料来调整
    if(position == 1) 
    {
        feetech_servo_move(3,right_3,2000,feet_acc);
        theta_servo_now = right_3;
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
        feetech_servo_move(3,left_3,2000,feet_acc);
        theta_servo_now = left_3;
        
    }
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
void get_and_pre_put(int position,int is_pile_up)
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
        if(position == 1) 
        {
            feetech_servo_move(3,right_3_pileup,2000,feet_acc);
            theta_servo_now = right_3_pileup;
        }
        else if(position == 2)
        {
            feetech_servo_move(3,middle_3_pileup,2000,feet_acc);    
            theta_servo_now = middle_3_pileup;
        }
        else if(position == 3)
        {
            feetech_servo_move(3,left_3_pileup,2000,feet_acc);
            theta_servo_now = left_3_pileup;
        }
    }
    else
    {
        if(position == 1) 
        {
            feetech_servo_move(3,right_3,2000,feet_acc);
            theta_servo_now = right_3;
        }
        else if(position == 2)
        {
            feetech_servo_move(3,middle_3,2000,feet_acc);    
            theta_servo_now = middle_3; 
        }
        else if(position == 3)
        {
            feetech_servo_move(3,left_3,2000,feet_acc);
            theta_servo_now = left_3;
        }
    }
    HAL_Delay(700);
    put_claw_down_state();
    HAL_Delay(300); //400
    close_claw();
    HAL_Delay(200);
    put_claw_up_top();
    HAL_Delay(400); //200
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
    HAL_Delay(200);
    if(is_pile_up == 1)
    {
        HAL_Delay(600);
        put_claw_down_pile();
        HAL_Delay(500);
    }
    else
    {
        put_claw_down_near_ground();
        HAL_Delay(1100);
    }
    if(is_pile_up != 1)
    {
    // HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
    }
}


/// @brief 根据物料放置到大致的位置，然后开始闭环调整
void get_and_pre_put_with_state_find_position(int position,int is_pile_up)
{
    state_spin_without_claw_avoid_collide(position);
    open_claw_avoid_collide();
    put_claw_down_state();
    HAL_Delay(500); 
    // open_claw_avoid_collide();
    arm_shrink(); 
    // state_spin_without_claw_avoid_collide(target_colour[i]);
    HAL_Delay(600);
    claw_spin_state();
    if(is_pile_up == 1)
    {
        if(position == 1) 
    {
        feetech_servo_move(3,right_3,2000,feet_acc);
        theta_servo_now = right_3;
    }
    else if(position == 2)
    {
        feetech_servo_move(3,middle_3,2000,feet_acc);    
        theta_servo_now = middle_3; 
    }
    else if(position == 3)
    {
        feetech_servo_move(3,left_3,2000,feet_acc);
        theta_servo_now = left_3;
        
    }

    }
    else{
    if(position == 1) 
    {
        feetech_servo_move(3,right_3_pileup,2000,feet_acc);
        theta_servo_now = right_3;
    }
    else if(position == 2)
    {
        feetech_servo_move(3,middle_3_pileup,2000,feet_acc);    
        theta_servo_now = middle_3; 
    }
    else if(position == 3)
    {
        feetech_servo_move(3,left_3_pileup,2000,feet_acc);
        theta_servo_now = left_3;
    }
    }
    HAL_Delay(500);
    state_spin(position);
    HAL_Delay(500); //400
    close_claw();
    HAL_Delay(600);
    put_claw_up_top();
    HAL_UART_Transmit(&huart3, (uint8_t*)"update", strlen("update"), 50); //发给树莓派，开始校正

    HAL_Delay(800); //200
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




/// @brief （废弃）根据物料放置到大致的位置，然后开始闭环调整
void get_and_pre_put_different_color(int position,int is_pile_up)
{
    state_spin(position);
    open_claw();
    put_claw_up_top();
    // HAL_Delay(500); //TODO 可能会撞到物料
    arm_shrink(); //TODO 待区分
    HAL_Delay(300);
    claw_spin_state();
    if(position == 2) 
    {
        feetech_servo_move(3,right_3,2000,feet_acc);
        theta_servo_now = right_3;
    }
    else if(position == 1)
    {
        feetech_servo_move(3,middle_3,2000,feet_acc);    
        theta_servo_now = middle_3; 
    }
    else if(position == 3)
    {
        feetech_servo_move(3,left_3,2000,feet_acc);
        theta_servo_now = left_3;
        
    }
    HAL_Delay(700);
    put_claw_down_state();
    HAL_Delay(700); //400
    close_claw();
    HAL_Delay(200);
    put_claw_up_top();
    HAL_Delay(500); //200
    claw_spin_front(); //TODO 是否可能撞到
    if(position == 2) 
    {
        
        feetech_servo_move(4,right_4,4000,feet_acc);
        r_servo_now = right_4;
    }
    else if(position == 1)
    {
        
        feetech_servo_move(4,middle_4,4000,feet_acc);
        r_servo_now = middle_4;
    }
    else if(position == 3)
    {
        
        feetech_servo_move(4,left_4,4000,feet_acc);
        r_servo_now = left_4;
    }
    HAL_Delay(200);
    if(is_pile_up == 1)
    {
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

/// @brief （废弃）根据物料编号在同一个位置放置三个物料
/// @param position 
void get_and_put_different_position(int position)
{
    state_spin(position);
    whole_arm_spin(1);
    arm_shrink(); // 机械臂收回至载物盘上
    open_claw();
    put_claw_up_top();
    HAL_Delay(800);
    claw_spin_state();
    HAL_Delay(700);    
    put_claw_down_state();
    HAL_Delay(700);

    close_claw();
    put_claw_up_top();
    HAL_Delay(500);
    if(position == 1) //红
    {
        feetech_servo_move(2,right_2,4095,feet_acc);
        feetech_servo_move(3,right_3,2000,feet_acc);
        feetech_servo_move(4,right_4,4095,feet_acc);
    }
    else if(position == 2)
    {
        feetech_servo_move(2,middle_2,4095,feet_acc);
        feetech_servo_move(3,middle_3,2000,feet_acc);
        feetech_servo_move(4,middle_4,4095,feet_acc);
    }
    else if(position == 3)
    {
        feetech_servo_move(2,left_2,4095,feet_acc);
        feetech_servo_move(3,left_3,2000,feet_acc);
        feetech_servo_move(4,left_4,4095,feet_acc);
    }
    put_claw_down_ground();    
    HAL_Delay(1500);
    // HAL_Delay(1500); //! 单独测试使用
    open_claw();
    HAL_Delay(500);
    put_claw_up_top();
    //// HAL_Delay(500);
    //// if(position == 1 || position == 3)
    //// {
    ////     claw_spin_front();
    //// }
    // arm_shrink();
    HAL_Delay(500);
}

/// @brief 废弃
/// @param position 
void get_and_put_different_position_pileup(int position)
{
    state_spin(position);
    whole_arm_spin(1);
    arm_shrink(); // 机械臂收回至载物盘上
    open_claw();
    put_claw_up_top();
    HAL_Delay(800);
    claw_spin_state();
    HAL_Delay(700);    
    put_claw_down_state();
    HAL_Delay(700);

    close_claw();
    put_claw_up_top();
    HAL_Delay(600);
    if(position == 1) //红
    {
        feetech_servo_move(2,right_2,4095,feet_acc);
        feetech_servo_move(3,right_3,1000,feet_acc);
        feetech_servo_move(4,right_4,4095,feet_acc);
    }
    else if(position == 2)
    {
        feetech_servo_move(2,middle_2,4095,feet_acc);
        feetech_servo_move(3,middle_3,1000,feet_acc);
        feetech_servo_move(4,middle_4,4095,feet_acc);
    }
    else if(position == 3)
    {
        feetech_servo_move(2,left_2,4095,feet_acc);
        feetech_servo_move(3,left_3,1000,feet_acc);
        feetech_servo_move(4,left_4,4095,feet_acc);
    }
    HAL_Delay(500);
    put_claw_down_pile();    
    HAL_Delay(1200);
    open_claw();
    HAL_Delay(500);
    put_claw_up_top();
    HAL_Delay(400);
    // if(position == 1 || position == 3)
    // {
    //     claw_spin_front();
    // }
    // arm_shrink();
    // HAL_Delay(500);
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
        feetech_servo_move(4,stretch_arm_longgest,4095,feet_acc);
        r_servo_now = stretch_arm_longgest;
    }
}


/// @brief 机械臂运动到放物料
/// @param  
void arm_shrink(void)
{
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arm_shrink_position);
    if( ReadPos(4) >1500)
    {
        feetech_servo_move(4,shrink_arm,4095,feet_acc);
        r_servo_now = shrink_arm;
    }
}

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
        feetech_servo_move(2,claw_spin_position_front,4000,feet_acc);
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
        feetech_servo_move(2,claw_spin_position_state,4000,feet_acc);
    }
    else
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"claw_tooooo_right", strlen("claw_tooooo_right"), 50);
    }
}

void claw_spin_state_without_claw(void)
{
    feetech_servo_move(2,claw_spin_without_claw_position_state,4000,feet_acc);
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


/**
 * @brief 
 * 
 * 
 // 定义PID控制器的参数
#define Kp_theta 0.5
#define Ki_theta 0.1
#define Kd_theta 0.05
#define Kp_r 0.5
#define Ki_r 0.1
#define Kd_r 0.05

// 定义PID控制器的状态
int previous_theta_error = 0, integral_theta = 0;
int previous_r_error = 0, integral_r = 0;

int adjust_position_with_camera(int x_error, int y_error)
{
    int r_adjust_values = 0, theta_adjust_values = 0;
    int is_theta_ok = 0, is_r_ok = 0;

    // PID控制 - theta方向
    int theta_error = x_error; // 当前误差
    integral_theta += theta_error; // 积分
    int derivative_theta = theta_error - previous_theta_error; // 微分
    previous_theta_error = theta_error; // 更新误差

    // 计算PID控制输出
    int theta_pid_output = (Kp_theta * theta_error) + (Ki_theta * integral_theta) + (Kd_theta * derivative_theta);

    // 控制theta调整值，乘以PID输出
    if (theta_pid_output > theta_big_limit) {
        theta_adjust_values = theta_pulse_servo_big * (theta_pid_output / theta_big_limit);
    } else if (theta_pid_output > theta_mid_limit) {
        theta_adjust_values = theta_pulse_servo_mid * (theta_pid_output / theta_mid_limit);
    } else if (theta_pid_output > theta_small_limit) {
        theta_adjust_values = theta_pulse_servo_small * (theta_pid_output / theta_small_limit);
    } else if (theta_pid_output < -theta_big_limit) {
        theta_adjust_values = -theta_pulse_servo_big * (theta_pid_output / -theta_big_limit);
    } else if (theta_pid_output < -theta_mid_limit) {
        theta_adjust_values = -theta_pulse_servo_mid * (theta_pid_output / -theta_mid_limit);
    } else if (theta_pid_output < -theta_small_limit) {
        theta_adjust_values = -theta_pulse_servo_small * (theta_pid_output / -theta_small_limit);
    } else {
        theta_adjust_values = 0; // 已经足够接近目标
        is_theta_ok = 1;
    }

    // PID控制 - r方向
    int r_error = y_error; // 当前误差
    integral_r += r_error; // 积分
    int derivative_r = r_error - previous_r_error; // 微分
    previous_r_error = r_error; // 更新误差

    // 计算PID控制输出
    int r_pid_output = (Kp_r * r_error) + (Ki_r * integral_r) + (Kd_r * derivative_r);

    // 控制r调整值，乘以PID输出
    if (r_pid_output > r_big_limit) {
        r_adjust_values = r_pulse_servo_big * (r_pid_output / r_big_limit);
    } else if (r_pid_output > r_mid_limit) {
        r_adjust_values = r_pulse_servo_mid * (r_pid_output / r_mid_limit);
    } else if (r_pid_output > r_small_limit) {
        r_adjust_values = r_pulse_servo_small * (r_pid_output / r_small_limit);
    } else if (r_pid_output < -r_big_limit) {
        r_adjust_values = -r_pulse_servo_big * (r_pid_output / -r_big_limit);
    } else if (r_pid_output < -r_mid_limit) {
        r_adjust_values = -r_pulse_servo_mid * (r_pid_output / -r_mid_limit);
    } else if (r_pid_output < -r_small_limit) {
        r_adjust_values = -r_pulse_servo_small * (r_pid_output / -r_small_limit);
    } else {
        r_adjust_values = 0; // 已经足够接近目标
        is_r_ok = 1;
    }

    // 更新theta和r方向的舵机
    if (theta_servo_now + theta_adjust_values < theta_right_position_limit && theta_servo_now + theta_adjust_values > theta_left_position_limit) {
        feetech_servo_move(3, theta_servo_now + theta_adjust_values, 3000, 50);
        theta_servo_now += theta_adjust_values;
    }
    HAL_Delay(20);

    if (r_servo_now + r_adjust_values < r_front_position_limit && r_servo_now + r_adjust_values > r_back_position_limit) {
        feetech_servo_move(4, r_servo_now + r_adjust_values, 3000, 50);
        r_servo_now += r_adjust_values;
    }

    if (is_r_ok == 1 && is_theta_ok == 1) {
        return 1;
    } else {
        return 0;
    }
}

 * 
 */