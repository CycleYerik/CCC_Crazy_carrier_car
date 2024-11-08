#include "my_servo.h"


extern TIM_HandleTypeDef htim1;
/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

// 普通舵机参数范围50-250
int open_claw_position = 145; 
int close_claw_position = 113;
int arm_stretch_position = 50;
int arm_shrink_position = 70;  // 155则收回撞到
int state_spin_position_1 = 65;
int state_spin_position_2 = 151;
int state_spin_position_3 = 235;

// 精密舵机参数范围0-4095
int put_claw_down_position = 2000;  // 3050是从地面抓取
int put_claw_up_position = 610;
int claw_spin_position_front = 1930;
int claw_spin_position_state = 300;
int right_arm = 2750;
int left_arm = 1350;
int middle_arm = 2150;



/// @brief 抓取并放置
/// @param  
void get_and_load(int position)
{
    // 伸长并打开夹爪
    arm_stretch();
    claw_spin_front();
    open_claw();
    HAL_Delay(2000);

    // 放下夹爪
    put_claw_down();
    HAL_Delay(2000);

    // 抓取
    close_claw();
    HAL_Delay(800);

    // 拉起夹爪
    put_claw_up();
    HAL_Delay(500);
    claw_spin_state();
    HAL_Delay(1000);

    // 收回
    arm_shrink();
    state_spin(position);
    HAL_Delay(1000);

    // 放下
    open_claw();
    HAL_Delay(500);
}




/// @brief 精密舵机串口初始化
void my_servo_init()
{
    Uart_Init(115200);
}

/// @brief 夹爪打开
/// @param  
void open_claw(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, open_claw_position);
}

/// @brief 夹爪关闭
/// @param  
void close_claw(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, close_claw_position);
}


/// @brief 机械臂伸长
/// @param  
void arm_stretch(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, arm_stretch_position);
}

/// @brief 机械臂收回
/// @param  
void arm_shrink(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, arm_shrink_position);
}

/// @brief 载物盘旋转到对应的位置
/// @param state_position 
void state_spin(int state_position)
{
    if(state_position == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, state_spin_position_1);
    }
    else if(state_position == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, state_spin_position_2);
    }
    else if(state_position == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, state_spin_position_3);
    }
    
}

/// @brief 绳驱转盘转动，夹爪下降
/// @param  
void put_claw_down(void)
{
    feetech_servo_move(1,put_claw_down_position,4095,50);
}

/// @brief 绳驱转盘转动，夹爪上升
/// @param  
void put_claw_up(void)
{
    feetech_servo_move(1,put_claw_up_position,4095,50);
}

/// @brief 夹爪旋转到朝向前方
/// @param  
void claw_spin_front(void)
{
    feetech_servo_move(2,claw_spin_position_front,2000,50);
}

/// @brief 夹爪旋转到朝向载物台
/// @param
void claw_spin_state(void)
{
    feetech_servo_move(2,claw_spin_position_state,2000,50);
}

/// @brief 中板旋转（待完善）
/// @param position 
void whole_arm_spin(int position)
{
    // if(position )
    feetech_servo_move(3,position,500,50);
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
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, position);
    }
    else if(servo_ID == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, position);
    }
    else if(servo_ID == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, position);
    }
}