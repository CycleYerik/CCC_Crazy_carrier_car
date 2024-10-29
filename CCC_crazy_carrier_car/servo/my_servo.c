#include "my_servo.h"


extern TIM_HandleTypeDef htim1;

/// @brief 精密舵机串口初始化
void my_servo_init()
{
    Uart_Init(115200);
}

void open_claw(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 150);
}

void close_claw(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 120);
}

void arm_stretch(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 180);
}

void arm_shrink(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 50);
}

void state_spin(int state_position)
{
    if(state_position == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 68);
    }
    else if(state_position == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 154);
    }
    else if(state_position == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 238);
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
    int position_abs = (Position- ReadPos(servo_ID))? (Position- ReadPos(servo_ID)): (ReadPos(servo_ID)-Position);

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