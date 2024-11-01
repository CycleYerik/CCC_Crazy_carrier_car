#include "my_servo.h"


extern TIM_HandleTypeDef htim1;


void get_and_load(void)
{
    // 伸长并打开夹爪
    arm_stretch();
    open_claw();
    HAL_Delay(2000);

    // 放下夹爪
    feetech_servo_move(1,3200,4095,50);
    HAL_Delay(2000);

    // 抓取
    close_claw();
    HAL_Delay(1000);

    // 拉起夹爪
    feetech_servo_move(1,1100,4095,50);
    HAL_Delay(2000);

    // 旋转
    feetech_servo_move(2,3857,2000,50);
    HAL_Delay(2000);

    // 收回
    arm_shrink();
    HAL_Delay(1500);

    // 旋转
    state_spin(1);
    HAL_Delay(1500);

    // 放下
    open_claw();
    HAL_Delay(1000);

    // 伸长
    arm_stretch();
    HAL_Delay(2000);

    // 旋转
    feetech_servo_move(2,1280,2000,50);
    HAL_Delay(2000);


}




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
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 90);
}


/// @brief 伸长去抓物料
/// @param  
void arm_stretch(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 140);
}


void arm_shrink(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 225);
}

void state_spin(int state_position)
{
    if(state_position == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 65);
    }
    else if(state_position == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 151);
    }
    else if(state_position == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 235);
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