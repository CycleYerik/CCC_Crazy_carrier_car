#include "my_servo.h"


/// @brief 精密舵机串口初始化
void my_servo_init()
{
    Uart_Init(115200);
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
    int time = (position_abs / Speed)*1000 + (Speed / (ACC*100))*1000;
    
    // 此处的延时计算还有问题
    HAL_Delay(time); //[(P1-P0)/V]*1000+[V/(A*100)]*1000

}