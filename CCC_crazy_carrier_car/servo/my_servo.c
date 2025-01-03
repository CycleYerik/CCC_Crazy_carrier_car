#include "my_servo.h"


extern TIM_HandleTypeDef htim4;
/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

/*-------以下数据如果重新装车需要重新调，务必注意！！！！！！！！！！！！-------*/

// 夹爪PD13  载物盘PD14 机械臂PD12

// 普通舵机参数范围25-125  270
int open_claw_180_position = 99; //87 为正常抓放的位置
int open_claw_position = 87; //87 为正常抓放的位置
int close_claw_position = 74;
int arm_stretch_position = 39; // 弃用
int arm_shrink_position = 75;  // 弃用
int arm_shrink_all_position = 75; // 弃用
int state_spin_position_1 = 25;  //120度对应44.44
int state_spin_position_2 = 69;
int state_spin_position_3 = 114; 


// 精密舵机参数范围0-4095
int feet_acc = 50;

int put_claw_down_pile_position = 2391;
int put_claw_down_state_position = 1246 ; //从车的载物盘上  
int put_claw_down_position = 2230;  // 从转盘上取物料
int put_claw_down_ground_position = 3400; // 放在地上
int put_claw_up_top_position = 743; // 最高点  -700
int put_claw_up_position =960; //  
int claw_spin_position_front = 1937; // 2号精密舵机回到前方
int claw_spin_position_state = 233; // 2号精密舵机回到载物盘// 230
int right_arm = 2935;
int left_arm = 2935;
int middle_arm = 2865; 

int shrink_arm = 1925;
int stretch_arm_longgest = 2102;
int stretch_camera = 413; 
int shrink_arm_all = 500;



int left_2 = 1620;
int left_3 = 2337; // +500
int left_4 = 1790;

int right_2 = 2250; 
int right_3 = 3375; // +500
int right_4 = 1830;

int middle_2 = 1937;
int middle_3 = 2865; // +500
int middle_4 = 430;  // 413



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

void get_and_load_different_position(int position)
{
    state_spin(position);
    whole_arm_spin(1);
    arm_shrink(); // 机械臂收回至载物盘上
    open_claw();
    put_claw_up();
    HAL_Delay(1000);
    if(position == 1) //红
    {
        feetech_servo_move(2,right_2,4095,feet_acc);
        feetech_servo_move(3,right_3,1000,feet_acc);
        feetech_servo_move(4,right_4,4095,feet_acc);
    }
    else if(position == 2)
    {
        feetech_servo_move(4,middle_4-20,4095,feet_acc); //! magic number
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
    put_claw_up();
    HAL_Delay(700);
    claw_spin_state();
    HAL_Delay(1200);
    open_claw();
    HAL_Delay(300);
    put_claw_up_top();
    claw_spin_front();
    HAL_Delay(500);
}

/// @brief 根据物料编号在同一个位置放置三个物料
/// @param position 
void get_and_put_different_position(int position)
{
    state_spin(position);
    whole_arm_spin(1);
    arm_shrink(); // 机械臂收回至载物盘上
    open_claw();
    put_claw_up_top();
    // HAL_Delay(500);
    claw_spin_state();
    HAL_Delay(800);    
    put_claw_down_state();
    HAL_Delay(800);

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
    put_claw_down_ground();    
    HAL_Delay(1600);
    open_claw();
    HAL_Delay(500);
    put_claw_up_top();
    HAL_Delay(500);
    if(position == 1 || position == 3)
    {
        claw_spin_front();
    }
    arm_shrink();
    HAL_Delay(500);
}

void get_and_put_different_position_pileup(int position)
{
    state_spin(position);
    whole_arm_spin(1);
    arm_shrink(); // 机械臂收回至载物盘上
    open_claw();
    put_claw_up_top();
    HAL_Delay(1000);
    claw_spin_state();
    HAL_Delay(800);    
    put_claw_down_state();
    HAL_Delay(1000);

    close_claw();
    put_claw_up();
    HAL_Delay(500);
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
    if(position == 1 || position == 3)
    {
        claw_spin_front();
    }
    arm_shrink();
    HAL_Delay(500);
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

/// @brief 夹爪关闭
/// @param  
void close_claw(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, close_claw_position);
}


/// @brief 机械臂伸长到摄像头
/// @param  
void arm_stretch(void)
{
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arm_stretch_position);
    feetech_servo_move(4,stretch_camera,4095,feet_acc);
}


/// @brief 机械臂收回到放物料
/// @param  
void arm_shrink(void)
{
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arm_shrink_position);
    feetech_servo_move(4,shrink_arm,4095,feet_acc);
}

void arm_shrink_all(void)
{
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, arm_shrink_all_position);
    feetech_servo_move(4,shrink_arm_all,4095,feet_acc);
}

/// @brief 载物盘旋转到对应的位置
/// @param state_position 
void state_spin(int state_position)
{
    if(state_position == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, state_spin_position_1);
    }
    else if(state_position == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, state_spin_position_2);
    }
    else if(state_position == 3)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, state_spin_position_3);
    }
    
}

void put_claw_down_ground(void)
{
    feetech_servo_move(1,put_claw_down_ground_position,4095,feet_acc);
}

/// @brief 夹爪下降到车的载物盘高度
/// @param  
void put_claw_down_state(void)
{
    feetech_servo_move(1,put_claw_down_state_position,4095,feet_acc);
}

/// @brief 绳驱转盘转动，夹爪下降,抓取转盘上的物品
/// @param  
void put_claw_down(void)
{
    feetech_servo_move(1,put_claw_down_position,4095,feet_acc);
}

/// @brief 绳驱转盘转动，夹爪上升
/// @param  
void put_claw_up(void)
{
    feetech_servo_move(1,put_claw_up_position,4095,feet_acc);
}

void put_claw_up_top(void)
{
    feetech_servo_move(1,put_claw_up_top_position,4095,feet_acc);
}

/// @brief 夹爪旋转到朝向前方
/// @param  
void claw_spin_front(void)
{
    feetech_servo_move(2,claw_spin_position_front,4000,feet_acc);
}

/// @brief 夹爪旋转到朝向载物台
/// @param
void claw_spin_state(void)
{
    feetech_servo_move(2,claw_spin_position_state,4000,feet_acc);
}

/// @brief 中板旋转,1为中间位置
/// @param position 
void whole_arm_spin(int status)
{
    if(status == 1)
    {
        feetech_servo_move(3,middle_arm,1000,feet_acc);
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
