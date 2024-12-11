/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*************************************自己的头文件引用区****************************************/
#include <stdlib.h>
#include "motor.h"
#include "uart_screen.h"
#include "my_usart.h"
#include "my_servo.h"
#include "my_gyroscope.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**************************************各种全局变量区*****************************************/

// 串口相关变量
extern uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50],rxdata_u5[50]; // usart2,3接收缓冲区
extern uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1,received_rxdata_u5,received_rxdata_u4; // 暂存usart2,3接收到的数据单字节变量
extern uchar rxflag_u2,rxflag_u3,rxflag_u1,rxflag_u4,rxflag_u5; // usart2,3接收标志位变量

extern float acceleration; // 加速度
extern float x_move_position, y_move_position; // x、y
extern int is_motor_start_move; 
extern int is_slight_move,motor_state,is_slight_spin;

float gyro_z = 90;

int open_loop_move_velocity = 80;

// 目标颜色数组
volatile int target_colour[6] = {3,2,1,2,1,3}; 
int move_sequence_bias = 0; // 根据不同顺序移动带来的位置相对色环位置的偏差，如中-左-右，则偏差为0、-x、+x 

/// @brief 用于判断当前是第几个case,
int case_count = 0; 
int timeout_limit = 1200; // 超时时间限制，单位10ms
extern int tim3_count;

int is_get_qrcode_target = 0;
int volatile is_start_get_plate = 0; // 开始从转盘抓
int volatile get_plate = 0; // 1 2 3 
int get_plate_count = 0;

extern volatile int test_slight_move; // 用于判断微调是否完成
extern int spin_which_direction;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/************************************函数声明及定义区****************************************/

// printf重定向，用于串口屏的显示


int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}
int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&huart5, &ch, 1, 0xffff);
    return ch;
}
void move_follow_sequence(int target_colour_input[6],int case_count);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */


    /*****************各种系统相关外设的初始化（串口、定时器等)***********************/
    

 
    HAL_UART_Receive_IT(&huart3, &received_rxdata_u3, 1); // 使能串口3接收中断
    // HAL_UART_Receive_IT(&huart1, &received_rxdata_u1, 1); // 使能串口1接收中断
    HAL_UART_Receive_IT(&huart4, &received_rxdata_u4, 1); // 使能串口4接收中断
    HAL_UART_Receive_IT(&huart5, &received_rxdata_u5, 1); // 使能串口5接收中断
    // HAL_UART_Receive_IT(&huart2, &received_rxdata_u2, 1); // 使能串口2接收中断



    HAL_TIM_Base_Start_IT(&htim2); // 使能定时器2中断
    HAL_TIM_Base_Start_IT(&htim3); // 使能定时器3中断
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // 开启TIM1通道1 PWM输出
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // 开启TIM1通道2 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 开启TIM1通道3 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 开启TIM1通道4 PWM输出  
    my_servo_init(); //精密电机初始化，使用精密电机则必须加入


    /*******************************实际功能的初始化******************************************/

    HAL_Delay(4000); // 等待电机初始化完成，本该是4000ms
    // 机械臂初始位置设定
    arm_shrink();
    whole_arm_spin(1);
    put_claw_up_top();
    claw_spin_front();
    open_claw();
    HAL_Delay(1000);

    /*********************************测试区域开始*************************************/
    

    //实际开始移动
    // move_all_direction_position(acceleration, open_loop_move_velocity, -15, 0);
    // HAL_Delay(1200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, 145);
    // HAL_Delay(3500);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)
    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);

    // //从转盘前往粗加工区
    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0,45); 
    // HAL_Delay(2000);
    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, 168);
    // HAL_Delay(4000);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)
    
    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0,82); // 右移40cm
    // HAL_Delay(2500);
    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2000);

    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, 75);
    // HAL_Delay(2500);
    // spin_left(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);

    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, 85);
    // HAL_Delay(3000);

    // move_all_direction_position(acceleration, open_loop_move_velocity, 32, 0);
    // HAL_Delay(3000);

    // 回到了转盘

    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);

    // move_all_direction_position(acceleration, open_loop_move_velocity, 0,45); // 右移40cm
    // HAL_Delay(2000);
    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);

    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, 168);
    // HAL_Delay(4000);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)

    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0,75); // 右移40cm
    // HAL_Delay(2500);
    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2000);

    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, 75);
    // HAL_Delay(2500);
    // spin_left(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);

    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, 93);
    // HAL_Delay(2500);

    // spin_right(open_loop_move_velocity,acceleration, 90);
    // HAL_Delay(2200);

    // move_all_direction_position(acceleration, open_loop_move_velocity, 0,182); // 回到起点
    // HAL_Delay(4000);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)

    // move_all_direction_position(acceleration, open_loop_move_velocity, -20, 0);

    // while(1)
    // {
    //     HAL_Delay(10);
    // }
    /*********************************测试区域结束*************************************/
    //! 调试到此为止，不会进入下面的主程序流程代码




    /**************************************以下为主程序流程代码********************************************/

    //! 小车离开起点并前往转盘
    /*-------------------小车离开起点并前往转盘-----------------------*/

    printf("t0.txt=\"start\"\xff\xff\xff"); // 开始
    HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50); // 开始识别二维码
    HAL_Delay(50);

    move_all_direction_position(acceleration, open_loop_move_velocity, -17 , 0); // 左移出库
    HAL_Delay(1500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, 60); // 前进至二维码
    HAL_Delay(5000);//  
    // 将target_colour转为字符串显示在串口屏上
    char* target_colour_str = (char*)malloc(6);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上
    free(target_colour_str);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, 85); // 前进至转盘
    HAL_Delay(3000);
    spin_right(open_loop_move_velocity,acceleration, 89);
    HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, -5); 
    // HAL_Delay(1000);
    // motor_state = 1;


    //! 开始从转盘抓取
    
    is_start_get_plate = 1; // 

    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); // 开始识别颜色并抓取
    tim3_count = 0; // 开始计时，+1 代表10ms
    // arm_shrink();

    while(get_plate_count < 3 && tim3_count < 5000) // 从转盘抓取三个色环或者超时
    {
        if(get_plate == 1)  //此处会不会一次识别发送了好几个，导致重复抓取同一个位置？
        {
            get_and_load(1);
            get_plate_count++;
            get_plate = 0;
        }
        else if(get_plate == 2)
        {
            get_and_load(2);
            get_plate_count++;
            get_plate = 0;
        }
        else if (get_plate == 3)
        {
            get_and_load(3);
            get_plate_count++;
            get_plate = 0;
        }
        HAL_Delay(10);
    }
    get_plate_count = 0;
    is_start_get_plate = 0;
    arm_stretch();
    //如果tim3_count超时，则加入一个标志位


    //! 小车前往粗加工区
    /*------------------前往粗加工区------------------------*/

    spin_right(open_loop_move_velocity,acceleration, 89);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,40); 
    HAL_Delay(2000);
    spin_right(open_loop_move_velocity,acceleration, 89);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, 166);
    HAL_Delay(5000);


    //! 识别色环移动并放置
    /*------------------识别色环移动并放置------------------------*/

    // 左蓝中绿右红， 1红2绿3蓝


    // 粗加工区第一个物料
    // move_follow_sequence(target_colour,1); 
    // 先校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);

    is_slight_spin = 1; // 使能轻微移动
    motor_state = 1;
    tim3_count = 0;
    while(is_slight_spin != 0 && tim3_count < timeout_limit)
    {
        HAL_Delay(10);
    }
    is_slight_spin = 0;
    stop();
    printf("t0.txt=\"end_of_line\"\xff\xff\xff"); // 校正结束，调试用，正式比赛中须删除
    HAL_Delay(1000);

    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    is_slight_move = 0;
    printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();
    // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    // HAL_Delay(80);
    if(target_colour[0] != 0)
    {
        get_and_put_different_position(target_colour[0]);
    }
    else
    {
        get_and_put_different_position(2);
    }
    if(target_colour[1] != 0)
    {
        get_and_put_different_position(target_colour[1]);
    }
    else
    {
        get_and_put_different_position(3);
    }
    if(target_colour[2] != 0)
    {
        get_and_put_different_position(target_colour[2]);
    }
    else
    {
        get_and_put_different_position(1);
    }

    HAL_Delay(2000);


    if(target_colour[0] != 0)
    {
        get_and_load_different_position(target_colour[0]);
    }
    else
    {
        get_and_load_different_position(2);
    }
    if(target_colour[1] != 0)
    {
        get_and_load_different_position(target_colour[1]);
    }
    else
    {
        get_and_load_different_position(3);
    }
    if(target_colour[2] != 0)
    {
        get_and_load_different_position(target_colour[2]);
    }
    else
    {
        get_and_load_different_position(1);
    }
    whole_arm_spin(1);
    claw_spin_front();
    arm_stretch();
    HAL_Delay(1000);


    // put_from_state();
    
    // printf("t0.txt=\"end_of_circle1\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第二个物料
    // move_follow_sequence(target_colour,2); 
    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit )
    // {
    //     HAL_Delay(10);
    // }
    // is_slight_move = 0;
    // stop();
    // if(target_colour[1] != 0)
    // {
    //     get_from_state(target_colour[1]); 
    // }
    // else
    // {
    //     get_from_state(3); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle2\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第三个物料
    // move_follow_sequence(target_colour,3); 
    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit)
    // {
    //     HAL_Delay(10);
    // }
    // is_slight_move = 0;
    // stop();
    // if(target_colour[2] != 0)
    // {
    //     get_from_state(target_colour[2]); 
    // }
    // else
    // {
    //     get_from_state(1); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle3\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除

    // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    // HAL_Delay(80);




    /*------------------再次抓取------------------------*/
    // open_claw();
    // move_follow_sequence(target_colour,1);
    // get_and_load_ground(target_colour[0]);
    // move_follow_sequence(target_colour,2);
    // get_and_load_ground(target_colour[1]);
    // move_follow_sequence(target_colour,3);
    // get_and_load_ground(target_colour[2]);




    /*------------------前往暂存区------------------------*/
    spin_right(open_loop_move_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,82+ move_sequence_bias); //!待测量
    HAL_Delay(2500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 85, 0);
    HAL_Delay(3000);
    move_sequence_bias = 0; //! 重置偏差


    /*------------------识别色环移动并放置------------------------*/
    // 粗加工区第一个物料
    // 先校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50);     //! 待修改,发给树莓派，开始校正直线
    HAL_Delay(50);

    is_slight_spin = 1; // 使能轻微移动
    while(is_slight_spin != 0)
    {
        HAL_Delay(10);
    }
    stop();
    printf("t0.txt=\"end_of_line\"\xff\xff\xff"); // 校正结束，调试用，正式比赛中须删除
    HAL_Delay(1000);

    // move_follow_sequence(target_colour,1); 

    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    is_slight_move = 0;
    printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();
    // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    // HAL_Delay(80);
    if(target_colour[0] != 0)
    {
        get_and_put_different_position(target_colour[0]);
    }
    else
    {
        get_and_put_different_position(2);
    }
    if(target_colour[1] != 0)
    {
        get_and_put_different_position(target_colour[1]);
    }
    else
    {
        get_and_put_different_position(3);
    }
    if(target_colour[2] != 0)
    {
        get_and_put_different_position(target_colour[2]);
    }
    else
    {
        get_and_put_different_position(1);
    }

    whole_arm_spin(1);
    claw_spin_front();
    arm_stretch();
    HAL_Delay(1000);

    // if(target_colour[0] != 0)
    // {
    //     get_from_state(target_colour[0]); 
    // }
    // else
    // {
    //     get_from_state(2); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle1\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第二个物料
    // move_follow_sequence(target_colour,2); 
    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit )
    // {
    //     HAL_Delay(10);
    // }
    // stop();
    // if(target_colour[1] != 0)
    // {
    //     get_from_state(target_colour[1]); 
    // }
    // else
    // {
    //     get_from_state(3); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle2\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第三个物料
    // move_follow_sequence(target_colour,3); 

    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit)
    // {
    //     HAL_Delay(10);
    // }
    // stop();
    // if(target_colour[2] != 0)
    // {
    //     get_from_state(target_colour[2]); 
    // }
    // else
    // {
    //     get_from_state(1); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle3\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除

    // HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束


    /*------------------再次前往转盘------------------------*/
    spin_right(open_loop_move_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,88);
    HAL_Delay(3000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 42, 0);
    HAL_Delay(3000);


    is_start_get_plate = 1; // 开始从转盘抓取

    HAL_UART_Transmit(&huart3, (uint8_t*)"DD", strlen("DD"), 50); // 开始识别颜色并抓取
    tim3_count = 0; // 开始计时，+1 代表10ms
    arm_shrink();

    while(get_plate_count < 3 && tim3_count < 5000) // 从转盘抓取三个色环或者超时
    {
        if(get_plate == 1)  //此处会不会一次识别发送了好几个，导致重复抓取同一个位置？
        {
            get_and_load(1);
            get_plate_count++;
            get_plate = 0;
        }
        else if(get_plate == 2)
        {
            get_and_load(2);
            get_plate_count++;
            get_plate = 0;
        }
        else if (get_plate == 3)
        {
            get_and_load(3);
            get_plate_count++;
            get_plate = 0;
        }
        HAL_Delay(10);
    }
    get_plate_count = 0;
    is_start_get_plate = 0;
    arm_stretch();
    //如果tim3_count超时，则加入一个标志位

    /*------------------前往粗加工区------------------------*/

    spin_right(open_loop_move_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,40); 
    HAL_Delay(2000);
    spin_right(open_loop_move_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, 168);
    HAL_Delay(5000);

    /*------------------识别色环移动并放置------------------------*/

    // 左蓝中绿右红， 1红2绿3蓝


    // 粗加工区第一个物料
    // move_follow_sequence(target_colour,1); 
    // 先校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);

    is_slight_spin = 1; // 使能轻微移动
    motor_state = 1;
    tim3_count = 0;
    while(is_slight_spin != 0 && tim3_count < timeout_limit)
    {
        HAL_Delay(10);
    }
    is_slight_spin = 0;
    stop();
    printf("t0.txt=\"end_of_line\"\xff\xff\xff"); // 校正结束，调试用，正式比赛中须删除
    HAL_Delay(1000);

    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    is_slight_move = 0;
    printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();

    if(target_colour[3] != 0)
    {
        get_and_put_different_position(target_colour[3]);
    }
    else
    {
        get_and_put_different_position(2);
    }
    if(target_colour[4] != 0)
    {
        get_and_put_different_position(target_colour[4]);
    }
    else
    {
        get_and_put_different_position(3);
    }
    if(target_colour[5] != 0)
    {
        get_and_put_different_position(target_colour[5]);
    }
    else
    {
        get_and_put_different_position(1);
    }

    HAL_Delay(2000);

    if(target_colour[3] != 0)
    {
        get_and_load_different_position(target_colour[3]);
    }
    else
    {
        get_and_load_different_position(2);
    }
    if(target_colour[4] != 0)
    {
        get_and_load_different_position(target_colour[4]);
    }
    else
    {
        get_and_load_different_position(3);
    }
    if(target_colour[5] != 0)
    {
        get_and_load_different_position(target_colour[5]);
    }
    else
    {
        get_and_load_different_position(1);
    }
    whole_arm_spin(1);
    claw_spin_front();
    arm_stretch();
    HAL_Delay(1000);




    // if(target_colour[0] != 0)
    // {
    //     get_from_state(target_colour[0]); 
    // }
    // else
    // {
    //     get_from_state(2); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle1\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第二个物料
    // move_follow_sequence(target_colour,2); 
    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit )
    // {
    //     HAL_Delay(10);
    // }
    // is_slight_move = 0;
    // stop();
    // if(target_colour[1] != 0)
    // {
    //     get_from_state(target_colour[1]); 
    // }
    // else
    // {
    //     get_from_state(3); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle2\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第三个物料
    // move_follow_sequence(target_colour,3); 
    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit)
    // {
    //     HAL_Delay(10);
    // }
    // is_slight_move = 0;
    // stop();
    // if(target_colour[2] != 0)
    // {
    //     get_from_state(target_colour[2]); 
    // }
    // else
    // {
    //     get_from_state(1); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle3\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除

    // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    // HAL_Delay(80);
    // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    // HAL_Delay(80);




    /*------------------再次抓取------------------------*/
    // open_claw();
    // move_follow_sequence(target_colour,1);
    // get_and_load_ground(target_colour[0]);
    // move_follow_sequence(target_colour,2);
    // get_and_load_ground(target_colour[1]);
    // move_follow_sequence(target_colour,3);
    // get_and_load_ground(target_colour[2]);




    /*------------------前往暂存区------------------------*/
    spin_right(open_loop_move_velocity,acceleration, 89);
    HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0,83+ move_sequence_bias); //!待测量
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,83); //!待测量
    HAL_Delay(2500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 80, 0);
    HAL_Delay(3000);
    move_sequence_bias = 0; //! 重置偏差


    /*------------------识别色环移动并放置------------------------*/
    // 粗加工区第一个物料
    // 先校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50);     //! 待修改,发给树莓派，开始校正直线
    HAL_Delay(50);

    is_slight_spin = 1; // 使能轻微移动
    while(is_slight_spin != 0)
    {
        HAL_Delay(10);
    }
    stop();
    printf("t0.txt=\"end_of_line\"\xff\xff\xff"); // 校正结束，调试用，正式比赛中须删除
    HAL_Delay(1000);

    // move_follow_sequence(target_colour,1); 

    is_slight_move = 1; // 使能微调
    motor_state = 1;
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();

    if(target_colour[3] != 0)
    {
        get_and_put_different_position_pileup(target_colour[3]);
    }
    else
    {
        get_and_put_different_position_pileup(2);
    }
    if(target_colour[4] != 0)
    {
        get_and_put_different_position_pileup(target_colour[4]);
    }
    else
    {
        get_and_put_different_position_pileup(3);
    }
    if(target_colour[5] != 0)
    {
        get_and_put_different_position_pileup(target_colour[5]);
    }
    else
    {
        get_and_put_different_position_pileup(1);
    }

    // HAL_Delay(2000);

    // if(target_colour[3] != 0)
    // {
    //     get_and_load_different_position(target_colour[3]);
    // }
    // else
    // {
    //     get_and_load_different_position(2);
    // }
    // if(target_colour[4] != 0)
    // {
    //     get_and_load_different_position(target_colour[4]);
    // }
    // else
    // {
    //     get_and_load_different_position(3);
    // }
    // if(target_colour[5] != 0)
    // {
    //     get_and_load_different_position(target_colour[5]);
    // }
    // else
    // {
    //     get_and_load_different_position(1);
    // }
    whole_arm_spin(1);
    claw_spin_front();
    arm_stretch();
    HAL_Delay(1000);



    // if(target_colour[0] != 0)
    // {
    //     get_from_state(target_colour[0]); 
    // }
    // else
    // {
    //     get_from_state(2); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle1\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第二个物料
    // move_follow_sequence(target_colour,2); 
    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit )
    // {
    //     HAL_Delay(10);
    // }
    // stop();
    // if(target_colour[1] != 0)
    // {
    //     get_from_state(target_colour[1]); 
    // }
    // else
    // {
    //     get_from_state(3); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle2\"\xff\xff\xff");  // 校正结束，调试用，正式比赛中须删除


    // // 粗加工区第三个物料
    // move_follow_sequence(target_colour,3); 

    // is_slight_move = 1;
    // tim3_count = 0;
    // while(is_slight_move != 0 && tim3_count < timeout_limit)
    // {
    //     HAL_Delay(10);
    // }
    // stop();
    // if(target_colour[2] != 0)
    // {
    //     get_from_state(target_colour[2]); 
    // }
    // else
    // {
    //     get_from_state(1); 
    // }
    // put_from_state();
    // printf("t0.txt=\"end_of_circle3\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除

    // HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束

    spin_right(open_loop_move_velocity,acceleration, 88);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, 93);
    HAL_Delay(2500);

    spin_right(open_loop_move_velocity,acceleration, 88);
    HAL_Delay(2200);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,250); // 回到起点
    HAL_Delay(5000);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)

    move_all_direction_position(acceleration, open_loop_move_velocity, -20, 0);



    



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50);
    HAL_Delay(100);



    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/// @brief 根据识别到的颜色顺序，移动到对应的位置,左蓝中绿右红， 1红2绿3蓝
/// @param target_colour_input 
/// @param case 放置的阶段，1 2 3 对应粗加工区，然后循环，1 2 3 对应暂存区 ，第二轮则又从 1 2 3 开始
void move_follow_sequence(int target_colour_input[6],int case_count_input)
{
    switch (case_count_input)
    {
        case 1:
            if(target_colour_input[0] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15 +move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = -15;
                
            }
            else if(target_colour_input[0] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;
            }
            else if(target_colour_input[0] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15+move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;
        
        case 2:
            if(target_colour_input[1] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15+move_sequence_bias, 0); //即相对中线右移15cm，再加上偏差
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if(target_colour_input[1] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;

            }
            else if(target_colour_input[1] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15+move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;

        case 3:
            if(target_colour_input[2] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15+move_sequence_bias, 0); //即相对中线右移15cm，再加上偏差
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if(target_colour_input[2] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;

            }
            else if(target_colour_input[2] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15+move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;



    }

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
