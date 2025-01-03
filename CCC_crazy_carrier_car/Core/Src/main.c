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

float volatile gyro_z = 90;

int open_loop_move_velocity = 400; //!200
int open_loop_spin_velocity = 200;

// 目标颜色数组
volatile int target_colour[6] = {2,3,1,2,1,3}; 
int move_sequence_bias = 0; // 根2不同顺序移动带来的位置相对色环位置的偏差，如中-左-右，则偏差为0、-x、+x 

/// @brief 用于判断当前是第几个case,
int case_count = 0; 
int timeout_limit = 3000; // 超时时间限制，单位10ms
extern int tim3_count;

int is_get_qrcode_target = 0; //!!!!!!
int volatile is_start_get_plate = 0; // 开始从转盘抓
int volatile get_plate = 0; // 1 2 3 
int get_plate_count = 0;

extern volatile int test_slight_move; // 用于判断微调是否完成
extern int spin_which_direction;
extern int put_claw_down_ground_position;

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
void move_follow_sequence(int target_colour_input[6], int case_count_input, int status);
void start_and_come_to_turntable(void);
void come_to_raw_processing_area(void);
void come_to_temporary_area(void);
void come_to_turntable_from_temparea(void);
void come_back_to_start_from_temparea(void);

void get_from_turntable_test(void);
void get_and_put_in_one_position_test(int time_status);
void get_and_load_in_one_position_test(void);

void get_from_turntable(int turntable_status);
void get_and_put_in_one_position(int time_status);
void get_and_load_in_one_position(int time_status);

void get_and_put_with_movement(int status,int is_pile_up);
void get_and_load_with_movement(int status);
void spin_adjust_line(void);

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
	HAL_Delay(1000);
    my_servo_init(); //精密舵机初始化，使用精密舵机则必须加入


    /*******************************实际功能的初始化******************************************/

    // HAL_Delay(2000); // !等待电机初始化完成，本该是4000ms
    // 机械臂初始位置设定
    // arm_shrink();
    arm_stretch();
    whole_arm_spin(1);
    put_claw_up_top();
    claw_spin_front();
    open_claw_180();


    // HAL_Delay(1000);

    /*********************************测试区域开始*************************************/
    // float gyro_error_last = 0;
    // float target_angle = 90; // 目标角度
    // float current_angle = 0; // 当前角度
    float gyro_error_last = 0; // 上一次陀螺仪角度

    // while(1)
    // {
    //     gyro_error_last = gyro_z;
    //     printf("t0.txt=\"%.3f\"\xff\xff\xff", gyro_z);
    //     spin_right_velocity(30, acceleration);
    //     HAL_Delay(500);
    //     while(abs(gyro_z - gyro_error_last) < 90)
    //     {
    //         // current_angle = gyro_z - gyro_error_last;
    //         // if(current_angle < 0)
    //         // {
    //         //     current_angle += 360; // 将负角度转换为正角度
    //         // }
    //         HAL_Delay(10);
    //         // printf("t0.txt=\"%.3f\"\xff\xff\xff", gyro_z);
    //     }
    //     stop();
    //     printf("t0.txt=\"%.3f\"\xff\xff\xff", gyro_z);
    //     HAL_Delay(2000);

    //     // gyro_error_last = gyro_z;
    //     // spin_left_velocity(50, acceleration);
    //     // while(abs(current_angle - target_angle) < 90)
    //     // {
    //     //     current_angle = gyro_z - gyro_error_last;
    //     //     if(current_angle < 0)
    //     //     {
    //     //         current_angle += 360; // 将负角度转换为正角度
    //     //     }
    //     //     HAL_Delay(10);
    //     // }
    //     // stop();
    //     // HAL_Delay(4000);
    // }
    


    // get_and_put_different_position(1);
    // get_and_put_different_position(2);
    // get_and_put_different_position(3);
    // get_and_load_different_position(1);
    // HAL_Delay(1000);
    // get_and_load_different_position(2);
    // HAL_Delay(1000);
    // get_and_load_different_position(3);
    // HAL_Delay(3000);
    // while(1)
    // {
    //     HAL_Delay(4000);
    // }
    // close_claw();
    // arm_stretch();
    // put_claw_down_ground();
    // HAL_Delay(4000);
    // get_and_put_in_one_position(1);
        // get_and_put_different_position(target_colour[0]);
        // get_and_put_different_position(target_colour[1]);
        // get_and_put_different_position(target_colour[2]);
        // while(1)
        // {
        //     get_and_put_in_one_position(1);
        //     arm_stretch();
        //     HAL_Delay(3000);
        // }
    

//     while(1)
//     {
// // 先校正车身位置
//     HAL_UART_Transmit(&huart3, (uint8_t*)"EE", strlen("EE"), 50); //发给树莓派，开始校正直线
//     HAL_Delay(50);

//     is_slight_spin = 1; // 使能轻微移动
//     motor_state = 1;
//     tim3_count = 0;
//     while(is_slight_spin != 0 && tim3_count < timeout_limit)
//     {
//         HAL_Delay(10);
//     }
// 	stop();
//     if(tim3_count >= timeout_limit)
//     {
//         HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
//         HAL_Delay(80);
//     }


	// while(1)
    // {
    //     HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正
    // HAL_Delay(50);
	// is_slight_move = 1;
	// motor_state = 1;
	// tim3_count = 0;
	// while(is_slight_move != 0)
	// {
	// 	HAL_Delay(10);
	// }
	// // if(tim3_count >= timeout_limit)
    // // {
    // //     HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
    // //     HAL_Delay(80);
    // // }
	// is_slight_move = 0;
	// stop();
	// HAL_Delay(2000);
	// }
    

    
//     is_slight_spin = 0;
//     stop();
//     HAL_Delay(4000);
    //    }
    //     while(1)
    //    {
    //         move_all_direction_position(acceleration, open_loop_move_velocity, 0, 150);
	// 		HAL_Delay(4000);
	// 		move_all_direction_position(acceleration, open_loop_move_velocity, 0, -150);
    //         HAL_Delay(4000);
    //    }

	





    /*********************************测试区域结束*************************************/
    //! 调试到此为止，不会进入下面的主程序流程代码



    /**************************************以下为主程序流程代码********************************************/

    //小车离开起点并前往转盘

    start_and_come_to_turntable(); // 从起点前往转盘
    get_from_turntable(1);  // 从转盘抓取物料

    // get_from_turntable_test();  // !从转盘抓取物料测试
    // HAL_Delay(4000); //! 单独路径测试

    //小车第一次前往粗加工区
    come_to_raw_processing_area();


// HAL_Delay(4000); //! 单独路径测试

    //粗加工区识别色环移动并放置
    /*************方案一**************/
    arm_stretch();
    get_and_put_in_one_position(1);
    get_and_load_in_one_position(1);

    // get_and_put_in_one_position_test(1); // !测试
    // get_and_load_in_one_position_test(); // !测试

    /************方案二*************/
    // get_and_put_with_movement(1,0);
    // get_and_load_with_movement(1);
    // move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
    // HAL_Delay(1000);
    // move_sequence_bias = 0;



    
    //小车第一次前往暂存区
    come_to_temporary_area();
    
    // HAL_Delay(4000); //! 单独路径测试

    //暂存区识别色环移动并放置
    /*************方案一**************/
    arm_stretch();
    get_and_put_in_one_position(2);

    // get_and_put_in_one_position_test(2); // !测试

    /*************方案二**************/
    // get_and_put_with_movement(1,0);
    // move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
    // HAL_Delay(1000);
    // move_sequence_bias = 0;



    // 第一次从暂存区去转盘
    come_to_turntable_from_temparea();

    // HAL_Delay(4000); //! 单独路径测试


    // 第二次从转盘抓取物料  
    arm_stretch();
    open_claw_180();
    get_from_turntable(2);

    // get_from_turntable_test();  // !从转盘抓取物料测试

    // 第二次前往粗加工区
    come_to_raw_processing_area();
    // HAL_Delay(4000); //! 单独路径测试


    //粗加工区识别色环移动并放置
    /*************方案一**************/
    arm_stretch();
    get_and_put_in_one_position(3);
    get_and_load_in_one_position(3);

    // get_and_put_in_one_position_test(3); // !测试
    // get_and_load_in_one_position_test(); // !测试

	arm_stretch();


    /************方案二*************/
    // get_and_put_with_movement(2,0);
    // get_and_load_with_movement(2);
    // move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
    // HAL_Delay(1000);



    // 第二次前往暂存区
    come_to_temporary_area();
    // HAL_Delay(4000); //! 单独路径测试


    //暂存区识别色环移动并放置
    
    /*************方案一**************/
    get_and_put_in_one_position(4);

    // get_and_put_in_one_position_test(4); // !测试

    /*************方案二**************/
    // get_and_put_with_movement(2,1);




    // 第二次从暂存区回原点
    come_back_to_start_from_temparea();

    //! 总体代码流程到此结束了

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


void spin_adjust_line(void)
{
    // 先校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"EE", strlen("EE"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);

    is_slight_spin = 1; // 使能轻微移动
    motor_state = 1;
    tim3_count = 0;
    while(is_slight_spin != 0 && tim3_count < timeout_limit)
    {
        HAL_Delay(10);
    }
    if(tim3_count >= timeout_limit)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
        HAL_Delay(80);
    }
}

/// @brief 根据识别到的颜色顺序，移动到对应的位置,左蓝中绿右红， 1红2绿3蓝
/// @param target_colour_input 
/// @param case 放置的阶段，1 2 3 对应粗加工区，然后循环，1 2 3 对应暂存区 ，第二轮则又从 1 2 3 开始
void move_follow_sequence(int target_colour_input[6], int case_count_input, int status)
{
    if (status == 1)
    {

        switch (case_count_input)
        {
        case 1:
            if (target_colour_input[0] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if (target_colour_input[0] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;
            }
            else if (target_colour_input[0] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;

        case 2:
            if (target_colour_input[1] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15 + move_sequence_bias, 0); // 即相对中线右移15cm，再加上偏差
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if (target_colour_input[1] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;
            }
            else if (target_colour_input[1] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;

        case 3:
            if (target_colour_input[2] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15 + move_sequence_bias, 0); // 即相对中线右移15cm，再加上偏差
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if (target_colour_input[2] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;
            }
            else if (target_colour_input[2] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;
        }
    }

    if (status == 2)
    {
        switch (case_count_input)
        {
        case 1:
            if (target_colour_input[3] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if (target_colour_input[3] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;
            }
            else if (target_colour_input[3] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;

        case 2:
            if (target_colour_input[4] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15 + move_sequence_bias, 0); // 即相对中线右移15cm，再加上偏差
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if (target_colour_input[4] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;
            }
            else if (target_colour_input[4] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;

        case 3:
            if (target_colour_input[5] == 1)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, 15 + move_sequence_bias, 0); // 即相对中线右移15cm，再加上偏差
                HAL_Delay(2500);
                move_sequence_bias = -15;
            }
            else if (target_colour_input[5] == 2)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 0;
            }
            else if (target_colour_input[5] == 3)
            {
                move_all_direction_position(acceleration, open_loop_move_velocity, -15 + move_sequence_bias, 0);
                HAL_Delay(2500);
                move_sequence_bias = 15;
            }
            break;
        }
    }
}

/// @brief 从起点出发，前进到转盘并调整好姿态准备抓取
/// @param  
void start_and_come_to_turntable(void)
{
    int start_move = 15;
    int move_to_qrcode = 45;
    int move_from_qrcode_to_table = 85;
    int spin_right_angle = 90;
    
    printf("t0.txt=\"start\"\xff\xff\xff"); // 开始
    HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50); // 开始识别二维码
    HAL_Delay(50);
    move_all_direction_position(acceleration, open_loop_move_velocity, -start_move , start_move); // 左移出库
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_to_qrcode); // 前进至二维码
    HAL_Delay(2000);
    // 将target_colour转为字符串显示在串口屏上
    char* target_colour_str = (char*)malloc(6);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table); // 前进至转盘
    HAL_Delay(2500);
    spin_right(open_loop_spin_velocity,acceleration, spin_right_angle);
    HAL_Delay(1300);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上
    free(target_colour_str);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, -5); 
    // HAL_Delay(1000);
}


/// @brief 从转盘抓取色环
/// @param  
void get_from_turntable(int turntable_status)
{
        //! 开始从转盘抓取
    int is_1_get = 0, is_2_get = 0, is_3_get = 0;
    is_start_get_plate = 1;
    if(turntable_status == 1)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); // 开始识别颜色并抓取
        HAL_Delay(100);
        HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); // 开始识别颜色并抓取
        HAL_Delay(100);
        HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); // 开始识别颜色并抓取
        HAL_Delay(100);
    }
    else if(turntable_status == 2)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"DD", strlen("DD"), 50); // 开始识别颜色并抓取
        HAL_Delay(200);
    }
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(get_plate_count < 3 ) // 从转盘抓取三个色环或者超时
    // while(1)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)&get_plate, 1, 50);
        if(get_plate == 1 && is_1_get == 0)  //此处会不会一次识别发送了好几个，导致重复抓取同一个位置？
        {
            get_plate = 0;
            // is_start_get_plate = 0;
            get_and_load(1);   //!
            get_plate_count++;
            HAL_UART_Transmit(&huart3, (uint8_t*)"red", strlen("red"), 50); 
            HAL_Delay(50);
            // is_start_get_plate = 1;
            get_plate = 0;
            is_1_get = 1; //!
            
        }
        else if(get_plate == 2 && is_2_get == 0)
        {
            get_plate = 0;
            // is_start_get_plate = 0;
            get_and_load(2);  //!
            get_plate_count++;
            HAL_UART_Transmit(&huart3, (uint8_t*)"green", strlen("green"), 50);
            HAL_Delay(50); 
            // is_start_get_plate = 1;
            get_plate = 0;
            is_2_get = 1; //!
            
        }
        else if (get_plate == 3 && is_3_get == 0)
        {
            get_plate = 0;
            // is_start_get_plate = 0;
            get_and_load(3);  //!
            get_plate_count++;
            HAL_UART_Transmit(&huart3, (uint8_t*)"blue", strlen("blue"), 50); 
            HAL_Delay(50);
            // is_start_get_plate = 1;
            get_plate = 0;
            is_3_get = 1; //!
            
        }
        HAL_UART_Transmit(&huart3, (uint8_t*)"loop", strlen("loop"), 50); // 开始识别颜色并抓取
        get_plate = 0;
        HAL_UART_Transmit(&huart3, (uint8_t*)&get_plate, 1, 50);
        HAL_Delay(100);
        

    }
    get_plate_count = 0;
    is_start_get_plate = 0;
    // arm_stretch();
    // HAL_Delay(500);
}

/// @brief 无视觉从转盘抓取物料，用于调试
/// @param  
void get_from_turntable_test(void)
{
    get_and_load(1);
    HAL_Delay(2000);
    get_and_load(2);
    HAL_Delay(2000);
    get_and_load(3);
    HAL_Delay(2000);
}

/// @brief 从转盘前往粗加工区
/// @param  
void come_to_raw_processing_area(void)
{
    // int move_right_length = 41;
    // int move_front_length = 170;
    // spin_right(open_loop_spin_velocity,acceleration, 90);
    // HAL_Delay(2200);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_right_length);
    // HAL_Delay(2000);
    // spin_right(open_loop_spin_velocity,acceleration, 90);
    // HAL_Delay(2000);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_front_length);
    // HAL_Delay(4000);


    //一次转动
    int move_right_length = 39;
    int move_front_length = 172;
    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length,0);
    HAL_Delay(2000);
    spin_right(open_loop_spin_velocity,acceleration, 178);
    HAL_Delay(3000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_front_length);
    HAL_Delay(5000);
    char* target_colour_str = (char*)malloc(6);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上


}

/// @brief 从粗加工区前往暂存区
/// @param  
void come_to_temporary_area(void)
{
    int move_front_length = 85; // 88
    int move_right_length = 85;
    spin_right(open_loop_spin_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length );
    HAL_Delay(2500);
    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length, 0);
    HAL_Delay(3200);
}

/// @brief 从暂存区前往转盘
/// @param  
void come_to_turntable_from_temparea(void)
{
    int move_right_length = 47;
    int move_front_length = 90;
    spin_right(open_loop_spin_velocity,acceleration, 90);
    arm_stretch();
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length);
    HAL_Delay(3000);

    // spin_adjust_line();  //! 实际联调需要加上

    move_all_direction_position(acceleration, open_loop_move_velocity,move_right_length, 0);
    HAL_Delay(3000);
}

/// @brief 从暂存区回到起点
void come_back_to_start_from_temparea(void)
{
    int move_45_length = -18;
    int move_front_length_1 = 89;
    int move_front_length_2 = 173;
    spin_right(open_loop_spin_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_1);
    HAL_Delay(2500);
    spin_right(open_loop_spin_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_2);
    HAL_Delay(4000);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)
    move_all_direction_position(acceleration, open_loop_move_velocity, move_45_length, -move_45_length);
    HAL_Delay(2000);
}

/// @brief 无视觉原地放置三个物料测试
/// @param  
void get_and_put_in_one_position_test(int time_status)
{
    if(time_status == 4)
    {
        get_and_put_different_position_pileup(2);
        get_and_put_different_position_pileup(1);
        get_and_put_different_position_pileup(3);
    }
    else
    {
        get_and_put_different_position(2);
        get_and_put_different_position(1);
        get_and_put_different_position(3);
    }
    whole_arm_spin(1);
    claw_spin_front();
    HAL_Delay(1000);
}

/// @brief 原地放置三个物料
/// @param time_status 放置的阶段，1为第一次放置，2为第二次放置，一直到4，第四次码垛
void get_and_put_in_one_position(int time_status)
{
    // //! 先校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);

    // is_slight_spin = 1; // 使能轻微移动
    motor_state = 1;
    // tim3_count = 0;
    // while(is_slight_spin != 0 && tim3_count < timeout_limit)
    // {
    //     HAL_Delay(10);
    // }
    // if(tim3_count >= timeout_limit)
    // {
    //     HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
    //     HAL_Delay(80);
    // }
    
    // is_slight_spin = 0;
    // stop();
    if(time_status == 4)
    {
        put_claw_up_top();
    }
    else
    {
        feetech_servo_move(1,put_claw_down_ground_position-520,4095,50);
        open_claw_180();
    }
    
    HAL_Delay(1000);



    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    if(tim3_count >= timeout_limit)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
        HAL_Delay(80);
    }
    is_slight_move = 0;
    // printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();
    // HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    // HAL_Delay(80);
    if(target_colour[0] != 0 )
    {
        if(time_status == 1|| time_status == 2)
        {
            get_and_put_different_position(target_colour[0]);
        }
        else if(time_status == 3)
        {
            get_and_put_different_position(target_colour[3]);
        }
        else if(time_status == 4)
        {
            get_and_put_different_position_pileup(target_colour[3]);
        }
    }
    else
    {
        get_and_put_different_position(2);
    }
    if(target_colour[1] != 0)
    {
        if(time_status == 1|| time_status == 2)
        {
            get_and_put_different_position(target_colour[1]);
        }
        else if(time_status == 3)
        {
            get_and_put_different_position(target_colour[4]);
        }
        else if(time_status == 4)
        {
            get_and_put_different_position_pileup(target_colour[4]);
        }
    }
    else
    {
        get_and_put_different_position(3);
    }
    if(target_colour[2] != 0)
    {
        if(time_status == 1|| time_status == 2)
        {
            get_and_put_different_position(target_colour[2]);
        }
        else if(time_status == 3)
        {
            get_and_put_different_position(target_colour[5]);
        }
        else if(time_status == 4)
        {
            get_and_put_different_position_pileup(target_colour[5]);
        }
    }
    else
    {
        get_and_put_different_position(1);
    }
    whole_arm_spin(1);
    claw_spin_front();
    // arm_stretch();
    HAL_Delay(1000);
}

/// @brief 无视觉
/// @param  
void get_and_load_in_one_position_test(void)
{
    get_and_load_different_position(2);
    get_and_load_different_position(1);
    get_and_load_different_position(3);
    whole_arm_spin(1);
    claw_spin_front();
    HAL_Delay(1000);
}

/// @brief 在原地将三个物料夹取到车上
/// @param time_status
void get_and_load_in_one_position(int time_status)
{
    if (target_colour[0] != 0)
    {
        if (time_status == 1 )
        {
            get_and_load_different_position(target_colour[0]);
        }
        else if (time_status == 3)
        {
            get_and_load_different_position(target_colour[3]);
        }
    }
    else
    {
        get_and_load_different_position(2);
    }
    if (target_colour[1] != 0)
    {
        if (time_status == 1 )
        {
            get_and_load_different_position(target_colour[1]);
        }
        else if (time_status == 3)
        {
            get_and_load_different_position(target_colour[4]);
        }
    }
    else
    {
        get_and_load_different_position(3);
    }
    if (target_colour[2] != 0)
    {
        if (time_status == 1 )
        {
            get_and_load_different_position(target_colour[2]);
        }
        else if (time_status == 3)
        {
            get_and_load_different_position(target_colour[5]);
        }
    }
    else
    {
        get_and_load_different_position(1);
    }
    whole_arm_spin(1);
    claw_spin_front();
    // arm_stretch();
    HAL_Delay(1000);
}

/// @brief 移动进行放置，1为第一轮，2为第二轮
/// @param status 
void get_and_put_with_movement(int status,int is_pile_up)
{
    // state_spin(position);
    move_follow_sequence(target_colour,1,status);
    arm_stretch();
    HAL_Delay(1000);


    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);

    is_slight_spin = 1; // 直线
    motor_state = 1;
    tim3_count = 0;
    while(is_slight_spin != 0 && tim3_count < timeout_limit)
    {
        HAL_Delay(10);
    }
    if(tim3_count >= timeout_limit)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
        // HAL_Delay(80);
    }
    is_slight_spin = 0;
    stop();
    // printf("t0.txt=\"end_of_line\"\xff\xff\xff"); // 校正结束，调试用，正式比赛中须删除
    HAL_Delay(1000);

    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    if(tim3_count >= timeout_limit)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
        // HAL_Delay(80);
    }
    is_slight_move = 0;
    // printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();

    if(status == 1) // 第一轮
    {
        get_from_state(target_colour[0]);
    }
    else if(status == 2) // 第二轮
    {
        get_from_state(target_colour[3]);
    }

    if(is_pile_up == 1)
    {
        put_from_state_pileup();
    }
    else
    {
        put_from_state();
    }
    arm_stretch();

    move_follow_sequence(target_colour,2,status);
    HAL_Delay(1000);

    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    if(tim3_count >= timeout_limit)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 50); // 通知树莓派结束
        // HAL_Delay(80);
    }
    is_slight_move = 0;
    // printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();

    if(status == 1) // 第一轮
    {
        get_from_state(target_colour[1]);
    }
    else if(status == 2) // 第二轮
    {
        get_from_state(target_colour[4]);
    }
    
    if(is_pile_up == 1)
    {
        put_from_state_pileup();
    }
    else
    {
        put_from_state();
    }
    arm_stretch();

    move_follow_sequence(target_colour,3,status);
    HAL_Delay(1000);

    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
    }
    is_slight_move = 0;
    // printf("t0.txt=\"1\"\xff\xff\xff"); //校正结束，调试用，正式比赛中须删除
    stop();

    if(status == 1) // 第一轮
    {
        get_from_state(target_colour[2]);
    }
    else if(status == 2) // 第二轮
    {
        get_from_state(target_colour[5]);
    }
    
    if(is_pile_up == 1)
    {
        put_from_state_pileup();
    }
    else
    {
        put_from_state();
    }
    arm_stretch();
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    HAL_Delay(80);
    HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    HAL_Delay(80);
    HAL_UART_Transmit(&huart3, (uint8_t*)"stop", strlen("stop"), 50); // 通知树莓派结束
    HAL_Delay(80);



}


void get_and_load_with_movement(int status)
{
    move_follow_sequence(target_colour,1,status);
    arm_stretch();
    HAL_Delay(1000);

    if(status == 1) // 第一轮
    {
        get_and_load_ground(target_colour[0]);
    }
    else if(status == 2) // 第二轮
    {
        get_and_load_ground(target_colour[3]);
    }

    move_follow_sequence(target_colour,2,status);
    arm_stretch();
    HAL_Delay(1000);

    if(status == 1) // 第一轮
    {
        get_and_load_ground(target_colour[1]);
    }
    else if(status == 2) // 第二轮
    {
        get_and_load_ground(target_colour[4]);
    }

    move_follow_sequence(target_colour,3,status);
    arm_stretch();
    HAL_Delay(1000);

    if(status == 1) // 第一轮
    {
        get_and_load_ground(target_colour[2]);
    }
    else if(status == 2) // 第二轮
    {
        get_and_load_ground(target_colour[5]);
    }
    arm_stretch();
    

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
