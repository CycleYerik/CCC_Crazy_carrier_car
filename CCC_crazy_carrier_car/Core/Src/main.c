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


extern uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50],rxdata_u5[50]; // usart2,3接收缓冲区
extern uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1,received_rxdata_u5,received_rxdata_u4; // 暂存usart2,3接收到的数据单字节变量
extern uchar rxflag_u2,rxflag_u3,rxflag_u1,rxflag_u4,rxflag_u5; // usart2,3接收标志位变量
extern float Motor_Cur_Pos_1, Motor_Cur_Pos_2, Motor_Cur_Pos_3, Motor_Cur_Pos_4; // 电机当前位置
extern float  Motor_Vel_1, Motor_Vel_2, Motor_Vel_3, Motor_Vel_4; // 电机当前速度
extern float x_velocity, y_velocity; // x、y轴速度
extern float acceleration; // 加速度
extern float x_move_position, y_move_position; // x、y
extern int is_motor_start_move; 
extern int is_slight_move,motor_state,is_slight_spin;

float x_error = 0, y_error = 0; // x、y轴误差
float gyro_z = 90;

int open_loop_move_velocity = 100;

int target_colour[6] = {0}; // 目标颜色

int is_get_qrcode_target = 0;
int volatile is_start_get_plate = 0; // 开始从转盘抓
int volatile get_plate = 0; // 1 2 3 
int get_plate_count = 0;

extern volatile int test_slight_move; // 用于判断微调是否完成


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/************************************函数声明区****************************************/

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

    HAL_Delay(1000); // 等待电机初始化完成，本该是4000ms
    // 机械臂初始位置设定
    whole_arm_spin(1);
    arm_stretch();
    put_claw_up_top();
    claw_spin_front();
    open_claw();

    /*********************************测试区域开始*************************************/

    // state_spin(1); // 载物盘旋转到1号位置
    // arm_stretch();
    // get_and_load(1);
    // get_and_load(2);
    // get_and_load(3);

    // get_from_state(1);
    // put_from_state();
    // get_from_state(2);
    // put_from_state();
    // get_from_state(3);
    // put_from_state();




    // while(1)
    // {
    //     HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50);
    //     HAL_Delay(100);
    // }



    // while(1)
    // {
    //     spin_right(open_loop_move_velocity,acceleration, 90);
    //     HAL_Delay(4000);
    //     spin_left(open_loop_move_velocity,acceleration, 90);
    //     HAL_Delay(4000);
    // }

    

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

    /*-------------------小车离开起点并前往转盘-----------------------*/

    // is_slight_move = 1; //! 使能轻微移动
    printf("t0.txt=\"start\"\xff\xff\xff"); // 开始
    HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50); // 开始识别二维码

    move_all_direction_position(acceleration, open_loop_move_velocity, -15, 0);
    HAL_Delay(1200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, 145);
    HAL_Delay(8000);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)  3500

    // 将target_colour转为字符串显示在串口屏上
    char target_colour_str[6] = {0};
    for(int i = 0; i < 6; i++)
    {
        target_colour_str[i] = target_colour[i] + '0';
    }
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上
    HAL_Delay(2000);



    spin_right(open_loop_move_velocity,acceleration, 90);
    HAL_Delay(2200);

    is_start_get_plate = 1; // 开始从转盘抓取

    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); // 开始识别颜色并抓取

    while(get_plate_count < 3) 
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

    while(1) //!!!!!!!!!!!!!!!!!!!!!!!! 停止
    {
        HAL_Delay(10);
    }        //!!!!!!!!!!!!!!!!!!!!!!!! 停止

    /*------------------前往粗加工区------------------------*/

    spin_right(open_loop_move_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,45); 
    HAL_Delay(2000);
    spin_right(open_loop_move_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, 168);
    HAL_Delay(4000);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)


    //! 此处还要加入根据所识别到的顺序，移动到对应的位置

    put_claw_up(); // 放低，识别色环


    /*------------------识别色环移动并放置------------------------*/

    // 先校正车身位置
    is_slight_spin = 1; // 使能轻微移动

    //! 发给树莓派，开始校正直线

    while(is_slight_spin != 0)
    {
        HAL_Delay(10);
    }
    HAL_Delay(1000);
    is_slight_move = 1;

    while(is_slight_move != 0)
    {
        HAL_Delay(10);
    }
    HAL_Delay(1000);
    stop();
    get_from_state(target_colour[0]); // 从转盘取色环
    put_from_state();

    

    //move_all_direction_position(acceleration, 60, 0, -200); //!!!!!!待测量
    //HAL_Delay(3000);//!!!!!!待测量


    is_slight_move = 1;
    while(is_slight_move != 0)
    {
        HAL_Delay(10);
    }
    HAL_Delay(1000);
    stop();
    get_from_state(target_colour[1]); // 从转盘取色环
    put_from_state();

    //move_all_direction_position(acceleration, 60, 0, -200); //!!!!!!待测量
    //HAL_Delay(3000);//!!!!!!待测量


    is_slight_move = 1;
    while(is_slight_move != 0)
    {
        HAL_Delay(10);
    }
    HAL_Delay(1000);
    stop();
    get_from_state(target_colour[2]); // 从转盘取色环
    put_from_state();









    


 
    int test_move_stop = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


    // //! 
    // if(is_slight_move == 0 && test_move_stop == 0)
    // {
    //     stop();
    //     get_from_state(1);
    //     put_from_state();
    //     test_move_stop = 1;
    // }


    // if(is_start_get_plate == 1)
    // {
    //     if(get_plate == 1)
    //     {
    //         get_and_load(1);
    //     }
    //     else if(get_plate == 2)
    //     {
    //         get_and_load(2);
    //     }
    //     else if (get_plate == 3)
    //     {
    //         get_and_load(3);
    //     }
    //     is_start_get_plate = 0;
    //     get_plate = 0;

    // }
    HAL_Delay(10);



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
