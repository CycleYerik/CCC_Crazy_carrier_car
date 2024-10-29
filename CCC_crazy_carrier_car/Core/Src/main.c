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


extern uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50]; // usart2,3接收缓冲区
extern uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1; // 暂存usart2,3接收到的数据单字节变量
extern uchar rxflag_u2,rxflag_u3,rxflag_u1; // usart2,3接收标志位变量
extern float Motor_Cur_Pos_1, Motor_Cur_Pos_2, Motor_Cur_Pos_3, Motor_Cur_Pos_4; // 电机当前位置
extern float  Motor_Vel_1, Motor_Vel_2, Motor_Vel_3, Motor_Vel_4; // 电机当前速度
extern float x_velocity, y_velocity; // x、y轴速度
extern float acceleration; // 加速度
extern float x_move_position, y_move_position; // x、y
extern int is_motor_start_move; 

float x_error = 0, y_error = 0; // x、y轴误差

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/************************************函数声明区****************************************/

// printf重定向
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

    /*********测试*********/


    /*****************各种系统相关外设的初始化（串口、定时器等)***********************/


 
    // HAL_UART_Receive_IT(&huart2, &received_rxdata_u2, 1); // 使能串口2接收中断
    // HAL_UART_Receive_IT(&huart3, &received_rxdata_u3, 1); // 使能串口3接收中断
    // HAL_UART_Receive_IT(&huart1, &received_rxdata_u1, 1); // 使能串口1接收中断

    HAL_TIM_Base_Start_IT(&htim2); // 使能定时器2中断
    HAL_TIM_Base_Start_IT(&htim3); // 使能定时器3中断
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 开启TIM2通道1 PWM输出
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // 开启TIM2通道2 PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // 开启TIM2通道3 PWM输出

    /*********************************实际功能的初始化*************************************/


    my_servo_init(); //精密电机初始化，使用精密电机则必须加入

    HAL_Delay(2000); //! 等待电机初始化完成

    /**************************************以下为主程序流程代码********************************************/

    /*-------------------小车离开起点-----------------------*/

    
    /*-----------------------前进到扫码点，扫码并显示在屏幕上------------------------------*/


        



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        /****************************************以下皆为各种测试******************************************/
        // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 117);
        // HAL_Delay(3000);
        // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 50);
        // HAL_Delay(3000);

        // 在比赛中整个的代码是一个单次的，所以不会进入循环，这里主要是一些测试代码

        // move_all_direction_position(20, 20, 0, 1500); // 位置控制
        // HAL_Delay(4000);

        // move_all_direction_position(20, 20, 1500, 0); // 位置控制
        // HAL_Delay(4000);

        // move_all_direction_position(20, 20, 0, -1500); // 位置控制
        // HAL_Delay(4000);

        // move_all_direction_position(20, 20, -1500, 0); // 位置控制
        // HAL_Delay(4000);

        // spin_left(100, 10, 90); // 左转
        // HAL_Delay(3000);
        // spin_right(100, 10, 90); // 右转
        // HAL_Delay(3000);

        // HAL_Delay(6000);



        // printf("t2.txt=\"M1: %.2f\"\xff\xff\xff", Motor_Vel_1);
        // printf("t3.txt=\"M2: %.2f\"\xff\xff\xff", Motor_Vel_2);
        // printf("t4.txt=\"M3: %.2f\"\xff\xff\xff", Motor_Vel_3);
        // printf("t5.txt=\"M4: %.2f\"\xff\xff\xff", Motor_Vel_4);
        // printf("t1.txt=\"x%.2f\"\xff\xff\xff", x_velocity);
        // printf("t0.txt=\"y%.2f\"\xff\xff\xff", y_velocity);



        // 每次读取电机速度就进行一次速度的调整，所以在主程序中要不断读取，然后修改目标速度，并将PID计算后的速度发送给电机，而其余的控制代码主要是修改目标速度
        // Emm_V5_Read_Sys_Params(1, S_VEL); 
        // HAL_Delay(10);
        // UART_handle_function_1();
        // Emm_V5_Read_Sys_Params(2, S_VEL);
        // HAL_Delay(10);
        // UART_handle_function_1();
        // Emm_V5_Read_Sys_Params(3, S_VEL);
        // HAL_Delay(10);
        // UART_handle_function_1();
        // Emm_V5_Read_Sys_Params(4, S_VEL);
        // HAL_Delay(10);
        // UART_handle_function_1();
        // // HAL_Delay(20);
        // move_all_direction_pid(1, x_velocity, y_velocity);
        // Forward_move(100, 10, 150); // 前进
        // HAL_Delay(5000);

        
        
        // move_left(100, 10, 200); // 左走
        // HAL_Delay(5000);
        // Backward_move(100, 10, 150); // 后退
        // HAL_Delay(5000);
        // move_right(100, 10, 200); // 右走
        // HAL_Delay(8000);
        // Forward_move(100, 10, 150); // 前进
        // HAL_Delay(150 / wheel_circumference / 100.0 / 60.0);
        // spin_left(30, 10, 90); // 左转
        // HAL_Delay(4000);
        // Forward_move(100, 10, 150); // 前进
        // HAL_Delay(4000);
        // spin_left(30, 10, 90); // 左转
        // HAL_Delay(4000);
        // Forward_move(100, 10, 150); // 前进
        // HAL_Delay(4000);

        // spin_right(30, 10, 270); // 右转
        // HAL_Delay(4000);
        // Forward_move(100, 10, 150); // 前进
        // HAL_Delay(4000);
        // spin_left(30, 10, 90); // 左转
        // HAL_Delay(8000);
        // HAL_Delay(100);

        

        // UART_handle_function_3();
        // HAL_Delay(100);

        // if(is_motor_start_move == 0)
        // {
        //     is_motor_start_move = 1;
        // }
        // HAL_Delay(2000);

        // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 250);
        // HAL_Delay(3000);
        // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 50);
        // HAL_Delay(3000);
        // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 250);
        // HAL_Delay(3000);

        // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 83);
        // HAL_Delay(3000);



        // WritePosEx(1, 4095,2250, 50);//舵机(ID1),以最高速度V=2250步/秒,加速度A=50(50*100步/秒^2),运行至P1=4095
        // HAL_Delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	
        // WritePosEx(1, 0, 2250, 50);//舵机(ID1),以最高速度V=2250步/秒,加速度A=50(50*100步/秒^2),运行至P1=0
        // HAL_Delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000

        // WritePosEx(0xfe, 4095, 2250, 50);//舵机(广播),以最高速度V=2250步/秒,加速度A=50(50*100步/秒^2),运行至P1=4095
        // HAL_Delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000
	
        // WritePosEx(0xfe, 0, 2250, 50);//舵机(广播),以最高速度V=2250步/秒,加速度A=50(50*100步/秒^2),运行至P1=0
        // HAL_Delay(2270);//[(P1-P0)/V]*1000+[V/(A*100)]*1000

        // feetech_servo_move(1, 1000, 2250, 50);
        // feetech_servo_move(1,0,2250,50);
        // HAL_Delay(3000);
        // printf("t0.txt=\"y%.2f\"\xff\xff\xff", y_velocity);
        // HAL_Delay(1000);


        state_spin(1);
        HAL_Delay(2000);
        state_spin(2);
        HAL_Delay(2000);
        state_spin(3);
        HAL_Delay(2000);


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
