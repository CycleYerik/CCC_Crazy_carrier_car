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
extern float acceleration_spin;
extern float x_move_position, y_move_position; // x、y
extern int is_motor_start_move; 
extern int is_slight_move,motor_state,is_slight_spin, is_slight_spin_and_move; // 微调底盘所用的标志位变量

extern volatile int x_camera_error, y_camera_error; // 视觉闭环微调时的x、y偏差值
int is_find_circle = 0;

int temp_plate=0;

float volatile gyro_z = 90;

/// @brief 底盘电机移动相关的速度
int open_loop_x_move_velocity = 120; //100
int open_loop_move_velocity = 180; //200
int open_loop_spin_velocity = 150; //150 

// 目标颜色数组
volatile int target_colour[6] = {1,2,3,1,2,3}; 
int move_sequence_bias = 0; // 根2不同顺序移动带来的位置相对色环位置的偏差，如中-左-右，则偏差为0、-x、+x 

/// @brief 用于判断当前是第几个case,
int case_count = 0; 
int timeout_limit = 1500; // 超时时间限制，单位10ms
extern int tim3_count;

int is_get_qrcode_target = 0; //!!!!!!
int volatile is_start_get_plate = 0; // 开始从转盘抓
int volatile get_plate = 0; // 1 2 3 
int is_adjust_plate_servo = 0; // 根据视觉定位在转盘处实现移动机械臂抓取物料
int is_adjust_plate_first = 0;
int get_plate_count = 0;

extern volatile int x_plate_error , y_plate_error ;

int is_adjust_motor_in_tim = 1; // 如果为1，则在定时器中进行电机调整，否则在main的while中进行电机调整
extern int acceleration_adjust;

float now_spin_which_direction = 0;

int is_start_judge_move_before_slight_adjust=0; // 是否开始判断在微调前是否需要移动
int is_move_before_slight_adjust=0 ; // 在微调前是否需要移动
int x_move_before_slight_move=0 ;

int is_1_get = 0, is_2_get = 0, is_3_get = 0;
int is_get_empty = 0,start_judge_empty = 0;
int is_get_empty_finish = 0; // 空抓判断完成

extern int middle_arm,stretch_camera;

int seeking_for_circle = 0; // 当还没看到完整色环时置1
int is_servo_adjust= 0; // 当在视觉闭环微调舵机时置1

extern volatile int  r_servo_now ; // 机械臂伸缩舵机的位置
extern volatile int  theta_servo_now ; // 机械臂中板旋转舵机的位置

extern int left_2, left_3, left_4;
extern int middle_2, middle_3, middle_4;
extern int right_2, right_3, right_4;

extern volatile int test_slight_move; // 用于判断微调是否完成
extern int spin_which_direction;
extern int put_claw_down_ground_position;

extern float angle_motor_1,angle_motor_2,angle_motor_3,angle_motor_4;
extern float motor_vel_1,motor_vel_2,motor_vel_3,motor_vel_4;

int servo_adjust_status = 5;

// 飞特舵机相关加速度
int acc_front_start = 200,acc_front_stop = 200;
int acc_x_same_start = 150,acc_x_same_stop = 150;
int acc_spin_start = 130,acc_spin_stop = 130;

float velocity_front_y42 = 120,velocity_x_y42 = 80,velocity_spin_y42 = 80; // 废弃

int is_get_massage = 0;

int x_plate_error_with_put = 0, y_plate_error_with_put = 0;
int is_adjust_plate_with_put = 0;
int is_plate_with_put_ok_1 = 0; //红，树莓派认为到位时置1
int is_plate_with_put_ok_2 = 0; //绿，树莓派认为到位时置1
int is_plate_with_put_ok_3 = 0; //蓝，树莓派认为到位时置1
int is_get_plate_put_1 = 0,is_get_plate_put_2 = 0,is_get_plate_put_3 = 0;

int temp_r_servo_position =0;
int temp_theta_servo_position = 0;
int adjust_plate_with_put_count = 0;

int is_plate_first_move = 0; // 在将物料放置在圆盘带有色环时是否开始移动
int is_plate_move_adjust = 0; // 在将物料放置在圆盘带有色环时是否开始调整
int put_plate_count = 0; // 将物料放置在圆盘带有色环时的计数
int is_third_preput = 0; // 是否可以进入第三次放置
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
void all_process_main(void);
void move_follow_sequence(int target_colour_input[6], int case_count_input, int status);
void start_and_come_to_turntable(void);
void come_to_raw_processing_area(void);
void come_to_temporary_area(void);
void come_to_temporary_area_v2(void);
void come_to_turntable_from_temparea(void);
void come_back_to_start_from_temparea(void);
void come_back_to_start_from_temparea_v2(void);

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

    /**********************************main函数说明****************************************/
    //!  本程序中包含各种测试流程和比赛流程，因为调试阶段各种动作和指令的耦合性比较强，故放弃了一些常用动作流程的封装，所以整体较为混乱
    //!  主要的程序即全流程代码
    //! 
    //! 
    //! 

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
    //! 各种需要调的参数及位置
    //TODO
    /**
     * main.c
     * 
     * timeout_limit：超时时间限制，单位10ms
     * 
     * 
     * 
     * my_usart.c
     * 
     * 0.1 视觉圆环识别到的x、y偏差值所乘的系数，用于底盘微调
     * Kp_slight_move、Ki_slight_move、Kd_slight_move：底盘微调的PID参数
     * 
     * x_plate_error、y_plate_error *= 2/7 ：转盘调整的x、y偏差值的系数
     * 
     * 
     * my_servo.c
     * 
     * 机械臂位置的big、mid、small阈值和对应的调整步进
     * 
     * 
     * motor.c
     * position_move_velocity
     * spin_move_velocity
     * 
     * 
     */



    /*****************各种系统相关外设的初始化（串口、定时器等)***********************/
    

 
    HAL_UART_Receive_IT(&huart3, &received_rxdata_u3, 1); // 使能串口3接收中断
    // HAL_UART_Receive_IT(&huart1, &received_rxdata_u1, 1); // 使能串口1接收中断
    // HAL_UART_Receive_IT(&huart4, &received_rxdata_u4, 1); // 使能串口4接收中断 //TODO 此处开启后造成串口接收消息出现问题
    HAL_UART_Receive_IT(&huart5, &received_rxdata_u5, 1); // 使能串口5接收中断
    // HAL_UART_Receive_IT(&huart2, &received_rxdata_u2, 1); // 使能串口2接收中断


    HAL_TIM_Base_Start_IT(&htim2); // 使能定时器2中断
    HAL_TIM_Base_Start_IT(&htim3); // 使能定时器3中断
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // 开启TIM1通道1 PWM输出
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // 开启TIM1通道2 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 开启TIM1通道3 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 开启TIM1通道4 PWM输出  
	HAL_Delay(1000); //TODO 比赛时考虑去除，增加启动速度
    my_servo_init(); //!精密舵机初始化，使用精密舵机则必须加入


    /*****************初始化动作姿态***********************/
    HAL_Delay(100);
    arm_stretch();
    whole_arm_spin(1); // 中板旋转到中间位置
    put_claw_up_top(); 
    claw_spin_front();
    open_claw_180();
    state_spin_without_claw(1);
    HAL_Delay(900); // TODO等待电机初始化完成，本该是4000ms,目前暂时减少时间

    /*****************单独调试程序***********************/


    //? 抓福建三物料
    // get_and_pre_put(2,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // HAL_Delay(1000);
    // get_and_load_openloop(2);
    // while(1)
    // {
    //     HAL_Delay(100);
    // }

    //? 抓经典物料底下
    // 400 载物盘   2600 near ground  
    // 300    617      2702
    // get_and_load_openloop_v2(1,2600,400);
    // get_and_load_openloop_v2(3,2600,400);
    // get_and_load_openloop_v2(2,2600,400);
    // HAL_Delay(1000);
    // get_and_pre_put_v2(3,2600,400,2600,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // get_and_pre_put_v2(1,2600,400,2600,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // get_and_pre_put_v2(2,2600,400,2600,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // while(1)
    // {
    //     HAL_Delay(100);
    // }

    //? 抓国赛物料
    // HAL_Delay(2000);
    // get_and_load_openloop_v3(2,2750,290,520);
    // get_and_load_openloop_v3(3,2600,617,300);
    // get_and_load_openloop_v3(2,2600,617,300);
    // HAL_Delay(1000);
    // get_and_pre_put_v3(2,2750,290,520,2700,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // get_and_pre_put_v2(1,2600,617,300,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // get_and_pre_put_v2(2,2600,617,300,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // while(1)
    // {
    //     HAL_Delay(100);
    // }


    //? 抓福建三物料
    // get_and_pre_put(target_colour[0], 0);
    // HAL_Delay(1000);
    // open_claw_180();
    // HAL_Delay(1000);
    // get_and_load_openloop(2);
    // while(1)
    // {
    //     HAL_Delay(100);
    // }


    // HAL_Delay(2000); 



    //? 将物料放置在转盘圆环上（测试）
    // HAL_UART_Transmit(&huart3, (uint8_t*)"LL", strlen("LL"), 50);
    // // put_claw_down();
    // for(int i = 0; i < 2; i++) //前两次可以放置
    // {
    //     // is_adjust_plate_servo = 1;
    //     get_and_pre_put_spin_plate(target_colour[i]);
    //     servo_adjust_status = target_colour[i];
    //     is_servo_adjust = 1;
    //     // x_camera_error = 0;
    //     // y_camera_error = 0;
    //     if(i == 0)
    //     {
    //         is_plate_first_move = 2; // 来的第一个圆
    //         is_adjust_plate_servo = 1;
    //         while(is_plate_first_move != 1)
    //         {
    //             HAL_Delay(50);
    //         }
    //         HAL_Delay(50);
    //         is_plate_first_move = 0;// 开始夹着物料准备调整
    //         adjust_plate(x_plate_error, y_plate_error);
    //         x_plate_error = 0;
    //         y_plate_error = 0;
    //     }
    //     // if(i == 0)
    //     // {
    //     //     adjust_plate(x_plate_error, y_plate_error);
    //     //     x_plate_error = 0;
    //     //     y_plate_error = 0;
    //     // }
    //     // HAL_Delay(200);
    //     is_adjust_plate_servo = 0;
    //     while(is_servo_adjust != 0)
    //     {
    //         adjust_position_with_camera(x_camera_error,y_camera_error,0);
    //         HAL_Delay(50);
    //     }
    //     is_servo_adjust = 0;
    //     put_claw_down();
    //     HAL_Delay(300);
    //     open_claw_180();
    //     HAL_Delay(300);
    // }
    // while(is_third_preput != 1)
    // {
    //     HAL_Delay(50);
    // }
    // is_third_preput = 2;
    // get_and_pre_put_spin_plate(target_colour[2]);
    // servo_adjust_status = target_colour[2];
    // is_servo_adjust = 1;
    // x_camera_error = 0;
    // y_camera_error = 0;
    // while(is_servo_adjust != 0)
    // {
    //     adjust_position_with_camera(x_camera_error,y_camera_error,0);
    //     HAL_Delay(50);
    // }
    // is_servo_adjust = 0;
    //     put_claw_down();
    //     HAL_Delay(300);
    //     open_claw_180();
    //     HAL_Delay(300);

    // while(1)
    // {
    //     HAL_Delay(100);
    // }





    //? 将物料放置在转盘上（测试）
    // HAL_UART_Transmit(&huart3, (uint8_t*)"HH", strlen("HH"), 50); 
    // put_claw_down();
    // HAL_Delay(100);

    // is_adjust_plate_with_put = 1;
    // while(is_adjust_plate_with_put != 0)
    // {
    //     adjust_position_with_camera(x_plate_error_with_put,y_plate_error_with_put,0);
    //     x_plate_error_with_put = 0;
    //     y_plate_error_with_put = 0;
    //     temp_r_servo_position = r_servo_now;
    //     temp_theta_servo_position = theta_servo_now;
    //     HAL_Delay(50);
    // }
    // is_adjust_plate_with_put = 2;
    // printf("t0.txt=\"ok\"\xff\xff\xff");
    // while(adjust_plate_with_put_count < 3)
    // {
    //     if((is_plate_with_put_ok_1 == 1 && is_get_plate_put_1 == 0) || (is_plate_with_put_ok_2 == 1 && is_get_plate_put_2 == 0)||( is_plate_with_put_ok_3 == 1 && is_get_plate_put_3 == 0))
    //     {
    //         adjust_plate_with_put_count++;
    //         if(is_plate_with_put_ok_1 == 1)
    //         {
    //             state_spin_without_claw(1);
    //             is_get_plate_put_1 = 1;
    //         }
    //         else if(is_plate_with_put_ok_2 == 1)
    //         {
    //             state_spin_without_claw(2);
    //             is_get_plate_put_2 = 1;
    //         }
    //         else if(is_plate_with_put_ok_3 == 1)
    //         {
    //             state_spin_without_claw(3);
    //             is_get_plate_put_3 = 1;
    //         }
    //         is_plate_with_put_ok_1 = 0;
    //         is_plate_with_put_ok_2 = 0;
    //         is_plate_with_put_ok_3 = 0;
    //         put_claw_up_top();
    //         arm_shrink();
    //         HAL_Delay(200);
    //         claw_spin_state();
    //         HAL_Delay(300);
    //         put_claw_down_state();
    //         HAL_Delay(300); 
    //         close_claw();
    //         HAL_Delay(300);
    //         put_claw_up_top();
    //         HAL_Delay(300); 
    //         claw_spin_front(); 
    //         feetech_servo_move(4,temp_r_servo_position,4000,180);
    //         feetech_servo_move(3,temp_theta_servo_position,4000,180);
    //         HAL_Delay(200);
    //         put_claw_down();
    //         HAL_Delay(600);
    //         open_claw_180();
    //         HAL_Delay(300);
    //         // put_claw_up_top();
    //         // HAL_Delay(500);
    //     }
    // }

    // while(1)
    // {
    //     HAL_Delay(100);
    // }






    //?圆台
    // int tai_ground = 2850;
    // feetech_servo_move(1,tai_ground,4095,240);
    // // put_claw_down_ground();
    // HAL_Delay(1000);
    // while(1)
    // {
		
        
    //     close_claw();
    //     HAL_Delay(3000);
    //     // feetech_servo_move(1,2600,4095,240);
    //     put_claw_up_top();
    //     HAL_Delay(1500);
    //     claw_spin_state();
    //     HAL_Delay(1500);
    //     claw_spin_front();
    //     HAL_Delay(1500);
    //     // claw_spin_state();
    //     // HAL_Delay(1500);
    //     // claw_spin_front();
    //     // HAL_Delay(1500);
    //     // claw_spin_state();
    //     // HAL_Delay(1500);
    //     // claw_spin_front();
    //     // HAL_Delay(1500);
    //     feetech_servo_move(1,tai_ground,4095,240);
    //     // put_claw_down_ground();
    //     HAL_Delay(3000);
    //     open_claw();
    //     HAL_Delay(3000);
    // }
    // while(1)
    // {
    //     feetech_servo_move(1,tai_ground,4095,240);
    //     state_spin_without_claw(1);
    //     HAL_Delay(1000);
    //     close_claw();
    //     HAL_Delay(1500);
    //     arm_shrink();
    //     put_claw_up_top();
    //     HAL_Delay(1000);
    //     claw_spin_state();
    //     HAL_Delay(1000);
    //     // feetech_servo_move(1,566,4095,240);
    //     HAL_Delay(1000);
    //     open_claw();
    //     HAL_Delay(1000);
    //     state_spin_without_claw(2);
    //     HAL_Delay(1000);
    //     close_claw();
    //     HAL_Delay(1500);
    //     put_claw_up_top();
    //     HAL_Delay(500);
    //     claw_spin_front();
    //     arm_stretch();
    //     HAL_Delay(1000);
    //     feetech_servo_move(1,tai_ground,4095,240);
    //     HAL_Delay(2000);
    //     open_claw();
    //     HAL_Delay(3000);

    // }


    //? 福建省物料抓取测试
    // while(1)
    // {
    //     put_claw_down_ground();
    // HAL_Delay(1000);
    // close_claw();
    // HAL_Delay(1500);
    // put_claw_up_top();
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // open_claw();
    // HAL_Delay(1000);

    // }

    

    //? 开机自启动测试程序
    // HAL_UART_Transmit(&huart3, (uint8_t*)"ZZ", strlen("ZZ"), 50); // 通知树莓派开始
    // while(is_get_massage != 1)
    // {
    //     HAL_Delay(100);
    // }
    // move_all_direction_position(100,100,0,5);


    //? 糖葫芦物料测试
    // while(1)
    // {
    //     HAL_Delay(100);
    // }
    // int state_tanghulu_1 = 45;
    // int state_tanghulu_2 = 89;
    // int state_tanghulu_3 = 134;
    // put_claw_down_ground();
    // HAL_Delay(1000);
    // close_claw();
    // HAL_Delay(500);
    // put_claw_up_top();
    // HAL_Delay(1000);
    // feetech_servo_move(4,3690,4095,180);
    // HAL_Delay(500);
    // feetech_servo_move(2,1600,4000,180);
    // HAL_Delay(1000);

    // feetech_servo_move(1,806,1000,180);
    // HAL_Delay(1000);
    // open_claw();
    // HAL_Delay(1000);
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, state_tanghulu_1);
    // feetech_servo_move(4,4050,1000,180);
    // HAL_Delay(2000);
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 108);
    // feetech_servo_move(2,3329,1000,180);
    // HAL_Delay(2000);

    // feetech_servo_move(2,1600,1000,180);
    // HAL_Delay(1500);
    // state_spin_without_claw(2);
    // HAL_Delay(100);
    // __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 98);
    // HAL_Delay(2000);
    // close_claw();
    // HAL_Delay(500);
    // put_claw_up_top();
    // HAL_Delay(500);
    // claw_spin_front();
    // put_claw_down_ground();
    // HAL_Delay(2000);
    // open_claw();
    // while(1)
    // {
    //     HAL_Delay(1000);
    // }

    //? 摄像头偏差值调整
    // close_claw();
    // put_claw_down_near_ground();
    // HAL_Delay(4000);
    // open_claw_180();
    // put_claw_up();
    // HAL_Delay(1000);
    // while(1)
    // {
    //     HAL_Delay(1000);
    // }

    // //? 最新色环定位和放置
    // HAL_Delay(1000);
    // put_claw_up();
    // HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); 
    // motor_state = 1;
    // is_slight_spin_and_move = 1;
    // tim3_count = 0;
    // while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    // {
    //     slight_spin_and_move(); // 直线和圆环一起调整
    //     HAL_Delay(50);
    // }
    // is_slight_spin_and_move = 0;
    // stop();
    // HAL_Delay(50);
    // for (int i = 0; i < 3; i++)
    // {
        
    //     get_and_pre_put(target_colour[i], 0);
    //     servo_adjust_status = target_colour[i];
    //     is_servo_adjust = 1;
    //     tim3_count = 0;
    //     HAL_Delay(700);
    //     x_camera_error = 0;
    //     y_camera_error = 0;
    //     while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
    //     {
    //         adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 这里的1是否有必要
    //         //TODO 加入变量的互斥锁机制
    //         HAL_Delay(50);  //100
    //     }
    //     // HAL_UART_Transmit(&huart3, (uint8_t*)"adjust_end", strlen("adjust_end"), 50); // 通知树莓派结束
    //     // if (tim3_count >= timeout_limit)
    //     // {
    //     //     HAL_Delay(20);
    //     // }
    //     is_servo_adjust = 0;
    //     put_claw_down_ground();
    //     HAL_Delay(500);
    //     open_claw();
    //     HAL_Delay(500);
    // }
    // put_claw_up_top();
    // HAL_Delay(500);
    // open_claw();
    // while(1)
    // {
    //     HAL_Delay(1000);
    // }



    //? 最新的转盘
    // HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    // HAL_Delay(100);
    // put_claw_down();
    // is_start_get_plate = 1;
    // while(get_plate_count < 3 ) // 从转盘抓取三个色环或者超时
    // {
    //     // HAL_UART_Transmit(&huart3, (uint8_t*)&get_plate, 1, 50);
    //     is_adjust_plate_servo = 1;
    //     HAL_Delay(10);
    //     // while(is_adjust_plate_servo != 0)
    //     // {
    //     //     HAL_Delay(10);
    //     // }
    //     int temp_plate=0;
    //     if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0))
    //     {
    //         is_adjust_plate_servo = 0;
    //         //TODO 第一次抓取前移动底盘，然后稍微延时一下
    //         temp_plate = get_plate;
    //         get_plate = 0;
    //         HAL_Delay(50);
    //         adjust_plate(x_plate_error, y_plate_error);
    //         x_plate_error = 0;
    //         y_plate_error = 0;


    //         state_spin_without_claw(temp_plate);
    //         close_claw();
    //         if(is_get_empty_finish == 0)
    //         {
    //             start_judge_empty = 1;
    //         }
    //         HAL_Delay(400);
    //         put_claw_up_top();
    //         HAL_Delay(10); //delate
    //         if(is_get_empty_finish == 0)
    //         {
    //             HAL_Delay(1500);
    //         }
    //         start_judge_empty = 0;
    //         if(is_get_empty == 1)
    //         {
    //             open_claw_180();
    //             put_claw_down();
    //             get_plate = 0;
    //         }
    //         else
    //         {
    //             int r_servo_now_temp = r_servo_now;
    //             is_get_empty_finish = 1;
    //             arm_shrink();
    //             HAL_Delay(300);
    //             claw_spin_state_without_claw();
    //             HAL_Delay(700);
    //             open_claw();
    //             HAL_Delay(300);
    //             // arm_stretch();
    //             r_servo_now = r_servo_now_temp;
    //             x_plate_error = 0;
    //             y_plate_error = 0;
    //             adjust_plate(x_plate_error, y_plate_error);
    //             x_plate_error = 0;
    //             y_plate_error = 0;
    //             claw_spin_front();
    //             open_claw_180();
    //             HAL_Delay(500);
    //             get_plate_count++;
    //             // if(temp_plate == 1)
    //             // {
    //             //     HAL_UART_Transmit(&huart3, (uint8_t*)"red", strlen("red"), 50); 
    //             //     is_1_get = 1;
    //             // }
    //             // else if(temp_plate == 2)
    //             // {
    //             //     HAL_UART_Transmit(&huart3, (uint8_t*)"green", strlen("green"), 50);
    //             //     is_2_get = 1;
    //             // }
    //             // else if(temp_plate == 3)
    //             // {
    //             //     HAL_UART_Transmit(&huart3, (uint8_t*)"blue", strlen("blue"), 50); 
    //             //     is_3_get = 1;
    //             // }
    //             get_plate = 0;
    //             put_claw_down();
    //         }
    //         is_get_empty = 0;
    //     }
    //     // get_plate = 0; //TODO ？
    //     HAL_Delay(10);
    // }
    // get_plate_count = 0;
    // is_1_get = 0;
    // is_2_get = 0;
    // is_3_get = 0;
    // is_get_empty_finish = 0;
    // is_get_empty = 0;
    // get_plate = 0;
    // while(1)
    // {
    //     HAL_Delay(1000);
    // }

    //? 旧版本转盘
    // HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    // HAL_Delay(100);
    // put_claw_down();
    // is_start_get_plate = 1;
    // while(get_plate_count < 3 ) // 从转盘抓取三个色环或者超时
    // {
    //     // HAL_UART_Transmit(&huart3, (uint8_t*)&get_plate, 1, 50);
    //     is_adjust_plate_servo = 1;
    //     HAL_Delay(10);
    //     // while(is_adjust_plate_servo != 0)
    //     // {
    //     //     HAL_Delay(10);
    //     // }
    //     int temp_plate=0;
    //     if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0))
    //     {
    //         is_adjust_plate_servo = 0;
    //         //TODO 第一次抓取前移动底盘，然后稍微延时一下
    //         temp_plate = get_plate;
    //         get_plate = 0;
    //         HAL_Delay(50);
    //         adjust_plate(x_plate_error, y_plate_error);
    //         x_plate_error = 0;
    //         y_plate_error = 0;


    //         state_spin_without_claw(temp_plate);
    //         close_claw();
    //         if(is_get_empty_finish == 0)
    //         {
    //             start_judge_empty = 1;
    //         }
    //         HAL_Delay(400);
    //         put_claw_up_top();
    //         HAL_Delay(10); //delate
    //         if(is_get_empty_finish == 0)
    //         {
    //             HAL_Delay(1500);
    //         }
    //         start_judge_empty = 0;
    //         if(is_get_empty == 1)
    //         {
    //             open_claw_180();
    //             put_claw_down();
    //             get_plate = 0;
    //         }
    //         else
    //         {
    //             int r_servo_now_temp = r_servo_now;
    //             is_get_empty_finish = 1;
    //             arm_shrink();
    //             HAL_Delay(300);
    //             claw_spin_state_without_claw();
    //             HAL_Delay(700);
    //             open_claw();
    //             HAL_Delay(300);
    //             // arm_stretch();
    //             r_servo_now = r_servo_now_temp;
    //             x_plate_error = 0;
    //             y_plate_error = 0;
    //             adjust_plate(x_plate_error, y_plate_error);
    //             x_plate_error = 0;
    //             y_plate_error = 0;
    //             claw_spin_front();
    //             open_claw_180();
    //             HAL_Delay(500);
    //             get_plate_count++;
    //             // if(temp_plate == 1)
    //             // {
    //             //     HAL_UART_Transmit(&huart3, (uint8_t*)"red", strlen("red"), 50); 
    //             //     is_1_get = 1;
    //             // }
    //             // else if(temp_plate == 2)
    //             // {
    //             //     HAL_UART_Transmit(&huart3, (uint8_t*)"green", strlen("green"), 50);
    //             //     is_2_get = 1;
    //             // }
    //             // else if(temp_plate == 3)
    //             // {
    //             //     HAL_UART_Transmit(&huart3, (uint8_t*)"blue", strlen("blue"), 50); 
    //             //     is_3_get = 1;
    //             // }
    //             get_plate = 0;
    //             put_claw_down();
    //         }
    //         is_get_empty = 0;
    //     }
    //     // get_plate = 0; //TODO ？
    //     HAL_Delay(10);
    // }
    // whole_arm_spin(1); 
    // arm_stretch();
    // put_claw_up();

    // while(1)
    // {
    //     HAL_Delay(1000);
    // }



    /***********************比赛初赛所用的全流程***********************/

    //! 全流程测试（2.28版本，更新新的色环放置，更新了最新的转盘放置）
    //! 全流程测试（2.28版本，更新新的色环放置，更新了最新的转盘放置）
    //! 全流程测试（2.28版本，更新新的色环放置，更新了最新的转盘放置）
    is_adjust_motor_in_tim = 0;
    HAL_Delay(1000); // 等待电机初始化
    HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50);  // 开始识别二维码
    int start_move_x = 16; //TODO 根据二维码的大小和车身位置调整
    int start_move_y = 15;
    int move_to_qrcode = 45;
    int move_from_qrcode_to_table = 83;
    int spin_right_angle = 90;
    int little_back_1 = 2;
    move_all_direction_position(acceleration, open_loop_move_velocity, -start_move_x , start_move_y); 
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_to_qrcode); 
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table); 
    HAL_Delay(2000);
    char* target_colour_str = (char*)malloc(6);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); 
    spin_right(open_loop_spin_velocity,acceleration_spin, spin_right_angle);
    free(target_colour_str);

    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    HAL_Delay(1000);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, -little_back_1);
    put_claw_down();
    is_start_get_plate = 1;
    while(get_plate_count < 3 ) //TODO 从转盘抓取三个色环或者超时，如果empty抓空，是否能给一个延时后直接离开
    {
        // HAL_UART_Transmi t(&huart3, (uint8_t*)&get_plate, 1, 50);
        is_adjust_plate_servo = 1;
        HAL_Delay(10);
        int temp_plate=0;
        if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0))
        {
            is_adjust_plate_servo = 0;
            //TODO 第一次抓取前移动底盘，然后稍微延时一下
            temp_plate = get_plate;
            get_plate = 0;
            HAL_Delay(50);
            adjust_plate(x_plate_error, y_plate_error);
            x_plate_error = 0;
            y_plate_error = 0;


            state_spin_without_claw(temp_plate);
            close_claw();
            if(is_get_empty_finish == 0)
            {
                start_judge_empty = 1;
            }
            HAL_Delay(400);
            put_claw_up_top();
            HAL_Delay(10); //delate
            if(is_get_empty_finish == 0)
            {
                HAL_Delay(1500);
            }
            start_judge_empty = 0;
            if(is_get_empty == 1)
            {
                open_claw_180();
                put_claw_down();
                get_plate = 0;
            }
            else
            {
                int r_servo_now_temp = r_servo_now;
                is_get_empty_finish = 1;
                arm_shrink();
                HAL_Delay(300);
                claw_spin_state_without_claw();
                HAL_Delay(700);
                open_claw();
                HAL_Delay(300);
                // arm_stretch();
                r_servo_now = r_servo_now_temp;
                x_plate_error = 0;
                y_plate_error = 0;
                adjust_plate(x_plate_error, y_plate_error);
                x_plate_error = 0;
                y_plate_error = 0;
                claw_spin_front();
                open_claw_180();
                HAL_Delay(500);
                get_plate_count++;
                get_plate = 0;
                put_claw_down();
            }
            is_get_empty = 0;
        }
        // get_plate = 0; //TODO ？
        HAL_Delay(10);
    }
    // 转盘相关的标志位置零，准备下一次使用
    get_plate_count = 0;
    is_1_get = 0;
    is_2_get = 0;
    is_3_get = 0;
    is_get_empty_finish = 0;
    is_get_empty = 0;
    get_plate = 0;

    /**************第一次从转盘前往粗加工区并放置*****************/
    put_claw_up();
    whole_arm_spin(1); 
    arm_stretch();
    int move_right_length_1 = 41; 
    int move_front_length_1 = 168;  
    move_all_direction_position(acceleration, open_loop_x_move_velocity, move_right_length_1,0);
    HAL_Delay(900);
    HAL_UART_Transmit(&huart3, (uint8_t*)"EE", strlen("EE"), 50); //在转盘旁的直线处进行姿态的校正
    HAL_Delay(200);
    is_slight_spin_and_move =1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    {
        slight_spin_and_move(); //在转盘旁的直线处进行姿态的校正
        HAL_Delay(50);
    }
    stop();
    HAL_Delay(50);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_1);
    HAL_Delay(3000);
    spin_right_180(open_loop_spin_velocity,acceleration_spin);
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); 
    HAL_Delay(2000);

    //到达粗加工区，开始校正车身位置
    motor_state = 1;
    is_slight_spin_and_move = 1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    {
        slight_spin_and_move(); // 直线和圆环一起调整
        HAL_Delay(50);
    }
    is_slight_spin_and_move = 0;
    stop();
    HAL_Delay(50);
    for (int i = 0; i < 3; i++)
    {
        
        get_and_pre_put(target_colour[i], 0);
        servo_adjust_status = target_colour[i];
        is_servo_adjust = 1;
        tim3_count = 0;
        HAL_Delay(700);
        x_camera_error = 0;
        y_camera_error = 0;
        while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
        // while (is_servo_adjust != 0 ) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(50);  //30
        }
        // HAL_UART_Transmit(&huart3, (uint8_t*)"adjust_end", strlen("adjust_end"), 50); // 通知树莓派结束
        // if (tim3_count >= timeout_limit)
        // {
        //     HAL_Delay(20);
        // }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up_top();
    HAL_Delay(500);
    open_claw();
    for(int i = 0; i < 3; i++)
    {
        get_and_load_openloop(target_colour[i]); // 开环抓取
    }
    whole_arm_spin(1);
    open_claw_180();
    arm_stretch();
    HAL_Delay(10);


//     // HAL_Delay(3000); //? 延时
    /**************第一次从粗加工区前往暂存区并放置*****************/
    int move_front_length_2 = 81; // 82
    int move_back_length_2 = 86; //85
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_2);
    HAL_Delay(2000);
    put_claw_up();
    arm_stretch();
    spin_right(open_loop_spin_velocity,acceleration_spin, 90);//! magic number
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_2 );
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); 
    HAL_Delay(1800);
    put_claw_up();
    is_slight_spin_and_move = 1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    {
        slight_spin_and_move(); // 直线和圆环一起调整
        HAL_Delay(50);
    }
    is_slight_spin_and_move = 0;
    stop();
    HAL_Delay(50);
    for (int i = 0; i < 3; i++)
    {
        
        get_and_pre_put(target_colour[i], 0);
        servo_adjust_status = target_colour[i];
        is_servo_adjust = 1;
        tim3_count = 0;
        HAL_Delay(700);
        x_camera_error = 0;
        y_camera_error = 0;
        while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
        // while (is_servo_adjust != 0 ) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(50);  //80
        }
        // HAL_UART_Transmit(&huart3, (uint8_t*)"adjust_end", strlen("adjust_end"), 50); // 通知树莓派结束
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up();
    HAL_Delay(500);
    open_claw_180();
    whole_arm_spin(1); 
    arm_stretch();



    /**************第二次前往转盘并抓取*****************/
    int move_right_length_b = 44;
    int move_front_length_b = 88;
    spin_right(open_loop_spin_velocity,acceleration_spin, 90);
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_b);
    HAL_Delay(2500);
    move_all_direction_position(acceleration, open_loop_x_move_velocity,move_right_length_b, 0);
    HAL_Delay(1500);
    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    HAL_Delay(100);
    put_claw_down();
    is_start_get_plate = 1;
    while(get_plate_count < 3 ) // 从转盘抓取三个色环或者超时
    {
        // HAL_UART_Transmi t(&huart3, (uint8_t*)&get_plate, 1, 50);
        is_adjust_plate_servo = 1;
        HAL_Delay(10);
        // while(is_adjust_plate_servo != 0)
        // {
        //     HAL_Delay(10);
        // }
        int temp_plate=0;
        if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0))
        {
            is_adjust_plate_servo = 0;
            //TODO 第一次抓取前移动底盘，然后稍微延时一下
            temp_plate = get_plate;
            get_plate = 0;
            HAL_Delay(50);
            adjust_plate(x_plate_error, y_plate_error);
            x_plate_error = 0;
            y_plate_error = 0;


            state_spin_without_claw(temp_plate);
            close_claw();
            if(is_get_empty_finish == 0)
            {
                start_judge_empty = 1;
            }
            HAL_Delay(400);
            put_claw_up_top();
            HAL_Delay(10); //delate
            if(is_get_empty_finish == 0)
            {
                HAL_Delay(1500);
            }
            start_judge_empty = 0;
            if(is_get_empty == 1)
            {
                open_claw_180();
                put_claw_down();
                get_plate = 0;
            }
            else
            {
                int r_servo_now_temp = r_servo_now;
                is_get_empty_finish = 1;
                arm_shrink();
                HAL_Delay(300);
                claw_spin_state_without_claw();
                HAL_Delay(700);
                open_claw();
                HAL_Delay(300);
                // arm_stretch();
                r_servo_now = r_servo_now_temp;
                x_plate_error = 0;
                y_plate_error = 0;
                adjust_plate(x_plate_error, y_plate_error);
                x_plate_error = 0;
                y_plate_error = 0;
                claw_spin_front();
                open_claw_180();
                HAL_Delay(500);
                get_plate_count++;
                // if(temp_plate == 1)
                // {
                //     HAL_UART_Transmit(&huart3, (uint8_t*)"red", strlen("red"), 50); 
                //     is_1_get = 1;
                // }
                // else if(temp_plate == 2)
                // {
                //     HAL_UART_Transmit(&huart3, (uint8_t*)"green", strlen("green"), 50);
                //     is_2_get = 1;
                // }
                // else if(temp_plate == 3)
                // {
                //     HAL_UART_Transmit(&huart3, (uint8_t*)"blue", strlen("blue"), 50); 
                //     is_3_get = 1;
                // }
                get_plate = 0;
                put_claw_down();
            }
            is_get_empty = 0;
        }
        // get_plate = 0; //TODO ？
        HAL_Delay(10);
    }
    whole_arm_spin(1); 
    arm_stretch();
    put_claw_up();

    /**************第二次从转盘前往粗加工区并放置*****************/
    int move_right_length_3 = 41; //38
    int move_front_length_3 = 168;  //172
    move_all_direction_position(acceleration, open_loop_x_move_velocity, move_right_length_3,0);
    HAL_Delay(900);
    HAL_UART_Transmit(&huart3, (uint8_t*)"EE", strlen("EE"), 50); 
    HAL_Delay(200); 
    // is_slight_spin = 1;
    is_slight_spin_and_move =1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    {
        slight_spin_and_move(); // 直线和圆环一起调整
        HAL_Delay(50);
    }
    
    stop();
    HAL_Delay(100);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_3);
    HAL_Delay(2500);
    put_claw_up();
    spin_right_180(open_loop_spin_velocity,acceleration_spin);
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); 
    HAL_Delay(2000);
    is_slight_spin_and_move = 1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    {
        slight_spin_and_move(); // 直线和圆环一起调整
        HAL_Delay(100);
    }
    is_slight_spin_and_move = 0;
    stop();
    HAL_Delay(50);
    for (int i = 3; i < 6; i++)
    {
        
        get_and_pre_put(target_colour[i], 0);
        servo_adjust_status = target_colour[i];
        is_servo_adjust = 1;
        tim3_count = 0;
        HAL_Delay(700);
        x_camera_error = 0;
        y_camera_error = 0;
        while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
        // while (is_servo_adjust != 0 ) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(50);  //30
        }
        // HAL_UART_Transmit(&huart3, (uint8_t*)"adjust_end", strlen("adjust_end"), 50); // 通知树莓派结束
        // if (tim3_count >= timeout_limit)
        // {
        //     HAL_Delay(20);
        // }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up_top();
    HAL_Delay(500);
    open_claw();
    //放置完成进行抓取
    // TODO 是否根据视觉真实放置的情况进行抓取，还是开环抓取
    for(int i = 3; i < 6; i++)
    {
        get_and_load_openloop(target_colour[i]); // 开环抓取
    }
    whole_arm_spin(1);
    open_claw_180();
    arm_stretch();
    HAL_Delay(10);

// HAL_Delay(3000); //? 延时
    /**************第二次从粗加工区前往暂存区并放置*****************/
    int move_front_length_4 = 81; // 82
    int move_back_length_4 = 86; //85
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_4);
    HAL_Delay(2000);
    put_claw_up();
    arm_stretch();
    spin_right(open_loop_spin_velocity,acceleration_spin, 90);//! magic number
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_4);
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); 
    HAL_Delay(1800);

    is_slight_spin_and_move = 1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    {
        slight_spin_and_move(); // 直线和圆环一起调整
        HAL_Delay(50);
    }
    is_slight_spin_and_move = 0;
    stop();
    HAL_Delay(50);
    for (int i = 0; i < 3; i++)
    {
        
        get_and_pre_put(target_colour[i+3], 1);
        servo_adjust_status = target_colour[i+3];
        is_servo_adjust = 1;
        tim3_count = 0;
        // HAL_Delay(700);
        x_camera_error = 0;
        y_camera_error = 0;
        // while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
        // // while (is_servo_adjust != 0 ) // TODO 超时处理
        // {
        //     adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
        //     HAL_Delay(30);  //80
        // }
        // HAL_UART_Transmit(&huart3, (uint8_t*)"adjust_end", strlen("adjust_end"), 50); // 通知树莓派结束
        // if (tim3_count >= timeout_limit)
        // {
        //     HAL_Delay(20);
        // }
        is_servo_adjust = 0;
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up();
    HAL_Delay(500);
    open_claw_180();
    whole_arm_spin(1); 
    arm_stretch();
    HAL_Delay(10);

// // HAL_Delay(3000); //? 延时
    /**************从暂存区回原点*****************/
   int move_45_length_5 = 30;
    int move_front_length_5 = 74;
    int move_back_length_5 = 160;
    int move_right_length_5 = 3;
    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length_5,0);
    HAL_Delay(900);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,-move_back_length_5);
    HAL_Delay(2500);
    spin_right(open_loop_spin_velocity,acceleration_spin, 90);
    HAL_Delay(1000);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_5);
    HAL_Delay(2000);
    move_all_direction_position(acceleration, open_loop_move_velocity, move_45_length_5, move_45_length_5);
    HAL_Delay(2000);

    while(1)
    {
        HAL_Delay(1000);
    }

    //! 全流程测试结束
    //! 全流程测试结束
    //! 全流程测试结束
    
    //?陀螺仪校正车身姿态
    // int Kp_yaw = 3;
    // int Ki_yaw = 0.1;
    // int Kd_yaw = 0.1;
    // float yaw_err_1 = 0;
    // float yaw_err_2 = 0;
    // int adjust_vel_yaw = 0;
    // int base_vel_yaw = 60;
    // while(1)
    // {
    //     printf("t0.txt=\"%f\"\n",gyro_z);
    //     adjust_vel_yaw = Kp_yaw * (gyro_z) + Ki_yaw * (gyro_z+yaw_err_1 + yaw_err_2 ) + Kd_yaw * (0 - gyro_z -2*yaw_err_1 +yaw_err_2 );
    //     yaw_err_2 = yaw_err_1;
    //     yaw_err_1 = gyro_z;
    //     if(adjust_vel_yaw > 50)
    //     {
    //         adjust_vel_yaw = 50;
    //     }
    //     if(adjust_	vel_yaw < -50)
    //     {
    //         adjust_vel_yaw = -50;
    //     }
    //     if(adjust_vel_yaw >=0)
    //     {
    //         Forward_move_with_yaw_adjust(base_vel_yaw  ,base_vel_yaw+adjust_vel_yaw,acceleration);
    //         HAL_Delay(50);
    //     } 
    //     else
    //     {
    //         Forward_move_with_yaw_adjust(base_vel_yaw-adjust_vel_yaw ,base_vel_yaw ,acceleration);
    //         HAL_Delay(50);
    //     }
    // }

    // while(1)
    // {
    //     HAL_Delay(1000);
    // }

    //? 旧版本物料区放置
    motor_state = 1;
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    is_start_judge_move_before_slight_adjust = 1;
    put_claw_up();
    // HAL_Delay(2000);
    printf("t0.txt=\"start_judge\"\xff\xff\xff");
    is_adjust_motor_in_tim = 0;
    // if(is_move_before_slight_adjust == 1)
    // {
    //     move_all_direction_position(acceleration, open_loop_move_velocity, x_move_before_slight_move, 0);
    //     is_move_before_slight_adjust = 0;
    // }

	is_slight_spin_and_move = 1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    {
        slight_spin_and_move(); // 直线和圆环一起调整
        HAL_Delay(50);
    }
    stop();
    HAL_Delay(50);
    for (int i = 0; i < 3; i++)
    {
        
        get_and_pre_put(target_colour[i], 0);
        servo_adjust_status = target_colour[i];
        is_servo_adjust = 1;
        tim3_count = 0;
        while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
        // while (is_servo_adjust != 0 ) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(100);  //80
        }
        HAL_UART_Transmit(&huart3, (uint8_t*)"adjust_end", strlen("adjust_end"), 50); // 通知树莓派结束
        // if (tim3_count >= timeout_limit)
        // {
        //     HAL_Delay(20);
        // }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up();
    HAL_Delay(500);
    open_claw_180();
    //放置完成进行抓取
    // TODO 是否根据视觉真实放置的情况进行抓取，还是开环抓取
    for(int i = 0; i < 3; i++)
    {
        get_and_load_openloop(target_colour[i]); // 开环抓取
    }
    whole_arm_spin(1);
    put_claw_up_top();
    arm_stretch();
    while(1)//! end
    {
        HAL_Delay(1000);
    }

    //? 非同时调整直线和圆环
    //TODO 这里的发送可以提前一点，保证一到位马上能进行视觉调整
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);
    motor_state = 1;
    if(1) 
    {
        is_slight_spin = 1; // 使能轻微移动
      
		tim3_count = 0;
        while(is_slight_spin != 0 && tim3_count < timeout_limit)
        {
            HAL_Delay(10);
            if(is_adjust_motor_in_tim == 0)
            {
                spin_all_direction_tim(0,spin_which_direction,1);
                HAL_Delay(10);
                spin_all_direction_tim(0,spin_which_direction,2);
                HAL_Delay(10);
                spin_all_direction_tim(0,spin_which_direction,3);
                HAL_Delay(10);
                spin_all_direction_tim(0,spin_which_direction,4);
                HAL_Delay(10);
                spin_all_direction_tim(0,spin_which_direction,5);
                HAL_Delay(10);
            }
        }
        if(tim3_count >= timeout_limit)
        {
            // HAL_UART_Transmit(&huart3, (uint8_t*)"next_line", strlen("next_line"), 50); // 通知树莓派结束
            // HAL_Delay(80);
        }
        is_slight_spin = 0;
        stop();
    }
    put_claw_up();
    open_claw_180();
    // 调圆定位
    is_slight_move = 1; // 使能微调
    tim3_count = 0; // 开始计时，+1 代表10ms
    while(is_slight_move != 0 && tim3_count < timeout_limit) // 超时则停止
    {
        HAL_Delay(10);
        if(is_adjust_motor_in_tim == 0)
        {
            move_all_direction_tim(acceleration_adjust, x_move_position, y_move_position, 1);
            HAL_Delay(10);
            move_all_direction_tim(acceleration_adjust, x_move_position, y_move_position, 2);
            HAL_Delay(10);
            move_all_direction_tim(acceleration_adjust, x_move_position, y_move_position, 3);
            HAL_Delay(10);
            move_all_direction_tim(acceleration_adjust, x_move_position, y_move_position, 4);
            HAL_Delay(10);
            move_all_direction_tim(acceleration_adjust, x_move_position, y_move_position, 5);
            HAL_Delay(10);
        }
    }
    if(tim3_count >= timeout_limit)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"next", strlen("next"), 50); // 通知树莓派结束
        //     HAL_Delay(200);
    }
    is_slight_move = 0;
    stop();
    // 视觉闭环放置


    // HAL_UART_Transmit(&huart3, (uint8_t*)"GG", strlen("GG"), 50); //发给树莓派，开始校正
    HAL_Delay(50);
    for (int i = 0; i < 3; i++)
    {
        get_and_pre_put(target_colour[i], 0);
        is_servo_adjust = 1;
        tim3_count = 0;
        while (is_servo_adjust != 0 && tim3_count < timeout_limit -150) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(100);  //80
        if (tim3_count >= timeout_limit-150)
        {
            // if(i == 0)
            // {
            //     HAL_UART_Transmit(&huart3, (uint8_t *)"next_1", strlen("next_1"), 50); // 通知树莓派结束
            // }
            // else if(i == 1)
            // {
            //     HAL_UART_Transmit(&huart3, (uint8_t *)"next_1", strlen("next_1"), 50); // 通知树莓派结束
            // }
            // else if(i == 2)
            // {
            //     HAL_UART_Transmit(&huart3, (uint8_t *)"next_1", strlen("next_1"), 50); // 通知树莓派结束
            // }
            
            HAL_Delay(80);
        }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }

    put_claw_up();
    HAL_Delay(500);
    open_claw_180();
    //放置完成进行抓取
    // TODO 是否根据视觉真实放置的情况进行抓取，还是开环抓取
    for(int i = 0; i < 3; i++)
    {
        get_and_load_openloop(target_colour[i]); // 开环抓取
    }
    whole_arm_spin(1);
    put_claw_up_top();
    arm_stretch();
    while(1)//! end
    {
        HAL_Delay(1000);
    }

    


    HAL_UART_Transmit(&huart3, (uint8_t*)"GG", strlen("GG"), 50); //发给树莓派，开始校正
    for(int i = 0; i < 3; i++)
    {
        get_and_pre_put(target_colour[i],0);
        HAL_Delay(100);
        switch(target_colour[i])
        {
            case 3:
                r_servo_now = left_4;
                theta_servo_now = left_3;
                break;
            case 2:
                r_servo_now = middle_4;
                theta_servo_now = middle_3;
                break;
            case 1:
                r_servo_now = right_4;
                theta_servo_now = right_3;
                break;
        }
        is_servo_adjust = 1;
        tim3_count = 0; 
        while(is_servo_adjust != 0 && tim3_count < timeout_limit) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(50);
        }
        if(tim3_count >= timeout_limit)
        {
            HAL_UART_Transmit(&huart3, (uint8_t*)"next", strlen("next"), 50); // 通知树莓派结束
            HAL_Delay(80);
        }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up();
    HAL_Delay(500);
    open_claw_180();
    //放置完成进行抓取
    // TODO 是否根据视觉真实放置的情况进行抓取，还是开环抓取
    for(int i = 0; i < 3; i++)
    {
        get_and_load_openloop(target_colour[i]); // 开环抓取
    }
    whole_arm_spin(1);
    put_claw_up_top();
    arm_stretch();

    // get_and_pre_put(2,0);
    // get_and_pre_put(3,0);
    // get_and_pre_put(1,0);

    // 抓取放置测试
    // HAL_Delay(2000);
    // get_and_pre_put(2,0);
    // HAL_Delay(1000);
    // put_claw_down_ground();
    // HAL_Delay(500);
    // open_claw(); 
    // HAL_Delay(500);
    // put_claw_up();
    // get_and_load_openloop(1);
    // get_and_load_openloop(2);
    // get_and_load_openloop(3);
    // arm_stretch();



    while(1)
    {
        HAL_Delay(1000);
    }

    //! 以下本为测试代码，但现在全部废弃
    //! 以下本为测试代码，但现在全部废弃
    //! 以下本为测试代码，但现在全部废弃
    //! 以下本为测试代码，但现在全部废弃
    //! 以下本为测试代码，但现在全部废弃

    /*******************************(暂时废弃）实际功能的初始化******************************************/
    // 等待电机初始化完成，本该是4000ms
    arm_stretch();
    whole_arm_spin(1);
    put_claw_up_top(); // 正式比赛时取消注释
    claw_spin_front();
    open_claw_180();
    state_spin(1);


    // HAL_Delay(1000);

    /*********************************测试区域开始*************************************/
    HAL_Delay(1000);


    // float gyro_error_last = 0;
    // float target_angle = 90; // 目标角度
    // float current_angle = 0; // 当前角度
    // float gyro_error_last = 0; // 上一次陀螺仪角度

    
    // 舵机调试
    



    //? 抓取放置动作
    // while(1)
    // {
    //     HAL_Delay(1000);
    //     arm_shrink();
    //     put_claw_up_top();
    //     open_claw();
    //     HAL_Delay(1000);
    //     claw_spin_state();
    //     HAL_Delay(1000);
    //     put_claw_down_state();
    //     HAL_Delay(1500);
    //     close_claw();
    //     put_claw_up_top();
    //     HAL_Delay(1000);
    //     claw_spin_front();
    //     HAL_Delay(1000);
    //     put_claw_down_ground();
    //     HAL_Delay(4000);
    //     open_claw();
    //     HAL_Delay(5000);



    // }

    //? 视觉闭环测试
    // close_claw();
    // HAL_UART_Transmit(&huart3, (uint8_t*)"GG", strlen("GG"), 50); //发给树莓派，开始校正
    // HAL_Delay(500);
    // put_claw_down_near_ground();
    // // HAL_Delay(3000);

			
			
	// is_servo_adjust = 1;
    // // int is_servo_adjust_ok =0;
    // while(is_servo_adjust != 0)
    // {
    //     // if(is_find_circle == 0)
    //     // {
    //     //     // adjust_position_with_camera(5,-5,1);
    //     // }
    //     // else
    //     // {
    //     adjust_position_with_camera(x_camera_error, y_camera_error,1);
    //     // }
    //     // x_camera_error = 0;
    //     // y_camera_error = 0;
    //     HAL_Delay(100);
    // }
    // put_claw_down_ground();
    // HAL_Delay(500);
    // open_claw_180();
    // HAL_Delay(5000);
    // while(1)
    // {
    //     HAL_Delay(1000);
    // }



    // // //! 前后移动测试
    // while(1)
    // {
    //     move_all_direction_position(acceleration, open_loop_move_velocity-100,0,100);
    //     HAL_Delay(1000);
    //     move_all_direction_position(acceleration, open_loop_move_velocity-100,0,100);
    //     HAL_Delay(1000);
    // }

    //? 陀螺仪测试
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
    

    
//     feetech_servo_move(1,put_claw_down_ground_position-520,4095,50);
//     HAL_Delay(2000);

//     while(1) //?直线校正测试
//     {
// // 先校正车身位置
//     HAL_UART_Transmit(&huart3, (uint8_t*)"EE", strlen("EE"), 50); //发给树莓派，开始校正直线
//     HAL_Delay(50);

//     is_slight_spin = 1; // 使能轻微移动
//     motor_state = 1;
//     tim3_count = 0;
//     while(is_slight_spin != 0)
//     {
//         HAL_Delay(10);
//     }
// 	stop();
//     HAL_Delay(3000);
//     }

    //?微调测试
    // feetech_servo_move(1,put_claw_down_ground_position-520,4095,50);
    // HAL_Delay(2000);
    // HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正
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
    // get_and_put_different_position(2);
    // get_and_put_different_position(1);
    // get_and_put_different_position(3);

	// while(1)
    // {
    //     HAL_Delay(1000);
    // }




    // HAL_Delay(2000);
    // //原地抓取放置测试
    // while(1)
    // {
    //     HAL_Delay(2000);
    //     claw_spin_front();
    //     close_claw();
    //     put_claw_down_ground();
    //     HAL_Delay(4000);
    //     open_claw_180();    
    //     put_claw_up_top();
    //     HAL_Delay(1500);
    //     get_and_put_in_one_position_test(1);
    //     get_and_load_in_one_position_test();
    //     HAL_Delay(4000);
    // }

	





    /*********************************测试区域结束*************************************/
    //! 以下为暂时废弃的主流程代码
    //! 以下为暂时废弃的主流程代码
    //! 以下为暂时废弃的主流程代码
    //! 以下为暂时废弃的主流程代码
    //! 以下为暂时废弃的主流程代码

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
    arm_stretch();
  
    // come_to_temporary_area();
    come_to_temporary_area_v2();
    
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
    // come_to_temporary_area();
    come_to_temporary_area_v2();
    // HAL_Delay(4000); //! 单独路径测试


    //暂存区识别色环移动并放置
    
    /*************方案一**************/
    open_claw_180();
    get_and_put_in_one_position(4);

    // get_and_put_in_one_position_test(4); // !测试

    /*************方案二**************/
    // get_and_put_with_movement(2,1);




    // 第二次从暂存区回原点
    // come_back_to_start_from_temparea();
    come_back_to_start_from_temparea_v2();

    //! 总体代码流程到此结束了

    // 640 480
    // 1280 720

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

/// @brief （暂时废弃）比赛的全流程代码(包含初始化)
/// @param  
void all_process_main(void)
{
    // 我知道这个代码又臭又长，但是这样确实能实现较为灵活的动作控制
    
    /****************初始化*****************/    
    HAL_UART_Receive_IT(&huart3, &received_rxdata_u3, 1); // 使能串口3接收中断
    // HAL_UART_Receive_IT(&huart1, &received_rxdata_u1, 1); // 使能串口1接收中断
    // HAL_UART_Receive_IT(&huart4, &received_rxdata_u4, 1); // 使能串口4接收中断 //TODO 此处开启后造成串口接收消息出现问题
    HAL_UART_Receive_IT(&huart5, &received_rxdata_u5, 1); // 使能串口5接收中断
    // HAL_UART_Receive_IT(&huart2, &received_rxdata_u2, 1); // 使能串口2接收中断


    HAL_TIM_Base_Start_IT(&htim2); // 使能定时器2中断
    HAL_TIM_Base_Start_IT(&htim3); // 使能定时器3中断
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // 开启TIM1通道1 PWM输出
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // 开启TIM1通道2 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 开启TIM1通道3 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 开启TIM1通道4 PWM输出  
	HAL_Delay(1000);
    my_servo_init();//!精密舵机初始化，使用精密舵机则必须加入
    HAL_Delay(100);
    arm_stretch();
    whole_arm_spin(1); // 中板旋转到中间位置
    put_claw_up_top(); 
    claw_spin_front();
    open_claw_180();
    state_spin_without_claw(1);//TODO 电机初始化是否完成

    /**************起步前往转盘*****************/
    int start_move = 15;
    int move_to_qrcode = 45;
    int move_from_qrcode_to_table = 84;
    int spin_right_angle = 90;
    // 开始识别二维码
    HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50); 
    HAL_Delay(50);
    // 左前移动出库
    move_all_direction_position(acceleration, open_loop_move_velocity, -start_move , start_move); 
    HAL_Delay(1000);
    // 前往二维码
    move_all_direction_position(acceleration, open_loop_move_velocity, move_to_qrcode, 0); 
    HAL_Delay(2000);
    // 将目标颜色显示在串口屏上
    char* target_colour_str = (char*)malloc(6);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); 
    // 前进至转盘
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table); 
    HAL_Delay(2500);
    // 右转90（以后可考虑换姿势出发）
    // TODO 考虑直接出发时就面朝转盘方向
    spin_right(open_loop_spin_velocity,acceleration, spin_right_angle);
    // 夹爪放下
    open_claw_180();
    put_claw_down();

    //TODO 待加入：根据视觉调整底盘位置
    

    
    /**************从转盘抓取*****************/

    // 提前发指令开始识别物料颜色并抓取
    is_start_get_plate = 1;
    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    HAL_Delay(100);
    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    HAL_Delay(100);
    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    HAL_Delay(100);

    HAL_Delay(1000); // 等待旋转完成

    //将目标颜色再次显示在串口屏上
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上
    while(get_plate_count < 3 ) // 从转盘抓取三个色环或者超时
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)&get_plate, 1, 50);
        is_adjust_plate_servo = 1;
        HAL_Delay(10);
        while(is_adjust_plate_servo != 0) //TODO 超时处理
        {
            HAL_Delay(10);
        }
        if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0))
        {
            is_adjust_plate_servo = 0;
            //TODO 第一次抓取前移动底盘，然后稍微延时一下
            temp_plate = get_plate;
            get_plate = 0;
            adjust_plate(x_plate_error, y_plate_error);
            x_plate_error = 0;
            y_plate_error = 0;
            state_spin_without_claw(temp_plate);
            close_claw();
            HAL_Delay(400);
            put_claw_up_top();
            start_judge_empty = 1;
            HAL_Delay(10); //delate
            HAL_Delay(700);
            if(is_get_empty == 1)
            {
                open_claw_180();
                put_claw_down();
                get_plate = 0;
            }
            else
            {
                int r_servo_now_temp = r_servo_now;
                arm_shrink();
                claw_spin_state_without_claw();
                HAL_Delay(1100);
                open_claw();
                HAL_Delay(300);
                // arm_stretch();
                r_servo_now = r_servo_now_temp;
                adjust_plate(x_plate_error, y_plate_error);
                x_plate_error = 0;
                y_plate_error = 0;
                claw_spin_front();
                open_claw_180();
                HAL_Delay(500);
                get_plate_count++;
                if(temp_plate == 1)
                {
                    HAL_UART_Transmit(&huart3, (uint8_t*)"red", strlen("red"), 50); 
                    is_1_get = 1;
                }
                else if(temp_plate == 2)
                {
                    HAL_UART_Transmit(&huart3, (uint8_t*)"green", strlen("green"), 50);
                    is_2_get = 1;
                }
                else if(temp_plate == 3)
                {
                    HAL_UART_Transmit(&huart3, (uint8_t*)"blue", strlen("blue"), 50); 
                    is_3_get = 1;
                }
                get_plate = 0;
                put_claw_down();
            }
            start_judge_empty = 0;
            is_get_empty = 0;

            
            
        }
        get_plate = 0;
        HAL_Delay(10);
    }

    /**************第一次从转盘前往粗加工区并放置*****************/
    int move_right_length_1 = 40; 
    int move_front_length_1 = 174;  
    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length_1,0);
    HAL_Delay(1200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_1);
    HAL_Delay(3000);
    spin_right(open_loop_spin_velocity,acceleration, 180);
    HAL_Delay(2000);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上
    free(target_colour_str);

    // 校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);
    motor_state = 1;
    // 调直线
    if(1) 
    {
        is_slight_spin = 1; // 使能轻微移动
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
        is_slight_spin = 0;
        stop();
    }
    put_claw_up(); //TODO 似乎仍需要考虑
    open_claw_180();
    // 调圆定位
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
    stop();


    // 视觉闭环放置
    for(int i = 0; i < 3; i++)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"GG", strlen("GG"), 50); //发给树莓派，开始校正
        get_and_pre_put(target_colour[i],0);
        switch(target_colour[i])
        {
            case 3:
                r_servo_now = left_4;
                theta_servo_now = left_3;
                break;
            case 2:
                r_servo_now = middle_4;
                theta_servo_now = middle_3;
                break;
            case 1:
                r_servo_now = right_4;
                theta_servo_now = right_3;
                break;
        }
        is_servo_adjust = 1;
        tim3_count = 0; 
        while(is_servo_adjust != 0 && tim3_count < timeout_limit) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(100);
        }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up();

    //放置完成进行抓取
    // TODO 是否根据视觉真实放置的情况进行抓取，还是开环抓取
    for(int i = 0; i < 3; i++)
    {
        get_and_load_openloop(target_colour[i]); // 开环抓取
    }
    whole_arm_spin(1);
    put_claw_up_top();
    arm_stretch();

    /**************第一次从粗加工区前往暂存区并放置*****************/

    int move_front_length_2 = 84; // 82
    int move_back_length_2 = 85; //85
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_2);
    HAL_Delay(2200);
    spin_right(open_loop_spin_velocity,acceleration, 90);//! magic number
    HAL_Delay(1500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_2 );
    HAL_Delay(1800);
    move_all_direction_position(acceleration, open_loop_move_velocity, -3, 0);  //TODO 可以去掉
    HAL_Delay(400);

    // 校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);
    motor_state = 1;
    // 调直线
    if(1) 
    {
        is_slight_spin = 1; // 使能轻微移动
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
        is_slight_spin = 0;
        stop();
    }
    put_claw_up();
    open_claw_180();
    // 调圆定位
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
    stop();


    // 视觉闭环放置
    for(int i = 0; i < 3; i++)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"GG", strlen("GG"), 50); //发给树莓派，开始校正
        get_and_pre_put(target_colour[i],0);
        switch(target_colour[i])
        {
            case 3:
                r_servo_now = left_4;
                theta_servo_now = left_3;
                break;
            case 2:
                r_servo_now = middle_4;
                theta_servo_now = middle_3;
                break;
            case 1:
                r_servo_now = right_4;
                theta_servo_now = right_3;
                break;
        }
        is_servo_adjust = 1;
        tim3_count = 0; 
        while(is_servo_adjust != 0 && tim3_count < timeout_limit) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(100);
        }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    whole_arm_spin(1);
    put_claw_up_top();
    open_claw_180();
    arm_stretch();

    /**************第二次前往转盘并抓取*****************/

    int move_right_length_b = 48;
    int move_front_length_b = 89;
    spin_right(open_loop_spin_velocity,acceleration, 90);
    HAL_Delay(1500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_b);
    HAL_Delay(2500);
    is_start_get_plate = 1;
    HAL_UART_Transmit(&huart3, (uint8_t*)"DD", strlen("DD"), 50); // 开始识别颜色并抓取
    move_all_direction_position(acceleration, open_loop_move_velocity,move_right_length_b, 0);
    HAL_Delay(1000);
    put_claw_down();
    open_claw_180();
    HAL_Delay(500);
    get_plate_count = 0;
    is_1_get = 0;
    is_2_get = 0;
    is_3_get = 0;

    //TODO 待加入：根据视觉调整底盘位置
    while(get_plate_count < 3 ) // 从转盘抓取三个色环或者超时
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)&get_plate, 1, 50);
        int temp_plate=0;
        if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0))
        {
            //TODO 第一次抓取前移动底盘，然后稍微延时一下
            temp_plate = get_plate;
            get_plate = 0;
            state_spin_without_claw(temp_plate);
            close_claw();
            HAL_Delay(500);
            put_claw_up_top();
            HAL_Delay(700);
            arm_shrink();
            claw_spin_state_without_claw();
            HAL_Delay(1100);
            open_claw();
            HAL_Delay(300);
            claw_spin_front();
            open_claw_180();
            HAL_Delay(500);
            get_plate_count++;
            if(temp_plate == 1)
            {
                HAL_UART_Transmit(&huart3, (uint8_t*)"red", strlen("red"), 50); 
                is_1_get = 1;
            }
            else if(temp_plate == 2)
            {
                HAL_UART_Transmit(&huart3, (uint8_t*)"green", strlen("green"), 50);
                is_2_get = 1;
            }
            else if(temp_plate == 3)
            {
                HAL_UART_Transmit(&huart3, (uint8_t*)"blue", strlen("blue"), 50); 
                is_3_get = 1;
            }
            get_plate = 0;
            put_claw_down();
        }
        get_plate = 0;
        HAL_Delay(100);
    }
    
    /**************第二次从转盘前往粗加工区并放置*****************/
    int move_right_length_3 = 40; //38
    int move_front_length_3 = 174;  //172
    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length_3,0);
    HAL_Delay(1200); 
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_3);
    HAL_Delay(3000);
    spin_right(open_loop_spin_velocity,acceleration, 180);
    HAL_Delay(2000);


    // 校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);
    motor_state = 1;
    // 调直线
    if(1) 
    {
        is_slight_spin = 1; // 使能轻微移动
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
        is_slight_spin = 0;
        stop();
    }
    put_claw_up();
    open_claw_180();
    // 调圆定位
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
    stop();


    // 视觉闭环放置
    for(int i = 0; i < 3; i++)
    {
        HAL_UART_Transmit(&huart3, (uint8_t*)"GG", strlen("GG"), 50); //发给树莓派，开始校正
        get_and_pre_put(target_colour[i],0);
        switch(target_colour[i])
        {
            case 3:
                r_servo_now = left_4;
                theta_servo_now = left_3;
                break;
            case 2:
                r_servo_now = middle_4;
                theta_servo_now = middle_3;
                break;
            case 1:
                r_servo_now = right_4;
                theta_servo_now = right_3;
                break;
        }
        is_servo_adjust = 1;
        tim3_count = 0; 
        while(is_servo_adjust != 0 && tim3_count < timeout_limit) // TODO 超时处理
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); // TODO 可以针对视觉调整的情况来进行方案的调整
            HAL_Delay(100);
        }
        is_servo_adjust = 0;
        put_claw_down_ground();
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    put_claw_up();

    //放置完成进行抓取
    // TODO 是否根据视觉真实放置的情况进行抓取，还是开环抓取
    for(int i = 0; i < 3; i++)
    {
        get_and_load_openloop(target_colour[i]); // 开环抓取
    }
    whole_arm_spin(1);
    put_claw_up_top();
    arm_stretch();

    /**************第二次从粗加工区前往暂存区并放置*****************/

    int move_front_length_4 = 84; // 82
    int move_back_length_4 = 85; //85
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_4);
    HAL_Delay(2200);
    spin_right(open_loop_spin_velocity,acceleration, 90);//! magic number
    HAL_Delay(1500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_4);
    HAL_Delay(1800);
    move_all_direction_position(acceleration, open_loop_move_velocity, -3, 0);  //TODO 可以去掉
    HAL_Delay(400);

    // 校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);
    motor_state = 1;
    // 调直线
    if(1) 
    {
        is_slight_spin = 1; // 使能轻微移动
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
        is_slight_spin = 0;
        stop();
    }
    put_claw_up();
    open_claw_180();
    // 调圆定位
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
    stop();


    // 视觉闭环放置
    for(int i = 0; i < 3; i++)
    {
        get_and_pre_put(target_colour[i],1); //TODO 码垛是否会撞倒
        HAL_Delay(500);
        open_claw();
        HAL_Delay(500);
    }
    whole_arm_spin(1);
    put_claw_up_top();
    open_claw_180();
    arm_stretch();

    /**************从暂存区回原点*****************/

    int move_45_length_5 = 18;
    int move_front_length_5 = 80;
    int move_back_length_5 = 174;
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,-move_back_length_5);
    HAL_Delay(3000);
    spin_right(open_loop_spin_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_5);
    HAL_Delay(2500);
    move_all_direction_position(acceleration, open_loop_move_velocity, move_45_length_5, move_45_length_5);
    HAL_Delay(2000);

}

/// @brief （废弃）
/// @param  
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

/// @brief （废弃）根据识别到的颜色顺序，移动到对应的位置,左蓝中绿右红， 1红2绿3蓝
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

/// @brief （废弃）从起点出发，前进到转盘并调整好姿态准备抓取
/// @param  
void start_and_come_to_turntable(void)
{
    int start_move = 15;
    int move_to_qrcode = 45;
    int move_from_qrcode_to_table = 84;
    int spin_right_angle = 90;
    
    // printf("t0.txt=\"start\"\xff\xff\xff"); // 开始
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


/// @brief （废弃）从转盘抓取色环
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
        get_plate_count = 0;
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

/// @brief （废弃）无视觉从转盘抓取物料，用于调试
/// @param  
void get_from_turntable_test(void)
{
    get_and_load(1);
    HAL_Delay(2000);
    get_and_load(2);
    HAL_Delay(2000);
    get_and_load(3);
    // HAL_Delay(2000);
}

/// @brief （废弃）从转盘前往粗加工区
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
    int move_right_length = 40; //38
    int move_front_length = 174;  //172
    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length,0);
    HAL_Delay(1200);
    // spin_right(open_loop_spin_velocity,acceleration, 180);//! magic number 
    // HAL_Delay(1700);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length);
    HAL_Delay(3000);
    spin_right(open_loop_spin_velocity,acceleration, 180);
    HAL_Delay(2000);
    char* target_colour_str = (char*)malloc(6);
    sprintf(target_colour_str, "%d%d%d%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]);
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); // 将目标颜色显示在串口屏上


}

/// @brief （废弃）从粗加工区前往暂存区
/// @param  
void come_to_temporary_area(void)
{
    int move_front_length = 87; // 85
    int move_right_length = 86;
    spin_right(open_loop_spin_velocity,acceleration, 90); 
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length );
    HAL_Delay(2000);
    move_all_direction_position(acceleration, open_loop_move_velocity-100, move_right_length, 0);
    HAL_Delay(2500);
}

/// @brief （废弃）先后退
/// @param  
void come_to_temporary_area_v2(void)
{
    int move_front_length = 84; // 82
    int move_back_length = 85; //85
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length);
    HAL_Delay(2200);
    spin_right(open_loop_spin_velocity,acceleration, 92);//! magic number
    HAL_Delay(1500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length );
    HAL_Delay(1800);
    move_all_direction_position(acceleration, open_loop_move_velocity, -3, 0);
    HAL_Delay(400);
}

/// @brief （废弃）从暂存区前往转盘
/// @param  
void come_to_turntable_from_temparea(void)
{
    int move_right_length = 48; //45
    int move_front_length = 89;
    spin_right(open_loop_spin_velocity,acceleration, 90);
    arm_stretch();
    HAL_Delay(1500);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length);
    HAL_Delay(2500);

    // spin_adjust_line();  //! 实际联调需要加上

    move_all_direction_position(acceleration, open_loop_move_velocity,move_right_length, 0);
    HAL_Delay(1500);
}

/// @brief （废弃）从暂存区回到起点
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
    HAL_Delay(3500);//  length /(0.47cm/s * velocity) *1000 = delaytime(ms)
    move_all_direction_position(acceleration, open_loop_move_velocity, move_45_length, -move_45_length);
    HAL_Delay(2000);
}

/// @brief （废弃）
/// @param  
void come_back_to_start_from_temparea_v2(void)
{
    int move_45_length = 18;
    int move_front_length = 80;
    int move_back_length = 174;
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,-move_back_length);
    HAL_Delay(3000);
    spin_right(open_loop_spin_velocity,acceleration, 90);
    HAL_Delay(2200);
    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length);
    HAL_Delay(2500);
    move_all_direction_position(acceleration, open_loop_move_velocity, move_45_length, move_45_length);
    HAL_Delay(2000);
}

/// @brief （废弃）无视觉原地放置三个物料测试
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
    // HAL_Delay(1000);
}

/// @brief （废弃）原地放置三个物料
/// @param time_status 放置的阶段，1为第一次放置，2为第二次放置，一直到4，第四次码垛
void get_and_put_in_one_position(int time_status)
{
    // //! 先校正车身位置
    HAL_UART_Transmit(&huart3, (uint8_t*)"CC", strlen("CC"), 50); //发给树莓派，开始校正直线
    HAL_Delay(50);
    // move_all_direction_position(acceleration, open_loop_move_velocity, 0, -3); // 后退5cm

    motor_state = 1;
    if(1) //!    全部采用或者24采用
    {
        is_slight_spin = 1; // 使能 轻微移动
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
        
        is_slight_spin = 0;
        stop();
    }
    if(time_status == 4)
    {
        put_claw_up_top();
    }
    else
    {
        feetech_servo_move(1,put_claw_down_ground_position-520,4095,50);
        open_claw_180();
    }
    
    HAL_Delay(500);



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
    // HAL_Delay(1000);
}

/// @brief （废弃）无视觉
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

/// @brief （废弃）在原地将三个物料夹取到车上
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

/// @brief （废弃）移动进行放置，1为第一轮，2为第二轮
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

/// @brief 废弃
/// @param status 
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
