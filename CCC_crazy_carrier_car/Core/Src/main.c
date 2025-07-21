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
#include "motor.h"       // 电机控制相关
#include "uart_screen.h" // 串口屏通信
#include "my_usart.h"    // 串口通信
#include "my_servo.h"    // 舵机控制
#include "my_gyroscope.h" // 陀螺仪
#include "stm32f407xx.h"
#include "stm32f4xx_hal_flash_ex.h"


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


//树莓派对应的引脚（USB端口在前，风扇在后）25.7.3
// 灰色：左列由后往前第三个 gnd
// 蓝色：右列由后往前第四个 tx pi  
// 橙色：右列由前往后第六个 rx
// PB10 TX



//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!      重要变量（可能要根据不同情况进行修改的）

//?至码垛运行时间 3.01  2.52   2.55   3.07  2.55  3.15  3.06  2.30（最快）   3.35（最慢）  3.05（2.12离开转盘）  3.03  2.53  3.09  2.58  3.06 2.41  2.31  2.54 3.10  2.31  3.14

int start = 0; //! 是否运行全流程

int is_put_adjust_with_material = 1 ; // 1则为夹着物料进行调整，0则为不夹着物料进行调整

int is_pile_adjust = 0; // 1则为码垛时细调整，0为不调整

int is_get_material_from_temp_area = 0; // 是否从暂存取物料

int is_single_route_test = 0; // 1则单独路径移动

int is_show_origin_xy_data = 0;
int is_show_origin_theta_data = 0;





//! 目标颜色数组a
volatile int target_colour[6] = {2,3,1,1,3,2}; // 物料颜色序列(1红,2绿,3蓝)
volatile int material_place[3] = {0,0,0}; //从暂存区夹取随机位置的物料时用的数组

//! 默认暂存区物料顺序
material_order default_material_order = { .left = 3, .middle = 2, .right = 1};


//!底盘调整相关参数
const float Kp_slight_move = 0.56;  // 底盘前后左右微调PID参数
const float Ki_slight_move = 0.01;
const float Kd_slight_move = 0.25;

const float Kp_line_spin = 1;      // 直线校正PID参数
const float Ki_line_spin = 0.1;
const float Kd_line_spin = 0.5;

const float xy_move_k = 0.2; //底盘微调时xy乘上的比例 
const float adjust_spin_scale = 0.8; // 底盘微调时旋转和移动的比例
const float adjust_move_scale = 1;


const float spin_limit_max = 10;   // 旋转速度的最大值
const float spin_limit_min = 0.4;  // 旋转速度的最小值
const float move_limit_max = 10;   // 移动速度的最大值
const float move_limit_min = 0.5;  // 移动速度的最小值

int motor_vel_adjust_with_spin = 50;  // 底盘直线调整时的最大速度20
int open_loop_x_move_velocity = 120;  // 开环横向移动速度120
int open_loop_move_velocity = 180;    // 开环前进速度180
int open_loop_spin_velocity = 150;    // 开环旋转速度150

// 步进电机加速度
float acceleration = 200;          // 直线运动加速度170  180会出现明显的震荡类似丢步
float acceleration_spin = 180;     // 旋转运动加速度180
float acceleration_adjust = 0; //一直用的180
int motor_pos_move_mode = 0; //如果是0则是位置模式按照上一条指令的目标位置进行相对移动；2则是按照当前的实际位置进行相对移动



//!机械臂调整相关参数
//!!!!!!!!      注意：机械臂还有大量参数在my_servo.c中
//!!!!!!!!      注意：机械臂还有大量参数在my_servo.c中
//!!!!!!!!      注意：机械臂还有大量参数在my_servo.c中

//? 以下是旧版本的PID控制参数（配合adjust_position_with_camera）
const float Kp_theta = 0.32;  // 机械臂旋转PID参数
const float Ki_theta = 0.01;
const float Kd_theta = 0.45;
const float Kp_r = 0.35;     // 机械臂伸缩PID参数
const float Ki_r = 0.01;
const float Kd_r = 0.3;

const float pixel_to_distance_theta = 1.2; // theta方向的像素到实际距离的比例
const float pixel_to_distance_r = 4; // r方向的像素到实际距离的比例

//? 暂时废弃以下是版本的PID控制参数（adjust_position_with_camera_new）
// const float Kp_theta = 0.3;  // 机械臂旋转PID参数
// const float Ki_theta = 0.008;
// const float Kd_theta = 0.012;
// const float Kp_r = 0.5;     // 机械臂伸缩PID参数
// const float Ki_r = 0.010;
// const float Kd_r = 0.018;

// const float pixel_to_distance_theta = 1; // theta方向的像素到实际距离的比例
// const float pixel_to_distance_r = 1; // r方向的像素到实际距离的比例


float error_decrease_gain = 0.5; //如果没有新的值，根据旧的值继续调时用的衰减系数

// 根据不同的误大小调整Kp的值
const float Kp_decrease_gain_big = 1.5;
const float Kp_decrease_gain_middle =1.3; 
const float Kp_decrease_gain_small = 1;

// 像素值范围
const float middle_limit = 20;
const float small_limit = 10;


//! 机械臂通用调整参数

int adjust_position_with_camera_time = 0; //机械臂细调的延时时间

//机械臂转盘单次微调系数
const float x_plate_k = 1.2;   // 转盘处机械臂微调系数
const float y_plate_k = 4.5;


//! 调整的超时时间
int timeout_limit = 1000; // 超时时间限制，单位10ms
int timeout_limit_line = 400;










//!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! 各种extern和声明（一般无需修改的）

//串口相关变量
extern int uart_receive_count3;
char wifi_massage[50]; // 串口屏发送的数据
int is_wifi_already_get_message = 0;
int is_raspi_get_massage = 0;
extern uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50],rxdata_u5[50]; // usart2,3接收缓冲区
extern uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1,received_rxdata_u5,received_rxdata_u4; // 暂存usart2,3接收到的数据单字节变量
extern uchar rxflag_u2,rxflag_u3,rxflag_u1,rxflag_u4,rxflag_u5; // usart2,3接收标志位变量

// #define max_data_length 50 // 串口屏数据最大长度
volatile char print_screen_buffer_xy[max_data_length][4];// 0为直线，1为x，2为y
volatile char print_screen_buffer_thetar[max_data_length][4]; // 串口屏打印缓冲区 0为直线，1为theta，2为r
volatile int print_screen_buffer_index_xy = 0; // 串口屏打印缓冲区索引
volatile int print_screen_buffer_index_thetar = 0; // 串口屏打印缓冲区索引

// 各种底盘标志位和变量
extern float acceleration; // 加速度
extern float acceleration_spin;
extern float x_move_position, y_move_position; // x、y
extern int is_motor_start_move; 
extern int is_slight_move,motor_state,is_slight_spin, is_slight_spin_and_move; // 微调底盘所用的标志位变量
extern volatile int x_camera_error, y_camera_error; // 视觉闭环微调时的x、y偏差值


int is_find_circle = 0; //调整时是否找到圆环
int temp_plate=0; //当次抓取的物料

float volatile gyro_z = 90; // 陀螺仪z轴角度（暂时废弃）

int move_sequence_bias = 0; // 根据不同顺序移动带来的位置相对色环位置的偏差，如中-左-右，则偏差为0、-x、+x （暂时废弃）

/// @brief 用于判断当前是第几个case,
int case_count = 0; 
extern int tim3_count;

int is_get_qrcode_target = 0; //是否已经获取二维码任务
int volatile is_start_get_plate = 0; // 开始从转盘抓
int volatile start_check_plate_back_state = 1;
int volatile get_plate = 0; // 1 2 3 
int is_adjust_plate_servo = 0; // 根据视觉定位在转盘处实现移动机械臂抓取物料
int is_adjust_plate_first = 0;
int get_plate_count = 0; // 从转盘上抓取物料的计数

extern volatile int x_plate_error , y_plate_error ;

int is_adjust_motor_in_tim = 1; // （废弃）如果为1，则在定时器中进行电机调整，否则在main的while中进行电机调整



float now_spin_which_direction = 0;

int is_start_judge_move_before_slight_adjust=0; // 是否开始判断在微调前是否需要移动
int is_move_before_slight_adjust=0 ; // 在微调前是否需要移动
int x_move_before_slight_move=0 ;

int is_1_get = 0, is_2_get = 0, is_3_get = 0;
int is_get_empty = 0,start_judge_empty = 0;
int is_get_empty_finish = 0; // 空抓判断完成

extern int stretch_camera;

int seeking_for_circle = 0; // 当还没看到完整色环时置1
int is_servo_adjust= 0; // 当在视觉闭环微调舵机时置1

extern volatile int  r_servo_now ; // 机械臂伸缩舵机的位置
extern volatile int  theta_servo_now ; // 机械臂中板旋转舵机的位置

extern const int feet_acc;

extern int left_2, left_3, left_4;
extern int middle_2, middle_3, middle_4;
extern int right_2, right_3, right_4;
extern const int middle_arm,stretch_arm;

extern volatile int test_slight_move; // 用于判断微调是否完成
extern int spin_which_direction;
extern int put_claw_down_ground_position;

extern float angle_motor_1,angle_motor_2,angle_motor_3,angle_motor_4;
extern float motor_vel_1,motor_vel_2,motor_vel_3,motor_vel_4;

int servo_adjust_status = 5;

// 飞特舵机相关加速度
// int acc_front_start = 200,acc_front_stop = 200;
// int acc_x_same_start = 150,acc_x_same_stop = 150;
// int acc_spin_start = 130,acc_spin_stop = 130;

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

int test_raspi_communication_start = 0; 
int test_raspi_communication_status = 0;

int is_plate_first_move = 0; // 在将物料放置在圆盘带有色环时是否开始移动
int is_plate_move_adjust = 0; // 在将物料放置在圆盘带有色环时是否开始调整
int put_plate_count = 0; // 将物料放置在圆盘带有色环时的计数
int is_third_preput = 0; // 是否可以进入第三次放置



int test_is_uart_message_lost = 0;
int uart_data = 0;

int is_put_material_in_plate = 0;

int theta_servo_value[4] = {0}; //TODO 需要修改
int r_servo_value[4] = {0};
int is_put_plate = 0;

char target_colour_str[16]; // 用于显示二维码识别结果
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/************************************函数声明及定义区****************************************/

void guosai_test(void);
void new_final_function_test(int test_num);
void new_material_test(int test_num,int is_avoid);
void new_motor_servo_screen_test(int test_num);
void old_function_test(int test_num);
void test_new_material_all_process(int is_avoid);
void old_new_init_car_arm(void);
void old_all_process(void);


// printf重定向，用于串口屏的显示
void print_to_screen(int t_num,char *pData);
void print_clear_screen(void);
void print_show_origin_data(int theta,int x, int y);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
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

HAL_StatusTypeDef Reliable_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

void test_new_material_all_process(int is_avoid);

void single_line_adjust(char *pData);
void single_line_circle_adjust(char *pData);
void single_circle_adjust(char* pData);
void old_get_from_plate_all_movement(int n);
void new_get_from_plate_all_movement(int n, plate_get_struct* plate_struct ,int is_get_without_vision);
void new_get_from_plate_all_movement_with_back_check(void);
void old_material_get_and_put_some_with_load_first( int times,int is_pile_up,int is_load);
void new_material_get_and_put_some_with_load_first(material_get_and_put_struct* new_get_and_put_struct, material_order* order);
void new_get_and_put_in_spin_plate_cricle_all(int times);
void new_get_and_put_in_spin_plate_cricle_all_v2(uint8_t *pData,int times,int is_adjust_once, int is_avoid);
void new_get_from_ground_in_random_position(int times);

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
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!  本程序中包含各种测试流程和比赛流程，因为调试阶段各种动作和指令的耦合性比较强，故放弃了一些常用动作流程的封装，所以整体较为混乱
    //!  主要的程序即全流程代码
 
    /**
     * 主要函数说明:(待更新)
     * 
     * 
     */



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

     * 
     * 
     * 
     * my_usart.c
     * 

     * 
     * 
     * my_servo.c
     * 

     * 
     * 
     * motor.c

     * 
     * 
     */
    //? 摄像头像素和底盘步进关系
    //! 5前后对应188像素
    //! 5左右对应像素181
    //! 旋转量5对应13°
    // //? 摄像头像素与实际步进的关系
    //! 1000r步进对应像素值变化量为180
    //! 100步进对应70 旋转



    /*****************各种系统相关外设的初始化（串口、定时器等)***********************/

    // 串口接收中断
    // HAL_UART_Receive_IT(&huart3, &received_rxdata_u3, 1); // 使能串口3接收中断 （树莓派，目前使用空闲中断，这个不需要）
    // HAL_UART_Receive_IT(&huart1, &received_rxdata_u1, 1); // 使能串口1接收中断（步进电机）
    // HAL_UART_Receive_IT(&huart4, &received_rxdata_u4, 1); // 使能串口4接收中断 （陀螺仪、WiFi模块）
    HAL_UART_Receive_IT(&huart5, &received_rxdata_u5, 1); // 使能串口5接收中断（串口屏）
    // HAL_UART_Receive_IT(&huart2, &received_rxdata_u2, 1); // 使能串口2接收中断（飞特舵机）

    // 串口空闲中断
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, rxdata_u3,40 );
    HAL_UARTEx_ReceiveToIdle_IT(&huart4, rxdata_u4,40 );


    HAL_TIM_Base_Start_IT(&htim2); // 使能定时器2中断
    HAL_TIM_Base_Start_IT(&htim3); // 使能定时器3中断
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // 开启TIM1通道1 PWM输出
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // 开启TIM1通道2 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 开启TIM1通道3 PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 开启TIM1通道4 PWM输出  
	HAL_Delay(1000); // TODO等待电机初始化完成，本该是4000ms,目前暂时减少时间
    my_servo_init(); //!精密舵机初始化，使用精密舵机则必须加入
    is_adjust_motor_in_tim = 0;
    motor_state = 1;


    /*****************初始化动作姿态***********************/
    for(int i = 0 ; i < max_data_length ; i++)
    {
        print_screen_buffer_xy[i][0] = 128;
        print_screen_buffer_xy[i][1] = 128;
        print_screen_buffer_xy[i][2] = 128;
        print_screen_buffer_xy[i][3] = 128;
    }
    is_raspi_get_massage = 0; //空闲中断标志位清零
    for(int i = 0 ; i < 4 ; i++)
    {
        theta_servo_value[i] = middle_arm;
        r_servo_value[i] = stretch_arm;
    }
    


    /*****************单独调试程序***********************/
    HAL_Delay(3000);


    //! 新物料抓取放置功能测试
    new_material_test(0,0);


    //! 各种外设测试程序
    new_motor_servo_screen_test(-1);


    //! 单独初赛功能测试
    old_function_test(-1);


    //! 单独决赛功能测试
    new_final_function_test(-1);


    /***********************初赛所用的全流程***********************/
    old_all_process();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // HAL_UART_Transmit(&huart3, (uint8_t*)"st", strlen("st"), 1000);
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

/// @brief 国赛决赛功能框架函数，直接参照该函数内的功能写法调用即可
/// @param  
void guosai_test(void)
{
    //! 下面是国赛框架


    //? 发送信息
    Reliable_UART_Transmit(&huart3, (uint8_t*)"AA\n", strlen("AA\n"), 1000); // 通知树莓派开始识别二维码


    //? 底盘位置移动
    int move_x_1 = 0;
    int move_y_1 = 0;
    move_all_direction_position(acceleration, open_loop_move_velocity, move_x_1, move_y_1);
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, move_x_1, move_y_1);


    //? 底盘旋转
    spin_right_180(open_loop_spin_velocity,acceleration_spin);  // 旋转180度面向色环
    HAL_Delay(2000);
    spin_left_90(open_loop_spin_velocity,acceleration_spin);  // 旋转180度面向色环
    HAL_Delay(1000);


    //? 底盘姿态调整
    single_line_adjust("EE\n"); //直线校正

    single_line_circle_adjust("CC12\n"); // 校正车身位置对准色环
    
    single_circle_adjust("CC12\n"); // 校正车身位置对准色环

    
    //? 物料抓放函数
    material_get_and_put_struct struct_get_and_put_1  = 
    {
        .times = 1, //暂时不调
        .run_round_number = 3,
        .is_load = 1, 
        .is_pile_up = 0,

        .is_avoid_collide = 1,
        .is_adjust_without_material = 0,
        .is_none_pile_up_put_adjust = 1,
        .is_pile_up_adjust = 0, //暂时不调
        .is_get_from_car_plate_update = 1, //暂时不调
        .is_get_from_ground_check = 0,  //暂时不调
    };
    put_claw_up();
    single_line_circle_adjust("KK12\n"); 
    new_material_get_and_put_some_with_load_first(&struct_get_and_put_1,&default_material_order);


    //? 从转盘抓取
    plate_get_struct struct_plate_get_1 = 
    {
        .is_avoid = 1,
        .empty_check = 0,
        .back_check = 1,
    };
    put_claw_down();
    new_get_from_plate_all_movement(3,&struct_plate_get_1,0);


    //? 从暂存区抓取随机顺序摆放物料
    put_claw_up();
    single_line_adjust("II\n");
    new_get_from_ground_in_random_position(1);


    //? 将物料放置在旋转转盘上
    new_get_and_put_in_spin_plate_cricle_all_v2((uint8_t*)"HH\n",1,1,1);
}

/// @brief 决赛功能测试函数
/// @param test_num  -1 不执行
///                     1 从暂存区抓取随机顺序摆放物料
///                     2 将物料放置在旋转转盘上
void new_final_function_test(int test_num)
{
    switch(test_num)
    {
        case 1:
            {
                //? 从暂存区抓取随机顺序摆放物料
                put_claw_up();
                single_line_adjust("II\n");
                new_get_from_ground_in_random_position(1);
                while(1)
                {
                    HAL_Delay(1000);
                }
            }
            break;
            case 2:
            {
                new_get_and_put_in_spin_plate_cricle_all_v2((uint8_t*)"LL\n",1,0,1);
                while(1)
                {
                    HAL_Delay(1000);
                }
            }
            break;
        default:
            break;
    }
}

/// @brief 国赛决赛新物料测试函数，直接将不同的值传入即可
/**
 * @param test_num 功能选择：
 *   -1 - 不执行
 *   0 - 只夹紧爪子
 *   1 - 不断原地夹取并放到车上avoid
 *   2 - 不断无视觉从转盘上抓取物料
 *   3 - 有视觉从转盘上抓取物料
 *   4 - 从车上抓取准备放置
 *   5 - 决赛抓放功能测试
 *   6 - 决赛物料半流程测试
 */
void new_material_test(int test_num,int is_avoid)
{
    
    switch(test_num)
    {
        case 0:
            //只夹紧爪子
            {
                close_claw();
                while(1)
                {
                    HAL_Delay(1000);
                }
            }
            break;
        
        case 1:
            {
                // 原地夹取并放到车上avoid
                old_new_init_car_arm();
                HAL_Delay(1000);
                while(1)
                {
                    if(is_avoid == 1)
                    {
                        new_get_and_load_openloop_avoid(2,1,&default_material_order);
                    }
                    else
                    {
                        new_get_and_load_openloop(2,1,&default_material_order);
                    }
                    HAL_Delay(1000);
                }
            }
            break;
        case 2:
            // 不断无视觉从转盘上抓取物料
            {
                old_new_init_car_arm();
                HAL_Delay(1000);
                plate_get_struct new_plate_get_struct = { .is_avoid = is_avoid, .empty_check = 0, .back_check = 0 };
                while (1)
                {
                    put_claw_down();
                    HAL_Delay(2000);
                    new_get_from_plate_all_movement(3,&new_plate_get_struct,1);
                }
            }
            break;
        case 3:
            // 有视觉从转盘上抓取物料
            {
                old_new_init_car_arm();
                HAL_Delay(1000);
                plate_get_struct new_plate_get_struct_2 = { .is_avoid = is_avoid, .empty_check = 0, .back_check = 1 };
                new_get_from_plate_all_movement(3,&new_plate_get_struct_2,0);
                while (1)
                {
                    HAL_Delay(1000);
                }
            }
            break;
        case 4:
            // 从车上抓取准备放置
            {
                old_new_init_car_arm();
                HAL_Delay(1000);
                while(1)
                {
                    for(int i = 0; i < 3; i++)
                    {
                        if(is_avoid == 1)
                        {
                            new_get_and_pre_put_avoid(i+1,0,1,0,&default_material_order);
                        }
                        else
                        {
                            new_get_and_pre_put(i+1,0,1,0,&default_material_order);
                        }
                        HAL_Delay(2000);
                        put_claw_down_ground_slightly();
                        HAL_Delay(1000);
                        open_claw();
                        HAL_Delay(500);
                        put_claw_up_top();
                        HAL_Delay(1000);
                    }
                    HAL_Delay(1000);
                }
            }
            break;
        case 5:
            // 决赛抓放功能测试
            {
                old_new_init_car_arm();
                HAL_Delay(1000);
                material_get_and_put_struct struct_1  = {
                    .times = 1, 
                    .run_round_number = 3, 
                    .is_load = 1, 
                    .is_pile_up = 0,
                    .is_avoid_collide = is_avoid, 
                    .is_adjust_without_material = 0,
                    .is_none_pile_up_put_adjust = 1, 
                    .is_pile_up_adjust = 0,
                    .is_get_from_car_plate_update = 0, 
                    .is_get_from_ground_check = 0,
                };
                put_claw_up();
                single_line_circle_adjust("CC12\n"); 
                new_material_get_and_put_some_with_load_first(&struct_1,&default_material_order);
                /**
                 *  avoid， 码垛，先空看调整再放
                 *  avoid, 普通，不调整直接放
                 *  avoid  普通，先空看调整再放
                 */
                while(1)
                {
                    HAL_Delay(1000); // 等待放置完成
                }
            }
            break;
        case 6:
            {
                old_new_init_car_arm();
                HAL_Delay(1000);
                // 决赛物料半流程测试
                test_new_material_all_process(is_avoid);
                while(1)
                {
                    HAL_Delay(1000);
                }
            }
            break;
        default:
            // 默认case，提示参数无效
            // print_to_screen(0,"fail");
            break;
    }
}


/// @brief 国赛初赛决赛测试外设函数，直接将不同的值传入即可
/**
 * @param test_num 功能选择：
 *   -1 - 不执行
 *   1 - 测试串口屏曲线
 *   2 - 数字舵机测试
 *   3 - 底盘直线测试
 */
void new_motor_servo_screen_test(int test_num)
{
    old_new_init_car_arm();
    
    switch(test_num)
    {
        case 1:
            {
                //? 测试串口屏曲线
                uint32_t time_now = HAL_GetTick();
                for(int i =0;i<10;i++)
                {
                    //向曲线s0的通道0传输1个数据,add指令不支持跨页面
                    printf("add s0.id,0,%d\xff\xff\xff",(int)(rand() % 256));
                }
                time_now = HAL_GetTick() - time_now;
                char time_str[50];
                sprintf(time_str,"Time : %d ms",time_now);
                print_to_screen(1,time_str); // 打印时间

                while(1)
                {
                    HAL_Delay(1000);
                }
            }
            break;
            
        case 2:
            // ? 数字舵机测试程序
            {
            while(1)
            {
                state_spin_angles(25);
                HAL_Delay(3000);
                state_spin_angles(130);
                HAL_Delay(3000);
            }
            }
            break;
        case 3:
            //? 单独测试底盘直线情况
            {
            while(1)
            {
                move_all_direction_position(acceleration,open_loop_move_velocity,0,150);
                move_all_direction_position_delay(acceleration,open_loop_move_velocity,0,150);
                move_all_direction_position(acceleration,open_loop_move_velocity,0,-150);
                move_all_direction_position_delay(acceleration,open_loop_move_velocity,0,-150);
                HAL_Delay(4000);
            }
            }
            break;
        default:
            break;
    }
}


/// @brief 国赛初赛功能单独测试函数
/**
 * @param test_num 功能选择：
 *   -1 - 不执行
 *   1 - 单独测试old转盘抓取
 *   2 - 单独测试old直线校正
 *   3 - 单独测试old色环放置功能
 *   4 - 单独调试底盘持续定位和放置
 *   5 - 单独测试底盘旋转
 */
void old_function_test(int test_num)
{
    old_new_init_car_arm();
    HAL_Delay(1000);
    switch(test_num)
    {
        case 1:
            {
                //? 单独测试old转盘抓取
                put_claw_down();
                HAL_Delay(1000);
                Reliable_UART_Transmit(&huart3, (uint8_t*)"BB1\n", strlen("BB1\n"), 50);  // 通知树莓派开始识别转盘
                theta_servo_now = middle_arm;
                old_get_from_plate_all_movement(3);
                while(1)
                {
                    HAL_Delay(1000);
                }
            }
            break;
        case 3:
            {
                //? 单独测试old色环放置功能
                put_claw_up();
                single_line_circle_adjust("CC12\n");  // 校正车身位置对准色环
                old_material_get_and_put_some_with_load_first(1,0,1);  // 执行放置动作序列
                while(1)
                {
                    HAL_Delay(1000);
                }
            }
            break;

        case 2:
            //? 单独测试old直线校正
            {
            put_claw_up();
            while(1)
            {   
                single_line_adjust("EE\n");
                int rand_spin = rand() % 2; // 0或1
                if(rand_spin == 0)
                {
                    spin_left_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                    spin_left_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                }
                else
                {
                    spin_right_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                    spin_right_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                }
                HAL_Delay(1000); 
            }
            }
            break;

        case 4:
            //? 单独调试底盘持续定位和放置
            {
            int rand_num_1, rand_num_2;
            put_claw_up();
            while(1)
            {   
                single_line_circle_adjust("CC12\n");  // 校正车身位置对准色环
                old_material_get_and_put_some_with_load_first(1,0,1);
                // HAL_Delay(2000);
                // 生成 -5 到 5 的随机整数（共11个可能值）
                rand_num_1 = rand() % 5 - 2;  // [0,10] -5 -> [-5,5]
                rand_num_2 = rand() % 5 - 2;  // [0,10] -5 -> [-5,5]
                move_all_direction_position(acceleration, open_loop_move_velocity, rand_num_1, rand_num_2);
                move_all_direction_position_delay(acceleration, open_loop_move_velocity, rand_num_1, rand_num_2);
                put_claw_up();
                // HAL_Delay(1000);    
                // 随机旋转动作


                int rand_spin = rand() % 2; // 0或1
                if(rand_spin == 0)
                {
                    spin_left_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                    spin_left_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                }
                else
                {
                    spin_right_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                    spin_right_180(open_loop_spin_velocity,acceleration_spin);
                    HAL_Delay(1500);
                }
            }
            }
            break;
        case 5:
            //? 单独测试底盘旋转
            {
            while(1)
            {
                spin_right_90(open_loop_spin_velocity,acceleration_spin);
                HAL_Delay(2000);
                spin_left_90(open_loop_spin_velocity,acceleration_spin);
                HAL_Delay(2000);
                spin_left_180(open_loop_spin_velocity,acceleration_spin);
                HAL_Delay(2000);
                spin_right_180(open_loop_spin_velocity,acceleration_spin);
                HAL_Delay(4000);
            }
            }
            break;
        default:
            break;
    }
}





/// @brief 国赛决赛新物料的流程测试函数
void test_new_material_all_process(int is_avoid)
{
    //! 以下路径为7.15版本参数，待修改
    // 从初始位置到转盘的移动参数
    // 往前144cm
    float start_move_x = 14; 
    float start_move_y = 14; 
    float move_to_qrcode = 46;
    float move_from_qrcode_to_table = 84;
    float little_back_1 = 2;
    


    //注意加速度改为acceleration-20
    move_all_direction_position(acceleration-20, open_loop_move_velocity, -start_move_x , start_move_y);  // 从启停区出来
    move_all_direction_position_delay(acceleration-20, open_loop_move_velocity, -start_move_x , start_move_y);  
    // HAL_Delay(500);



    move_all_direction_position(acceleration-20, open_loop_move_velocity, 0, move_to_qrcode);  // 出来后移动到二维码前
    move_all_direction_position_delay(acceleration-20, open_loop_move_velocity, 0, move_to_qrcode);



    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table);  // 移动到转盘前
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table);

    spin_right(open_loop_spin_velocity,acceleration_spin,90);  // 右转90度面向转盘
    Reliable_UART_Transmit(&huart3, (uint8_t*)"NN1\n", strlen("NN1\n"), 1000);  // 通知树莓派开始识别转盘
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -little_back_1);  // 微调位置往后
    // move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, -little_back_1);

    put_claw_down();  // 放下机械爪准备抓取

    plate_get_struct new_plate_get_struct = 
    {
        .is_avoid = is_avoid,
        .empty_check = 0,
        .back_check = 1,
    };
    new_get_from_plate_all_movement(2,&new_plate_get_struct,0);  // 执行从转盘抓取物料的动作序列


    put_claw_up();  
    whole_arm_spin(1); 
    arm_stretch();


    //移动到粗加工区参数
    float move_right_length_1 = 40; 
    float move_front_length_1 = 170;  


    move_all_direction_position(acceleration-20, open_loop_x_move_velocity, move_right_length_1,0);  // 向右移动到中轴线
    move_all_direction_position_delay(acceleration-20, open_loop_x_move_velocity, move_right_length_1,0);


    single_line_adjust("EE\n");  // 校正车身姿态与直线平行
    
    
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_1);// 后退到粗加工区
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, -move_front_length_1);


    spin_right_180(open_loop_spin_velocity,acceleration_spin);  // 旋转180度面向色环
    HAL_Delay(1500); 



    material_get_and_put_struct get_and_put = 
    {
        .times = 1,
        .run_round_number = 2,
        .is_avoid_collide = is_avoid,
        .is_load = 1,
        .is_pile_up = 0,
        .is_adjust_without_material = 0,
        .is_none_pile_up_put_adjust = 1,
        .is_pile_up_adjust = 0,
        .is_get_from_car_plate_update = 0,
        .is_get_from_ground_check = 0,

    };
    single_line_circle_adjust("CC12\n");
    new_material_get_and_put_some_with_load_first(&get_and_put,&default_material_order);  // 执行放置动作序列


}



/// @brief 国赛初赛决赛使用 单独的直线校正
/// @param pData 要发送的字符串
void single_line_adjust(char *pData)
{
    static int adjust_delay = 0; //TODO 原先为50
    Reliable_UART_Transmit(&huart3, (uint8_t*)pData, strlen(pData), 1000); // 发送传入的字符串
    is_slight_spin_and_move = 1;
    // HAL_Delay(10);
    tim3_count = 0;
    clear_motor_error();
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit_line)
    // while(is_slight_spin_and_move != 0 )
    {
        slight_spin_and_move(1,0); // 在转盘旁的直线处进行姿态的校正
        HAL_Delay(adjust_delay);
    }
    is_slight_spin_and_move = 0;
    stop();
    // HAL_Delay(5000);

    // 打印曲线
    if(is_show_origin_xy_data == 1)
    {
        print_show_origin_data(1,0,0); // 打印角度误差，x误差，y误差
        print_clear_screen(); // 清空屏幕缓冲区
    }




    HAL_Delay(50);
}



/// @brief 国赛初赛决赛使用 单独的底盘定位
/// @param  
void single_line_circle_adjust(char *pData)
{
    static int adjust_delay = 0; //TODO 原先为50
    Reliable_UART_Transmit(&huart3, (uint8_t*)pData, strlen(pData), 1000);
    is_slight_spin_and_move = 1;
    // HAL_Delay(100);
    tim3_count = 0;
    clear_motor_error();
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    // while(is_slight_spin_and_move != 0 )
    {
        slight_spin_and_move(1,1); // 直线和圆环一起调整
        HAL_Delay(adjust_delay);
    }
    //将电机的误差清零
    stop();
    is_slight_spin_and_move = 0;
    // HAL_Delay(3000);

    // 打印曲线
    if(is_show_origin_xy_data == 1)
    {
        print_show_origin_data(1,1,1); // 打印角度误差，x误差，y误差
        print_clear_screen(); // 清空屏幕缓冲区
    }


    HAL_Delay(50);
}

/// @brief 国赛决赛使用，单独的圆环校正
/// @param pData 
void single_circle_adjust(char* pData)
{
    static int adjust_delay = 50; //TODO 原先为50
    Reliable_UART_Transmit(&huart3, (uint8_t*)pData, strlen(pData), 1000);
    is_slight_spin_and_move = 1;
    // HAL_Delay(100);
    tim3_count = 0;
    clear_motor_error();
    while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    // while(is_slight_spin_and_move != 0 )
    {
        slight_spin_and_move(0,1); // 直线和圆环一起调整
        HAL_Delay(adjust_delay);
    }
    stop();
    is_slight_spin_and_move = 0;
    HAL_Delay(3000);

    // 打印曲线
    if(is_show_origin_xy_data == 1)
    {
        print_show_origin_data(1,1,1); // 打印角度误差，x误差，y误差
        print_clear_screen(); // 清空屏幕缓冲区
    }


    HAL_Delay(50);
}


/// @brief （暂时废弃，用V2）省赛决赛待修改 将物料放置在转盘色环上的全流程函数（省赛版本，非常复杂）
/// @param times 
void new_get_and_put_in_spin_plate_cricle_all(int times)
{
    /**
     * @brief 目前是在前一个颜色转走后，判断下一个颜色即是要放置的颜色，树莓派就发送信号，然后开始抓取并放置。放完就在原地等，以此类推
     * 
     */
    
    Reliable_UART_Transmit(&huart3, (uint8_t*)"PP\n", strlen("PP\n"), 1000);
    int add_count = 0;
    if(times == 2)
    {
        add_count = 3;
    }

    // 先张开看去调整
    put_claw_down();
    open_claw_180();

    servo_adjust_status = 1;
    is_servo_adjust = 1; //! 
    while (is_servo_adjust != 0 ) //等待接收0x39
    {
        adjust_position_with_camera(x_camera_error, y_camera_error,1); 
        HAL_Delay(adjust_position_with_camera_time); 
    }
    //调整完了位置，可以直接放置了

    for(int i = 0; i < 3; i++) 
    {
        is_put_plate = 0;
        tim3_count = 0;
        while(is_put_plate != 1 ) //等待接受信号就放
        {
            HAL_Delay(50);
        }
        
        
        new_get_and_pre_put_spin_plate_avoid_collide(target_colour[i+add_count]);
        is_put_material_in_plate = 0;
        is_third_preput = 0;
        
        put_claw_down();
        HAL_Delay(300);
        open_claw_180();
        HAL_Delay(300);



    }

    arm_stretch();
    put_claw_up();
    whole_arm_spin(1);

    //! 下列为原先版本的，暂时注释掉，可能还能用

    // //? 转回去夹起来等
    // state_spin_without_claw_avoid_collide(target_colour[2+add_count]);
    // open_claw_180();
    // put_claw_up_top();
    // // HAL_Delay(500); //TODO 可能会撞到物料
    // int temp_r_servo_position_plate = r_servo_now;
    // int temp_theta_servo_position_plate = theta_servo_now;
    // arm_shrink(); //TODO 待区分
    // HAL_Delay(700);
    // claw_spin_state_without_claw();
    // HAL_Delay(700);
    // put_claw_down_state();
    // state_spin_without_claw(target_colour[2+add_count]);
    // HAL_Delay(700); //400
    
    // HAL_UART_Transmit(&huart3, (uint8_t*)"wait", strlen("wait"), 1000);
    // is_third_preput = 0;
    // while(is_third_preput != 1)
    // {
    //     HAL_Delay(50);
    // }
    // close_claw();
    // HAL_Delay(600);
    // //close_claw_2(); 
    // put_claw_up_top();
    // HAL_Delay(500); //200

    // claw_spin_front(); //TODO 是否可能撞到
    // feetech_servo_move(4,temp_r_servo_position_plate,4000,180);
    // feetech_servo_move(3,temp_theta_servo_position_plate,4000,180);
    // r_servo_now = temp_r_servo_position_plate;
    // theta_servo_now = temp_theta_servo_position_plate;
    
    // HAL_Delay(400);
    // put_claw_down_near_plate();
    // HAL_Delay(300);

    // is_put_material_in_plate = 0;
    // while(is_put_material_in_plate != 1)
    // {
    //     HAL_Delay(50);
    // }
    // is_put_material_in_plate = 0;


    // //? 不用avoid
    // // state_spin(position);
    // // open_claw();
    // // put_claw_up_top();
    // // // HAL_Delay(500); //TODO 可能会撞到物料
    // // int temp_r_servo_position_plate = r_servo_now;
    // // int temp_theta_servo_position_plate = theta_servo_now;
    // // arm_shrink(); //TODO 待区分
    // // HAL_Delay(500);
    // // claw_spin_state();
    // // // feetech_servo_move(3,middle_3,2000,feet_acc);    
    // // HAL_Delay(700);
    // // put_claw_down_state();
    // // HAL_Delay(700); //400
    // // HAL_UART_Transmit(&huart3, (uint8_t*)"wait", strlen("wait"), 1000);
    // // is_third_preput = 0;
    // // while(is_third_preput != 1)
    // // {
    // //     HAL_Delay(50);
    // // }
    // // close_claw();
    // // HAL_Delay(400);
    // // put_claw_up_top();
    // // HAL_Delay(500); //200
    // // close_claw_2(); 
    // // claw_spin_front(); //TODO 是否可能撞到
    // // feetech_servo_move(4,temp_r_servo_position_plate,4000,feet_acc);
    // // feetech_servo_move(3,temp_theta_servo_position_plate,4000,feet_acc);
    // // r_servo_now = temp_r_servo_position_plate;
    // // theta_servo_now = temp_theta_servo_position_plate;
    // // HAL_Delay(200);
    // // put_claw_down_near_plate();
    // // HAL_Delay(300);



    // put_claw_down();
    // HAL_Delay(300);
    // open_claw_180();
    // HAL_Delay(300);
}



/// @brief 国赛决赛使用 将物料放置在转盘色环上的全流程函数（重写版本，实现单次识别单次放置 //TODO 需要考虑到容易掉落的物料，放慢速度
/// @param times 
void new_get_and_put_in_spin_plate_cricle_all_v2(uint8_t *pData,int times,int is_adjust_once, int is_avoid)
{
    // 发信号开始调整位置
    HAL_UART_Transmit(&huart3, pData, strlen(pData), 1000);
    // Reliable_UART_Transmit(&huart3, (uint8_t*)"LL\n", strlen("LL\n"), 1000);
    int add_count = 0;
    if(times == 2)
    {
        add_count = 3;
    }

    // 先张开看去调整
    put_claw_down();
    open_claw_180();


    //保留得到的调整值
    int theta_servo_adjust_plate = 0;
    int r_servo_adjust_plate = 0;
    //! 注意：以下分支需要切换，保留一个
    //? 只调一次
    if(is_adjust_once == 1)
    {
        is_adjust_plate_servo = 1;
        while(is_adjust_plate_servo != 0)
        {
            HAL_Delay(50);
        }

        //将x_plate_error和y_plate_error存起来
        theta_servo_adjust_plate = x_plate_error;
        r_servo_adjust_plate = y_plate_error;
    }
    else //! 多次调整
    {
        servo_adjust_status = 1;
        is_servo_adjust = 1; //! 
        tim3_count = 0;
        while (is_servo_adjust != 0) //等待接收0x39
        {
            adjust_position_with_camera(x_camera_error, y_camera_error,1); 
            HAL_Delay(adjust_position_with_camera_time); 
        }
    }





    //发信号开始准备放置
    for(int i = 0; i < 3; i++) 
    {
        //TODO 发信号
        is_put_plate = 0;
        while(is_put_plate != 1 ) //等待接受信号就放
        {
            HAL_Delay(50);
        }
        //! 根据是否需要avoid进行选择
        if(is_avoid == 1)
        {
            if(is_adjust_once == 1 && i == 0)
             {
                adjust_plate(theta_servo_adjust_plate, r_servo_adjust_plate);
            }
            new_get_and_pre_put_spin_plate_avoid_collide(target_colour[i+add_count]); 
        }
        else
        {
            if(is_adjust_once == 1 && i == 0)
            {
                adjust_plate(theta_servo_adjust_plate, r_servo_adjust_plate);
            }
            new_get_and_put_spin_plate(target_colour[i+add_count]); 
        }
        open_claw_180(); //放下后张开
    }


}




/// @brief 国赛初赛使用 的物料放置和夹取函数
/// @param times 
/// @param is_pile_up 
void old_material_get_and_put_some_with_load_first( int times,int is_pile_up,int is_load)
{
    int times_count = 0; //用于计算target_colour的索引
    if(times == 3 || times == 4)
    {
        times_count = 3;
    }

    if(is_pile_up == 1) // 码垛，不调整
    {
        for (int i = 0; i < 3; i++)
        {
            
            get_and_pre_put(target_colour[i+times_count],is_pile_up,1, &default_material_order); //夹着物料放置
            servo_adjust_status = target_colour[i+times_count];
            is_servo_adjust = 1;
            tim3_count = 0;
            x_camera_error = 0;
            y_camera_error = 0;
            is_servo_adjust = 0;
            open_claw();
            HAL_Delay(300);
        }
        put_claw_up_top();
        HAL_Delay(500);
    }
    else
    {
        for (int i = 0; i < 3; i++) 
        {
            get_and_pre_put(target_colour[i+times_count], is_pile_up,1, &default_material_order); //夹着物料放置
            servo_adjust_status = target_colour[i+times_count];
            is_servo_adjust = 1;
            tim3_count = 0;
            
            // uint32_t time = HAL_GetTick();
            Reliable_UART_Transmit(&huart3, (uint8_t*)"nearground\n", strlen("nearground\n"), 1000); //发给树莓派，开始校正
            // time = HAL_GetTick() - time; // 计算发送时间
            // char time_str[20];
            // sprintf(time_str, "time: %d ms", time); // 格式化时间字符串
            // print_to_screen(1,time_str); // 打印发送时间到口屏


    


            uart_receive_count3 = 0;
            while (is_servo_adjust != 0 && tim3_count < timeout_limit_line) 
            {
                // x_camera_error_array[x_camera_error_array_index] = x_camera_error;
                // y_camera_error_array[y_camera_error_array_index] = y_camera_error;
                // x_camera_error_array_index++;
                // y_camera_error_array_index++;
                adjust_position_with_camera(x_camera_error, y_camera_error,1);  
				// adjust_position_with_camera_new(x_camera_error,y_camera_error,adjust_position_with_camera_time);
                HAL_Delay(adjust_position_with_camera_time);  //10
            }

            // char temp_str[20];
            // sprintf(temp_str, "uart: %d", uart_receive_count3);
            // print_to_screen(0,temp_str);
            // uart_receive_count3 = 0;


            // HAL_Delay(1000);
            theta_servo_value[target_colour[i+times_count]] = theta_servo_now;
            r_servo_value[target_colour[i+times_count]] = r_servo_now;
            is_servo_adjust = 0;
            put_claw_down_ground_slightly();
            HAL_Delay(500);

            // 打印曲线
            if(is_show_origin_xy_data == 1)
            {
                print_show_origin_data(0,1,1); // 打印角度误差，x误差，y误差
                print_clear_screen(); // 清空屏幕缓冲区
            }

            open_claw();
            HAL_Delay(300);


            
            
        }
        put_claw_up_top();
        HAL_Delay(500);
        open_claw_bigger(); //防止夹不到物料
    }
    if(is_load == 1)
    {
        int times_count = 0;
        if(times == 3 || times == 4)
        {
            times_count = 3;
        }
        for(int i = 0; i < 3; i++)
        {
            get_and_load_openloop(target_colour[i+times_count],0,&default_material_order); // 开环抓取
        }
    }
    whole_arm_spin(1);
    open_claw_180();
    arm_stretch();
    HAL_Delay(10);
    
}



/// @brief 国赛决赛使用 物料放置和夹取函数（新版本）
/// @param new_get_and_put_struct 物料抓放结构体
void new_material_get_and_put_some_with_load_first(material_get_and_put_struct* new_get_and_put_struct, material_order* order)
{
    
    
    int times_count = 0; //用于计算target_colour的索引
    if(new_get_and_put_struct->times == 3 || new_get_and_put_struct->times == 4)
    {   
        times_count = 3;
    }

    // 从车上抓取（是否更新位置）
    for(int i = 0 ; i< new_get_and_put_struct->run_round_number ; i++)
    {
        if(new_get_and_put_struct-> is_adjust_without_material == 1)// 先调整好再回去抓，无法更新偏差值
        {
            new_get_and_pre_put_void(target_colour[i+times_count], new_get_and_put_struct->is_pile_up, order); 
            servo_adjust_status = target_colour[i+times_count];
            is_servo_adjust = 1;
            tim3_count = 0;
            put_claw_down_near_ground();
            Reliable_UART_Transmit(&huart3, (uint8_t*)"nearground\n", strlen("nearground\n"), 1000);
            while (is_servo_adjust != 0 ) 
            {
                adjust_position_with_camera(x_camera_error, y_camera_error,1);  
                HAL_Delay(adjust_position_with_camera_time); 
            }
            theta_servo_value[target_colour[i+times_count]] = theta_servo_now;
            r_servo_value[target_colour[i+times_count]] = r_servo_now;
            is_servo_adjust = 0;

            //? 回去直接抓然后放
            if(new_get_and_put_struct->is_avoid_collide == 1)
            {
                new_get_and_pre_put_avoid(target_colour[i+times_count], new_get_and_put_struct->is_pile_up,0,new_get_and_put_struct->is_get_from_car_plate_update, order); 
            }
            else
            {
                new_get_and_pre_put(target_colour[i+times_count], new_get_and_put_struct->is_pile_up,0,new_get_and_put_struct->is_get_from_car_plate_update, order); 
            }


            if(new_get_and_put_struct->is_pile_up == 1)
            {
                put_claw_down_pile_slightly();
            }
            else
            {
                put_claw_down_ground_slightly();
                HAL_Delay(200);
            }
            HAL_Delay(1000);
            open_claw();
            HAL_Delay(300);


        }
        else //直接抓着调
        {
            //是否需要avoid
            if(new_get_and_put_struct->is_avoid_collide == 1)
            {
                new_get_and_pre_put_avoid(target_colour[i+times_count], new_get_and_put_struct->is_pile_up, 1,new_get_and_put_struct->is_get_from_car_plate_update,order); 
            }
            else
            {
                new_get_and_pre_put(target_colour[i+times_count], new_get_and_put_struct->is_pile_up, 1,new_get_and_put_struct->is_get_from_car_plate_update,order); 
            }

            // 需要调整（普通或者码垛）
            if((new_get_and_put_struct->is_pile_up_adjust && new_get_and_put_struct->is_pile_up == 1)|| (new_get_and_put_struct->is_pile_up == 0 && new_get_and_put_struct->is_none_pile_up_put_adjust == 1))
            {
                servo_adjust_status = target_colour[i+times_count];
                is_servo_adjust = 1;
                tim3_count = 0;
                Reliable_UART_Transmit(&huart3, (uint8_t*)"nearground\n", strlen("nearground\n"), 1000); 
                while (is_servo_adjust != 0 && tim3_count < 400) 
                {
                    adjust_position_with_camera(x_camera_error, y_camera_error,1);  
                    HAL_Delay(adjust_position_with_camera_time);
                }
                theta_servo_value[target_colour[i+times_count]] = theta_servo_now;
                r_servo_value[target_colour[i+times_count]] = r_servo_now;
                is_servo_adjust = 0;
            }

            //放置
            if(new_get_and_put_struct->is_pile_up == 1)
            {
                put_claw_down_pile_slightly();
            }
            else
            {
                put_claw_down_ground_slightly();
            }
            HAL_Delay(700);
            open_claw();
            HAL_Delay(300);
        }
        
    }
    put_claw_up_top();
    HAL_Delay(500);
    open_claw_bigger(); //防止夹不到物料

    // 抓取（是否抓取）
    if(new_get_and_put_struct->is_load == 1)
    {
        for(int i = 0; i < new_get_and_put_struct->run_round_number; i++)
        {
            if(new_get_and_put_struct->is_avoid_collide == 1) //是否从侧面过
            {
                new_get_and_load_openloop_avoid(target_colour[i+times_count],!new_get_and_put_struct->is_none_pile_up_put_adjust,order);
            }
            else
            {
                new_get_and_load_openloop(target_colour[i+times_count],!new_get_and_put_struct->is_none_pile_up_put_adjust,order); 
            }
        }
    }
    whole_arm_spin(1);
    open_claw_180();
    arm_stretch();
    HAL_Delay(10);

}



/// @brief 国赛初赛决赛使用  从转盘抓物料的动作（需要提前完成夹爪放低的动作，这个函数的第一个动作就是抓取）
/// @param  
void old_get_from_plate_all_movement(int n)
{
    is_start_get_plate = 1; //开始从转盘抓取物料
    int r_servo_now_temp = 0;
    int theta_servo_now_temp = 0;
    int is_first_get = 1;
    tim3_count = 0;
    while(get_plate_count < n && tim3_count < 6000) //TODO 从转盘抓取n个色环或者超时，如果empty抓空，是否能给一个延时后直接离开
    {
        is_adjust_plate_servo = 1; //开始根据物料在转盘上的位置调整机械臂
        HAL_Delay(10);
        int temp_plate=0;
        if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0)) //从树莓派得到任务且未重复
        {
            is_adjust_plate_servo = 0;
            temp_plate = get_plate;
            get_plate = 0;
            HAL_Delay(50);
            char temp_str[20];
            sprintf(temp_str, "x=%d,y=%d", x_plate_error, y_plate_error);
            print_to_screen(1,temp_str);
            adjust_plate(x_plate_error, y_plate_error);

            // //将x_plate_error和y_plate_error printf
            // char temp[20];
            // sprintf(temp, "t0.txt=\"%d,%d\"\xff\xff\xff", x_plate_error, y_plate_error); 

					// printf("%s", temp);
            // char temp_str[20];
            sprintf(temp_str, "x=%d,y=%d", x_plate_error, y_plate_error);
            print_to_screen(2,temp_str);

            x_plate_error = 0;
            y_plate_error = 0;
            state_spin_without_claw(temp_plate);
            close_claw();
            if(is_get_empty_finish == 0)
            {
                start_judge_empty = 1; //首次抓取则开始判断是否抓空
            }
            HAL_Delay(600);
            // close_claw_2(); //! 暂时使用，
            put_claw_up_top();
            HAL_Delay(10); //delate
            if(is_get_empty_finish == 0) //首次抓取则延时1.5s，判断是否抓空
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
                if(is_first_get == 1)
                {
                    r_servo_now_temp = r_servo_now;
                    theta_servo_now_temp = theta_servo_now;
                    is_first_get = 0; //第一次抓取时记录r_servo_now
                }
                
                is_get_empty_finish = 1;
                arm_shrink();
                HAL_Delay(300);
                claw_spin_state_without_claw();
                HAL_Delay(600); 
                put_claw_down_state(); 
                HAL_Delay(300); 
                open_claw();
                HAL_Delay(300);
                x_plate_error = 0;
                y_plate_error = 0;
                put_claw_up_top(); 
                HAL_Delay(400); 
                claw_spin_front();
                open_claw_180();
                // feetech_servo_move(4,r_servo_now_temp,4000,feet_acc);
                // feetech_servo_move(3,theta_servo_now_temp,4000,feet_acc);
                HAL_Delay(500);
                arm_stretch();
                whole_arm_spin(1); //TODO 可能会撞到物料
                get_plate_count++;
                if(temp_plate == 1)
                {
                    is_1_get = 1;
                }
                else if(temp_plate == 2)
                {
                    is_2_get = 1;
                }
                else if(temp_plate == 3)
                {
                    is_3_get = 1;
                }
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
    is_start_get_plate = 0;
}

/// @brief 国赛决赛使用的转盘抓取物料函数
/// @param n 
/// @param plate_struct 
void new_get_from_plate_all_movement(int n, plate_get_struct* plate_struct ,int is_get_without_vision)
{
    is_start_get_plate = 1; //开始从转盘抓取物料
    int r_servo_now_temp = 0;
    int theta_servo_now_temp = 0;
    int is_first_get = 1;
    tim3_count = 0;
    while(get_plate_count < n && tim3_count < 6000) //TODO 从转盘抓取n个色环或者超时，如果empty抓空，是否能给一个延时后直接离开
    {
        is_adjust_plate_servo = 1; //开始根据物料在转盘上的位置调整机械臂
        HAL_Delay(10);
        int temp_plate=0;
        if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0) || is_get_without_vision == 1) //从树莓派得到任务且未重复
        {
            if(is_get_without_vision == 1) //! 无视觉的抓取
            {
                get_plate = 1;
            }
            is_adjust_plate_servo = 0;
            temp_plate = get_plate;
            get_plate = 0;
            HAL_Delay(50);
            adjust_plate(x_plate_error, y_plate_error);
            x_plate_error = 0;
            y_plate_error = 0;
            state_spin_without_claw(temp_plate);
            close_claw();
            HAL_Delay(1000);
            put_claw_up_top_slight();
            arm_shrink();
            HAL_Delay(800);
            claw_spin_state_without_claw_slight();
            HAL_Delay(800); 


            start_check_plate_back_state = 0;
            //?判断空抓
            if(plate_struct->back_check == 1)
            {
                start_check_plate_back_state = 1;
                Reliable_UART_Transmit(&huart3, (uint8_t*)"check\n", strlen("check\n"), 1000); //发给树莓派，开始判断是否抓空
                HAL_Delay(1000); //只是延时，等待树莓派判断
            }
            if(plate_struct->back_check == 1 && start_check_plate_back_state == 0) //抓空
            {
                claw_spin_front();
                HAL_Delay(500);
                open_claw_180();
                HAL_Delay(500);
                arm_stretch();
                put_claw_down();
                get_plate = 0;
            }
            else
            {
                if(plate_struct->is_avoid == 0)
                {
                    put_claw_down_state(); 
                    HAL_Delay(300); 
                    open_claw();
                    HAL_Delay(200);
                    put_claw_up_top();
                    HAL_Delay(400); 
                    claw_spin_front();
                    open_claw_180();
                }
                else
                {
                    put_claw_down_state(); 
                    HAL_Delay(300);  
                    open_claw_avoid_collide();
                    HAL_Delay(200);
                    // put_claw_up_top();
                    state_spin_without_claw_avoid_collide(temp_plate);
                    HAL_Delay(500);
                    claw_spin_front();
                    HAL_Delay(600);
                    open_claw_180();
                    put_claw_up_top(); 
                }



                HAL_Delay(500);
                arm_stretch();
                whole_arm_spin(1); 
                get_plate_count++;
                if(temp_plate == 1)
                {
                    is_1_get = 1;
                }
                else if(temp_plate == 2)
                {
                    is_2_get = 1;
                }
                else if(temp_plate == 3)
                {
                    is_3_get = 1;
                }
                get_plate = 0;
                put_claw_down();
                HAL_Delay(1000);
            } 
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
    is_start_get_plate = 0;
}


/// @brief 国赛决赛使用 （未优化）转盘抓取（带有判断空抓）（先回到物料盘再判断）（需要提前完成爪子放低的动作）
/// @param  
void new_get_from_plate_all_movement_with_back_check(void)
{
    is_start_get_plate = 1; //开始从转盘抓取物料
    while(get_plate_count < 3 ) //TODO 从转盘抓取三个色环或者超时，如果empty抓空，是否能给一个延时后直接离开
    {
        is_adjust_plate_servo = 1; //开始根据物料在转盘上的位置调整机械臂
        HAL_Delay(10);
        int temp_plate=0;
        if((get_plate == 1 && is_1_get == 0)|| (get_plate == 2 && is_2_get == 0) || (get_plate == 3 && is_3_get == 0)) //从树莓派得到任务且未重复
        {
            is_adjust_plate_servo = 0;
            temp_plate = get_plate;
            get_plate = 0;
            HAL_Delay(50);
            adjust_plate(x_plate_error, y_plate_error);
            x_plate_error = 0;
            y_plate_error = 0;
            state_spin_without_claw(temp_plate);
            close_claw();
            HAL_Delay(600);
            put_claw_up_top();
            HAL_Delay(10); //delate
                int r_servo_now_temp = r_servo_now;
                arm_shrink();
                HAL_Delay(300);
                // close_claw_2(); 
                claw_spin_state_without_claw();
                HAL_Delay(700); 

                start_check_plate_back_state = 1;
                Reliable_UART_Transmit(&huart3, (uint8_t*)"check\n", strlen("check\n"), 1000); //发给树莓派，开始判断是否抓空
                HAL_Delay(1000); //只是延时，等待树莓派判断
                if(start_check_plate_back_state == 0) //抓空
                {
                    claw_spin_front();
                    HAL_Delay(500);
                    open_claw_180();
                    HAL_Delay(500);
                    arm_stretch();
                    put_claw_down();
                    get_plate = 0;
                }
                else
                {
                HAL_Delay(600); //? 
                put_claw_down_state(); //?
                HAL_Delay(300);  //?
                open_claw();
                HAL_Delay(300);
                // arm_stretch();
                // r_servo_now = r_servo_now_temp;
                x_plate_error = 0;
                y_plate_error = 0;
                // adjust_plate(x_plate_error, y_plate_error);
                x_plate_error = 0;
                y_plate_error = 0;
                put_claw_up_top(); 
                HAL_Delay(400); //?
                state_spin_without_claw_avoid_collide(temp_plate);
                HAL_Delay(800);
                claw_spin_front();
                open_claw_avoid_collide();
                // open_claw_180();
                // arm_stretch();
                HAL_Delay(700);
                arm_stretch();
                open_claw_180();
                HAL_Delay(500);
                get_plate_count++;
                if(temp_plate == 1)
                {
                    is_1_get = 1;
                    printf("t1.txt=\"1\"\xff\xff\xff");
                }
                else if(temp_plate == 2)
                {
                    is_2_get = 1;
                    printf("t2.txt=\"2\"\xff\xff\xff");
                }
                else if(temp_plate == 3)
                {
                    is_3_get = 1;
                    printf("t3.txt=\"3\"\xff\xff\xff");
                }
                get_plate = 0;
                put_claw_down();
                }
                start_check_plate_back_state = 0;
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
    is_start_get_plate = 0;
}


/// @brief 国赛决赛使用 从地上抓取随机顺序摆放的物料全流程函数(注意，这个函数使用之前先纯直线，然后在第一个nearground后调整位置)
/// @param times 1为第一轮，2为第二轮
void new_get_from_ground_in_random_position(int times)
{
    put_claw_down_near_ground();
    HAL_Delay(1000);
    Reliable_UART_Transmit(&huart3, (uint8_t*)"nearground\n", strlen("nearground\n"), 1000); //开始识别最中间的物料
    is_slight_spin_and_move =1;
    tim3_count = 0;
    while(is_slight_spin_and_move != 0) 
    {
        slight_spin_and_move(0,1); //根据色块定位
        HAL_Delay(50);
    }
    is_slight_spin_and_move = 0;
    stop(); //根据中间物料的中心调整底盘


    is_get_material_from_temp_area = 2; 
    new_get_and_pre_put_void(1,0, &default_material_order); //看最右侧的物料
    HAL_Delay(500);
    Reliable_UART_Transmit(&huart3, (uint8_t*)"nearground\n", strlen("nearground\n"), 1000); //识别最右侧的物料
    while(is_get_material_from_temp_area != 3) //等待树莓派返回识别结果
    {
        HAL_Delay(100);
    }
    put_claw_up_top();
    HAL_Delay(1000);
    int temp_add = 0;
    if(times == 2)
    {
        temp_add = 3;
    }
    int temp_get_order[3] = {2,1,3};//本次抓取的位置顺序，123对应的是右中左
    for(int i = 0 ; i < 3 ; i++) //核心代码判断
    {
        for(int j = 0 ; j < 3 ; j++)
        {
            if(target_colour[i+temp_add] == material_place[j])
            {
                temp_get_order[i] = j+1;
                HAL_Delay(10);
                break; 
            }
        }
    }
    for(int i = 0; i < 3; i++)
    {
        new_get_and_load_openloop_with_temp_put(temp_get_order[i],target_colour[i+temp_add]); // 开环抓取，avoid
    }
    whole_arm_spin(1);
    arm_stretch();
    put_claw_up_top();
}


/// @brief 初始化车的机械臂动作
/// @param  
void old_new_init_car_arm(void)
{
    HAL_Delay(100);
    arm_stretch();                // 机械臂伸缩位置初始化
    whole_arm_spin(1);           // 中板旋转位置初始化
    put_claw_up_top();           // 机械爪抬起
    claw_spin_front();           // 机械爪旋转到正前方
    open_claw_180();             // 机械爪完全张开
    state_spin_without_claw(1);  // 载物盘旋转到1号位
}

/// @brief 国赛初赛全流程函数
/// @param  
void old_all_process(void)
{
    /************初始化和第一次前往转盘*************/
    /************初始化和第一次前往转盘*************/
    /************初始化和第一次前往转盘*************/

    //? 初始化机械臂
    old_new_init_car_arm();


    is_adjust_motor_in_tim = 0;
    motor_state = 1;

    // HAL_Delay(1000); //TODO 7.9去除，为了加快速度
    if(is_single_route_test != 1)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 1000);  // 开始识别二维码
        Reliable_UART_Transmit(&huart3,(uint8_t*)"AA\n", strlen("AA\n"), 1000); // 通知树莓派开始识别二维码
    }
    

    //! wifi接收（不用的话保持注释）
    //TODO 仍待优化和测试
    // HAL_Delay(2000);
    // Reliable_UART_Transmit(&huart3, (uint8_t*)"MM\n", strlen("MM\n"), 1000);
    // char temp_wifi_printf[50];
    // Reliable_UART_Transmit(&huart4, (uint8_t*)"AT+CIPMUX=0\r\n", strlen("AT+CIPMUX=0\r\n"), 1000);
    // HAL_Delay(500);
    // Reliable_UART_Transmit(&huart4, (uint8_t*)"AT+CIPMODE=1\r\n", strlen("AT+CIPMODE=1\r\n"), 1000);
    // HAL_Delay(500);
    // Reliable_UART_Transmit(&huart4, (uint8_t*)"AT+CIPSTART=\"TCP\",\"192.168.43.42\",8089\r\n", strlen("AT+CIPSTART=\"TCP\",\"192.168.43.42\",8089\r\n"), 1000);
    // HAL_Delay(500);
    // Reliable_UART_Transmit(&huart4, (uint8_t*)"AT+CIPMODE=1\r\n", strlen("AT+CIPMODE=1\r\n"), 1000);
    // HAL_Delay(500);
    // while(is_wifi_already_get_message != 1)
    // {
    //     HAL_Delay(100);
    // } 
    // sprintf(temp_wifi_printf, "%s",wifi_massage);
    // Reliable_UART_Transmit(&huart3, (uint8_t*)temp_wifi_printf, strlen(temp_wifi_printf), 1000); //发送wifi数据
    // HAL_Delay(100);



    // 从初始位置到转盘的移动参数
    // 往前144cm
    float start_move_x = 14; 
    float start_move_y = 14; 
    float move_to_qrcode = 46;
    float move_from_qrcode_to_table = 84;
    float little_back_1 = 2;
    


    //注意加速度改为acceleration-20
    move_all_direction_position(acceleration-20, open_loop_move_velocity, -start_move_x , start_move_y);  // 从启停区出来
    move_all_direction_position_delay(acceleration-20, open_loop_move_velocity, -start_move_x , start_move_y);  
    // HAL_Delay(500);



    move_all_direction_position(acceleration-20, open_loop_move_velocity, 0, move_to_qrcode);  // 出来后移动到二维码前
    move_all_direction_position_delay(acceleration-20, open_loop_move_velocity, 0, move_to_qrcode);



    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table);  // 移动到转盘前
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table);

    
    //显示二维码信息
    sprintf(target_colour_str, "%d%d%d+%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]); 
    print_to_screen(0,target_colour_str);


    spin_right(open_loop_spin_velocity,acceleration_spin,90);  // 右转90度面向转盘


    if(is_single_route_test != 1)
    {
        // HAL_UART_Transmit(&huart3, (uint8_t*)"BB1", strlen("BB1"), 1000);  // 通知树莓派开始识别转盘
        Reliable_UART_Transmit(&huart3, (uint8_t*)"BB1\n", strlen("BB1\n"), 1000);  // 通知树莓派开始识别转盘
    }


    HAL_Delay(1000);
    put_claw_up();


    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -little_back_1);  // 微调位置往后
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, -little_back_1);

    if(is_single_route_test != 1)
    {
        put_claw_down();  // 放下机械爪准备抓取
        old_get_from_plate_all_movement(3);  // 执行从转盘抓取物料的动作序列
    }
    else
    {
        HAL_Delay(3000);
    }


    //! 姿态的恢复
    put_claw_up();  
    whole_arm_spin(1); 
    arm_stretch();








    /**************第一次  转盘  粗加工区*****************/
    /**************第一次  转盘  粗加工区*****************/
    /**************第一次  转盘  粗加工区*****************/





    
    //移动到粗加工区参数
    float move_right_length_1 = 40; 
    float move_front_length_1 = 170;  


    move_all_direction_position(acceleration-20, open_loop_x_move_velocity, move_right_length_1,0);  // 向右移动到中轴线
    move_all_direction_position_delay(acceleration-20, open_loop_x_move_velocity, move_right_length_1,0);


    single_line_adjust("EE\n");  // 校正车身姿态与直线平行
    
    
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_1);// 后退到粗加工区
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, -move_front_length_1);


    spin_right_180(open_loop_spin_velocity,acceleration_spin);  // 旋转180度面向色环
    HAL_Delay(1500); 


    if(is_single_route_test != 1)
    {
        single_line_circle_adjust("CC12\n");  // 校正车身位置对准色环
        old_material_get_and_put_some_with_load_first(1,0,1);  // 执行放置动作序列
    }
    else
    {
        single_line_circle_adjust("CC4\n");
        HAL_Delay(3000);
    }
    





    /**************第一次  粗加工区  暂存区*****************/
    /**************第一次  粗加工区  暂存区*****************/
    /**************第一次  粗加工区  暂存区*****************/





    //移动到暂存区参数
    int move_front_length_2 = 82; 
    int move_back_length_2 = 83; 
    
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_2);  // 后退到十字中心
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, -move_back_length_2);


    put_claw_up();//! 姿态的恢复
    arm_stretch();


    spin_right_90(open_loop_spin_velocity,acceleration_spin);  // 右转90度面向暂存区
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_2 );  // 前进到暂存区
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0,move_front_length_2 );


    if(is_single_route_test != 1)
    {
        single_line_circle_adjust("CC12\n");  // 校正位置对准暂存区
        old_material_get_and_put_some_with_load_first(2,0,0);  // 执行放置动作序列
    }
    else
    {
        single_line_circle_adjust("CC4\n");  // 校正位置对准暂存区
        HAL_Delay(3000);
    }
    
    

    /**************第二次  转盘*****************/
    /**************第二次  转盘*****************/
    /**************第二次  转盘*****************/




    //移动回转盘参数
    int move_right_length_b = 46.5;
    int move_front_length_b = 88;
    

    spin_right(open_loop_spin_velocity,acceleration_spin, 90);  // 右转90度
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_b);  // 前进到转盘左侧
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0,move_front_length_b);

    move_all_direction_position(acceleration, open_loop_x_move_velocity,move_right_length_b, 0);  // 向右移动到转盘
    move_all_direction_position_delay(acceleration, open_loop_x_move_velocity,move_right_length_b, 0);


    if(is_single_route_test != 1)
    {
        Reliable_UART_Transmit(&huart3, (uint8_t*)"BB2\n", strlen("BB2\n"), 1000);  // 通知树莓派开始识别转盘
    }

    if(is_single_route_test != 1)
    {
        put_claw_down();
        old_get_from_plate_all_movement(3);
    }
    else
    {
        HAL_Delay(3000);
    }

    put_claw_up();//! 姿态的恢复
    whole_arm_spin(1); 
    arm_stretch();




    /**************第二次  转盘  粗加工区*****************/
    /**************第二次  转盘  粗加工区*****************/
    /**************第二次  转盘  粗加工区*****************/




    //移动到粗加工区参数
    float move_right_length_3 = 39; 
    float move_front_length_3 = 170;  


    move_all_direction_position(acceleration-20, open_loop_x_move_velocity, move_right_length_3,0);  // 向右移动到中轴线
    move_all_direction_position_delay(acceleration-20, open_loop_x_move_velocity, move_right_length_3,0);


    single_line_adjust("EE\n");


    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_3);  // 后退到粗加工区
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, -move_front_length_3);


    put_claw_up();//! 姿态的恢复

    spin_right_180(open_loop_spin_velocity,acceleration_spin);  // 旋转180度面向色环
    HAL_Delay(1500);

    if(is_single_route_test != 1)
    {
        single_line_circle_adjust("CC3\n");
        old_material_get_and_put_some_with_load_first(3,0,1);
    }
    else
    {
        single_line_circle_adjust("CC4\n");
        HAL_Delay(3000);
    }


    /**************第二次  粗加工区  暂存区*****************/
    /**************第二次  粗加工区  暂存区*****************/
    /**************第二次  粗加工区  暂存区*****************/

    //移动到暂存区参数
    float move_front_length_4 = 82; 
    float move_back_length_4 = 83; 

    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_4);  // 后退到十字中心
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0, -move_back_length_4);
    // HAL_Delay(2000);

    put_claw_up();//! 姿态的恢复
    arm_stretch();

    spin_right_90(open_loop_spin_velocity,acceleration_spin);  // 右转90度面向暂存区
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_4);  // 前进到暂存区
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0,move_front_length_4);
    // HAL_Delay(1800);


    if(is_single_route_test != 1)
    {
    //! 注意：两个函数的配合使用（包括发送的字母标志位）
        single_line_circle_adjust("CC4\n");
        old_material_get_and_put_some_with_load_first(4,1,0);
    }
    else
    {
        single_line_circle_adjust("CC4\n");
        HAL_Delay(3000);
    }
    

    /**************暂存区  原点*****************/
    /**************暂存区  原点*****************/
    /**************暂存区  原点*****************/

    //移动回原点参数
    int move_45_length_5_x = 20.5;
    int move_45_length_5_y = 28;
    int move_front_length_5 = 75.5;
    int move_back_length_5 = 169;
    int move_right_length_5 = 0.1;

    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length_5,0);  
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, move_right_length_5,0);


    move_all_direction_position(acceleration, open_loop_move_velocity, 0,-move_back_length_5);  
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0,-move_back_length_5);


    open_claw_180();
    whole_arm_spin(1); 
    arm_stretch();


    spin_right(open_loop_spin_velocity,acceleration_spin, 90);
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_5);  
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, 0,move_front_length_5);  


    move_all_direction_position(acceleration, open_loop_move_velocity, move_45_length_5_x, move_45_length_5_y);
    move_all_direction_position_delay(acceleration, open_loop_move_velocity, move_45_length_5_x, move_45_length_5_y);


    Reliable_UART_Transmit(&huart3, (uint8_t*)"end\n", strlen("end\n"), 1000);

    while(1)
    {
        HAL_Delay(1000);
    }
}


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!                以下函数不用修改

/// @brief 带有重试机制的UART发送函数
/// @param huart 
/// @param pData 
/// @param Size 
/// @param Timeout 
/// @return 
HAL_StatusTypeDef Reliable_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    HAL_StatusTypeDef status = HAL_ERROR; // 初始状态设置为非OK，确保进入循环
    int retry_count = 0;

    while (status != HAL_OK && retry_count < 10)
    {
        if (retry_count > 0)
        {
            // 如果不是第一次尝试，打印重试信息 (可选，需要有调试输出功能)
            // 例如：printf("UART Transmit failed (Status: %d). Retrying %d/%d...\r\n", status, retry_count, MAX_UART_TRANSMIT_RETRIES);
            print_to_screen(1,"fail");
            // 或者在调试器中观察 retry_count
            HAL_Delay(100); // 等待一小段时间再重试
        }

        // 调用 HAL 发送函数
        status = HAL_UART_Transmit(huart, pData, Size, Timeout);

        if (status != HAL_OK)
        {
            retry_count++;
        }
    }

    if (status != HAL_OK)
    {
        // 达到最大重试次数后仍未成功，打印失败信息 (可选)
        // 例如：printf("UART Transmit failed after %d retries (Final Status: %d).\r\n", MAX_UART_TRANSMIT_RETRIES, status);
    }
    else
    {
        // 成功发送 (可选)
        // 例如：printf("UART Transmit successful after %d retries.\r\n", retry_count);
    }

    // 返回最终的状态
    return status;
}

/// @brief 国赛初赛决赛使用 串口屏调试函数
/// @param t_num 串口屏里的tx
/// @param pData 要发送的字符串
void print_to_screen(int t_num,char *pData)
{
    char temp_data[40];
    // t_num 对应"tnum.txt=\"%s\"\xff\xff\xff"
    sprintf(temp_data, "t%d.txt=\"%s\"\xff\xff\xff", t_num, pData);
    printf("%s", temp_data);
    HAL_Delay(20);
}

/// @brief 将串口屏波形清零一段
void print_clear_screen()
{
    for(int i = 0 ; i < 50; i++)
    {
        printf("add s0.id,0,128\xff\xff\xff");
        HAL_Delay(1);
        printf("add s0.id,1,128\xff\xff\xff");
        HAL_Delay(1);
        printf("add s0.id,2,128\xff\xff\xff");
        HAL_Delay(1);
    }
}

/// @brief 将数据发送到串口屏波形图
/// @param theta 
/// @param x 
/// @param y 
void print_show_origin_data(int theta,int x, int y)
{
    int k_max = 6;
    int xy_k = 5;
    int theta_k = 2;
    for(int i = 0 ; i<= print_screen_buffer_index_xy; i++)
    {
        if(theta == 1)
        {
            int mapped_theta = (int)(((print_screen_buffer_xy[i][0] *theta_k+ 5) * 256.0) / 10.0);
            if(mapped_theta < 0) mapped_theta = 0;
            if(mapped_theta > 256) mapped_theta = 256;
            for(int k = 0; k < k_max; k++)
            {
                printf("add s0.id,0,%d\xff\xff\xff", mapped_theta); // 打印映射后的角度误差
                HAL_Delay(1);
            }
            // HAL_Delay(5);
        }
        else
        {
            for(int k = 0; k < k_max; k++)
            {
                printf("add s0.id,0,128\xff\xff\xff"); // 打印映射后的角度误差
                HAL_Delay(1);
            }
        }
        if(x == 1)
        {
            int mapped_x = print_screen_buffer_xy[i][1] * xy_k + 128;

            if(mapped_x < 0) mapped_x = 0;
            if(mapped_x > 256) mapped_x = 256;
            for(int k = 0; k < k_max; k++)
            {
                printf("add s0.id,1,%d\xff\xff\xff", mapped_x); // 打印映射后的x误差
                HAL_Delay(1);
            }
        }
        else
        {
            for(int k = 0; k < k_max; k++)
            {
                printf("add s0.id,1,128\xff\xff\xff"); // 打印映射后的角度误差
                HAL_Delay(1);
            }
        }
        if(y == 1)
        {
            int mapped_y = print_screen_buffer_xy[i][2] * xy_k + 128;
            if(mapped_y < 0) mapped_y = 0;
            if(mapped_y > 256) mapped_y = 256;
            for(int k = 0; k < k_max; k++)
            {
                printf("add s0.id,2,%d\xff\xff\xff", mapped_y); // 打印映射后的y误差
            }
        }
        else
        {
            for(int k = 0; k < k_max; k++)
            {
                printf("add s0.id,2,128\xff\xff\xff"); // 打印映射后的角度误差
                HAL_Delay(1);
            }
        }
        // HAL_Delay(5);
    }
    print_screen_buffer_index_xy = 0; // 清空屏幕缓冲区索引
    //将数组清空
    for(int i = 0 ; i < max_data_length ; i++)
    {
        print_screen_buffer_xy[i][0] = 128;
        print_screen_buffer_xy[i][1] = 128;
        print_screen_buffer_xy[i][2] = 128;
    }
}


/// @brief 国赛初赛决赛使用 串口空闲中断回调函数
/// @param huart 
/// @param Size 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_IT(huart, rxdata_u3, 40);
        is_raspi_get_massage = 1;
        UART_handle_function_3();
    }
    else if(huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_IT(huart, rxdata_u4, 40);
        if(rxdata_u4[3] == '+' )
        {
            for(int i = 0 ; i < 7 ; i ++)
            {
                wifi_massage[i] = rxdata_u4[i];
            }
            if(is_wifi_already_get_message == 0)
            {
                is_wifi_already_get_message = 1;
            }
        }
        memset(rxdata_u4, 0, sizeof(rxdata_u4));
    }
}



//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//!      以下皆是单独功能测试程序




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
