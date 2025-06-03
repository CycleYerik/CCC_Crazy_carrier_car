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


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!      重要变量（可能要根据不同情况进行修改的）


int is_put_adjust_with_material = 1 ; // 1则为夹着物料进行调整，0则为不夹着物料进行调整
int is_pile_adjust = 0; // 1则为码垛时细调整，0为不调整


int is_get_material_from_temp_area = 0; // 是否从暂存取物料

int is_single_route_test = 0; // 1则单独路径移动

//! 目标颜色数组
volatile int target_colour[6] = {2,3,1,1,3,2}; // 物料颜色序列(1红,2绿,3蓝)
volatile int material_place[3] = {0,0,0}; //从暂存区夹取随机位置的物料时用的数组




//!底盘调整相关参数
const float Kp_slight_move = 0.5;  // 底盘前后左右微调PID参数
const float Ki_slight_move = 0.01;
const float Kd_slight_move = 0.8;

const float Kp_line_spin = 1;      // 直线校正PID参数
const float Ki_line_spin = 0.04;
const float Kd_line_spin = 0.5;

const float xy_move_k = 0.2; //底盘微调时xy乘上的比例 
const float adjust_spin_scale = 1; // 底盘微调时旋转和移动的比例
const float adjust_move_scale = 1;


const float spin_limit_max = 10;   // 旋转速度的最大值
const float spin_limit_min = 0.4;  // 旋转速度的最小值
const float move_limit_max = 15;   // 移动速度的最大值
const float move_limit_min = 0.5;  // 移动速度的最小值

int motor_vel_adjust_with_spin = 20;  // 底盘直线调整时的最大速度20
int open_loop_x_move_velocity = 120;  // 开环横向移动速度120
int open_loop_move_velocity = 180;    // 开环前进速度180
int open_loop_spin_velocity = 150;    // 开环旋转速度150

// 步进电机加速度
float acceleration = 170;          // 直线运动加速度170  180会出现明显的震荡类似丢步
float acceleration_spin = 180;     // 旋转运动加速度180
float acceleration_adjust = 180;
int motor_pos_move_mode = 0; //如果是0则是位置模式按照上一条指令的目标位置进行相对移动；2则是按照当前的实际位置进行相对移动



//!机械臂调整相关参数
//!!!!!!!!      注意：机械臂还有大量参数在my_servo.c中
//!!!!!!!!      注意：机械臂还有大量参数在my_servo.c中
//!!!!!!!!      注意：机械臂还有大量参数在my_servo.c中

//? 以下是旧版本的PID控制参数（配合adjust_position_with_camera）
const float Kp_theta = 0.28;  // 机械臂旋转PID参数
const float Ki_theta = 0.01;
const float Kd_theta = 0.05;
const float Kp_r = 0.35;     // 机械臂伸缩PID参数
const float Ki_r = 0.01;
const float Kd_r = 0.08;

const float pixel_to_distance_theta = 1.2; // theta方向的像素到实际距离的比例
const float pixel_to_distance_r = 4; // r方向的像素到实际距离的比例

//? 以下是新版本的PID控制参数（adjust_position_with_camera_new）
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


//? 机械臂通用调整参数

int adjust_position_with_camera_time = 10; //机械臂细调的延时时间

//机械臂转盘单次微调系数
const float x_plate_k = 1;   // 转盘处机械臂微调系数
const float y_plate_k = 7;


//! 调整的超时时间
int timeout_limit = 1000; // 超时时间限制，单位10ms
int timeout_limit_line = 500;






//!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! 各种extern和声明（一般无需修改的）

//串口相关变量
char wifi_massage[50]; // 串口屏发送的数据
int is_wifi_already_get_message = 0;
int is_raspi_get_massage = 0;
extern uint8_t rxdata_u2[50],rxdata_u3[50],rxdata_u1[128],rxdata_u4[50],rxdata_u5[50]; // usart2,3接收缓冲区
extern uint8_t received_rxdata_u2,received_rxdata_u3,received_rxdata_u1,received_rxdata_u5,received_rxdata_u4; // 暂存usart2,3接收到的数据单字节变量
extern uchar rxflag_u2,rxflag_u3,rxflag_u1,rxflag_u4,rxflag_u5; // usart2,3接收标志位变量


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
int volatile start_check_plate_back_state = 0;
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


/************************************函数声明及定义区****************************************/

// printf重定向，用于串口屏的显示

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


void single_line_adjust(char *pData);
void single_line_circle_adjust(char *pData);
void get_from_plate_all_movement(void);
void get_from_plate_all_movement_with_back_check(void);
void single_get_and_put_some_with_load_first( int times,int is_pile_up,int is_load);
void single_get_and_put_some_with_load( int times, int is_load,int is_pile_up,int is_avoid_collide);
void get_and_put_in_spin_plate_cricle_all(int times);
void get_from_ground_in_random_position(int times);

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
     * 主要函数说明:
     * 
     * 单次的调整（用于在转盘抓取时的调整）
        void adjust_plate(int x_plate_error_in,int y_plate_error_in)

        根据偏差值进行放置的调整
        adjust_position_with_camera

        avoid版本，抓哪个位置，是不是根据上次记录的值进行抓取
        void get_and_load_openloop_avoid(int position,int is_default_position)

        从暂存区的地上抓取物料
        void get_and_load_openloop_with_temp_put(int position,int state_position)

        初赛用的从转盘圆环上抓取
        void get_and_load_openloop(int position,int is_default_position)

        将物料放置在转盘上的测试程序
        void get_and_pre_put_spin_plate_avoid_collide(int position)
        void get_and_pre_put_spin_plate(int position)

        avoid版本，pre_put
        void get_and_pre_put_avoid(int position,int is_pile_up)

        从地上抓取物料，配合调整的动作
        void pre_put_to_get_ground_material(int position)

        从地上抓物料（不带avoid）
        void get_material_from_ground(int state_position)

        不夹物料的preput调整
        void get_and_pre_put_void(int position,int is_pile_up)

        初赛用的，夹着物料去preput
        void get_and_pre_put(int position,int is_pile_up)

        在preput后调整物料中心偏差值，使得放置更准（无avoid)
        void get_and_pre_put_with_state_find_position(int position,int is_pile_up)
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
	HAL_Delay(1000); //TODO 比赛时考虑去除，增加启动速度
    my_servo_init(); //!精密舵机初始化，使用精密舵机则必须加入
    is_adjust_motor_in_tim = 0;
    motor_state = 1;


    /*****************初始化动作姿态***********************/
    HAL_Delay(100);
    arm_stretch();                // 机械臂伸缩位置初始化
    whole_arm_spin(1);           // 中板旋转位置初始化
    put_claw_up();           // 机械爪抬起
    claw_spin_front();           // 机械爪旋转到正前方
    open_claw_180();             // 机械爪完全张开
    state_spin_without_claw(1);  // 载物盘旋转到1号位
    // HAL_Delay(900); // TODO等待电机初始化完成，本该是4000ms,目前暂时减少时间
    is_raspi_get_massage = 0; //空闲中断标志位清零
    for(int i = 0 ; i < 4 ; i++)
    {
        theta_servo_value[i] = middle_arm;
        r_servo_value[i] = stretch_arm;
    }


    /*****************单独调试程序***********************/

    // HAL_Delay(1000); 
    // single_line_circle_adjust("CC");
    // single_get_and_put_some_with_load_first(1,0,1);

    // HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50);  // 通知树莓派开始识别转盘

    // while(1)
    // {
    //     HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50);
    //     put_claw_down();  // 放下机械爪准备抓取
    //     get_from_plate_all_movement();
    //     HAL_Delay(2000);
    // }

    // while(1)
    // {
    //         put_claw_up();  // 抬起机械爪
    //         single_line_circle_adjust("CC");
    // single_get_and_put_some_with_load_first(1,0,1);
    //     HAL_Delay(1000);
        // put_claw_up();
        // HAL_Delay(2000);
    // }
    // while(1)
    // {
    //     HAL_Delay(1000);
    // }


    // /***********************比赛初赛所用的全流程***********************/
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


    /************初始化和第一次前往转盘*************/
    /************初始化和第一次前往转盘*************/
    /************初始化和第一次前往转盘*************/


    is_adjust_motor_in_tim = 0;
    motor_state = 1;
    HAL_Delay(1000); // 等待电机初始化
    HAL_UART_Transmit(&huart3, (uint8_t*)"AA", strlen("AA"), 50);  // 开始识别二维码

    //! wifi接收（不用的话保持注释）
    //TODO 仍待优化和测试
    // HAL_Delay(2000);
    // HAL_UART_Transmit(&huart3, (uint8_t*)"MM", strlen("MM"), 50);
    // char temp_wifi_printf[50];
    // HAL_UART_Transmit(&huart4, (uint8_t*)"AT+CIPMUX=0\r\n", strlen("AT+CIPMUX=0\r\n"), 50);
    // HAL_Delay(500);
    // HAL_UART_Transmit(&huart4, (uint8_t*)"AT+CIPMODE=1\r\n", strlen("AT+CIPMODE=1\r\n"), 50);
    // HAL_Delay(500);
    // HAL_UART_Transmit(&huart4, (uint8_t*)"AT+CIPSTART=\"TCP\",\"192.168.43.42\",8089\r\n", strlen("AT+CIPSTART=\"TCP\",\"192.168.43.42\",8089\r\n"), 50);
    // HAL_Delay(500);
    // HAL_UART_Transmit(&huart4, (uint8_t*)"AT+CIPMODE=1\r\n", strlen("AT+CIPMODE=1\r\n"), 50);
    // HAL_Delay(500);
    // while(is_wifi_already_get_message != 1)
    // {
    //     HAL_Delay(100);
    // } 
    // sprintf(temp_wifi_printf, "%s",wifi_massage);
    // HAL_UART_Transmit(&huart3, (uint8_t*)temp_wifi_printf, strlen(temp_wifi_printf), 50); //发送wifi数据
    // HAL_Delay(100);



    // 从初始位置到转盘的移动参数
    float start_move_x = 15; 
    float start_move_y = 22.5; 
    float move_to_qrcode = 37.5;
    float move_from_qrcode_to_table = 85;
    float little_back_1 = 2;
    



    move_all_direction_position(acceleration, open_loop_move_velocity, -start_move_x , start_move_y);  // 从启停区出来
    HAL_Delay(1200);


    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_to_qrcode);  // 出来后移动到二维码前
    HAL_Delay(2000);


    move_all_direction_position(acceleration, open_loop_move_velocity, 0, move_from_qrcode_to_table);  // 移动到转盘前
    HAL_Delay(2000);

    
    //显示二维码信息
    char* target_colour_str = (char*)malloc(6);
    sprintf(target_colour_str, "%d%d%d+%d%d%d", target_colour[0], target_colour[1], target_colour[2], target_colour[3], target_colour[4], target_colour[5]); 
    printf("t0.txt=\"%s\"\xff\xff\xff",target_colour_str); 
    free(target_colour_str);


    spin_right_90(open_loop_spin_velocity,acceleration_spin);  // 右转90度面向转盘
    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50);  // 通知树莓派开始识别转盘
    HAL_Delay(1000);


    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -little_back_1);  // 微调位置往后

    if(is_single_route_test != 1)
    {
        //! 注意：两个函数的配合使用（包括发送的字母标志位）
        put_claw_down();  // 放下机械爪准备抓取
        get_from_plate_all_movement();  // 执行从转盘抓取物料的动作序列
    }
    else
    {
        HAL_Delay(3000);
    }
    //! 姿态的恢复
    put_claw_up();  
    whole_arm_spin(1); 
    arm_stretch();








    /**************第一次从转盘前往粗加工区并放置*****************/
    /**************第一次从转盘前往粗加工区并放置*****************/
    /**************第一次从转盘前往粗加工区并放置*****************/

    
    //移动到粗加工区参数
    float move_right_length_1 = 41; 
    float move_front_length_1 = 170;  


    move_all_direction_position(acceleration, open_loop_x_move_velocity, move_right_length_1,0);  // 向右移动到中轴线
    HAL_Delay(1100);

    if(is_single_route_test != 1)
    {
        //先校正直线
        single_line_adjust("EE");  // 校正车身姿态与直线平行
    }
    else
    {
        HAL_Delay(3000);
    }
    
    
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_1);// 后退到粗加工区
    HAL_Delay(3000);

    spin_right_180(open_loop_spin_velocity,acceleration_spin);  // 旋转180度面向色环
    HAL_Delay(2000); 

    if(is_single_route_test != 1)
    {
    //到达粗加工区，开始校正车身位置
    //! 注意：两个函数的配合使用（包括发送的字母标志位）
    single_line_circle_adjust("CC");  // 校正车身位置对准色环
    single_get_and_put_some_with_load_first(1,0,1);  // 执行放置动作序列
    }
    else
    {
        HAL_Delay(3000);
    }
    


    /**************第一次从粗加工区前往暂存区并放置*****************/
    /**************第一次从粗加工区前往暂存区并放置*****************/
    /**************第一次从粗加工区前往暂存区并放置*****************/

    //移动到暂存区参数
    int move_front_length_2 = 82; 
    int move_back_length_2 = 86; 
    
    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_2);  // 后退到十字中心
    HAL_Delay(2000);


    put_claw_up();//! 姿态的恢复
    arm_stretch();


    spin_right_90(open_loop_spin_velocity,acceleration_spin);  // 右转90度面向暂存区
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_2 );  // 前进到暂存区
    HAL_Delay(1800);


    if(is_single_route_test != 1)
    {
    //! 注意：两个函数的配合使用（包括发送的字母标志位）
    single_line_circle_adjust("CC");  // 校正位置对准暂存区
    single_get_and_put_some_with_load_first(2,0,0);  // 执行放置动作序列
    }
    else
    {
        HAL_Delay(3000);
    }
    
    

    /**************第二次前往转盘并抓取*****************/
    /**************第二次前往转盘并抓取*****************/
    /**************第二次前往转盘并抓取*****************/

    //移动回转盘参数
    int move_right_length_b = 44;
    int move_front_length_b = 88;
    

    spin_right(open_loop_spin_velocity,acceleration_spin, 90);  // 右转90度
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_b);  // 前进到转盘左侧
    HAL_Delay(2500);

    move_all_direction_position(acceleration, open_loop_x_move_velocity,move_right_length_b, 0);  // 向右移动到转盘
    HAL_Delay(1500);


    HAL_UART_Transmit(&huart3, (uint8_t*)"BB", strlen("BB"), 50); 
    HAL_Delay(100);

    if(is_single_route_test != 1)
    {
    //! 注意：两个函数的配合使用（包括发送的字母标志位）
    put_claw_down();
    get_from_plate_all_movement();
    }
    else
    {
        HAL_Delay(3000);
    }

    put_claw_up();//! 姿态的恢复
    whole_arm_spin(1); 
    arm_stretch();




    /**************第二次从转盘前往粗加工区并放置*****************/
    /**************第二次从转盘前往粗加工区并放置*****************/
    /**************第二次从转盘前往粗加工区并放置*****************/


    //移动到粗加工区参数
    float move_right_length_3 = 41; 
    float move_front_length_3 = 170;  


    move_all_direction_position(acceleration, open_loop_x_move_velocity, move_right_length_3,0);  // 向右移动到中轴线
    HAL_Delay(1100);
    
    if(is_single_route_test != 1)
    {
        //先校正直线
        single_line_adjust("EE");  // 校正车身姿态与直线平行
    }
    else
    {
        HAL_Delay(3000);
    }

    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_front_length_3);  // 后退到粗加工区
    HAL_Delay(2500);

    put_claw_up();//! 姿态的恢复

    spin_right_180(open_loop_spin_velocity,acceleration_spin);  // 旋转180度面向色环
    HAL_Delay(2000);

    if(is_single_route_test != 1)
    {
    //! 注意：两个函数的配合使用（包括发送的字母标志位）
    single_line_circle_adjust("CC");
    single_get_and_put_some_with_load_first(3,0,1);
    }
    else
    {
        HAL_Delay(3000);
    }


    /**************第二次从粗加工区前往暂存区并放置*****************/
    /**************第二次从粗加工区前往暂存区并放置*****************/
    /**************第二次从粗加工区前往暂存区并放置*****************/

    //移动到暂存区参数
    float move_front_length_4 = 82.5; 
    float move_back_length_4 = 86; 

    move_all_direction_position(acceleration, open_loop_move_velocity, 0, -move_back_length_4);  // 后退到十字中心
    HAL_Delay(2000);

    put_claw_up();//! 姿态的恢复
    arm_stretch();

    spin_right_90(open_loop_spin_velocity,acceleration_spin);  // 右转90度面向暂存区
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_4);  // 前进到暂存区
    HAL_Delay(1800);


    if(is_single_route_test != 1)
    {
    //! 注意：两个函数的配合使用（包括发送的字母标志位）
    single_line_circle_adjust("CC");
    single_get_and_put_some_with_load_first(4,1,0);
    }
    else
    {
        HAL_Delay(3000);
    }
    

    /**************从暂存区回原点*****************/
    /**************从暂存区回原点*****************/
    /**************从暂存区回原点*****************/

    //移动回原点参数
    int move_45_length_5 = 28;
    int move_front_length_5 = 75.5;
    int move_back_length_5 = 169;
    int move_right_length_5 = 0.1;

    move_all_direction_position(acceleration, open_loop_move_velocity, move_right_length_5,0);  
    HAL_Delay(900);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,-move_back_length_5);  
    HAL_Delay(1000);

    open_claw_180();
    whole_arm_spin(1); 
    arm_stretch();
    HAL_Delay(1500);

    spin_right(open_loop_spin_velocity,acceleration_spin, 90);
    HAL_Delay(1000);

    move_all_direction_position(acceleration, open_loop_move_velocity, 0,move_front_length_5);  
    HAL_Delay(2000);

    move_all_direction_position(acceleration, open_loop_move_velocity, move_45_length_5-7.5, move_45_length_5);
    HAL_Delay(2000);

    HAL_UART_Transmit(&huart3, (uint8_t*)"end", strlen("end"), 50);

    while(1)
    {
        HAL_Delay(1000);
    }

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!






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

/// @brief 串口空闲中断回调函数
/// @param huart 
/// @param Size 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_IT(huart, rxdata_u3, 40);
        is_raspi_get_massage = 1;
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

/// @brief 单独的直线校正
/// @param pData 要发送的字符串
void single_line_adjust(char *pData)
{
    static int adjust_delay = 50;
    HAL_UART_Transmit(&huart3, (uint8_t*)pData, strlen(pData), 50); // 发送传入的字符串
    HAL_Delay(10);
    is_slight_spin_and_move = 1;
    tim3_count = 0;
    // while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit_line)
    while(is_slight_spin_and_move != 0 )
    {
        slight_spin_and_move(); // 在转盘旁的直线处进行姿态的校正
        HAL_Delay(adjust_delay);
    }
    stop();
    HAL_Delay(50);
}

/// @brief 单独的底盘定位
/// @param  
void single_line_circle_adjust(char *pData)
{
    static int adjust_delay = 50;
    HAL_UART_Transmit(&huart3, (uint8_t*)pData, strlen(pData), 50);
    HAL_Delay(200);
    is_slight_spin_and_move = 1;
    tim3_count = 0;
    // while(is_slight_spin_and_move != 0 && tim3_count < timeout_limit)
    while(is_slight_spin_and_move != 0 )
    {
        slight_spin_and_move(); // 直线和圆环一起调整
        HAL_Delay(adjust_delay);
    }
    is_slight_spin_and_move = 0;
    stop();
    HAL_Delay(50);
}




/// @brief 将物料放置在转盘色环上的全流程函数（省赛版本，非常复杂）
/// @param times 
void get_and_put_in_spin_plate_cricle_all(int times)
{
    /**
     * @brief 目前是在前一个颜色转走后，判断下一个颜色即是要放置的颜色，树莓派就发送信号，然后开始抓取并放置。放完就在原地等，以此类推
     * 
     */
    
    HAL_UART_Transmit(&huart3, (uint8_t*)"PP", strlen("PP"), 50);
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
        
        
        get_and_pre_put_spin_plate_avoid_collide(target_colour[i+add_count]);
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
    
    // HAL_UART_Transmit(&huart3, (uint8_t*)"wait", strlen("wait"), 50);
    // is_third_preput = 0;
    // while(is_third_preput != 1)
    // {
    //     HAL_Delay(50);
    // }
    // close_claw();
    // HAL_Delay(600);
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
    // // HAL_UART_Transmit(&huart3, (uint8_t*)"wait", strlen("wait"), 50);
    // // is_third_preput = 0;
    // // while(is_third_preput != 1)
    // // {
    // //     HAL_Delay(50);
    // // }
    // // close_claw();
    // // HAL_Delay(400);
    // // put_claw_up_top();
    // // HAL_Delay(500); //200
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

/// @brief 国赛初赛使用的物料放置和夹取函数
/// @param times 
/// @param is_pile_up 
void single_get_and_put_some_with_load_first( int times,int is_pile_up,int is_load)
{
    int times_count = 0; //用于计算target_colour的索引
    if(times == 3 || times == 4)
    {
        times_count = 3;
    }

    if(is_pile_up == 1) // 码垛，不调整
    {
        int times_count = 0;
        if(times == 3 || times == 4)
        {
            times_count = 3;
        }
        for (int i = 0; i < 3; i++)
        {
            
            get_and_pre_put(target_colour[i+times_count], 1);
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
            get_and_pre_put(target_colour[i+times_count], is_pile_up); //夹着物料放置
            servo_adjust_status = target_colour[i+times_count];
            is_servo_adjust = 1;
            tim3_count = 0;
            HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
            while (is_servo_adjust != 0 ) 
            {
                adjust_position_with_camera(x_camera_error, y_camera_error,1);  
                // adjust_position_with_camera_new(x_camera_error,y_camera_error,adjust_position_with_camera_time);
                HAL_Delay(adjust_position_with_camera_time);  //10
            }
            theta_servo_value[target_colour[i+times_count]] = theta_servo_now;
            r_servo_value[target_colour[i+times_count]] = r_servo_now;
            is_servo_adjust = 0;
            if(is_put_adjust_with_material == 1)
            {
                put_claw_down_ground_slightly();
                HAL_Delay(500);
                open_claw();
                HAL_Delay(300);
            }
            
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
                get_and_load_openloop(target_colour[i+times_count],0); // 开环抓取
            }
    }
    whole_arm_spin(1);
    open_claw_180();
    arm_stretch();
    HAL_Delay(10);
    
}




/// @brief 色环放置(保留版本，这个是屎山)
/// @param is_load  放置完后是否需要抓取物料， 1为需要，0为不需要
/// @param is_pile_up  是否需要码垛, 1为需要，0为不需要
/// @param is_avoid_collide  是否从侧面过, 1为需要，0为不需要
/// @param times  到达物料区的次数
void single_get_and_put_some_with_load( int times, int is_load,int is_pile_up,int is_avoid_collide)
{
    // 主要问题:


    //! 问题： 如果决赛改流程，那么这个函数其实封装这么多意义不大，更重要的应该是封装好
    
    // 1. 代码结构问题:
    // - 函数过长(300+行),应拆分成多个子函数以提高可读性和可维护性
    // - 大量重复代码块,如放置动作序列在多处重复出现
    // - if-else嵌套层次过深,逻辑复杂
    
    // 2. 参数设计问题:
    // - 参数顺序与命名不符合函数名称
    // - times参数用途不明确,仅用于计算times_count
    // - is_pile_up和is_pile_adjust组合导致复杂分支
    
    // 3. 时序控制问题:
    // - 大量硬编码的延时数值(HAL_Delay)
    // - 延时值缺乏注释说明原因
    // - 部分延时值前后不一致(如300/500/600ms)
    
    // 4. 错误处理问题:
    // - 缺少超时处理和错误恢复机制
    // - while循环中缺少错误处理
    // - 通信失败时缺少重试机制
    
    // 5. 其他问题:
    // - 全局变量使用过多
    // - 注释不完整
    // - 代码缩进和格式不统一
    // - 部分TODO注释未处理
    
    // 建议改进:
    // 1. 将代码拆分为以下子函数:
    // - handle_pile_up_no_adjust()
    // - handle_pile_up_with_adjust() 
    // - handle_no_pile_up()
    // - handle_material_loading()
    
    // 2. 引入状态机管理复杂流程
    
    // 3. 统一延时参数并添加配置文件
    
    // 4. 添加错误处理和恢复机制
    
    // 5. 减少全局变量,改用参数传递
    
    // 6. 完善注释和文档
    
    if(is_pile_up == 1 && is_pile_adjust == 0) // 码垛，不调整
    {
        int times_count = 0;
        if(times == 3 || times == 4)
        {
            times_count = 3;
        }
        for (int i = 0; i < 3; i++)
        {
            if(is_avoid_collide == 1)
            {
                get_and_pre_put_avoid(target_colour[i+times_count], 1);
            }
            else
            {
                get_and_pre_put(target_colour[i+times_count], 1);
            }        
            servo_adjust_status = target_colour[i+times_count];
            is_servo_adjust = 1;
            tim3_count = 0;
            x_camera_error = 0;
            y_camera_error = 0;
            is_servo_adjust = 0;
            open_claw();
            HAL_Delay(500);
        }
        put_claw_up_top();
        HAL_Delay(500);
    }
    else if(is_pile_up == 1 && is_pile_adjust == 1) //码垛，调整
    {
        int times_count = 0;
        if(times == 3 || times == 4)
        {
            times_count = 3;
        }
        for (int i = 0; i < 3; i++)
        {
            
            get_and_pre_put_void(target_colour[i+times_count], 1); //不夹着物料放置
            put_claw_down_near_ground();
            servo_adjust_status = target_colour[i+times_count];
            is_servo_adjust = 1;
            tim3_count = 0;
            HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
            while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
            {
                adjust_position_with_camera(x_camera_error, y_camera_error,1); 
                HAL_Delay(adjust_position_with_camera_time);  //30
            }
            theta_servo_value[target_colour[i+times_count]] = theta_servo_now;
            r_servo_value[target_colour[i+times_count]] = r_servo_now;
            is_servo_adjust = 0;
            //以下的动作为回去夹取物料然后转过来放置
            if(is_avoid_collide == 0)
            {
                int now_servo = r_servo_now;
                open_claw();
                put_claw_up_top();
                HAL_Delay(1000); //TODO 可能会撞到物料
                arm_shrink(); 
                HAL_Delay(600);
                claw_spin_state();
                HAL_Delay(800);
                put_claw_down_state();
                HAL_Delay(500); //400
                close_claw();
                HAL_Delay(500);
                put_claw_up_top();
                HAL_Delay(600); //200
                feetech_servo_move(4,now_servo,4000,100);
                claw_spin_front();
                HAL_Delay(500);
                put_claw_down_pile();
                HAL_Delay(900);
                open_claw();
                HAL_Delay(600);
            }
            else
            {
                //?加入avoid_collide的动作
                int now_servo = r_servo_now;
                state_spin_without_claw_avoid_collide(target_colour[i+times_count]);
                put_claw_down_state();
                HAL_Delay(1000);
                arm_shrink(); 
                open_claw_avoid_collide();
                HAL_Delay(300);
                claw_spin_state();
                HAL_Delay(500);
                state_spin(target_colour[i+times_count]);
                HAL_Delay(200); //400
                close_claw();
                HAL_Delay(700);
                put_claw_up_top();
                HAL_Delay(500); //200
                feetech_servo_move(4,now_servo,4000,100);
                claw_spin_front();
                HAL_Delay(500);
                put_claw_down_pile();
                HAL_Delay(900);
                open_claw();
                HAL_Delay(600);
            }
            
        }
        put_claw_up_top();
        HAL_Delay(500);
    }
    else if(is_pile_up == 0) //不需要码垛
    {
        int times_count = 0; //用于计算target_colour的索引
        if(times == 3 || times == 4)
        {
            times_count = 3;
        }
        for (int i = 0; i < 3; i++) 
        {
            if(is_put_adjust_with_material == 1)
            {
                if(is_avoid_collide == 0)
                {
                    get_and_pre_put(target_colour[i+times_count], is_pile_up); //夹着物料放置
                }
                else
                {
                    get_and_pre_put_avoid(target_colour[i+times_count], is_pile_up); //夹着物料放置
                }
            }
            else
            {
                get_and_pre_put_void(target_colour[i+times_count], is_pile_up); //不夹着物料放置
            }
            servo_adjust_status = target_colour[i+times_count];
            is_servo_adjust = 1;
            tim3_count = 0;
            HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //发给树莓派，开始校正
            while (is_servo_adjust != 0 && tim3_count < timeout_limit) 
            {
                adjust_position_with_camera(x_camera_error, y_camera_error,1); 
                HAL_Delay(adjust_position_with_camera_time);  //30
            }
            theta_servo_value[target_colour[i+times_count]] = theta_servo_now;
            r_servo_value[target_colour[i+times_count]] = r_servo_now;
            is_servo_adjust = 0;
            if(is_put_adjust_with_material == 1)
            {
                put_claw_down_ground();
                HAL_Delay(500);
                open_claw();
                HAL_Delay(500);
            }
            else
            {
                if(is_avoid_collide == 0)
                {
                    //以下的动作为回去夹取物料然后转过来放置
                    int now_servo = r_servo_now;
                    open_claw();
                    put_claw_up_top();
                    arm_shrink(); 
                    HAL_Delay(300);
                    claw_spin_state();
                    HAL_Delay(500);
                    put_claw_down_state();
                    HAL_Delay(300); //400
                    close_claw();
                    HAL_Delay(200);
                    put_claw_up_top();
                    HAL_Delay(400); //200
                    feetech_servo_move(4,now_servo,4000,100);
                    claw_spin_front();
                    HAL_Delay(500);
                    put_claw_down_ground();
                    HAL_Delay(1100);
                    open_claw();
                    HAL_Delay(300);
                }
                else
                {
                    int now_servo = r_servo_now;
                    open_claw_avoid_collide();
                    state_spin_without_claw_avoid_collide(target_colour[i+times_count]);
                    put_claw_down_state();
                    HAL_Delay(500);
                    arm_shrink(); 
                    HAL_Delay(300);
                    claw_spin_state();
                    HAL_Delay(500);
                    state_spin(target_colour[i+times_count]);
                    HAL_Delay(200); //400
                    close_claw();
                    HAL_Delay(700);
                    put_claw_up_top();
                    HAL_Delay(500); //200
                    feetech_servo_move(4,now_servo,4000,100);
                    claw_spin_front();
                    HAL_Delay(500);
                    put_claw_down_ground();
                    HAL_Delay(1100);
                    open_claw();
                    HAL_Delay(600);
                }                
            }
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
                if(is_avoid_collide == 1)
                {
                    get_and_load_openloop_avoid(target_colour[i+times_count],0); 
                }
                else
                {
                    get_and_load_openloop(target_colour[i+times_count],0); // 开环抓取
                }
            }
    }
    whole_arm_spin(1);
    open_claw_180();
    arm_stretch();
    HAL_Delay(10);
}

/// @brief 从转盘抓物料的动作（需要提前完成夹爪放低的动作，这个函数的第一个动作就是抓取）
/// @param  
void get_from_plate_all_movement(void)
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
            if(is_get_empty_finish == 0)
            {
                start_judge_empty = 1; //首次抓取则开始判断是否抓空
            }
            HAL_Delay(600);
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
                int r_servo_now_temp = r_servo_now;
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
                HAL_Delay(500);
                arm_stretch();
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




/// @brief 转盘抓取（带有判断空抓）（先回到物料盘再判断）（需要提前完成爪子放低的动作）
/// @param  
void get_from_plate_all_movement_with_back_check(void)
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
                claw_spin_state_without_claw();
                HAL_Delay(700); 

                start_check_plate_back_state = 1;
                HAL_UART_Transmit(&huart3, (uint8_t*)"check", strlen("check"), 50); //发给树莓派，开始判断是否抓空
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


/// @brief 从地上抓取随机顺序摆放的物料全流程函数
/// @param times 1为第一轮，2为第二轮
void get_from_ground_in_random_position(int times)
{
    put_claw_down_near_ground();
    HAL_Delay(1000);
    HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //开始识别最中间的物料
    is_get_material_from_temp_area = 2; 
    get_and_pre_put_void(1,0); //看最右侧的物料
    HAL_Delay(500);
    HAL_UART_Transmit(&huart3, (uint8_t*)"near ground", strlen("near ground"), 50); //识别最右侧的物料
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
    for(int i = 0 ; i < 3 ; i++)
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
        get_and_load_openloop_with_temp_put(temp_get_order[i],target_colour[i+temp_add]); // 开环抓取，avoid
    }
    whole_arm_spin(1);
    arm_stretch();
    put_claw_up();
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
