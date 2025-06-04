#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "main.h"
#include "my_usart.h"


// #define pi 3.1415926
// #define wheel_circumference 31.4 //轮子周长
// #define pulse_per_circle 3200 //每圈脉冲数
// #define spin_radius_180 21.51 //自转半径，数据不准确 // 21.495 90 
// #define spin_radius_90 21.495 //自转半径，数据不准确 // 21.495 90
// #define spin_radius 21.495 //自转半径，数据不准确 // 21.495 90
// #define speed_ratio 0.52 //速度比例系数，0.47 cm/s 对应速度为1 （此处修改为0.52cm/s 对应速度为1）

// 0.523 cm/s 对应速度为1

/**
 * @brief 
 * 
 * 
 * 距离 / 周长 / 转速 = 时间
 * 
 * 
 * 
 * 
 * 
 */


#define		ABS(x)		((x) > 0 ? (x) : -(x)) 

typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;

extern float x_velocity, y_velocity; // x、y轴速度

void slight_spin_and_move(void);
uint32_t get_clk(float distance);
float get_angle(float distance);
int get_distance_time(float distance, float velocity);
// void PID_vel_Control(uint8_t addr,uint8_t acc, float target_vel);
// int PID_motor_control(float x_bias, float y_bias);
void move_all_direction(uint8_t acc,float x_move_velocity,float y_move_velocity);
// void move_all_direction_pid(uint8_t acc,float x_move_velocity,float y_move_velocity);
void move_all_direction_position(uint8_t acc,uint16_t velocity, float x_move_length,float y_move_length);
void move_all_direction_position_delay(uint8_t acc,uint16_t velocity, float x_move_length,float y_move_length);
void move_all_direction_position_y42(uint16_t acc_start,uint16_t acc_stop, float vel,float x_move_length,float y_move_length);
void move_all_direction_position_tim(uint8_t acc,uint16_t velocity, float x_move_length,float y_move_length,int times_count);
void move_all_direction_tim(uint8_t acc, float x_vel,float y_vel,int times_count);
void spin_all_direction_tim(uint8_t acc, float spin_direction, int times_count);
void stop(void);
void stop_tim(int times_count);
void Forward_move_with_yaw_adjust(uint16_t vel_left,uint16_t vel_right,uint8_t acc);
void Forward_move_velocity(uint16_t vel,uint8_t acc);
void Forward_move( uint16_t vel,uint8_t acc, uint32_t distance);
void Backward_move_velocity(uint16_t vel,uint8_t acc);
void Backward_move( uint16_t vel,uint8_t acc, uint32_t distance);
void spin_right_180(uint16_t vel,uint8_t acc);
void spin_left_180(uint16_t vel,uint8_t acc);
void spin_right_90(uint16_t vel,uint8_t acc);
void spin_left_90(uint16_t vel,uint8_t acc);
void spin_left_velocity(uint16_t vel,uint8_t acc);
void spin_left(uint16_t vel,uint8_t acc, uint32_t angle);
void spin_right_velocity(uint16_t vel,uint8_t acc);
void spin_right(uint16_t vel,uint8_t acc, uint32_t angle);
void move_left_velocity(uint16_t vel,uint8_t acc);
void move_left(uint16_t vel,uint8_t acc, uint32_t distance);
void move_right_velocity(uint16_t vel,uint8_t acc);
void move_right(uint16_t vel,uint8_t acc, uint32_t distance);

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk,uint8_t raF, uint8_t snF);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF);
void Emm_V5_Stop_Now(uint8_t addr, uint8_t snF);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void ZDT_X42_V2_Traj_Position_Control(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF);
void ZDT_X42_V2_Velocity_Control(uint8_t addr, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF);
void ZDT_X42_V2_Stop_Now(uint8_t addr, uint8_t snF);
void ZDT_X42_V2_Synchronous_motion(uint8_t addr);
void stop_y42(void);
void spin_right_90_y42(float vel, uint16_t acc_start,uint16_t acc_stop);
void spin_left_90_y42(float vel, uint16_t acc_start,uint16_t acc_stop);
void spin_right_180_y42(float vel,uint16_t acc_start, uint16_t acc_stop);
void spin_left_180_y42(float vel,uint16_t acc_start, uint16_t acc_stop);

void set_motor_ID(uint8_t addr,uint8_t modify_to_addr);

void move_leftright_front_with_spin(int left_or_right, float vel , float R_spin);
void move_leftright_front_with_spin_position(int left_or_right, float vel  , float R_spin, float angles);
#endif
