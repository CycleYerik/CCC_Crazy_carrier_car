#ifndef __MY_SERVO_H
#define __MY_SERVO_H

#include "main.h"

#include "SCServo.h"
#include "uart.h"

//50ms
#define pixel_to_distance_r 0.8
#define pixel_to_distance_theta 0.6
#define Kp_theta 0.6  //0.22
#define Ki_theta 0.15 //0.1
#define Kd_theta 0.25 //0.01
#define Kp_r 1.0  //0.45
#define Ki_r 0.15  //0.12
#define Kd_r 0.35  //0.01
void adjust_plate(int x_plate_error_in,int y_plate_error_in);
void get_and_load_ground(int position);
void get_and_load(int position);
void get_and_put_different_position(int position);
void get_and_load_different_position(int position);
void get_and_load_openloop(int position);
void my_servo_init(void);
void feetech_servo_move(uint8_t servo_ID,int16_t Position,uint16_t Speed,uint8_t ACC);
void servo_move(int servo_ID, int angle);
void open_claw_180(void);
void open_claw(void);
void close_claw(void);
void arm_stretch(void);
void arm_shrink(void);
void arm_shrink_all(void);
void state_spin(int state_position);
void state_spin_without_claw(int state_position);

int adjust_position_with_camera(int x_error, int y_error,int is_min_1 );

void get_and_pre_put(int position,int is_pile_up);
void get_and_put_different_position_pileup(int position);
void put_claw_down_pile(void);
void get_from_state(int position);
void put_from_state(void);
void put_from_state_pileup(void);
void put_claw_down_ground(void);
void put_claw_down_near_ground(void);
void put_claw_down_state(void);
void put_claw_down(void);
void put_claw_up(void);
void put_claw_up_top(void);
void claw_spin_front(void);
void claw_spin_state(void);
void claw_spin_state_without_claw(void);
void whole_arm_spin(int status);

#endif

