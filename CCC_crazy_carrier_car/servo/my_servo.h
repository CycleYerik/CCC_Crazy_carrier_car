#ifndef __MY_SERVO_H
#define __MY_SERVO_H

#include "main.h"

#include "SCServo.h"
#include "uart.h"

//50ms



void servo_move_with_limit(int d_theta_move_position,int d_r_move_position);

void adjust_plate(int x_plate_error_in,int y_plate_error_in);

int adjust_position_with_camera(int x_error, int y_error,int is_min_1 );
void adjust_position_with_camera_new(int x_error, int y_error, int dt);

void my_servo_init(void);
void feetech_servo_move(uint8_t servo_ID,int16_t Position,uint16_t Speed,uint8_t ACC);





void servo_move(int servo_ID, int angle);

void put_claw_down_pile(void);
void put_claw_down_ground(void);
void put_claw_down_ground_slightly(void);
void put_claw_down_pile_slightly(void);
void put_claw_down_near_ground(void);
void put_claw_down_state(void);
void put_claw_down(void);
void put_claw_down_slightly(void);
void put_claw_up(void);
void put_claw_up_top(void);
void put_claw_up_top_slight(void);
void put_claw_down_near_plate(void);

void claw_spin_front(void);
void claw_spin_state(void);
void claw_spin_front_slight(void);
void claw_spin_state_without_claw(void);
void claw_spin_state_without_claw_slight(void);

void whole_arm_spin(int status);

void open_claw_avoid_collide(void);
void open_claw_180(void);
void open_claw_bigger(void);
void open_claw(void);
void close_claw(void);
void close_claw_2(void);
void close_bit(void);

void arm_stretch(void);
void arm_shrink(void);
void arm_shrink_all(void);

void state_spin(int state_position);
void state_spin_angles(int angle);
void state_spin_without_claw(int state_position);
void state_spin_without_claw_avoid_collide(int state_position);




void get_and_load_openloop(int position,int is_default_position,material_order* order);
void new_get_and_load_openloop(int position,int is_default_position,material_order* order);
void new_get_and_load_openloop_with_temp_put(int position,int state_position);
void new_get_and_load_openloop_avoid(int position,int is_default_position,material_order* order);

void new_get_and_pre_put_avoid(int position,int is_pile_up, int is_default_position,int is_update, const material_order* order);
void new_get_and_pre_put_spin_plate(int position);
void new_get_and_put_spin_plate(int position);
void get_and_pre_put(int position,int is_pile_up, int is_default_position, const material_order* order);
void new_get_and_pre_put(int position,int is_pile_up, int is_default_position,int is_update, material_order* order);
void new_get_and_pre_put_void(int position,int is_pile_up, const material_order* order);
void new_get_and_pre_put_with_state_find_position(int position,int is_pile_up, const material_order* order);
void new_get_and_pre_put_spin_plate_avoid_collide(int position);


#endif

