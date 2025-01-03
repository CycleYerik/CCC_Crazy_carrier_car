#ifndef __MY_SERVO_H
#define __MY_SERVO_H

#include "main.h"

#include "SCServo.h"
#include "uart.h"

void get_and_load_ground(int position);
void get_and_load(int position);
void get_and_put_different_position(int position);
void get_and_load_different_position(int position);

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

void get_and_put_different_position_pileup(int position);
void put_claw_down_pile(void);
void get_from_state(int position);
void put_from_state(void);
void put_from_state_pileup(void);
void put_claw_down_ground(void);
void put_claw_down_state(void);
void put_claw_down(void);
void put_claw_up(void);
void put_claw_up_top(void);
void claw_spin_front(void);
void claw_spin_state(void);
void whole_arm_spin(int status);

#endif

