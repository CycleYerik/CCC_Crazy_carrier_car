#ifndef __MY_SERVO_H
#define __MY_SERVO_H

#include "main.h"

#include "SCServo.h"
#include "uart.h"

void my_servo_init(void);
void feetech_servo_move(uint8_t servo_ID,int16_t Position,uint16_t Speed,uint8_t ACC);
void servo_move(int servo_ID, int angle);
void open_claw(void);
void close_claw(void);
void arm_stretch(void);
void arm_shrink(void);
void state_spin(int state_position);

#endif

