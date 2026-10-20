#ifndef __DEMO_H__
#define __DEMO_H__

#include "fashion_star_uart_servo.h"

void demo_ping(void);
void demo_servo1_rotate_0_to_360(void);
void demo_servo1_2_rotate_0_to_360(void);
void demo_servo1_2_check_and_rotate(void);
void demo_servo2_check_and_rotate(void);
void demo_find_servo_id_by_motion(void);
void demo_project_two_servo_init_position(void);
void demo_setAngle_SingleServo(void);
void demo_setAngle_MultiServo(void);
void demo_setAngle_MultiCircle(void);
void demo_DampingModeAndQueryAngle(void);
void demo_WheelKeepMove(void);
void demo_WheelMoveTime(void);
void demo_WheelMoveCircle(void);
void demo_QueryServoStatus(void);
void demo_SYNC(void);
void demo_MONTIOR(void);
void demo_StopOnControlMode(void);
void demo_Begin_EndAsync(void);

#endif
