#ifndef INCLUDE_1_H
#define INCLUDE_1_H

#include "Arduino.h"

#define MAX_SPEED1 1000 //in rpm
#define MAX_SPEED2 1000
#define MAX_SPEED3
#define MAX_SPEED4

#define ENCODER_ACC 10 // pules per revolution

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define clamp(v, l, h) min(max(v, l), h)

#define INTERRUPT_PIN 2
#define INTERRUPT_FEED1 3
#define INTERRUPT_FEED2 4
#define INTERRUPT_FEED3 1
#define INTERRUPT_FEED4 11




#define MOTOR1_PWM 5
#define MOTOR2_PWM 6
#define MOTOR3_PWM
#define MOTOR4_PWM

#define MOTOR1_DIR1 7
#define MOTOR1_DIR2 8
#define MOTOR2_DIR1 12
#define MOTOR2_DIR2 13
#define MOTOR3_DIR1
#define MOTOR3_DIR2
#define MOTOR4_DIR1
#define MOTOR4_DIR2

#define MOTOR1_SPEED_IN A0
#define MOTOR2_SPEED_IN A1
#define MOTOR3_SPEED_IN
#define MOTOR4_SPEED_IN

#define MOTOR1_DIR_IN 9
#define MOTOR2_DIR_IN 10
#define MOTOR3_DIR_IN
#define MOTOR4_DIR_IN


#endif