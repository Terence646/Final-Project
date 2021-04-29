/*
 * servo.h
 *
 *  Created on: Jul 14, 2020
 *      Author: Tyler Davidson
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "pca9685.h"

//Servo Definitions
//-----Servo-----Channel
#define URL         0
#define LRL         1

#define URA         4
#define LRA         5

#define ULL         9
#define LLL         10

#define ULA         13
#define LLA         14

//URA-> Upper Right Arm
//LLL-> Lower Left Leg
//etc.

#define RA          0
#define LA          1
#define LL          2
#define RL          3




#define KEEP        9999
#define A1          35      //length of femur
#define A2          75      //length of tibia

float               speed;  //overall speed

void servo_timer_init(void);
void set_pos(int limb, float x, float y, float z);
void wait_reach(int limb);
void wait_all_reach(void);
void servo_service(void);
void servo_startup(void);


#endif /* SERVO_H_ */
