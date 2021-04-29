/*
 * servo.c
 *
 *  Created on: Jul 14, 2020
 *      Author: Tyler Davidson
 */
#include "servo.h"
#include "math.h"

void xyz_to_angle(int limb, volatile float *t1, volatile float *t2, volatile float x, volatile float y, volatile float z);
void servo_service(void);
void angle_to_servo(int limb, float t1, float t2);


volatile bool timer_flag = false;
const int Servo[4][2] = { {URA,LRA} , {ULA,LLA} , {ULL,LLL} , {URL,LRL} };
//    Servo Matrix
//  __|_0_|_1_|_2_|_3_
//  0_|URA|ULA|URL|ULL
//  1_|LRA|LLA|LRL|LLL
//  NOTE! Columns 1 and 2 have opposite servo direction to column 0 and 3

volatile float  pos_now[4][3];         //coords of each foot (xyz)
volatile float  pos_expected[4][3];    //expected coords of each foot
float           temp_speed[4][3];       //temporary speed


void servo_startup(){
    set_pos(RA, 115, 0, 0);
    set_pos(LA, -115, 0, 0);
    set_pos(LL, -115, 0, 0);
    set_pos(RL, 115, 0, 0);

    int i;
    int j;
    for(i=0; i<4; i++){
        for(j=0; j<3; j++){
            pos_now[i][j] = pos_expected[i][j];
        }
    }


}

void servo_timer_init(void){
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A0->CCR[0] = 60000;               //count to 60,000 at 3MHz (T = 20ms)
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK , UP mode
            TIMER_A_CTL_MC__UP;
    NVIC_EnableIRQ(TA0_0_IRQn);


}



void servo_service(void){
//    P1->OUT |= BIT0;

    static float t1, t2;

    int i;
    int j;
    for(i = 0; i<4; i++){
        for(j = 0; j<3; j++){
            if(abs(pos_now[i][j] - pos_expected[i][j]) >= abs(temp_speed[i][j]))
                pos_now[i][j] += temp_speed[i][j];
            else
                pos_now[i][j] = pos_expected[i][j];
        }
        xyz_to_angle(i, &t1, &t2, pos_now[i][0],pos_now[i][1],pos_now[i][2]);
        angle_to_servo(i,t1,t2);
    }

//    P1->OUT &= ~BIT0;
    timer_flag = false; //lower the flag, servos were updated


}

/*
 * Sets a point for the selected leg to move towards.
 * Use "KEEP" to keep that dimension the same
 */
void set_pos(int limb, float x, float y, float z){

    //distance to move in each direction
    float dist_x = 0;
    float dist_y = 0;
    float dist_z = 0;

    if(x != KEEP) dist_x = x - pos_now[limb][0]; //update distance to target in each dimension
    if(y != KEEP) dist_y = y - pos_now[limb][1];
    if(z != KEEP) dist_z = z - pos_now[limb][2];

    //calculate total distance from current point to new point
    float total_distance = sqrt(pow(dist_x,2) + pow(dist_y,2) + pow(dist_z,2));

    //divide distance in a direction by a scaled amount based on speed
    temp_speed[limb][0] = dist_x / total_distance * speed;
    temp_speed[limb][1] = dist_y / total_distance * speed;
    temp_speed[limb][2] = dist_z / total_distance * speed;

    //updated expected position if keeping position
    if(x != KEEP) pos_expected[limb][0] = x;
    if(y != KEEP) pos_expected[limb][1] = y;
    if(z != KEEP) pos_expected[limb][2] = z;

}

/*
 * Wait for Limb to reach the expected position
 */
void wait_reach(int limb){
    while(1){
        if(timer_flag)
            servo_service();
        if(pos_now[limb][0] == pos_expected[limb][0])
            if(pos_now[limb][1] == pos_expected[limb][1])
                if(pos_now[limb][2] == pos_expected[limb][2])
                    return;
    }



}

/*
 * wait for ALL limbs to reach expected position
 */
void wait_all_reach(void){
    int i;
    for(i=0; i<4; i++)
        wait_reach(i);
}



/*
 * Calculates the angle needed for the servos to position the foot at xyz
 * t1 is Yaw of leg
 * t2 is pitch of leg
 */
void xyz_to_angle(int limb, volatile float *t1, volatile float *t2, volatile float x, volatile float y, volatile float z){


    if(limb == LA){
        x = -x;
        y = -y;
    }

    if(limb == LL){
            x = -x;
            y = -y;
        }




    *t1 = (180/3.14) * atan2(y,x);
    *t2 = (180/3.14) * atan2(z,sqrt(pow(x,2) + pow(y,2))-A1);

}

/*
 * Corrects Direction of servos facing other directions
 */
void angle_to_servo(int limb, float t1, float t2){

    //transformation to be compatible w/ degree to count equation
    t1 += 90;
    t2 += 90;


    if(limb == RA) t2 = 180 - t2;
    if(limb == LL) t2 = 180 - t2;




    servo_write(Servo[limb][0], t1);
    servo_write(Servo[limb][1], t2);

}

void TA0_0_IRQHandler(void) {
    // Clear the compare interrupt flag
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

    P1->OUT ^= BIT0;
    timer_flag = true;


}
