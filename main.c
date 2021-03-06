#include "msp.h"
#include "servo.h"




volatile int state = 0;
const int servos[4][2] = { {URA,LRA} , {ULA,LLA} , {ULL,LLL} , {URL,LRL} };

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    //set up Button Interrupt



    P1->DIR &= ~BIT1;
    P1->OUT |= BIT1;
    P1->REN |= BIT1;
    P1->IES |= BIT1;
    P1->IFG = 0;
    P1->IE  |= BIT1;


    // Debug LED init
    P1->OUT &= ~BIT0;
    P1->DIR |= BIT0;

    P2->OUT &= ~BIT0;
    P2->DIR |= BIT0;


    //enable interrupts
    NVIC_EnableIRQ(PORT1_IRQn);

    __enable_irq();


    I2C_OPEN_STRUCT_TypeDef i2c_open_struct;

    i2c_open_struct.uca10 = 0;                              //no self 10-bit Address
    i2c_open_struct.ucsla10 = 0;                            //no 10-bit Addressing
    i2c_open_struct.ucmm = 0;                               //No Multi-Master
    i2c_open_struct.ucmst = EUSCI_B_CTLW0_MST;              //Master Mode
    i2c_open_struct.ucsselx = EUSCI_B_CTLW0_SSEL__SMCLK;    //SMCLK to be selected (3MHz)
    i2c_open_struct.ucbrx = 30;                             //Prescaler for Selected Clock.
                                                            //(100kHz)



    //Debug Toggle LED P1.0
    P1->OUT |= BIT0;
    //This sets up the I2C driver to operate with the
    //correct settings.
    //EUSCI_B0 uses P6.5 as SCL and P6.4 as SDA
    i2c_open(I2C_MODULE, &i2c_open_struct);

    pca9685_init(); //sets up PCA to output at the correct frequency

    //Debug Toggle LED P2

    P2->OUT |= BIT0;

    P1->DIR |= BIT0;
    P1->OUT &= ~BIT0;


    P2->DIR |= BIT4;
    P2->OUT &= ~BIT4;

    P2->DIR |= BIT5;
    P2->OUT &= ~BIT5;

    P2->DIR |= BIT6;
    P2->OUT &= ~BIT6;

    P2->DIR |= BIT7;
    P2->OUT &= ~BIT7;

    P5->DIR |= BIT6;
    P5->OUT &= ~BIT6;

    P5->DIR |= BIT7;
    P5->OUT &= ~BIT7;

    P6->DIR |= BIT6;
    P6->OUT &= ~BIT6;

    P6->DIR |= BIT7;
    P6->OUT &= ~BIT7;

    P1->OUT |= BIT0;
    P2->OUT |= BIT4;
    P2->OUT |= BIT5;
    P2->OUT |= BIT6;
    P2->OUT |= BIT7;

    P5->OUT |= BIT6;
    P5->OUT |= BIT7;

    P6->OUT |= BIT6;
    P6->OUT |= BIT7;


    volatile uint32_t k;
//    int i;
//    int j;

    //Leg angles
    int angleURL;
    int angleULL;
    int angleLLL;
    int angleLRL;
    //Arm angles
    int angleURA = 45;
    int angleULA = 45;
    int angleLLA = 45;
    int angleLRA = 45;
    //Generic angle
    int angle;

    while(state == 0);

    while(state == 2);      // Turn to the Right when object is Detected

        //Turning gait:
        //Right legs step backwards and then pull forward
        //Left legs step forwards and then pull backward

    while(state == 1){      //Forward Movement
        //Upper right arm/leg
        for(angleURL=160; angleURL<180; angleURL++){ //Starting condition is known
            servo_write(servos[2][0], angleURL); //Raise URL
            for(k=0; k<1000;k++);
        }
        for(angle=angleURA; angle<135; angle++){ //Starting condition may not be known. Implement a temporary variable to set equal to starting condition
            //angleULA = angle; //Update angleULA
            angleURA = angle; //Update angleURA
            servo_write(servos[0][0], angleURA); //Move URA forward
            for(k=0; k<1000;k++);
        }

        for(angle = 0; angle<30; angle++){ //Move each other arm backwards
            angleULA = angleULA - angle; //Update the other angles
            angleLLA = angleLLA - angle;
            angleLRA = angleLRA - angle;
            servo_write(servos[1][1], angleLLA); //LLA
            servo_write(servos[1][0], angleULA); //ULA
            servo_write(servos[0][1], angleLRA); //LRA
            for(k=0; k<1000;k++);
        }
        for(angleURL=180; angleURL<160; angleURL--){
            servo_write(servos[2][0], angleURL); //Lower URL
            for(k=0; k<1000;k++);
        }

        // Upper left arm/leg
        for(angleULL=85; angleULL>20; angleULL--){
            servo_write(servos[3][0], angleULL); //Raise ULL
            for(k=0; k<1000;k++);
        }
        for(angle=angleULA; angle<135; angle++){
            servo_write(servos[1][0], angleULA); //Move ULA forward
            for(k=0; k<1000;k++);
        }
        for(angle = 0; angle<30; angle++){
            angleURA = angleURA - angle;
            angleLLA = angleLLA - angle;
            angleLRA = angleLRA - angle;
            servo_write(servos[1][1], angleLLA); //LLA
            servo_write(servos[0][0], angleURA); //URA
            servo_write(servos[0][1], angleLRA); //LRA
            for(k=0; k<1000;k++);
        }
        for(angleULL=20; angleULL<85; angleULL++){
            servo_write(servos[3][0], angleULL); //Lower ULL
            for(k=0; k<1000;k++);
        }

        // Lower right arm/leg
        for(angleLRL=85; angleLRL>20; angleLRL--){
            servo_write(servos[2][1], angleLRL); //Raise LRL
            for(k=0; k<1000;k++);
        }
        for(angle=angleLRA; angle<135; angle++){
            servo_write(servos[0][1], angleLRA); //Move LRA forward
            for(k=0; k<1000;k++);
        }
        for(angle = 0; angle<30; angle++){
            angleURA = angleURA - angle;
            angleLLA = angleLLA - angle;
            angleULA = angleULA - angle;
            servo_write(servos[1][1], angleLLA); //LLA
            servo_write(servos[0][0], angleURA); //URA
            servo_write(servos[1][0], angleULA); //ULA
            for(k=0; k<1000;k++);
        }
        for(angleLRL=20; angleLRL<85; angleLRL++){
            servo_write(servos[2][1], angleLRL); //Lower LRL
            for(k=0; k<1000;k++);
        }


        // Lower left arm/leg
        for(angleLLL=85; angleLLL>20; angleLLL--){
            servo_write(servos[3][1], angleLLL); //Raise LLL
            for(k=0; k<1000;k++);
        }
        for(angle=angleLLA; angle<135; angle++){
            servo_write(servos[1][1], angleLLA); //Move LLA forward
            for(k=0; k<1000;k++);
        }
        for(angle = 0; angle<30; angle++){
            angleURA = angleURA - angle;
            angleLRA = angleLRA - angle;
            angleULA = angleULA - angle;
            servo_write(servos[0][1], angleLRA); //LRA
            servo_write(servos[0][0], angleURA); //URA
            servo_write(servos[1][0], angleULA); //ULA
            for(k=0; k<1000;k++);
        }
        for(angleLLL=20; angleLLL<85; angleLLL++){
            servo_write(servos[3][1], angleLLL); //Lower LLL
            for(k=0; k<1000;k++);
        }
        /*
                for(angle=45; angle<135; angle++){
                    servo_write(servos[0][0], angle); //URA
                    servo_write(servos[1][1], angle); //LLA
                    for(k=0; k<1000;k++);
                    servo_write(servos[1][0], angle); //ULA
                    servo_write(servos[0][1], angle); //LRA
                }
                for(angle=135; angle>45; angle--){
                    servo_write(servos[0][0], angle); //URA
                    servo_write(servos[1][1], angle); //LLA
                    for(k=0; k<1000;k++);
                    servo_write(servos[1][0], angle); //ULA
                    servo_write(servos[0][1], angle); //LRA
                }
            }
         */
    }
}



/* Port1 ISR */

void PORT1_IRQHandler(void){
    volatile uint32_t k;

    //Change State to Standing Position
    if(P1->IFG & BIT1)
        state = 1;

    // Delay for switch debounce
    for(k = 0; k < 100000; k++);

    P1->IFG &= ~BIT1;
}
