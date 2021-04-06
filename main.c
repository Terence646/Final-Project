#include "msp.h"
#include "DistSensor.h"
#include "i2c.h"

/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Sensor();
}

void Sensor(void){
    timerA_stop();
    timerA_config();
    timerA_start();
    gpio_config();
    config_NVIC();
        while(1){
           ElapsedTime = ElapsedTicks * TickLength;        //convert ticks to time
           Distance = ElapsedTime * SpeedOfSound / 2;      //centimeters
       }
}
