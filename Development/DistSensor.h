/*
 * DistSensor.h
 *
 *  Created on: Apr 5, 2021
 *      Author: terencewilliams
 */

#ifndef DISTSENSOR_H_
#define DISTSENSOR_H_

#define TICKS ((uint16_t)0xFFFF)
#define TICKS2 ((uint16_t)0x6DDD)


void timerA_stop(void);
void timerA2_stop(void);
void timerA2_start(void);
void timerA2_config(void);
void timerA_config(void);
void timerA_start(void);
void config_NVIC(void);
void gpio_config(void);


#endif /* DISTSENSOR_H_ */
