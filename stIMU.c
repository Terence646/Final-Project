/*
 * stIMU.c
 *
 *  Created on: Aug 2, 2020
 *      Author: Ariel's Yoga
 */

#include "stIMU.h"

const int16_t sens4g = 6842;

/****** TOP MAGNETOMETER ******/
void config_LIS3MDL(void){
    //SEND WHO AM I
        //printf to console for debugging
       int i;
       for (i=0; i<10000;i++); // Start-up time
       printf("Connecting to LIS3MDL...\n");
       for(i=0; i<100000; i++);
       printf("...\n");
       for(i=0; i<100; i++);
       printf("...\n");
       for(i=0; i<100; i++);

       uint8_t whoami;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_WHO_AM_I_REG);
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &whoami, 1, LIS3MDL_MAG_WHO_AM_I_REG);
       printf("%d\n", whoami);
       if(whoami == 61){
           printf("MAGNETOMETER Accessed\n");
       }

       uint8_t array[2];
       //uint8_t data[2];
       array[0] = LIS3MDL_MAG_CTRL_REG1;
       array[1] = CTR_REG1_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);
       //i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);

       //i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &data, 2, LIS3MDL_MAG_CTRL_REG1);

       //printf("%d\n", data);

       //i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &data, 2, LIS3MDL_MAG_STATUS_REG);

       //printf("%d\n", data);

       array[0] = LIS3MDL_MAG_CTRL_REG2;
       array[1] = CTR_REG2_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);
       //i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);

       array[0] = LIS3MDL_MAG_CTRL_REG3;
       array[1] = CTR_REG3_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);
       //i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);

       array[0] = LIS3MDL_MAG_CTRL_REG4;
       array[1] = CTR_REG4_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);
       //i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);

       array[0] = LIS3MDL_MAG_CTRL_REG5;
       array[1] = CTR_REG5_SET;
       i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);
       //i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, WRITE, array, 2, 0x00);
       printf("...\n");

}
int16_t read_magnetometer_x(void){

        int8_t mxh, mxl;
        int16_t mx;
        const int16_t sens4g = 6842;
        int i;

        mx = 0;

        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxh, 1, LIS3MDL_MAG_OUTX_H);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxh, 1, LIS3MDL_MAG_OUTX_H);

        for(i=0; i<10; i++);

        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxl, 1, LIS3MDL_MAG_OUTX_L);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mxl, 1, LIS3MDL_MAG_OUTX_L);

        mx = mxh << 8;
        mx = mx + mxl;
        //mx *= sens4g;

        return mx;

	
};
int16_t read_magnetometer_y(void){
        int8_t myh, myl;
        int16_t my;
        const int16_t sens4g = 6842;
        int i;

        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myh, 1, LIS3MDL_MAG_OUTY_H);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myh, 1, LIS3MDL_MAG_OUTY_H);

        for(i=0; i<10; i++);

        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myl, 1, LIS3MDL_MAG_OUTY_L);
        i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &myl, 1, LIS3MDL_MAG_OUTY_L);
	

        my = myh << 8;
        my = (my + myl);
        //my /= sens4g;

        return my;
	
};
int16_t read_magnetometer_z(void){
	    int8_t mzh,mzl;
	    int16_t mz;
	    const int16_t sens4g = 6842;
	    int i;

	    mz = 0;
	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzh, 1, LIS3MDL_MAG_OUTZ_H);
	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzh, 1, LIS3MDL_MAG_OUTZ_H);

	    for(i=0; i<10; i++);

	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzl, 1, LIS3MDL_MAG_OUTZ_L);
	    i2c_start(EUSCI_B0, LIS3MDL_MAG_I2C_ADDRESS, READ, &mzl, 1, LIS3MDL_MAG_OUTZ_L);
	

	    mz = mzh << 8;
	    mz = mz + mzl;
	    //mz = mz / sens4g;
	
	    return mz;
	
};
/****** BOTTOM MAGNETOMETER ******/
