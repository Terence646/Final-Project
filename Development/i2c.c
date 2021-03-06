/*
 * i2c.c
 *
 *  Created on: Jun 16, 2020
 *      Author: Tyler Davidson
 */

#include "i2c.h"

#define SCL_PULSE   50 //number to control how much to toggle scl during bus_clear


//private variables
static I2C_PAYLOAD_TypeDef i2c_payload_struct;


//private functions


/*
 * This private function is designed to test the settings to be used in i2c_open
 * to make sure that all inputs are valid. Each setting has only a few valid possible
 * values, so this function will prevent any bad inputs
 */
void i2c_openChecker(EUSCI_B_Type *i2c, I2C_OPEN_STRUCT_TypeDef *i2c_open_struct){

    //check UCA10 setting
    if(i2c_open_struct->uca10 != 0)
        if(i2c_open_struct->uca10 != EUSCI_B_CTLW0_A10)
            assert(false);

    //check UCSLA10 setting
    if(i2c_open_struct->ucsla10 != 0)
        if(i2c_open_struct->ucsla10 != EUSCI_B_CTLW0_SLA10)
            assert(false);

    //check UCMM setting
    if(i2c_open_struct->ucmm != 0)
        if(i2c_open_struct->ucmm != EUSCI_B_CTLW0_MM)
            assert(false);

    //check UCMST setting
    if(i2c_open_struct->ucmst != 0)
        if(i2c_open_struct->ucmst != EUSCI_B_CTLW0_MST)
            assert(false);

    //check UCSSELx setting
    if(i2c_open_struct->ucsselx != EUSCI_B_CTLW0_SSEL__UCLKI)
        if(i2c_open_struct->ucsselx != EUSCI_B_CTLW0_SSEL__ACLK)
            if(i2c_open_struct->ucsselx != EUSCI_B_CTLW0_SSEL__SMCLK)
                if(i2c_open_struct->ucsselx != EUSCI_B_CTLW0_UCSSEL_3) //this is also SMCLK
                    assert(false);
}



/*
 * According to the I2C specifications, if the i2c bus ever becomes blocked (the slave holding SDA
 * low) it can be cleared and reset by toggling the SCL line multiple times
 * followed by a stop condition. This method is somewhat clumsy
 */
void bus_clear(EUSCI_B_Type *i2c){

    volatile uint8_t i;

    if(i2c == EUSCI_B0){
        P1->DIR |= BIT7;
        for(i=0; i<SCL_PULSE; i++){
            P1->OUT ^= BIT7; //toggle 1.7 (B0 SCL) several times to reset bus
        }
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP; //send a stop bit
    }
    else if(i2c == EUSCI_B1){
        P6->DIR |= BIT5;
        for(i=0; i<SCL_PULSE; i++){
            P6->OUT ^= BIT5; //toggle 6.5 (B1 SCL) several times to reset bus
        }
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP; //send a stop bit
    }
    else if(i2c == EUSCI_B2){
        P3->DIR |= BIT7;
        for(i=0; i<SCL_PULSE; i++){
            P3->OUT ^= BIT7; //toggle 3.7 (B2 SCL) several times to reset bus
        }
        EUSCI_B2->CTLW0 |= EUSCI_B_CTLW0_TXSTP; //send a stop bit
    }
    else if(i2c == EUSCI_B3){
        P6->DIR |= BIT7;
        for(i=0; i<SCL_PULSE; i++){
            P6->OUT ^= BIT7; //toggle 6.7 (B3 SCL) several times to reset bus
        }
        EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTP; //send a stop bit
    }
    else assert(false); //Wrong I2C input

}



//public functions



/*
 * THis function will take a struct of settings for the i2c peripheral, check to
 * make sure the settings are valid, and then apply those settings to the specified
 * i2c module. This function also enables the RXIFG0, TXIFG0 and NACK interrupts
 */
void i2c_open(EUSCI_B_Type *i2c, I2C_OPEN_STRUCT_TypeDef *i2c_open_struct){

    bus_clear(i2c);

    //determine which eUSCI_B module is being used to configure pins and enable IRQ
    if(i2c == EUSCI_B0){
        P1->SEL0 |= BIT6 | BIT7; //configure P1.6 and P1.7 for I2C
        P1->SEL1 &= ~BIT6 & ~BIT7;
        NVIC_SetPriority(EUSCIB0_IRQn,NVIC_IPR0_PRI_0_M);
        NVIC_EnableIRQ(EUSCIB0_IRQn);
    }
    else if(i2c == EUSCI_B1){
        P6->SEL0 |= BIT4 | BIT5; //configure P6.4 and P6.5 for I2C
        P6->SEL1 &= ~BIT4 & ~BIT5;
        NVIC_SetPriority(EUSCIB1_IRQn,NVIC_IPR0_PRI_1_M);
        NVIC_EnableIRQ(EUSCIB1_IRQn);
    }
    else if(i2c == EUSCI_B2){
        P3->SEL0 |= BIT5 | BIT6; //configure P3.5 and P3.6 for I2C
        P3->SEL1 &= ~BIT5 & ~BIT6;
        NVIC_SetPriority(EUSCIB2_IRQn,NVIC_IPR0_PRI_2_M);
        NVIC_EnableIRQ(EUSCIB2_IRQn);
    }
    else if(i2c == EUSCI_B3){
        P6->SEL0 &= ~BIT6 & ~BIT7; //configure P6.6 and P6.7 for I2C
        P6->SEL1 |= BIT6 & BIT7;
        NVIC_SetPriority(EUSCIB3_IRQn,NVIC_IPR0_PRI_3_M);
        NVIC_EnableIRQ(EUSCIB3_IRQn);
    }
    else assert(false); //if none of the modules are selected, error

    i2c_openChecker(i2c, i2c_open_struct); //this function ensures settings are valid

    //Set UCSWRST bit to enable module modification
    i2c->CTLW0 |= EUSCI_B_CTLW0_SWRST;
    i2c->CTLW0 = EUSCI_B_CTLW0_SWRST |      //Stay in Reset State
            i2c_open_struct->uca10 |        //Own Addressing Mode
            i2c_open_struct->ucsla10 |      //Slave Addressing Mode
            i2c_open_struct->ucmm |         //Multi-Master Select
            i2c_open_struct->ucmst |        //Master Mode Select
            EUSCI_B_CTLW0_MODE_3 |          //eUSCI_B I2C Mode
            EUSCI_B_CTLW0_SYNC |            //Synchronous Mode - Always R/W as 1
            i2c_open_struct->ucsselx;       //eUSCI_B Clock Source Select

    //Set Clock Prescaler
    i2c->BRW = i2c_open_struct->ucbrx; //baudrate = clock source / prescaler

    //Turn off Reset Mode
    i2c->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;


    //clear and enable interrupt flags
    i2c->IFG &= ~EUSCI_B_IFG_NACKIFG & ~EUSCI_B_IFG_TXIFG0;
    i2c->IE |= EUSCI_B_IE_TXIE0 |   //TX interrupt
            EUSCI_B_IE_RXIE0    |   //RX interrupt
            EUSCI_B_IE_NACKIE;      //NACK interrupt

}


/*
 * This function is what starts communication on the i2c bus. It is designed to load data into
 * a struct that can be accessed by different parts in the IRQ. It also loads the Slave Address
 * into the respective register and sends a start condition on the bus.
 *
 * the *data should be the name of a struct from which to read/write the data to/from
 * the deviceReg is only used for read operations. Can be any value for write operations
 *
 */
void i2c_start(EUSCI_B_Type *i2c, uint8_t Address, bool RW, uint8_t *data, uint8_t dataBytes, uint8_t deviceReg){

    //make sure any previous transmission has stopped
    while(i2c->STATW & EUSCI_B_STATW_BBUSY);

    //load payload struct
    i2c_payload_struct.SA = Address;
    i2c_payload_struct.RW = RW;
    i2c_payload_struct.byteCount = 0;
    i2c_payload_struct.data = data;
    i2c_payload_struct.dataBytes = dataBytes;
    i2c_payload_struct.deviceReg = deviceReg;
    i2c_payload_struct.i2c = i2c;


    //Load Slave Address to the I2C register
    i2c->I2CSA = Address;


    i2c->CTLW0 |= EUSCI_B_CTLW0_TR |    //Set Master as Transmitter
            EUSCI_B_CTLW0_TXSTT;        //Generate Start Condition

}



//Need an IRQ for EACH eUSCI_Bx Module. This is the IRQ for eUSCI_B0
void EUSCIB0_IRQHandler(void){

//~~~~~~~~~~~~~~~~~~~~~~RX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(i2c_payload_struct.RW)
    {
        if(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)
        {
            assert(false);

            //clear flag
            EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }


        if(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0)
        {
            if((EUSCI_B0->STATW & EUSCI_B_STATW_BCNT_MASK) == 0)
            {
                EUSCI_B0->TXBUF = i2c_payload_struct.deviceReg;
                i2c_payload_struct.byteCount++;
            }
            if((EUSCI_B0->STATW & EUSCI_B_STATW_BCNT_MASK) > 0){
                EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;    //Set Master as Receiver
                EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;  //Generate Start Condition
            }
        }

        if(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0)
        {
            uint8_t rx = EUSCI_B0->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;
            *i2c_payload_struct.data = rx; //put data in the place pointed to by *data
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXNACK;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }

//~~~~~~~~~~~~~~~~~~~~~~TX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(!i2c_payload_struct.RW)
    {
    //If a NACK is RXed, transmission will start again
        if(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG)
        {

            //clear flag
            EUSCI_B0->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }

        //if TXBuf gets emptied successfully
        if(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0)
        {

            //clear flag
            EUSCI_B0->IFG &= ~EUSCI_B_IFG_TXIFG0;

            //if the count = number of bytes, it means the last byte was sent
            if(i2c_payload_struct.byteCount == i2c_payload_struct.dataBytes)
            {


//                EUSCI_B0->IFG &= ~EUSCI_B_IFG_TXIFG0;

                //transmission is done, stop procedure
                EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;


                //ensure flag is cleared
                EUSCI_B0->IFG &= ~EUSCI_B_IFG_TXIFG0;


            }

            else
            {
                //TX byte from data[]
                EUSCI_B0->TXBUF = i2c_payload_struct.data[i2c_payload_struct.byteCount];

                //increase byte count
                i2c_payload_struct.byteCount++;
            }

        }
    }
}














//Need an IRQ for EACH eUSCI_Bx Module. This is the IRQ for eUSCI_B1
void EUSCIB1_IRQHandler(void){

//~~~~~~~~~~~~~~~~~~~~~~RX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(i2c_payload_struct.RW)
    {
        if(EUSCI_B1->IFG & EUSCI_B_IFG_NACKIFG)
        {
            assert(false);

            //clear flag
            EUSCI_B1->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }


        if(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0)
        {
            if((EUSCI_B1->STATW & EUSCI_B_STATW_BCNT_MASK) == 0)
            {
                EUSCI_B1->TXBUF = i2c_payload_struct.deviceReg;
                i2c_payload_struct.byteCount++;
            }
            if((EUSCI_B1->STATW & EUSCI_B_STATW_BCNT_MASK) > 0){
                EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;    //Set Master as Receiver
                EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;  //Generate Start Condition
            }
        }

        if(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0)
        {
            uint8_t rx = EUSCI_B1->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;
            *i2c_payload_struct.data = rx; //put data in the place pointed to by *data
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXNACK;
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }

//~~~~~~~~~~~~~~~~~~~~~~TX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(!i2c_payload_struct.RW)
    {
    //If a NACK is RXed, transmission will start again
        if(EUSCI_B1->IFG & EUSCI_B_IFG_NACKIFG)
        {

            //clear flag
            EUSCI_B1->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }

        //if TXBuf gets emptied successfully
        if(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0)
        {

            //clear flag
            EUSCI_B1->IFG &= ~EUSCI_B_IFG_TXIFG0;

            //if the count = number of bytes, it means the last byte was sent
            if(i2c_payload_struct.byteCount == i2c_payload_struct.dataBytes)
            {


//                EUSCI_B1->IFG &= ~EUSCI_B_IFG_TXIFG0;

                //transmission is done, stop procedure
                EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;


                //ensure flag is cleared
                EUSCI_B1->IFG &= ~EUSCI_B_IFG_TXIFG0;


            }

            else
            {
                //TX byte from data[]
                EUSCI_B1->TXBUF = i2c_payload_struct.data[i2c_payload_struct.byteCount];

                //increase byte count
                i2c_payload_struct.byteCount++;
            }

        }
    }
}











//Need an IRQ for EACH eUSCI_Bx Module. This is the IRQ for eUSCI_B2
void EUSCIB2_IRQHandler(void){

//~~~~~~~~~~~~~~~~~~~~~~RX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(i2c_payload_struct.RW)
    {
        if(EUSCI_B2->IFG & EUSCI_B_IFG_NACKIFG)
        {
            assert(false);

            //clear flag
            EUSCI_B2->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B2->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }


        if(EUSCI_B2->IFG & EUSCI_B_IFG_TXIFG0)
        {
            if((EUSCI_B2->STATW & EUSCI_B_STATW_BCNT_MASK) == 0)
            {
                EUSCI_B2->TXBUF = i2c_payload_struct.deviceReg;
                i2c_payload_struct.byteCount++;
            }
            if((EUSCI_B2->STATW & EUSCI_B_STATW_BCNT_MASK) > 0){
                EUSCI_B2->CTLW0 &= ~EUSCI_B_CTLW0_TR;    //Set Master as Receiver
                EUSCI_B2->CTLW0 |= EUSCI_B_CTLW0_TXSTT;  //Generate Start Condition
            }
        }

        if(EUSCI_B2->IFG & EUSCI_B_IFG_RXIFG0)
        {
            uint8_t rx = EUSCI_B2->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;
            *i2c_payload_struct.data = rx; //put data in the place pointed to by *data
            EUSCI_B2->CTLW0 |= EUSCI_B_CTLW0_TXNACK;
            EUSCI_B2->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }

//~~~~~~~~~~~~~~~~~~~~~~TX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(!i2c_payload_struct.RW)
    {
    //If a NACK is RXed, transmission will start again
        if(EUSCI_B2->IFG & EUSCI_B_IFG_NACKIFG)
        {

            //clear flag
            EUSCI_B2->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B2->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }

        //if TXBuf gets emptied successfully
        if(EUSCI_B2->IFG & EUSCI_B_IFG_TXIFG0)
        {

            //clear flag
            EUSCI_B2->IFG &= ~EUSCI_B_IFG_TXIFG0;

            //if the count = number of bytes, it means the last byte was sent
            if(i2c_payload_struct.byteCount == i2c_payload_struct.dataBytes)
            {


//                EUSCI_B2->IFG &= ~EUSCI_B_IFG_TXIFG0;

                //transmission is done, stop procedure
                EUSCI_B2->CTLW0 |= EUSCI_B_CTLW0_TXSTP;


                //ensure flag is cleared
                EUSCI_B2->IFG &= ~EUSCI_B_IFG_TXIFG0;


            }

            else
            {
                //TX byte from data[]
                EUSCI_B2->TXBUF = i2c_payload_struct.data[i2c_payload_struct.byteCount];

                //increase byte count
                i2c_payload_struct.byteCount++;
            }

        }
    }
}














//Need an IRQ for EACH eUSCI_Bx Module. This is the IRQ for eUSCI_B3
void EUSCIB3_IRQHandler(void){

//~~~~~~~~~~~~~~~~~~~~~~RX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(i2c_payload_struct.RW)
    {
        if(EUSCI_B3->IFG & EUSCI_B_IFG_NACKIFG)
        {
            assert(false);

            //clear flag
            EUSCI_B3->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }


        if(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG0)
        {
            if((EUSCI_B3->STATW & EUSCI_B_STATW_BCNT_MASK) == 0)
            {
                EUSCI_B3->TXBUF = i2c_payload_struct.deviceReg;
                i2c_payload_struct.byteCount++;
            }
            if((EUSCI_B3->STATW & EUSCI_B_STATW_BCNT_MASK) > 0){
                EUSCI_B3->CTLW0 &= ~EUSCI_B_CTLW0_TR;    //Set Master as Receiver
                EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;  //Generate Start Condition
            }
        }

        if(EUSCI_B3->IFG & EUSCI_B_IFG_RXIFG0)
        {
            uint8_t rx = EUSCI_B3->RXBUF & EUSCI_B_RXBUF_RXBUF_MASK;
            *i2c_payload_struct.data = rx; //put data in the place pointed to by *data
            EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXNACK;
            EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }

//~~~~~~~~~~~~~~~~~~~~~~TX~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    if(!i2c_payload_struct.RW)
    {
    //If a NACK is RXed, transmission will start again
        if(EUSCI_B3->IFG & EUSCI_B_IFG_NACKIFG)
        {

            //clear flag
            EUSCI_B3->IFG &= ~EUSCI_B_IFG_NACKIFG;

            //reset byte count and send Start again
            i2c_payload_struct.byteCount = 0;
            EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        }

        //if TXBuf gets emptied successfully
        if(EUSCI_B3->IFG & EUSCI_B_IFG_TXIFG0)
        {

            //clear flag
            EUSCI_B3->IFG &= ~EUSCI_B_IFG_TXIFG0;

            //if the count = number of bytes, it means the last byte was sent
            if(i2c_payload_struct.byteCount == i2c_payload_struct.dataBytes)
            {


//                EUSCI_B3->IFG &= ~EUSCI_B_IFG_TXIFG0;

                //transmission is done, stop procedure
                EUSCI_B3->CTLW0 |= EUSCI_B_CTLW0_TXSTP;


                //ensure flag is cleared
                EUSCI_B3->IFG &= ~EUSCI_B_IFG_TXIFG0;


            }

            else
            {
                //TX byte from data[]
                EUSCI_B3->TXBUF = i2c_payload_struct.data[i2c_payload_struct.byteCount];

                //increase byte count
                i2c_payload_struct.byteCount++;
            }

        }
    }
}
