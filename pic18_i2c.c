#include <xc.h>
#include "pic18_i2c.h"
//#ifndef _XTAL_FREQ
//#define _XTAL_FREQ 64000000
//#endif

void pic18_i2c_enable(void) {

    TRISD |= 0b01100000;
    RD5PPS = 0x1C;
    RD6PPS = 0x1B;

    SSP2CON1 = 0x28;
    SSP2CON2 = 0x00;
    SSP2CON3 = 0x00;
    SSP2STAT = 0x00;
    SSP2ADD = 0x159;
}

/* Scratchpad with various changes
TRISD =0b01100000; //amit
    LATD = 0b01100000;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00;
    RD5PPS = 0x1C;              // MSSP2 SDA
    RD6PPS = 0x1B;
    SSP2DATPPS = 0x1D;
    SSP2CLKPPS = 0x1E;


    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01;

        SSP2CON1 = 0x28;
    SSP2CON2 = 0x00;
    SSP2CON3 = 0x00;
    SSP2STAT = 0x00;
    SSP2ADD = 0x159;

    SSP2CON2bits.SEN = 1;
    __delay_ms(100);

    SSP2BUF = 0x52;
    __delay_ms(100);
    SSP2BUF = 0;
    __delay_ms(100);
    SSP2CON2bits.PEN = 1;
    __delay_ms(100);
 */
void pic18_i2c_disable(void) {
//    SSP2CON1bits.SSPEN = 0;     //Disable MSSP
}

int8_t pic18_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t length) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN);
    SSP2BUF = slave_addr << 1;
    if(SSP2CON1bits.WCOL);
    while (SSP2STATbits.R_W || SSP2STATbits.BF){
        //TODO: Check and clear WCOL
    };
    if (SSP2CON2bits.ACKSTAT) {
        SSP2CON2bits.PEN = 1;
        while (SSP2CON2bits.PEN);
        return -1;
    }
    SSP2BUF = reg_addr;
    if(SSP2CON1bits.WCOL);
    while (SSP2STATbits.R_W || SSP2STATbits.BF);
    while (length > 0) {
        SSP2BUF = *data;
        if(SSP2CON1bits.WCOL);
        while (SSP2STATbits.R_W);
        if (SSP2CON2bits.ACKSTAT == 1) {
            SSP2CON2bits.PEN = 1;
            while (SSP2CON2bits.PEN == 1);
            return -1;
        }
        --length;
        ++data;
    }
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN);
    return 0;
}

int8_t pic18_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t length) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = slave_addr << 1;
    while (SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT) goto i2c_rx_err;

    SSP2BUF = reg_addr;
    while (SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT) goto i2c_rx_err;
    SSP2CON2bits.RSEN = 1;
    while (SSP2CON2bits.RSEN);
    SSP2BUF = (slave_addr << 1) + 1; //address with R/W set for read
    while (SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT) goto i2c_rx_err;
    PIR3bits.BCL2IF = 0;
    PIR3bits.SSP2IF = 0;
    while (length > 0) {
        SSP2CON2bits.RCEN = 1;
        while(!SSP2CON2bits.RCEN);
        while (!SSP2STATbits.BF);
        *data = SSP2BUF;
        PIR3bits.SSP2IF = 0;
        if (length > 1) {
            SSP2CON2bits.ACKDT = 0;
        } else {
            SSP2CON2bits.ACKDT = 1;
        }
        SSP2CON2bits.ACKEN = 1;
        while (SSP2CON2bits.ACKEN);
        --length;
        ++data;
    }
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN);
    return 0;
i2c_rx_err:
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN);
    return -1;
}
