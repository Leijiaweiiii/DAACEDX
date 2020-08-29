#include <xc.h>
#include "pic18_i2c.h"

void pic18_i2c_enable(void) {
    TRISD |= 0b11000000;        //our MMSP2 uses RD7 as SCL, RD6 as SDA, both set as inputs
    SSP2DATPPS = 0x1E;          // RD6
    SSP2CLKPPS = 0x1F;          // RD7
    RD7PPS = 0x1C;              // MSSP2 SDA
    RD6PPS = 0x1B;              // MSSP2 SCL
//    SSP2DATPPS = 0x1F;          // RD7
//    SSP2CLKPPS = 0x1E;          // RD6
//    RD7PPS = 0x1B;              // MSSP2 SCL
//    RD6PPS = 0x1C;              // MSSP2 SDA
    SSP2ADD = 160;                //100kHz with 64MHz clock
    SSP2CON1bits.SSPM = 0b1000; //I2C Master mode
    SSP2CON1bits.SSPEN = 1;     //Enable MSSP
}

void pic18_i2c_disable(void) {
    SSP2CON1bits.SSPEN = 0;     //Disable MSSP
}

int8_t pic18_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t length) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = slave_addr << 1;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT == 1) {
        SSP2CON2bits.PEN = 1;
        while (SSP2CON2bits.PEN == 1);
        return -1;
    }
    SSP2BUF = reg_addr;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    while (length > 0) {
        SSP2BUF = *data;
        while (SSP2STATbits.BF || SSP2STATbits.R_W){
            NOP();
        };
        --length;
        ++data;
    }
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN == 1);
    return 0;
}

int8_t pic18_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t length) {
    SSP2CON2bits.SEN = 1;
    while (SSP2CON2bits.SEN == 1);
    SSP2BUF = slave_addr << 1;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    if (SSP2CON2bits.ACKSTAT == 1) {
        SSP2CON2bits.PEN = 1;
        while (SSP2CON2bits.PEN == 1);
        return -1;
    }
    SSP2BUF = reg_addr;
    while (SSP2STATbits.BF || SSP2STATbits.R_W);
    SSP2CON2bits.RSEN = 1;
    while (SSP2CON2bits.RSEN == 1);
    SSP2BUF = (slave_addr << 1) + 1; //address with R/W set for read
    while (SSP2STATbits.BF || SSP2STATbits.R_W){
        NOP();
    };
    if(!SSP2STATbits.BF)
        NOP();
    while (length > 0) {
        SSP2CON2bits.RCEN = 1;
        while (!SSP2STATbits.BF){
            NOP();
        };
        *data = SSP2BUF;
        if (length > 1) {
            SSP2CON2bits.ACKDT = 0;
        } else {
            SSP2CON2bits.ACKDT = 1;
        }
        SSP2CON2bits.ACKEN = 1;
        while (SSP2CON2bits.ACKEN != 0);
        --length;
        ++data;
    }
    SSP2CON2bits.PEN = 1;
    while (SSP2CON2bits.PEN == 1);
    return 0;
}
