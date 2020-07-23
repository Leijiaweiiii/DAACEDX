#include "i2c.h"
#define I2C_RX_ADDR(d)  ((d<<7) | 0x01)
#define I2C_TX_ADDR(d)  ((d<<7) & 0xFE)

void i2c_init(){
    SSP2DATPPS = 0x1E;   //RD6->MSSP2:SDA2;    
    RD6PPS = 0x1C;   //RD6->MSSP2:SDA2;    
    RD7PPS = 0x1B;   //RD7->MSSP2:SCL2;    
    SSP2CLKPPS = 0x1F;   //RD7->MSSP2:SCL2;    
//    // Set SSP2DATPPS to PIN 49 (RD7)
//    SSP2DATPPS = 0x1F;
//    // Set SSP2CLKPPS to PIN 50 (RD6)
//    SSP2CLKPPS = 0x1E;
//    // Set PIN inputs to SSP dat and clk in PPS 
//    RD7PPS = 0x1C;
//    RD6PPS = 0x1B;
    TRISAbits.TRISA4 = 1;
    TRISAbits.TRISA5 = 1;
    // Set TRIS bits as input 
    SSP2ADD = 0x27;                 // TODO: Check if 0x26 or 0x28 ... 
    SSP2CON1bits.SSPM = 0b1000;     // I2C Master, clock = Fosc/(4 * SSP2ADD + 1)
    // 7 bits addresing
    SSP2CON1bits.SSPEN = 1;         // Enable SSP    
}


void i2c_spin_wait(){
    while(SSP2IF == 0)
        NOP();
    SSP2IF = 0;
}

int8_t i2c_tx_byte(uint8_t data){
    SSP2BUF = data;
    i2c_spin_wait();
    if (SSP2CON2bits.ACKSTAT){
        return -1;
    }
    
    return 0;
}

#define I2C_TX(d)   {\
    ret=i2c_tx_byte(d);\
    if (ret < 0) {\
        SSP2CON2bits.PEN = 1;\
        return ret;\
        }\
    }

#define I2C_STOP()  {SSP2CON2bits.PEN = 1;} 
#define I2C_START() {SSP2CON2bits.SEN = 1;}
#define I2C_RX_EN() {SSP2CON2bits.RCEN = 1;} 

//int8_t i2c_read(uint8_t device, uint8_t start_address, uint8_t size, uint8_t *data){
//    int8_t ret = 0;
//    SSP2IF = 0;
//    I2C_START()
////    i2c_spin_wait();
//    I2C_TX(I2C_TX_ADDR(device));
//    I2C_TX(start_address);
//    I2C_STOP();
//    I2C_START();
//    I2C_TX(I2C_RX_ADDR(device));
//    for(uint8_t i = 0;i<size;i++) {
//        I2C_RX_EN();
//        i2c_spin_wait();
//        data[i] = SSP2BUF;
//        SSP2CON2bits.ACKDT = (size==i); // Last byte should send NACK
//        SSP2CON2bits.ACKEN = 1;
//        i2c_spin_wait();
//    }
//    I2C_STOP();
//    return ret;
//}
//
//
//                    
//int8_t i2c_write(uint8_t device, uint8_t start_address, uint8_t size, uint8_t *data){
//    int8_t ret = 0;
//    SSP2IF = 0;
//    I2C_START();
//    i2c_spin_wait();
//    I2C_TX(I2C_TX_ADDR(device));
//    I2C_TX(start_address);
//    while(size--){
//        I2C_TX(data[size]);
//    }
//    I2C_STOP();
//    return ret;
//}

int8_t i2c_read(uint8_t device, uint8_t start_address, uint8_t size, uint8_t *data){
    
    return 0;
}

int8_t i2c_write(uint8_t device, uint8_t start_address, uint8_t size, uint8_t *data){
    return 0;
}