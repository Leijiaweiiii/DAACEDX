#ifndef PIC18_I2C_H
#define	PIC18_I2C_H

#include <xc.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

    //i2c functions
    void pic18_i2c_enable(void);
    void pic18_i2c_reset(void);
    void pic18_i2c_disable(void);
    int8_t pic18_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t length);
    int8_t pic18_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t length);
#ifdef	__cplusplus
}
#endif

#endif	/* PIC18_I2C_H */

