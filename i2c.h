
#ifndef I2C_H
#define	I2C_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
void i2c_init();
int8_t i2c_read(uint8_t device, uint8_t start_address, uint8_t size, uint8_t *data);
int8_t i2c_write(uint8_t device, uint8_t start_address, uint8_t size, uint8_t *data);
#endif	/* XC_HEADER_TEMPLATE_H */

