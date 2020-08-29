
#ifndef SPI_H
#define	SPI_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
void spi_init();
uint8_t spi_write(uint8_t data);
uint8_t spi_read();
uint8_t spi_write_bulk(uint8_t * data, uint8_t size);
uint8_t spi_write_bulk_const(uint8_t data, uint8_t size);
#endif	/* XC_HEADER_TEMPLATE_H */

