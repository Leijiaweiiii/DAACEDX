#include "eeprom.h"
#include "DAACED.h"
#include "spi.h"

static volatile uint8_t test = 0b01010101;
void eeprom_write_read_test(void){
    char * tx_data = "DEADBEAF";
//    uint8_t tx_data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//    uint8_t tx_data[8] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
    
    eeprom_write_array_bulk(SettingsEndAddress, tx_data, 8);
    TBool __c = True;
    while(__c){
        uint8_t rx_data[10] = { 0 };
        eeprom_read_array(SettingsEndAddress, rx_data, 8);
        NOP();
    }
}


void eeprom_reset_op(void){
    EEPROM_CS_SELECT();
    Delay(1);
    EEPROM_HOLD_EN();
    Delay(1);
    EEPROM_CS_DESELECT();
    Delay(1);
    EEPROM_HOLD_DIS();
}

void eeprom_init(void) {
    EEPROM_CS_INIT();
    EEPROM_HOLD_INIT();
    EEPROM_WP_INIT();

    EEPROM_CS_DESELECT();
    EEPROM_HOLD_DIS();
    EEPROM_WP_DIS();
    eeprom_reset_op();
//    eeprom_write_read_test();
}

#define eeprom_wait_deselect() {EEPROM_CS_DESELECT();EEPROM_CS_SELECT();}

/**
 * Write data in bulk mode. It's faster than byte by byte.
 * Caution needed not to cross 64 byte page boundary
 * @param address - start address
 * @param data - pointer to data
 * @param size - size in  bytes to write
 */
void eeprom_write_data_bulk(uint16_t address, uint8_t * data, uint8_t size) {
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    spi_write(CMD_WRSR);
    spi_write(0x02); // Enable Write Latch.
    eeprom_wait_deselect();
    spi_write(CMD_WREN);
    eeprom_wait_deselect();
    spi_write(CMD_WRITE);
    spi_write((uint8_t)MSB(address));
    spi_write((uint8_t)LSB(address));
    spi_write_bulk(data, size);
    EEPROM_CS_DESELECT();
    NOP();
}

/**
 * Write data in bulk mode. It's faster than byte by byte.
 * Caution needed not to cross 64 byte page boundary
 * @param address - start address
 * @param data - pointer to data
 * @param size - size in  bytes to write
 */
void eeprom_write_const_data_bulk(uint16_t address, uint8_t  data, uint8_t size) {
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    spi_write(CMD_WRSR);
    spi_write(0x02); // Enable Write Latch.
    eeprom_wait_deselect();
    spi_write(CMD_WREN);
    eeprom_wait_deselect();
    spi_write(CMD_WRITE);
    spi_write((uint8_t)MSB(address));
    spi_write((uint8_t)LSB(address));
    spi_write_bulk_const(data, size);
    EEPROM_CS_DESELECT();
}

void eeprom_write_data(uint16_t address, uint8_t data) {
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    spi_write(CMD_WRSR);
    spi_write(0x02); // Enable Write Latch.
    eeprom_wait_deselect();
    spi_write(CMD_WREN);
    eeprom_wait_deselect();
    spi_write(CMD_WRITE);
    spi_write((uint8_t)MSB(address));
    spi_write((uint8_t)LSB(address));
    spi_write(data);
    EEPROM_CS_DESELECT();
    NOP();
}

void eeprom_clear_block_bulk(uint16_t address, uint16_t size) {
    while(size>0){
        uint16_t next_write_size = EEPROM_PAGE_SIZE - EEPROM_RES_SIZE(address);
        next_write_size = MIN(size, next_write_size);
        eeprom_write_const_data_bulk(address, 0x00, next_write_size);
        address += next_write_size;
        size -= next_write_size;
    }
}

void eeprom_write_array_bulk(uint16_t address, uint8_t * data, uint16_t size) {
    while(size>0){
        uint16_t next_write_size = EEPROM_PAGE_SIZE - EEPROM_RES_SIZE(address);
        next_write_size = MIN(size, next_write_size);
        eeprom_write_data_bulk(address, data, next_write_size);
        address += next_write_size;
        size -= next_write_size;
        data += next_write_size;
    }
}

void eeprom_clear_block(uint16_t start_address, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        eeprom_write_data(start_address + i, 0x00);
    }
}

void eeprom_write_array(uint16_t address, uint8_t * data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        eeprom_write_data(address + i, data[i]);
    }
}

void eeprom_write_wdata(uint16_t address, uint16_t data) {
    eeprom_write_data(address, data & 0xFF);
    eeprom_write_data(address + 1, (data >> 8) & 0xFF);
}

void eeprom_write_tdata(uint16_t address, uint24_t data) {
    eeprom_write_data(address, data & 0xFF);
    eeprom_write_data(address + 1, (data >> 8) & 0xFF);
    eeprom_write_data(address + 2, (data >> 16) & 0xFF);
}

uint8_t eeprom_read_data(uint16_t address) {
    uint8_t read_data;
    EEPROM_CS_SELECT();
    spi_write(CMD_READ);
    spi_write(MSB(address));
    spi_write(LSB(address));
    read_data = spi_read();
    EEPROM_CS_DESELECT();
    return (read_data);
}

uint16_t eeprom_read_array(uint16_t address, uint8_t *data, uint16_t no_of_bytes) {
    uint16_t index;

    if (address > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    spi_write(CMD_READ);
    spi_write(MSB(address));
    spi_write(LSB(address));
    for (index = 0; index < no_of_bytes; index++) {
        if (address + index > EEPROM_MAX_SIZE) return (index);
        data[index] = spi_read();
    }
    EEPROM_CS_DESELECT();
    return (index);
}

uint16_t eeprom_read_wdata(uint16_t address) {
    union {
        uint16_t _d;
        struct{
            uint8_t read_least;
            uint8_t read_most;
        };
    } _u;
    if (address + 1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    spi_write(CMD_READ);
    spi_write(MSB(address));
    spi_write(LSB(address));
    _u.read_least = spi_read();
    _u.read_most = spi_read();
    EEPROM_CS_DESELECT();
    return _u._d;
}

uint24_t eeprom_read_tdata(uint16_t address) {
    union{
        uint24_t _d;
        struct{
            uint8_t read_least;
            uint8_t read_mid;
            uint8_t read_most;
        };
    }_u;
    if (address + 1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    spi_write(CMD_READ);
    spi_write(MSB(address));
    spi_write(LSB(address));
    _u.read_least = spi_read();
    _u.read_mid = spi_read();
    _u.read_most = spi_read();
    EEPROM_CS_DESELECT();
    return _u._d;
}



uint8_t eeprom_read_status_reg(void) {
    EEPROM_CS_SELECT();
    spi_write(CMD_RDSR);
    uint8_t status_reg = spi_read();
    EEPROM_CS_DESELECT();
    return status_reg;
}
