#include "eeprom.h"

// <editor-fold defaultstate="collapsed" desc="EEPROM Functions">
// Helper functions.

void eeprom_spi_init() {
    EEPROM_CS_INIT();
    EEPROM_HOLD_INIT();
    EEPROM_WP_INIT();

    EEPROM_CS_DESELECT();
    EEPROM_HOLD_DIS();
    EEPROM_WP_DIS();

    RD7PPS = 0x1C; // SPI2 dataout (RD7)
    SSP2DATPPS = 0x1D; // SPI2 datain.
    RD6PPS = 0x1B; // SPI2 clock.

    SSP2STAT &= 0x3F;
    SSP2CON1 = 0x00; // power on state.
//    SSP2CON1bits.SSPM = 0b0010;
    SSP2CON1bits.SSPM = 0b0001;
    SSP2STATbits.SMP = 0;

    SSP2CON1bits.CKP = 1;
    SSP2STATbits.CKE = 0;
    SSP2CON1bits.SSPEN = 1;
    SSP2CON1bits.WCOL = 0;
}

uint8_t eeprom_spi_write(uint8_t data) {
    uint8_t temp_var = SSP2BUF; // Clear buffer.
    UNUSED(temp_var);
    PIR3bits.SSP2IF = 0; // Clear interrupt flag bit
    SSP2CON1bits.WCOL = 0; // Clear write collision bit if any collision occurs
    SSP2BUF = data;
    while (SSP2STATbits.BF == 0);
    PIR3bits.SSP2IF = 0; // clear interrupt flag bit
    return SSP2BUF;
}

void eeprom_init() {
    eeprom_spi_init();
}

void eeprom_write_data(uint16_t address, uint8_t data) {
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_WRSR);
    eeprom_spi_write(0x02); // Enable Write Latch.
    EEPROM_CS_DESELECT();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_WREN);
    EEPROM_CS_DESELECT();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_WRITE);
    eeprom_spi_write((uint8_t)MSB(address));
    eeprom_spi_write((uint8_t)LSB(address));
    eeprom_spi_write(data);
    EEPROM_CS_DESELECT();
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
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(MSB(address));
    eeprom_spi_write(LSB(address));
    read_data = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_data);
}

uint16_t eeprom_read_array(uint16_t address, uint8_t *data, uint16_t no_of_bytes) {
    uint16_t index;

    if (address > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(MSB(address));
    eeprom_spi_write(LSB(address));
    for (index = 0; index < no_of_bytes; index++) {
        if (address + index > EEPROM_MAX_SIZE) return (index);
        data[index] = eeprom_spi_write(0x00);
    }
    EEPROM_CS_DESELECT();
    return (index);
}

uint16_t eeprom_read_wdata(uint16_t address) {
    uint8_t read_least, read_most;
    if (address + 1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(MSB(address));
    eeprom_spi_write(LSB(address));
    read_least = eeprom_spi_write(0x00);
    read_most = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_most << 8) | read_least;
}

uint24_t eeprom_read_tdata(uint16_t address) {
    uint8_t read_least, read_mid, read_most;
    if (address + 1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(MSB(address));
    eeprom_spi_write(LSB(address));
    read_least = eeprom_spi_write(0x00);
    read_mid = eeprom_spi_write(0x00);
    read_most = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_most << 16) | (read_mid << 8) | read_least;
}

void eeprom_busy_wait() {
    while (eeprom_read_status_reg() & 0x01);
}

uint8_t eeprom_read_status_reg() {
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_RDSR);
    uint8_t status_reg = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return status_reg;
}
// </editor-fold>
