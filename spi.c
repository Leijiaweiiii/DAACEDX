#include "spi.h"

#define UNUSED(x)               (void)(x)

void spi_init() {
    /*
     * FUNCTION| PIN | PORT  | BIT
     * LCD_CS  | 15  | PORTF |  3
     * LCD_A0  | 14  | PORTF |  4
     * LCD_RST | 12  | PORTF |  6
     * SPI_SCL | 34  | PORTC |  3
     * SPI_OUT | 35  | PORTC |  4
     * SPI_IN  | 51  | PORTD |  5   OLD
     * SPI_IN  | 11  | PORTF |  7   NEW
     * EE_CS   | 18  | PORTF |  0
     * EE_WR   | 17  | PORTF |  1
     * EE_HOLD | 16  | PORTF |  2
     */
    
    
    RC3PPS = 0x19;      // SPI clock output         PPS: 011 001    PORTD 1
    RC4PPS = 0x1A;      // data-output lcd and EEPROM  PPS: 011 010    PORTD 2
//    SSP1DATPPS = 0x1D;  // data-input for EEPROM    PPS: 011 101    PORTD 5
    SSP1DATPPS = 0x2F;  // data-input for EEPROM    PPS: 101 111    PORTF 7
    SSP1CLKPPS = 0x19;
    TRISDbits.TRISD5 = 1; // Set EEPROM input to input
    
//    SSP1STAT &= 0x3F; // Power on state
    
    PIE3bits.SSP1IE = 0; // Disable interrupt.
    SSP1STATbits.CKE = 0; // Data transmission on rising edge
    SSP1STATbits.SMP = 1; // Data sampled/latched at end of clock.
//    SSP1CON1 = 0x21; // Enable synchronous serial port , CKL ,FOSC_DIV_16 page 394
    SSP1CON1bits.SSPM = 0x01;               // SPI Clock FOSC_DIV_16
    SSP1CON1bits.CKP = 1;
    SSP1CON1bits.WCOL = 0;
    SSP1CON1bits.SSPEN = 1;                 // Enable synchronous serial port
    
}

uint8_t spi_write(uint8_t data) {
    unsigned char temp_var = SSP1BUF; // Clear buffer.
    (void)(temp_var);
    PIR3bits.SSP1IF = 0;              // clear interrupt flag bit
    SSP1CON1bits.WCOL = 0;            // clear write collision bit if any collision occurs
    SSP1BUF = data;                   // transmit data
    while (!PIR3bits.SSP1IF);         // waiting for the process to complete
    PIR3bits.SSP1IF = 0;              // clear interrupt flag bit
    while(!SSP1STATbits.BF);
    uint8_t ret = SSP1BUF;
    return ret;                       // return receive data
}

uint8_t spi_read(void) {
    unsigned char temp_var = SSP1BUF; // Clear buffer.
    (void)(temp_var);
    PIR3bits.SSP1IF = 0;              // clear interrupt flag bit
    SSP1CON1bits.WCOL = 0;            // clear write collision bit if any collision occurs
    SSP1BUF = 0xFF;                   // transmit data
    while(!SSP1STATbits.BF);
    uint8_t ret = SSP1BUF;
    return ret;                       // return receive data
}

uint8_t spi_write_bulk(uint8_t * data, uint8_t size) {
    uint8_t temp_var = SSP1BUF; // Clear buffer.
    UNUSED(temp_var);           // Suppress warning
    while( 0 < size-- ){
        PIR3bits.SSP1IF = 0;        // Clear interrupt flag bit
        SSP1CON1bits.WCOL = 0;      // Clear write collision bit if any collision occurs
        SSP1BUF = *(data++);
        while (SSP1STATbits.BF == 0);
    }
    PIR3bits.SSP1IF = 0; // clear interrupt flag bit
    return SSP1BUF;
}

uint8_t spi_write_bulk_const(uint8_t data, uint8_t size) {
    uint8_t temp_var = SSP1BUF; // Clear buffer.
    UNUSED(temp_var);           // Suppress warning
    while( 0 < size-- ){
        PIR3bits.SSP1IF = 0;        // Clear interrupt flag bit
        SSP1CON1bits.WCOL = 0;      // Clear write collision bit if any collision occurs
        SSP1BUF = data;
        while (SSP1STATbits.BF == 0);
    }
    PIR3bits.SSP1IF = 0; // clear interrupt flag bit
    return SSP1BUF;
}