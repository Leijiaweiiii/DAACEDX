#include "uart.h"
#include "DAACEDcommon.h"

void putch(unsigned char data) {
    while (!PIR3bits.TX1IF); // wait until the transmitter is ready
    TX1REG = data; // send one character
}

void uart_rx_handled() {
    for (int i = 0; i < UART_RX_BUF_SIZE; i++) {
        uart_rx_buffer[i] = 0;
    }
    rx_head = 0;
    RC1IE = 1;
}

void uart_tx_completed() {
    for (int i = 0; i < UART_TX_BUF_SIZE; i++) {
        uart_tx_buffer[i] = 0;
    }
}

void init_uart(void) {
    // Configure C6 = TX and C7 = RX
    RX1PPS = 0x17; // RX - C7
    RC6PPS = 0x0C; // TX - C6
    TX1STAbits.SYNC = 0; // ASYNC mode
    BAUD1CONbits.BRG16 = 1; // 16 bit baud rate counter
    TX1STAbits.BRGH = 0;
    BAUD1CONbits.SCKP = 0; // Data on falling edge
    uart_set_high_speed;
    TX1STAbits.TX9 = 0; // 8 bits TX
    TX1STAbits.TXEN = 1; // TX Enabled
    TX1IF = 0;
    PIE3bits.TX1IE = 1; // Enable interrupt
    RC1STAbits.SPEN = 1;
    RC1STAbits.RX9 = 0;
    RCSTA1bits.CREN = 1;
    RC1IF = 0;
    RC1IE = 1;
    uart_flags.tx_complete = 1;
    uart_rx_handled();
}

#define send_next_byte() { TX1REG = uart_tx_buffer[tx_head]; tx_head++; TX1IE = 1; }

void uart_start_tx_string(const char * str, const uint8_t size) {
    // if no TX enabled just don't send anything and unlock everyone who may be waiting this
    if(TX1STAbits.TXEN == 0){
        uart_flags.tx_complete = 1;
        return;
    }
    // Wait until previous buffer sent
    while (!uart_flags.tx_complete){
        uint8_t const_tx_head = tx_head;
        UNUSED(const_tx_head);
        Delay(1);
        if(tx_head==const_tx_head){
            break;
        }
    }
    strncpy(uart_tx_buffer, str, size);
    if(size<tx_size){
        // old command longer
        for (uint8_t i = size;i<tx_size;i++){
            uart_tx_buffer[i] = 0;
        }
    }
    tx_size = size;
    tx_head = 0;
    uart_flags.tx_complete = 0;
    send_next_byte();
}

void uart_tx_int_handler() {
    if (tx_head < tx_size) {
        send_next_byte();
    } else {
        uart_flags.tx_complete = 1;
        TX1IE = 0;
    }
}

void uart_rx_int_handler() {
    uint8_t status = RC1STA;
    UNUSED(status);
    if (RC1STAbits.OERR) RCSTA1bits.CREN = 0;
    // Handle error flags
    if (rx_head < UART_RX_BUF_SIZE) {
        uart_rx_buffer[rx_head] = RC1REG;
        rx_head++;
    } else {
        RC1IE = 0;
    }
    RCSTA1bits.CREN = 1;
}
