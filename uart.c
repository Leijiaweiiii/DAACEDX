#include "uart.h"
#include "DAACEDcommon.h"

void putch(unsigned char data) {
    while (!PIR3bits.TX1IF); // wait until the transmitter is ready
    TX1REG = data; // send one character
}

void uart_rx_handled() {

    uart_flags.rx_handled = 1;
    for (int i = 0; i < UART_RX_BUF_SIZE; i++) {
        uart_rx_buffer[i] = 0;
    }
    rx_head = 0;
    RC1IE = 1;
}

void init_uart(void) {
    // Configure C6 = TX and C7 = RX
    RX1PPS = 0x17; // RX - C7
    RC6PPS = 0x0C; // TX - C6
    TX1STAbits.SYNC = 0; // ASYNC mode
    BAUD1CONbits.BRG16 = 1; // 16 bit baud rate counter
    TX1STAbits.BRGH = 0;
    BAUD1CONbits.SCKP = 0; // Data on falling edge
    uart_set_low_speed;
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
    // TODO: Init UART RX
}

void send_next_byte() {
    TX1REG = uart_tx_buff[tx_head];
    tx_head++;
    TX1IE = 1;
}

void uart_start_tx_string(const char * str, const uint8_t size) {
    // Wait until previous buffer sent
    while (!uart_flags.tx_complete);
    strmycpy(uart_tx_buff, str);
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
        uart_flags.rx_handled = 0;
        uart_rx_buffer[rx_head] = RC1REG;
        rx_head++;
    } else {
        RC1IE = 0;
    }
    RCSTA1bits.CREN = 1;
}
