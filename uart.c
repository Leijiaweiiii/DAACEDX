#include "uart.h"
#include "DAACEDcommon.h"

void putch(unsigned char data) {
    while (!PIR3bits.TX1IF) // wait until the transmitter is ready
        continue;
    TX1REG = data; // send one character
}

void init_uart(void) {
    // Configure C6 = TX and C7 = RX
    //    TRISCbits.TRISC6 = 1;
    //    TRISCbits.TRISC5 = 1;
    SP1BRG = 103; // for 9600 rate, calculated for 64 MHz
    BAUD1CONbits.SCKP = 1; // Data on raising edge
    BAUD1CONbits.BRG16 = 0; // 16 bit baud rate counter
    RC6PPS = 0x0C; // Set RC6 pin to be TX1 (see Page 336 of the spec)
    TX1STAbits.SYNC = 0; // ASYNC mode
    TX1STAbits.TX9 = 0; // 8 bits TX
    RC1STAbits.SPEN = 1;
    PIE3bits.TX1IE = 1; // Enable interrupt
    TX1STAbits.TXEN = 1; // TX Enabled
    
    // TODO: Init UART RX
}

void send_next_byte(){
    TX1REG = uart_tx_buff[tx_head];
    tx_head++;
}

void uart_start_tx_string(const char * str, const uint8_t size) {
    // Wait until previous buffer sent
    while (!uart_flags.tx_complete);
    strmycpy(str, uart_tx_buff);
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
    }
}

void uart_rx_int_handler() {
    // Handle error flags
    if(rx_head<UART_RX_BUF_SIZE){
        uart_rx_buffer[rx_head] = RC1REG;
        rx_head++;
    }
}