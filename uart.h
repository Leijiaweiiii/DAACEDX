/* 
 * File:   uart.h
 * Author: navado
 *
 * Created on 23 March 2018, 19:08
 */

#ifndef UART_H
#define	UART_H
#include "stdint.h"
#ifdef	__cplusplus
extern "C" {
#endif
#define UART_RX_BUF_SIZE 48
#define UART_TX_BUF_SIZE 32
    char uart_rx_buffer[UART_RX_BUF_SIZE];
    volatile unsigned char rx_head=0;
    char uart_tx_buffer[UART_TX_BUF_SIZE];
    volatile unsigned char tx_head=0,tx_size=0;
    volatile struct{
        unsigned rx_handled     :1;
        unsigned tx_complete    :1;
        unsigned                :6;
    } uart_flags;
    void putch(unsigned char data);
    void init_uart(void);
    void uart_tx_int_handler(void);
    void uart_rx_int_handler(void);
    void uart_start_tx_string(const char * str, const uint8_t size);
    void uart_rx_handled(void);
    void uart_tx_completed(void);
#define uart_disable() { RC1IE = 0; PIE3bits.TX1IE = 0; TX1STAbits.TXEN = 0; RCSTA1bits.CREN = 0; RC1STAbits.SPEN = 0; }
#define uart_set_high_speed {SP1BRG = 34; /* for 115200 rate, calculated for 64 MHz */}
#define uart_set_low_speed  {SP1BRG = 416; /* for 9600 rate, calculated for 64 MHz */}


#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

