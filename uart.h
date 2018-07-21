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
#define UART_RX_BUF_SIZE 96
#define UART_TX_BUF_SIZE 128
    char uart_rx_buffer[UART_RX_BUF_SIZE];
    volatile unsigned char rx_head=0;
    char uart_tx_buff[UART_TX_BUF_SIZE];
    volatile unsigned char tx_head=0,tx_size=0;
    struct{
        unsigned rx_handled     :1;
        unsigned tx_complete    :1;
    } uart_flags;
    void putch(unsigned char data);
    void init_uart();
    void uart_tx_int_handler();
    void uart_rx_int_handler();
    void uart_start_tx_string(const char * str, const uint8_t size);
    void uart_rx_handled();


#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

