/* 
 * File:   uart.h
 * Author: navado
 *
 * Created on 23 March 2018, 19:08
 */

#ifndef UART_H
#define	UART_H
#include <xc.h>
#ifdef	__cplusplus
extern "C" {
#endif
    void putch(unsigned char data);
    void init_uart();



#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

