
#ifndef DAACED_COMMON_H
#define	DAACED_COMMON_H

#include <xc.h> // include processor files - each processor file is guarded.  

#include <stdint.h>
#include "DAACEDfont.h"
#include "__size_t.h"
#include "__null.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <time.h>

#define FW_VERSION (45)
// <editor-fold defaultstate="collapsed" desc="General">
#ifndef _XTAL_FREQ
#define _XTAL_FREQ            (64000000UL) //64MHz
#endif

#define MSB(x)              ((x & 0xFF00)>>8)
#define LSB(x)              (x & 0x00FF)

#define	True 1
#define	False 0
#define	On 1
#define	Off 0
#define	Pos 1
#define	Neg 0

#include "tbool.h"

#define HEX2DEC(x)            (x > '9') ? (x - 'a')+10 : x-'0'
#define DEC(x)                (x-'0')
#define Delay(t)           {for (int w=0 ; w<t ; w++)  __delay_ms(1);}
#define __delay_ms_in_lp(x) _delay((unsigned long)((x)*(1000000UL/4000.0)))
#define DelayLP(t)           {for (int w=0 ; w<t ; w++)  __delay_ms_in_lp(1);}

#define IO_AS_OUTPUT            (0)
#define IO_AS_INPUT             (1)
#define BIT(bit_position)       (1<<bit_position)
#define SWAP(x,y)               { x = x + y; y = x - y; x = x - y;}
#define UNUSED(x)               (void)(x)
#define ABS(x)                  ((x<0)?-x:x)
#define MIN(x,y)                ((x<y)?x:y)
#define MAX(x,y)                ((x<y)?y:x)

#define LCD_CS_SELECT()         (LATFbits.LF3 = 0)
#define LCD_CS_DESELECT()       (LATFbits.LF3 = 1)
// </editor-fold> 

extern void	qsort(void *, size_t, size_t, int (*)(const void *, const void *));
extern void generate_sinus(uint8_t amplitude, uint16_t frequency, int16_t duration);
#define Beep() {generate_sinus(1, 1000, 50);}
#endif	/* DAACED_COMMON_H */

