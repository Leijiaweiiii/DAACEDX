/* 
 * File:   adc.h
 * Author: navado
 *
 * Created on 25 March 2018, 14:10
 */

#ifndef ADC_H
#define	ADC_H

#include "DAACEDcommon.h"

#ifdef	__cplusplus
extern "C" {
#endif
// <editor-fold defaultstate="collapsed" desc="uC-ADC Definitions">
#define	ACCELEROMETER  0b000000          // AN0
#define	MICROPHONE     0b000001          // AN1
#define	ENVELOPE       0b000010          // AN2
#define	BATTERY        0b000011          // AN3
#define BAT_divider    4   // resistors relationship, will be changed to 4.96 (12.4K/10.0K)*4 to allow measure of 4.5V
TBool BatteryLow = False;
// </editor-fold>

#define ADC_INT_ENABLE     {ADCON0bits.ADCONT = 1;PIR1bits.ADIF=0;INTCONbits.PEIE=1;PIE1bits.ADIE=1;}
#define ADC_INT_DISABLE     {ADCON0bits.ADCONT = 0;}

void init_adc_interrupt();
void ADC_init();
uint16_t ADC_Read(char selectedADC);
#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

