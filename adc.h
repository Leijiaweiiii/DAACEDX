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

void ADC_init();
uint16_t ADC_Read(char selectedADC);
uint16_t ADC_Read_average(char selectedADC);

#define ADC_BUFFER_SIZE             7
#define ADC_MID_BUFFER              4

uint16_t samples[ADC_BUFFER_SIZE];
uint8_t head_index = 0;
#define ADC_MIFDDLE_INDEX           ((head_index+ADC_MID_BUFFER)%ADC_BUFFER_SIZE)
#define ADC_INC_HEAD_INDEX          {head_index = (head_index + 1)%ADC_BUFFER_SIZE;}
#define ADC_BUFFER_PUT(x)           {samples[head_index] = x;ADC_INC_HEAD_INDEX;}
#define ADC_MIDDLE_VALUE            (samples[ADC_MIFDDLE_INDEX])
#define ADC_LATEST_VALUE            (samples[head_index])
#define ADC_DETECTION_THRESHOLD     200
#define ADC_SAMPLE_REG_16_BIT       (ADRESL|(ADRESH << 8))

TBool AdcDetect();
uint16_t median_v = 0;
uint16_t median();
volatile uint16_t cma_n = 0;
volatile uint16_t adc_battery = 0;
uint16_t MeanValue();
#define ADC_CMA_MEMORY_FACTOR       16
#define ADC_SET_CMA_NEXT(x)         {cma_n = cma_n+(x-cma_n)/ADC_CMA_MEMORY_FACTOR;}
#define ADC_SAMPLE                  {ADC_BUFFER_PUT(ADC_Read(ENVELOPE));ADC_SET_CMA_NEXT(median());}
#define ADC_ENABLE_INTERRUPT_ENVELOPE              {ADPCH = ENVELOPE;ADCON0bits.ADGO = 1;PIE1bits.ADIE=1;}
#define ADC_ENABLE_INTERRUPT_BATTERY               {ADPCH = BATTERY;ADCON0bits.ADGO = 1;PIE1bits.ADIE=1;}
#define ADC_ENABLE_INTERRUPT_ACCELEROMETR          {ADPCH = ACCELEROMETER;ADCON0bits.ADGO = 1;PIE1bits.ADIE=1;}
#define ADC_DISABLE_INTERRUPT       {PIE1bits.ADIE=0;}

#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

