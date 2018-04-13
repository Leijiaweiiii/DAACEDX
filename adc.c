#include "adc.h"
// <editor-fold defaultstate="collapsed" desc="MicroController ADC Interface">

void ADC_init() {
    TRISA = 0b11111111; // ADC inputs 0..3
    ANSELA = 0b00001111;
    //  ADCON0 = 0b10000000;        // Enable ADC	 - single byte mode	   return ADRESH;
//    ADCON0 = 0b10000100; // Enable ADC	 - single 10 bit mode	return (ADRESH<<8)|ADRESL;

    ADCON1 = 0b00000001; // Select ADC Double Sample
    ADCON2 = 0b00001000; // Normal ADC operation
    ADCON3 = 0b00001000; // Normal ADC operation
    ADCLK = 0b00100000; // ADC CLK = OSC/64
//    ADCLK = 0b111111;   // 2uS per sample
    ADREFbits.ADNREF = 0;   // VSS
    ADREFbits.ADPREF = 0;   // VDD
    ADREF = 0b00000011; // ADC connected to FVR
    FVRCONbits.EN = 1;
    while (!FVRCONbits.RDY);
    FVRCONbits.ADFVR=0b10;
    ADCON0bits.ADCONT = 0;
    ADCON0bits.ADFM = 1;
    ADCON0bits.ADON = 1;
    PIE1bits.ADIE=0;
    PIR1bits.ADIF=0;

}

uint16_t ADC_Read(char selectedADC) {
    ADPCH = selectedADC; // Select ADC input
    ADCON0bits.ADGO = 1; // Start conversion
    while (GO_nDONE);
    return (ADRESH << 8) | ADRESL;
}


uint16_t ADC_Read_average(char selectedADC) {
    uint32_t res = 0;
    for (size_t i = 0; i<64;i++){
        res = res + ADC_Read(selectedADC);
    }
    return res>>5;
}

// </editor-fold>
int comp (const void * elem1, const void * elem2) 
{
    if (*((uint16_t*)elem1) > *((uint16_t*)elem2)) return  1;
    if (*((uint16_t*)elem1) < *((uint16_t*)elem2)) return -1;
    return 0;
}

uint16_t median(){
    uint16_t res[ADC_BUFFER_SIZE];
    for(uint8_t i = 0;i<ADC_BUFFER_SIZE;i++){
        res[i]=samples[i];
    }
    qsort(res,ADC_BUFFER_SIZE,sizeof(uint16_t),comp);
    median_v = res[ADC_MID_BUFFER];
    return median_v;
}

TBool AdcDetect(){
    return ADC_MIDDLE_VALUE - cma_n>ADC_DETECTION_THRESHOLD;
}

uint16_t MeanValue(){
    uint16_t avg = 0;
    for(uint16_t i=0;i<ADC_BUFFER_SIZE;i++)
        avg += samples[i];
    
    return avg>>4;
}
