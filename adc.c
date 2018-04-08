#include "adc.h"
// <editor-fold defaultstate="collapsed" desc="MicroController ADC Interface">

void ADC_init() {
    TRISA = 0b11111111; // ADC inputs 0..3
    ANSELA = 0b00001111;
    //  ADCON0 = 0b10000000;        // Enable ADC	 - single byte mode	   return ADRESH;
    ADCON0 = 0b10000100; // Enable ADC	 - single 10 bit mode	return (ADRESH<<8)|ADRESL;

    ADCON1 = 0b00000001; // Select ADC Double Sample
    ADCON2 = 0b00001000; // Normal ADC operation
    ADCON3 = 0b00001000; // Normal ADC operation
    ADCLK = 0b00100000; // ADC CLK = OSC/64
    ADREF = 0b00000011; // ADC connected to FVR
    FVRCON = 0b11000010; // FVR set to 2048mV
    ADCON0bits.ADCONT = 0;
    PIE1bits.ADIE=0;
    PIR1bits.ADIF=0;

}

uint16_t ADC_Read(char selectedADC) {
    ADPCH = selectedADC; // Select ADC input
    ADCON0bits.ADGO = 1; // Start conversion
    while (GO_nDONE);
    return (ADRESH << 8) | ADRESL;
}


uint16_t ADC_Read_average(char selectedADC, uint8_t cycles) {
    uint16_t res = 0;
    for (size_t i = 0; i<cycles;i++){
        res = res + ADC_Read(selectedADC);
    }
    return res/cycles;
}

// </editor-fold>

