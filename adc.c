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

}

uint16_t ADC_Read(char selectedADC) {
    ADPCH = selectedADC; // Select ADC input
    ADCON0bits.ADGO = 1; // Start conversion
    while (GO_nDONE);
    return (ADRESH << 8) | ADRESL;
}

void init_adc_interrupt() {

    /*
     ?
 EN and POL bits
 CxIE bit of the PIE2 register
 INTP bit (for a rising edge detection)
 INTN bit (for a falling edge detection)
 PEIE and GIE bits of the INTCON register
     */
    //    CM1CON0bits.EN = 1;         //Enable comparator
    //    CM1CON0bits.POL = 0;        // Do't invert polarity - OUT=1 iff VC1P > VC1N
    //    CM1CON0bits.SYNC = 1;       // Output synchronized to Timer1 clock (10ms timer in our case)
    //    CM1CON1bits.INTP = 1;       // Interrupt on positive edge
    //    CM1CON1bits.INTN = 0;       // Disable negative edge interrupt
    //    CM1PCHbits.PCH = 0b001;     // Positive edge - input pin 22 - ANA2
    //    CM1NCHbits.NCH = 0b110;     // Voltage reference - FVR Buffer
    //    FVRCONbits.FVREN = 1;       // Enable FVR
    //    FVRCONbits.CDAFVR = 0b01;   // Reference is 1.024V
    ADC_INT_ENABLE;
}

// </editor-fold>

