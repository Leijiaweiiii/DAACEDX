#include "adc.h"
// <editor-fold defaultstate="collapsed" desc="MicroController ADC Interface">

void ADC_init(void) {
    TRISA = 0b11111111; // ADC inputs 0..3
    ANSELA = 0b00001110;
    ADCON1 = 0b00000001; // Select ADC Double Sample
    ADCON2 = 0b00001000; // Normal ADC operation
    ADCON3 = 0b00001000; // Normal ADC operation
    ADCLK = 0b00111111; // ADC CLK = OSC/64
//    ADREFbits.ADNREF = 0;   // VSS
//    ADREFbits.ADPREF = 0;   // VDD
    ADREF = 0b00000011; // ADC connected to FVR
    PMD0bits.FVRMD = 0; // Turn ON FRV perepherial
    FVRCONbits.EN = 1;
//    while (!FVRCONbits.RDY);
    FVRCONbits.ADFVR=0b10;
//    ADCON0bits.ADCONT = 0;
//    ADCON0bits.ADFM = 1;
//    ADCON0bits.ADON = 1;
    ADCON0 = 0b10101110;
    PIE1bits.ADIE=0;
    PIR1bits.ADIF=0;

}

void ADC_HW_detect_init(uint16_t dc, uint16_t lth, uint16_t uth){
    ADLTH               = lth;
    ADUTH               = uth;
    ADSTPT              = dc;          // Setpoint set to DC level
//    ADRPT               = 64;
    ADCON0bits.ADCONT   = 1;           // Continue conversion continously
    ADCON0bits.ADCS     = 0;           // Conversion clock derived from oscillator
    ADCON1bits.ADDSEN   = 0;           // Calculate ADERR every second conversion
    ADCON2bits.ADMD     = 0b000;       // Basic mode
//    ADCON2bits.ADMD     = 0b010;       // Averaging mode
    ADCON3 = 0b00011010;                // Stop on interrupt set to avoid false shots when long handlong the interrupt
    ADC_HW_detect_shot_start_init();
//    ADCON3bits.ADTMD    = 0b010;       // Interrupt if ADERR > ADLTH
//    ADCON3bits.ADCALC   = 0b001;       // Comparison with setpoint
//    ADCON3bits.ADSOI    = 0;           // Don't stop on interrupt
    ADCLKbits.ADCS      = 0b111111;    // 2uS Conversion period
    IPR1bits.ADTIP      = 0;           // High priority interrupt
    PIE1bits.ADIE       = 0;           // Disable ADC conversion interrupt
    PIR1bits.ADIF       = 0;
}

void ADC_HW_filter_timer_start(uint8_t filter){
    if(filter > MAX_FILTER) filter = MAX_FILTER;
    if(filter == 0) filter = 1;
    PIE1bits.ADTIE = 0; // Disable detection interrupt
    PIR5bits.TMR8IF = 0;
    // Configure TMR6 to count mS
    T6CLKCONbits.CS = 0b0110;     // 32768Hz extosc
    T6PR            = filter_pr_setting[filter - 1];
    T6TMR           = 1;        // Init timer
    T6HLTbits.MODE = 0b01000;   // One shot software start
    T6HLTbits.CKSYNC = 0;       // Don't Sync with system clock

    // Configure interrupt on timer overflow
    
//    T6CONbits.ON = 1;
//    T6CONbits.CKPS  = 0b100;    // Prescale 1:16 i.e counting in ~0.5mS intervals
//    T6CONbits.OUTPS = 0b0000;   // Postscale 1:1
    T6CON = 0b11000000;
    TMR6IE = 1;
    TMR6IF = 0;
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

uint16_t MeanValue(){
    uint16_t avg = 0;
    for(uint16_t i=0;i<ADC_BUFFER_SIZE;i++)
        avg += samples[i];
    
    return avg>>4;
}
