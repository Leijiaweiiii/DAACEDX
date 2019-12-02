#include "sinus.h"
#include "rtc.h"

const uint8_t sinus_table[MAX_VOLUME_LEVELS][32] = {
    {0x8, 0x8, 0x8, 0x7, 0x7, 0x6, 0x6, 0x5, 0x4, 0x3, 0x2, 0x2, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x2, 0x2, 0x3, 0x4, 0x5, 0x6, 0x6, 0x7, 0x7, 0x8, 0x8},
    {0x10, 0x10, 0xF, 0xF, 0xE, 0xC, 0xB, 0xA, 0x8, 0x6, 0x5, 0x4, 0x2, 0x1, 0x1, 0x0, 0x0, 0x0, 0x1, 0x1, 0x2, 0x4, 0x5, 0x6, 0x8, 0xA, 0xB, 0xC, 0xE, 0xF, 0xF, 0x10},
    {0x1F, 0x1F, 0x1E, 0x1C, 0x1A, 0x18, 0x15, 0x13, 0x10, 0xC, 0xA, 0x7, 0x5, 0x3, 0x1, 0x0, 0x0, 0x0, 0x1, 0x3, 0x5, 0x7, 0xA, 0xC, 0x10, 0x13, 0x15, 0x18, 0x1A, 0x1C, 0x1E, 0x1F},
//    {30, 27, 24, 18, 13, 8, 5, 2, 1, 0, 2, 5, 7, 10, 11, 12, 11, 10, 7, 5, 2, 0, 1, 2, 5, 8, 13, 18, 24, 27, 30, 31 },
//    {30, 28, 24, 19, 14, 9, 6, 2, 1, 0, 1, 2, 4, 6, 7, 7, 7, 6, 4, 2, 1, 0, 1, 2, 6, 9, 14, 19, 24, 28, 30, 31}
};

void sinus_dac_init(TBool ref) {
    LATEbits.LATE1 = 1;
    TRISFbits.TRISF5 = 0;
    FVRCONbits.EN = True;
    FVRCONbits.CDAFVR = 0b10;
    ANSELFbits.ANSELF5 = 0;
    if(ref){
        DAC1CON0 = DACON_VALUE_NEW;
    } else {
        DAC1CON0 = DACON_VALUE_DEFAULT;
    }
    LATEbits.LATE2 = 1;
}

void sinus_duration_expired(){
    if(0 < (--beep_duration_residue)) return;
    stop_sinus();
    // Stop value timer - TMR4
    T4CONbits.ON = 0;
    PIE5bits.TMR4IE = 0;
    // Stop duration timer - TMR8
    T8CONbits.ON = 0;
    PIE5bits.TMR8IE = 0;
}

void sinus_value_expired(){
    uint8_t dac_value = sinus_table[amplitude_index][current_sample_index];
    DAC1CON1 = dac_value;
    current_sample_index++;
    current_sample_index &= 0x1F; // %32
}

/*
 * Duration in ms.
 * Timer configured to count milliseconds assuming decrement of remining 
 */
void sinus_duration_timer_init(uint16_t duration){
    beep_duration_residue = duration + 1;
    // TMR8
    PIE5bits.TMR8IE = 0;
    T8CLKCONbits.CS = 0b0011;   // HFOSC 64MHz
//    T8HLTbits.CKPOL = 1;
//    T8HLTbits.PSYNC = 1;
//    T8HLTbits.CKSYNC = 0;       // don't delay start on ON
//    T8HLTbits.MODE = 0b00000;   // Free Running
    T8HLT           = 0b11000000;
    T8PR = 100;                 // Count to 100
    IPR5bits.TMR8IP = 1;        // Timer8 high priority interrupt (sound is significant)
//    T8CONbits.CKPS = 0b110;     // 1/64 pre-scale
//    T8CONbits.OUTPS = 0b1001;   // 1/10 post-scale
//    T8CONbits.ON = 1;
    T8CON         = 0b11101001;
    PIE5bits.TMR8IE = 1;
}

void sinus_value_timer_init(uint8_t f_n){
    sinusSetting_t setting = sinus_s[f_n - 1];  
    current_sample_index = 0;
    PIE5bits.TMR4IE = 0;            // Disable interrupt
    T4CLKCONbits.CS = 0b0011;       // HFOSC
//    T4HLTbits.CKPOL = 0;
//    T4HLTbits.PSYNC = 0;
//    T4HLTbits.CKSYNC = 0;           // don't delay start on ON
//    T4HLTbits.MODE = 0b00000;       // Free running
    T4HLT           = 0b00000000;
    T4CONbits.OUTPS = setting.POS - 1;
    T4CONbits.CKPS  = setting.PRE;
    T4PR            = setting.PR;
    IPR5bits.TMR4IP = 1;            // High priority interrupt (sound is significant)
    PIE5bits.TMR4IE = 1;            // Enable the interrupt
    T4CONbits.ON = 1;
}
