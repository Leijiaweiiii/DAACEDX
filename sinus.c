#include "sinus.h"
#include "rtc.h"


void sinus_duration_expired(){
    if(0 < (--beep_duration_residue)) return;
    stop_sinus();
    // Stop value timer - TMR6
    T6CONbits.ON = 0;
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
void sinus_duration_timer_init(int16_t duration){
    beep_duration_residue = duration;
    // TMR8
    PIE5bits.TMR8IE = 0;
    T8CLKCONbits.CS = 0b0011;   // HFOSC 64MHz
    T8HLTbits.CKPOL = 1;
    T8HLTbits.PSYNC = 1;
    T8HLTbits.CKSYNC = 0;       // don't delay start on ON
    T8HLTbits.MODE = 0b00000;   // Free Running
    T8CONbits.CKPS = 0b110;     // 1/64 pre-scale
    T8CONbits.OUTPS = 0b1001;   // 1/10 post-scale
    T8PR = 100;                 // Count to 100
    PIE5bits.TMR8IE = 1;
    T8CONbits.ON = 1;
}

void sinus_value_timer_init(uint8_t f_n){
    sinusSetting_t setting = sinus_s[f_n - 1];  
    current_sample_index = 0;
    PIE5bits.TMR4IE = 0;            // Disable interrupt
    T4CLKCONbits.CS = 0b0011;       // HFOSC
    T4HLTbits.CKPOL = 0;
    T4HLTbits.PSYNC = 0;
    T4HLTbits.CKSYNC = 0;           // don't delay start on ON
    T4HLTbits.MODE = 0b00000;       // Free running
    T4CONbits.OUTPS = setting.POS - 1;
    T4CONbits.CKPS  = setting.PRE;
    T4PR            = setting.PR;
    PIE5bits.TMR4IE = 1;            // Enable the interrupt
    T4CONbits.ON = 1;
}