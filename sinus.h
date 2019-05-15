/* 
 * File:   sinus.h
 * Author: Boris Sorochkin
 * Nestlogic Inc.
 * Created on 24 March 2019, 20:18
 */

#ifndef SINUS_H
#define	SINUS_H
#include "DAACEDcommon.h"
#ifdef	__cplusplus
extern "C" {
#endif

    typedef struct{
        uint8_t PRE;
        uint8_t POS;
        uint8_t PR;
    } sinusSetting_t;

    /* PS - Post scaler
     * PR - Comparision value
     * 0 - 100Hz
     * 1 - 200Hz
     * ...
     * N - N*100Hz
     */
sinusSetting_t sinus_s[]={
{4,10,125},
{4,8,78},
{4,5,83},
{4,6,52},
{4,5,50},
{4,8,26},
{3,3,119},
{4,13,12},
{1,11,101},
{4,5,25},
{2,5,91},
{2,8,52},
{0,9,171},
{2,3,119},
{2,3,111},
{2,3,104},
{3,3,49},
{3,1,139},
{0,9,117},
{3,1,125},
{3,1,119},
{2,1,227},
{2,7,31},
{3,1,104},
{3,1,100},
{1,5,77},
{2,1,185}, 
{0,3,238},// ->2100 
{1,5,69}, // ->2400
{0,6,111},
{1,2,161},
{1,4,78},
{0,6,101}
};

const uint8_t sinus_table[3][32] = {
    {0x8, 0x8, 0x8, 0x7, 0x7, 0x6, 0x6, 0x5, 0x4, 0x3, 0x2, 0x2, 0x1, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x1, 0x2, 0x2, 0x3, 0x4, 0x5, 0x6, 0x6, 0x7, 0x7, 0x8, 0x8},
    {0x10, 0x10, 0xF, 0xF, 0xE, 0xC, 0xB, 0xA, 0x8, 0x6, 0x5, 0x4, 0x2, 0x1, 0x1, 0x0, 0x0, 0x0, 0x1, 0x1, 0x2, 0x4, 0x5, 0x6, 0x8, 0xA, 0xB, 0xC, 0xE, 0xF, 0xF, 0x10},
    {0x1F, 0x1F, 0x1E, 0x1C, 0x1A, 0x18, 0x15, 0x13, 0x10, 0xC, 0xA, 0x7, 0x5, 0x3, 0x1, 0x0, 0x0, 0x0, 0x1, 0x3, 0x5, 0x7, 0xA, 0xC, 0x10, 0x13, 0x15, 0x18, 0x1A, 0x1C, 0x1E, 0x1F}
};


// <editor-fold defaultstate="collapsed" desc="Sinus Generator">
volatile uint8_t current_sample_index = 0;
volatile uint8_t amplitude_index;
volatile uint16_t beep_duration_residue = 0;
volatile time_t beep_start;
volatile time_t beep_duration;


#define sinus_dac_init() { LATEbits.LATE1 = 1; TRISFbits.TRISF5 = 0; ANSELFbits.ANSELF5 = 0; DAC1CON0 = 0b10100000; LATEbits.LATE2 = 1;}
void generate_sinus(uint8_t amplitude, uint16_t frequency, int16_t duration);
//void start_sinus(uint16_t frequency);
#define stop_sinus() {DAC1CON0bits.EN = 0; /* DAC Off */   LATEbits.LATE2 = 0; /* Driver OFF */ LATEbits.LATE1 = 0; /*5v Power Pupply Off*/}
void sinus_duration_expired();
void sinus_value_expired();
void sinus_duration_timer_init(int16_t duration);
// f_n - index of the frequency in settings array
void sinus_value_timer_init(uint8_t f_n);
// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* SINUS_H */

