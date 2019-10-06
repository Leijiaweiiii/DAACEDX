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

#define MAX_VOLUME_LEVELS   5
extern const uint8_t sinus_table[MAX_VOLUME_LEVELS][32];


// <editor-fold defaultstate="collapsed" desc="Sinus Generator">
volatile uint8_t current_sample_index = 0;
volatile uint8_t amplitude_index;
volatile uint16_t beep_duration_residue = 0;
volatile time_t beep_start;
volatile time_t beep_duration;
#define DACON_VALUE_DEFAULT     0b10100000
#define DACON_VALUE_NEW         0b10100000

void sinus_dac_init(TBool ref);
void generate_sinus(uint8_t amplitude, uint16_t frequency, int16_t duration);
//void start_sinus(uint16_t frequency);
#define stop_sinus() {DAC1CON0 = 0; /* DAC Off */   LATEbits.LATE2 = 0; /* Driver OFF */ LATEbits.LATE1 = 0; /*5v Power Pupply Off*/}
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

