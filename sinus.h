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
    unsigned unused : 1;
    unsigned PRE    : 3;
    unsigned POS    : 4;
    uint8_t PR;
} sinusSetting_t;

/* PS - Post scaler
 * PR - Comparision value
 * 0 - 100Hz
 * 1 - 200Hz
 * ...
 * N - N*100Hz
 */
const sinusSetting_t sinus_s[33]={
    { 0, 4,10,125},
    { 0, 4,8,78},
    { 0, 4,5,83},
    { 0, 4,6,52},
    { 0, 4,5,50},
    { 0, 4,8,26},
    { 0, 3,3,119},
    { 0, 4,13,12},
    { 0, 1,11,101},
    { 0, 4,5,25},
    { 0, 2,5,91},
    { 0, 2,8,52},
    { 0, 0,9,171},
    { 0, 2,3,119},
    { 0, 2,3,111},
    { 0, 2,3,104},
    { 0, 3,3,49},
    { 0, 3,1,139},
    { 0, 0,9,117},
    { 0, 3,1,125},
    { 0, 3,1,119},
    { 0, 2,1,227},
    { 0, 2,7,31},
    { 0, 3,1,104},
    { 0, 3,1,100},
    { 0, 1,5,77},
    { 0, 2,1,185},
    { 0, 0,3,238},// ->2100
    { 0, 1,5,69}, // ->2400
    { 0, 0,6,111},
    { 0, 1,2,161},
    { 0, 1,4,78},
    { 0, 0,6,101}
};

#define MAX_VOLUME_LEVELS   3
extern const uint8_t sinus_table[MAX_VOLUME_LEVELS][32];


// <editor-fold defaultstate="collapsed" desc="Sinus Generator">
volatile uint8_t current_sample_index = 18; // Start signal from 0 to avoid click
volatile uint8_t amplitude_index;
volatile uint8_t amplitude_index_const;
volatile uint16_t beep_duration_residue = 0;
volatile time_t beep_start;
volatile time_t beep_duration;
#define DACON_VALUE_DEFAULT     0b10100000
#define DACON_VALUE_NEW         0b10100000

void sinus_dac_init(void);
void generate_sinus(uint8_t amplitude, uint16_t frequency, uint16_t duration);
//void start_sinus(uint16_t frequency);
#define stop_sinus() {DAC1CON0 = 0; /* DAC Off */   LATEbits.LATE2 = 0; /* Driver OFF */ LATEbits.LATE1 = 0; /*5v Power Pupply Off*/}
void sinus_duration_expired(void);
void sinus_value_expired(void);
void sinus_duration_timer_init(uint16_t duration);
// f_n - index of the frequency in settings array
void sinus_value_timer_init(uint8_t f_n);
// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* SINUS_H */

