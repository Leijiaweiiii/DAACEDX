#include "rtc.h"

#include "pcf85063a.h"

// <editor-fold defaultstate="collapsed" desc="RTC timer functions">

// Software RTC is implemented using TIMER1 on chip with 32.768 KHz timer.

time_t time_ms(void){
    union{
        time_t _t;
        struct {
            uint16_t _l;
            uint16_t _h;
        };
    } res;
    res._l = TMR1;
    res._h = TMR3;
    return res._t;
}

uint8_t get_time_source(void) {
    if (OSCSTATbits.EXTOR)
        return 'E';
    if (OSCSTATbits.SOR)
        return 'S';
    return 'I';
}

void initialize_rtc_timer(void) {
    /*
     * TMR0 - 1ms. Interrupt, input=SOSCI
     * TMR1 | (TMR3<<16) - unix_time_ms, TMR1 input - TMR6OF, TMR3 input - TMR1
     * TMR5 - Filter timer, input TMR6OF
     */
    TMR0H = 125;            // preset to get 1ms period
    T0CON1 = 0b01111001;    // HFINTOSC, async, 512 prescaler
    T0CON0 = 0b10000000;    // Enable timer 0 without postscaler and 8 bit mode
    TMR0IE = 1;             // Enable Timer0 interrupt
    
    TMR1CLK = 0b11111000;   // TMR1 Clock is TMR0 overflow
    T1GCON = 0b10000000;    // TMR1 always counting
    T1CON = 0b11001011;     // Enable timer 1 with 16 bit read and no prescaler
    
    TMR3CLK = 0b11111001;   // Input is TMR1
    T3GCON = 0b11000000;    // TMR3 always counting
    T3CON = 0b11001111;     // Enable timer 3 with 16 bit read and no prescaler
}



// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Time setting functions">

// </editor-fold>

void set_time(uint8_t h, uint8_t m, TBool is24h) {
    if (is24h){
        prdtdDateTime.hours._tens = h/10;
        prdtdDateTime.hours._units = h%10;
    } else {
        prdtdDateTime.hours._tens12 = h/10;
        prdtdDateTime.hours._units12 = h%10;
    }
    prdtdDateTime.minutes._tens = m/10;
    prdtdDateTime.minutes._units = m%10;
    getRtcControlData();
    prcdControl.control1.b1224 = is24h;
    setRtcControlData();
    setRtcDateTimeData();
}

void read_time(void){
    getRtcControlData();
    getRtcDateTimeData();
}

uint8_t minutes(void){
    return prdtdDateTime.minutes._tens * 10 + prdtdDateTime.minutes._units;
}

uint8_t hours(void){
    return prdtdDateTime.hours._tens * 10 + prdtdDateTime.hours._units;
}

TBool is1224(void){
    return prcdControl.control1.b1224;
}

/**
 * 
 * @param buff - buffer to print time there
 * @param format24h - boolean indicating if it's 24 or 12 hours format
 * @return bytes count written to buffer
 */
uint8_t rtc_print_time(char * b) {
    uint8_t res = 0;
     if (prcdControl.control1.b1224){
        res = sprintf(b,"%d%d:%d%d",
            prdtdDateTime.hours._tens,
            prdtdDateTime.hours._units,
            prdtdDateTime.minutes._tens,
            prdtdDateTime.minutes._units
            );
    } else {
        res = sprintf(b,"%d%d:%d%d%c",
            prdtdDateTime.hours._tens12,
            prdtdDateTime.hours._units12,
            prdtdDateTime.minutes._tens,
            prdtdDateTime.minutes._units,
                (prdtdDateTime.hours.bAMPM)?'a':'p'
            );
    }
    return res;
}