#include "rtc.h"

#include "pcf85063a.h"

// <editor-fold defaultstate="collapsed" desc="RTC timer functions">

// Software RTC is implemented using TIMER1 on chip with 32.768 KHz timer.

uint8_t get_time_source() {
    if (OSCSTATbits.EXTOR)
        return 'E';
    if (OSCSTATbits.SOR)
        return 'S';
    return 'I';
}

void initialize_rtc_timer() {
    // Real time counter will count 2 seconds forever.
    RTC_TIMER_IE = 0; // Disable interrupt.
    RTC_TIMER_IF = 0; // Clear Interrupt flag.
    PMD1bits.TMR1MD = 0; // Enable perepherial timer 1
    TMR1CLKbits.CS = 0b0100; // TIMER1 clock source is internal 32KHz
    T1CON = 0b00000111;
    RTC_TIMER_IE = 1; // Enable timer interrupt.
    INTCONbits.PEIE = 1;
}

void init_ms_timer0() {
    PIE0bits.TMR0IE = 0;

    // Time calculations:
    //    T0CON1bits.T0CS = 0b010; // HFINTOSC/4
    //    T0CON1bits.T0PS = 0b0010; // 1:4 prescalar -> 65536 / 64MHz * 4 = 4.0959mS
    //    T0CON1bits.T0ASYNC = 1;
    T0CON1 = 0b01010010;
    //    T0CON0bits.T016BIT = 0; // Enable 16-bit mode.
    //    T0CON0bits.T0OUTPS = 0b0000; // Postscalar 1:1
    //    T0CON0bits.T0OUTPS = 0b0000; // 1:1 postscaler 
    //    T0CON0bits.T0EN = 1; // Start timer.
    T0CON0 = 0b11100000;
    PIE0bits.TMR0IE = 1; // Enable Interrupt
    INTCONbits.PEIE = 1;
}

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Time setting functions">

// </editor-fold>

uint8_t rtc_print_time_full(char * buff, uint8_t _h, uint8_t _m, TBool format24h) {
    if (format24h)
        return sprintf(buff, "%02u:%02u", _h, _m);

    uint8_t h = _h;
    if (h > 12) h -= 12;
    if (h == 0) h = 12;
    return sprintf(buff, "%02u:%02u%c", h, _m, IsHourAM(_h) ? 'a' : 'p');
}

void set_time(uint8_t h, uint8_t m, TBool is24h) {
    if (is24h){
        rtcd.prdtdDateTime.hours._tens = h/10;
        rtcd.prdtdDateTime.hours._units = h%10;
    } else {
        rtcd.prdtdDateTime.hours._tens12 = h/10;
        rtcd.prdtdDateTime.hours._units12 = h%10;
    }
    rtcd.prdtdDateTime.minutes._tens = m/10;
    rtcd.prdtdDateTime.minutes._tens = m%10;
    getRtcControlData();
    rtcd.prcdControl.control1.b1224 = is24h;
    setRtcControlData();
    setRtcDateTimeData();
}

void read_time(void){
    getRtcControlData();
    getRtcDateTimeData();
}
uint8_t minutes(void){
    return rtcd.prdtdDateTime.minutes._tens * 10 + rtcd.prdtdDateTime.minutes._units;
}

uint8_t hours(void){
    return rtcd.prdtdDateTime.hours._tens * 10 + rtcd.prdtdDateTime.hours._units;
}

TBool is1224(void){
    return rtcd.prcdControl.control1.b1224;
}

/**
 * 
 * @param buff - buffer to print time there
 * @param format24h - boolean indicating if it's 24 or 12 hours format
 * @return bytes count written to buffer
 */
uint8_t rtc_print_time(char * buff) {
    return rtc_print_time_full(buff, hours(), minutes(), is1224());
}