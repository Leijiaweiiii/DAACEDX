#include "rtc.h"

// <editor-fold defaultstate="collapsed" desc="RTC timer functions">


// Software RTC is implemented using TIMER1 on chip with 32.768 KHz timer.

void initialize_rtc_timer() {
    // Real time counter will count 2 seconds forever.
    // Timer1 for sync
    RTC_TIMER_IE = 0; // Disable interrupt.
    RTC_TIMER_IF = 0; // Clear Interrupt flag.
    OSCENbits.SOSCEN = 1; // Secondary Oscillator Manual Request Enable bit.
    while (!OSCSTATbits.SOR); // Wait for Secondary Oscillator to be ready to use.
    TMR1CLKbits.CS = 0b0110; // TIMER1 clock source = 32.768KHz Secondary Oscillator.
    T1CONbits.CKPS = 0b00; // Prescale = 1:1.
    T1CONbits.NOT_SYNC = 1; // asynchronous counter mode to operate during sleep
    T1CONbits.RD16 = 1;

    // Configure RTC counter which will be counting seconds and should be used for time setting
    PMD1bits.TMR3MD = 0; // Enable perepherial timer 3
    TMR3CLKbits.CS = 0b1001; // TIMER3 clock source is TIMER1
    T3CONbits.CKPS = 0b00; // don't prescale - we need seconds
    T3CONbits.NOT_SYNC = 1; // Async for operation during sleep
    T3CONbits.RD16 = 1;
    T3GCONbits.GE = 0;

    PMD1bits.TMR5MD = 0; // Enable perepherial timer 5
    TMR5CLKbits.CS = 0b1010; // TIMER5 clock source is TIMER3
    T5CONbits.CKPS = 0b00; // don't prescale - we need seconds
    T5CONbits.NOT_SYNC = 1; // Async for operation during sleep
    T5CONbits.RD16 = 1;
    T5GCONbits.GE = 0;

    T1CONbits.ON = 1; //TIMER1 start.
    T3CONbits.ON = 1; //TIMER3 start.
    T5CONbits.ON = 1; //TIMER5 start.
    RTC_TIMER_IE = 1; // Enable timer interrupt.
    INTCONbits.PEIE = 1;
}

void init_ms_timer0() {
    PIE0bits.TMR0IE = 0;
    T0CON0bits.T016BIT = 0; // Enable 16-bit mode.
    T0CON0bits.T0OUTPS = 0b0000; // Postscalar 1:1
    T0CON1bits.T0CS = 0b010; // Fosc/4 Clock Source - clock itself
    T0CON1bits.T0PS = 0b0111; // 1:64 prescalar
    T0CON0bits.T0EN = 1; // Start timer.
    PIE0bits.TMR0IE = 1; // Enable Interrupt
    INTCONbits.PEIE = 1;
}

// </editor-fold>

uint8_t get_hour() {
    time_t const_time = rtc_time_sec;
    return gmtime(&const_time)->tm_hour;
}

uint8_t get_minute() {
    time_t const_time = rtc_time_sec;
    return gmtime(&const_time)->tm_min;
}

uint8_t get_second() {
    time_t const_time = rtc_time_sec;
    return gmtime(&const_time)->tm_sec;
}

void set_time(uint8_t h, uint8_t m, uint8_t s) {
    time_t const_time = rtc_time_sec;
    struct tm * t = gmtime(&const_time);
    t->tm_min = m;
    t->tm_hour = h;
    t->tm_sec = s;
    time_t newtime = mktime(t);
    set_rtc_time(newtime);
}

time_t get_corrected_time_msec(){
    time_t result = 0;
    time_t sec = rtc_time_sec;

    result+=rtc_time_sec*1000;
    result += get_ms_corrected();
    return result;
}

uint16_t get_ms_corrected(){
    time_t msec = rtc_time_msec;
    return msec - msec%41;
}