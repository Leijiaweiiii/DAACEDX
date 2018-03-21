#include "rtc.h"

// <editor-fold defaultstate="collapsed" desc="RTC timer functions">


// Software RTC is implemented using TIMER1 on chip with 32.768 KHz timer.
void initialize_rtc_timer() {
    RTC_TIMER_IE = 0;       // Disable interrupt.
    RTC_TIMER_IF = 0;       // Clear Interrupt flag.
    OSCENbits.SOSCEN = 1;   // Secondary Oscillator Manual Request Enable bit.
    while(!OSCSTATbits.SOR);// Wait for Secondary Oscillator to be ready to use.
    TMR1CLKbits.CS= 0b0110; // TIMER1 clock source = 32.768KHz Secondary Oscillator.
    T1CONbits.CKPS = 0b11;  // Prescale = 8.
    T1CONbits.NOT_SYNC =1;  // asynchronous counter mode to operate during sleep
    TMR1H = 0xF0;           // preset for timer1 MSB register (1 second delay)
    TMR1L = 0x00;           // preset for timer1 LSB register (1 second delay)
    T1CONbits.ON = 1;       //TIMER1 start.
    ei();
    INTCONbits.PEIE = 1;    // Enable Peripheral interrupt.
    RTC_TIMER_IE = 1;       // Enable timer interrupt.
    
}

void init_1ms_timer0() {
    T0CON0bits.T0EN = 0;        // Stop timer.
    T0CON0bits.T016BIT = 1;     // Enable 16-bit mode.
    T0CON1bits.T0CS = 0b010;    // Fosc/4 Clock Source.
    T0CON1bits.T0PS = 0b0001;    // 1:2 prescalar
    uint16_t preset = 80000+dt_correction*100;
    TMR0H = MSB(preset);               // preset for Timer0 MSB register
    TMR0L = LSB(preset);               // preset for Timer0 LSB register
    T0CON0bits.T0EN = 1;        // Start timer.
    PIE0bits.TMR0IE = 1;        // Enabled Interrupt
    INTCONbits.PEIE = 1;
}


void handle_preceise_time(){
        rtc_time_sec++;
        T1CONbits.ON = 0;
        TMR1CLKbits.CS= 0b0110; // TIMER1 clock source = 32.768KHz Secondary Oscillator.
        T1CONbits.CKPS = 0b11;  // Prescale = 8.
        T1CONbits.NOT_SYNC =1;  // asynchronous counter mode to operate during sleep
        TMR1H = 0xF0;           // preset for timer1 MSB register (1 second delay)
        TMR1L = 0x00;           // preset for timer1 LSB register (1 second delay)
        T1CONbits.ON = 1;       //TIMER1 start.
        if(rtc_time_msec < 100){
            dt_correction--;
        } else if (rtc_time_msec > 100){
            dt_correction++;
        }
        timers_diff = 100 - rtc_time_msec;
        
        rtc_time_msec = 0;
//        init_10ms_timer0();     // Reset timer for hight accuracy.
}
// </editor-fold>

uint8_t get_hour(){
    time_t const_time = rtc_time_sec;
    return gmtime(&const_time)->tm_hour;
}
uint8_t get_minute(){
    time_t const_time = rtc_time_sec;
    return gmtime(&const_time)->tm_min;
}

void set_time(uint8_t h,uint8_t m,uint8_t s){
    time_t const_time = rtc_time_sec;
    struct tm * t = gmtime(&const_time);
    t->tm_min = m;
    t->tm_hour = h;
    t->tm_sec = s;
    time_t newtime = mktime(t);
    rtc_time_sec = newtime;
}