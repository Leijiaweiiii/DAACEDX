#ifndef RTC_H
#define RTC_H
#include "xc.h"
#include "DAACEDcommon.h"
#include <time.h>

#define RTC_TIMER_IE    (PIE5bits.TMR1IE)
#define RTC_TIMER_IF    (PIR5bits.TMR1IF)

#define SYS_MODE_NORMAL (0)
#define SYS_MODE_SLEEP  (1)
#define delay_rtc(x)    {uint32_t __st= rtc_time_sec;while(rtc_time_sec-__st<x);}

#define SHOOT_IF            (PIR1bits.ADIF)
#define SHOOT_IE            (PIE1bits.ADIE)


// Timer3 counting output of timer1 which is 2 seconds
#define rtc_time_sec        ((TMR3|(TMR5<<16))<<1|(TMR1>>15))
#define set_rtc_time(x)     {time_t __ts=x>>1;TMR3=0x0000FFFF & __ts; TMR5=(0xFFFF0000 & __ts)>>16;}

#define rtc_time_msec       (TMR1>>6)

#define delay_rtc_ms(x)    {uint8_t __st = rtc_time_msec;while((rtc_time_msec-__st)<x);}

volatile uint32_t button_down_time, button_up_time;
void initialize_rtc_timer();
void init_ms_timer0();

uint8_t get_hour();
uint8_t get_minute();
uint8_t get_second();
uint16_t get_ms_corrected();
time_t get_corrected_time_msec();
void set_time(uint8_t h, uint8_t m, uint8_t s);
#endif /* RTC_H */