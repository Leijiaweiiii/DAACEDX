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

volatile time_t rtc_time_sec = 12345;
volatile time_t rtc_time_msec = 12345000;//rtc_time_sec*1000
TBool time_changed = true;

#define delay_rtc_ms(x)    {time_t __st = rtc_time_msec;while((rtc_time_msec-__st)<x);}

volatile uint32_t button_down_time, button_up_time;
void initialize_rtc_timer();
//void handle_preceise_time();
void init_10ms_timer0();

uint8_t get_hour();
uint8_t get_minute();
uint8_t get_second();
void set_time(uint8_t h, uint8_t m, uint8_t s);
#endif /* RTC_H */