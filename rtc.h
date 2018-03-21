#ifndef RTC_H
#define RTC_H
#include "xc.h"
#include "DAACED.h"
#include <stdint.h>
#include <time.h>

#ifndef _XTAL_FREQ
#define _XTAL_FREQ            (64000000UL) //64MHz
#endif

#define RTC_TIMER_IE    (PIE5bits.TMR1IE)
#define RTC_TIMER_IF    (PIR5bits.TMR1IF)

#define SYS_MODE_NORMAL (0)
#define SYS_MODE_SLEEP  (1)
#define delay_rtc(x)    {uint32_t __st= rtc_time_sec;while(rtc_time_sec-__st<x);}

#define MSB(x)              ((x & 0xFF00)>>8)
#define LSB(x)              (x & 0x00FF)

volatile time_t rtc_time_sec = 12345;
volatile time_t rtc_time_msec = 43;
volatile int8_t timers_diff = 0;
volatile int16_t dt_correction = 0;

volatile uint32_t button_down_time, button_up_time;
void initialize_rtc_timer();
void handle_preceise_time();
void init_1ms_timer0();

uint8_t get_hour();
uint8_t get_minute();
void set_time(uint8_t h,uint8_t m,uint8_t s);
#endif /* RTC_H */