#ifndef RTC_H
#define RTC_H
#include "xc.h"
#include "DAACEDcommon.h"


#define RTC_TIMER_IE    (PIE5bits.TMR1IE)
#define RTC_TIMER_IF    (PIR5bits.TMR1IF)

#define SYS_MODE_NORMAL (0)
#define SYS_MODE_SLEEP  (1)


#define SHOOT_IF            (PIR1bits.ADIF)
#define SHOOT_IE            (PIE1bits.ADIE)


// Timer3 counting output of timer1 which is 2 seconds
extern const uint16_t correction_table[];
typedef struct{
    union{
        time_t sec;
        struct{
            uint16_t sec_lsb;
            uint16_t sec_msb;
        };
    };
    time_t unix_time_ms;
    uint16_t msec;
}rtc_time_t;
rtc_time_t rtc_time;
uint8_t _hour,_minute,_2sec;
void set_rtc_time(time_t x);
// Taking only 10 bits - the MSB is 2 seconds
#define rtc_time_2k_msec    (TMR1>>5)
void update_rtc_time();
//#define delay_rtc(x)        {time_t __st = rtc_time.sec; while((rtc_time.sec  - __st)<x);}
#define delay_rtc_ms(x)     {time_t __st = rtc_time.unix_time_ms;while((rtc_time.unix_time_ms -  __st)<x);}

volatile uint32_t button_down_time, button_up_time;
void initialize_rtc_timer();
void init_ms_timer0();

uint8_t get_hour(TBool format24h);
uint8_t get_minute();
void tic_2_sec();
uint16_t get_ms_corrected();
#define get_corrected_time_msec()   (rtc_time.unix_time_ms)
#define set_time(h, m)              {_hour=h;_minute=m;}
uint8_t get_time_source();
uint8_t rtc_print_time(char * buff, TBool format24h);

#define IsHourAM(x)    ((x < 12))
#endif /* RTC_H */