#ifndef RTC_H
#define RTC_H
#include "xc.h"
#include "DAACEDcommon.h"
#include "pcf85063A.h"

#define RTC_TIMER_IE    (PIE5bits.TMR1IE)
#define RTC_TIMER_IF    (PIR5bits.TMR1IF)

#define SYS_MODE_NORMAL (0)
#define SYS_MODE_SLEEP  (1)


#define SHOOT_IF            (PIR1bits.ADIF)
#define SHOOT_IE            (PIE1bits.ADIE)


// Timer3 counting output of timer1 which is 2 seconds
extern const uint16_t correction_table[];
volatile time_t unix_time_ms, unix_time_ms_sec = 0;
#define _hour       rtc_time.hours.raw
#define _minute     rtc_time.minutes.raw

// Taking only 10 bits - the MSB is 2 seconds
#define rtc_time_2k_msec    (TMR1>>5)
void update_rtc_time();

void initialize_rtc_timer();
void init_ms_timer0();

void tic_2_sec();
#define set_time(h, m)              {_hour=h;_minute=m;}

uint8_t rtc_print_time(char * buff, TBool format24h);
uint8_t rtc_print_time_full(char * buff, uint8_t h, uint8_t  m, TBool format24h);

#define IsHourAM(x)    ((x < 12))
#endif /* RTC_H */