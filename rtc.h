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


uint32_t time_ms(void);
void clear_time_ms(void);

void initialize_rtc_timer(void);

void set_time(uint8_t h, uint8_t m, TBool is24h);
void read_time(void);
uint8_t hours(void);
uint8_t hours24(void);
uint8_t minutes(void);
TBool is1224(void);
TBool isAMPM(void);

uint8_t rtc_print_time(char * buff);
uint8_t rtc_print_time_full(char * buff, uint8_t h, uint8_t  m, TBool format24h);

#define IsHourAM(x)    ((x < 12))
#endif /* RTC_H */