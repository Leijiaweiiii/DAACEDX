#include "DAACEDcommons.h"
#include <stdint.h>
#define RTC_TIMER_IE    (PIE5bits.TMR1IE)
#define RTC_TIMER_IF    (PIR5bits.TMR1IF)

#define SYS_MODE_NORMAL (0)
#define SYS_MODE_SLEEP  (1)
#define delay_rtc(x)    {uint32_t __st= get_rtc_time();while(get_rtc_time()-__st<x);}

volatile uint32_t rtc_time_sec;
volatile uint8_t precise_time = 0;
volatile uint32_t button_down_time, button_up_time;
void initialize_rtc_timer();
void set_rtc_time(uint32_t time);
uint32_t get_rtc_time();
void handle_preceise_time();
void init_10ms_timer0();