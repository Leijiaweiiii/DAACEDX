#ifndef PCF_85063_H
#define	PCF_85063_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "tbool.h"
#include "stdint.h"


#define RTC_DEVICE_ADDR 0x51

// Control and status registers
#define RTC_REG_CONTROL_1       0x00
#define RTC_REG_CONTROL_2       0x01
#define RTC_REG_OFFSET          0x02
#define RTC_REG_RAM_BYTE        0x03
// Time registers
#define RTC_REG_SECONDS         0x04
#define RTC_REG_MINUTES         0x05
#define RTC_REG_HOURS           0x06
#define RTC_REG_DAYS            0x07
#define RTC_REG_WEEKDAYS        0x08
#define RTC_REG_MONTHS          0x09
#define RTC_REG_YEARS           0x0A
#define RTC_NUM_TIME_REG        (7)
// Alarm registers
#define RTC_REG_SECOND_ALARM    0x0B
#define RTC_REG_MINUTE_ALARM    0x0C
#define RTC_REG_HOUR_ALARM      0X0D
#define RTC_REG_DAY_ALARM       0x0E
#define RTC_REG_WEEKDAY_ALARM   0x0F
// Timer registers
#define RTC_REG_TIMER_VALUE     0x10
#define RTC_REG_TIMER_MODE      0x11

// Special values
#define SOFT_RESET      0x58

typedef struct{
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t days;
    uint8_t weekdays;
    uint8_t months;
    uint8_t years;
} rtc_time_t;

rtc_time_t rtc_time;
TBool is_24h = true;

#define IS_24H_BIT(x)      ((x>>1) & 0x01)

void read_rtc_time();
void write_rtc_time();
void set_timer(uint8_t sec);


#endif	/* XC_HEADER_TEMPLATE_H */

