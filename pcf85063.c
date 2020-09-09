#include "pcf85063A.h"
#include "pic18_i2c.h"

#define IS_24H_BIT(x)      ((x>>1) & 0x01)
#define set_24H_BIT(x)      ()
void read_rtc_time(){
    uint8_t ctrl1 = 0;
    return;
    pic18_i2c_enable();
    pic18_i2c_read(RTC_DEVICE_ADDR, RTC_REG_CONTROL_1, &ctrl1, 1);
    is_24h = IS_24H_BIT(ctrl1);
    pic18_i2c_read(RTC_DEVICE_ADDR, RTC_REG_SECONDS, rtc_time.raw, RTC_NUM_TIME_REG);
    pic18_i2c_disable();
}

void write_rtc_time(){
    uint8_t ctrl1 = 0;
    return;
    pic18_i2c_enable();
    pic18_i2c_read(RTC_DEVICE_ADDR, RTC_REG_CONTROL_1, &ctrl1, 1);
    is_24h = IS_24H_BIT(ctrl1);
    pic18_i2c_write(RTC_DEVICE_ADDR, RTC_REG_SECONDS, rtc_time.raw, RTC_NUM_TIME_REG);
    pic18_i2c_disable();
}

void set_timer(uint8_t sec){
//    uint8_t data[2];
//    data[0] = sec;
//    data[1] = 0b00010111; // 1 sec source (10), TE(1), TIE(1), TI_TP(1)
//    i2c_write(RTC_DEVICE_ADDR, RTC_REG_TIMER_VALUE, 2, data);
}