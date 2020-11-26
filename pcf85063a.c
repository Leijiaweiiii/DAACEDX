#include <stdio.h>
#include <stdint.h>
#include "pic18_i2c.h"
#include "pcf85063a.h"

#define DEVICE_ADDR 0x51

#define REGISTER_SEGMENT_BASE_ADDR 0x00
#define CONTROL_SEGMENT_ADDR REGISTER_SEGMENT_BASE_ADDR
#define TIME_SEGMENT_ADDR (CONTROL_SEGMENT_ADDR + sizeof(struct RtcControlData))
#define ALARM_SEGMENT_ADDR (TIME_SEGMENT_ADDR + sizeof(struct RtcDateTimeData))
#define TIMER_SEGMENT_ADDR (ALARM_SEGMENT_ADDR + sizeof(struct RtcAlarmData))

void getRtcData(void) {
	getRtcControlData();
	getRtcDateTimeData();
//	getRtcAlarmData();
//	getRtcTimerData();
}

void setRtcData(void) {
    setRtcControlData();
    setRtcDateTimeData();
//    setRtcAlarmData();
//    setRtcTimerData();
}

void getRtcControlData(void) {
    pic18_i2c_read(DEVICE_ADDR, CONTROL_SEGMENT_ADDR, &prcdControl, sizeof(struct RtcControlData));
}

void setRtcControlData(void) {
    pic18_i2c_write(DEVICE_ADDR, CONTROL_SEGMENT_ADDR,  &prcdControl, sizeof(struct RtcControlData));
}

void getRtcDateTimeData(void) {
	pic18_i2c_read(DEVICE_ADDR, TIME_SEGMENT_ADDR, &prdtdDateTime, sizeof(struct RtcDateTimeData));
}

void setRtcDateTimeData(void) {
    pic18_i2c_write(DEVICE_ADDR, TIME_SEGMENT_ADDR, &prdtdDateTime, sizeof(struct RtcDateTimeData));
}
//
//void getRtcAlarmData(void) {
//    pic18_i2c_enable();
//    pic18_i2c_read(DEVICE_ADDR, ALARM_SEGMENT_ADDR, &pradAlarm, sizeof(struct RtcAlarmData));
//    pic18_i2c_disable();
//}
//
//void setRtcAlarmData(void) {
//    pic18_i2c_enable();
//    pic18_i2c_write(DEVICE_ADDR, ALARM_SEGMENT_ADDR, &pradAlarm, sizeof(struct RtcAlarmData));
//    pic18_i2c_disable();
//}
//
//void getRtcTimerData(void) {
//    pic18_i2c_enable();
//    pic18_i2c_read(DEVICE_ADDR, TIMER_SEGMENT_ADDR, &prtdTimer, sizeof(struct RtcTimerData));
//    pic18_i2c_disable();
//}
//
//void setRtcTimerData(void) {
//    pic18_i2c_enable();
//    pic18_i2c_write(DEVICE_ADDR, TIMER_SEGMENT_ADDR, &prtdTimer, sizeof(struct RtcTimerData));
//    pic18_i2c_disable();
//}
