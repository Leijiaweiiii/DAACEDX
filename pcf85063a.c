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

uint8_t getRtcData(struct RtcData *data) {
    pic18_i2c_enable();
	getRtcControlData(&data->prcdControl);
	getRtcDateTimeData(&data->prdtdDateTime);
	getRtcAlarmData(&data->pradAlarm);
	getRtcTimerData(&data->prtdTimer);
    pic18_i2c_disable();
}

uint8_t setRtcData(struct RtcData *data) {
    pic18_i2c_enable();
    setRtcControlData(&data->prcdControl);
    setRtcDateTimeData(&data->prdtdDateTime);
    setRtcAlarmData(&data->pradAlarm);
    setRtcTimerData(&data->prtdTimer);
    pic18_i2c_disable();
}

uint8_t getRtcControlData(struct RtcControlData *control) {
    pic18_i2c_enable();
    pic18_i2c_read(DEVICE_ADDR, CONTROL_SEGMENT_ADDR, control, sizeof(*control));
    pic18_i2c_disable();
}

uint8_t setRtcControlData(struct RtcControlData *control) {
    pic18_i2c_enable();
    pic18_i2c_write(DEVICE_ADDR, CONTROL_SEGMENT_ADDR, control, sizeof(*control));
    pic18_i2c_disable();
}

uint8_t getRtcDateTimeData(struct RtcDateTimeData *time) {
    pic18_i2c_enable();
	pic18_i2c_read(DEVICE_ADDR, TIME_SEGMENT_ADDR, time, sizeof(*time));
    pic18_i2c_disable();
}

uint8_t setRtcDateTimeData(struct RtcDateTimeData *time) {
    pic18_i2c_enable();
    pic18_i2c_write(DEVICE_ADDR, TIME_SEGMENT_ADDR, time, sizeof(*time));
    pic18_i2c_disable();
}

uint8_t getRtcAlarmData(struct RtcAlarmData *alarm) {
    pic18_i2c_enable();
    pic18_i2c_read(DEVICE_ADDR, ALARM_SEGMENT_ADDR, alarm, sizeof(*alarm));
    pic18_i2c_disable();
}

uint8_t setRtcAlarmData(struct RtcAlarmData *alarm) {
    pic18_i2c_enable();
    pic18_i2c_write(DEVICE_ADDR, ALARM_SEGMENT_ADDR, alarm, sizeof(*alarm));
    pic18_i2c_disable();
}

uint8_t getRtcTimerData(struct RtcTimerData *timer) {
    pic18_i2c_enable();
    pic18_i2c_read(DEVICE_ADDR, TIMER_SEGMENT_ADDR, timer, sizeof(*timer));
    pic18_i2c_disable();
}

uint8_t setRtcTimerData(struct RtcTimerData *timer) {
    pic18_i2c_enable();
    pic18_i2c_write(DEVICE_ADDR, TIMER_SEGMENT_ADDR, timer, sizeof(*timer));
    pic18_i2c_disable();
}