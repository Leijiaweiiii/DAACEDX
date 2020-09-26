#ifndef PCF85063A
#define PCF85063A
#include <stdint.h>

struct RtcControlData {
        struct {
                uint8_t bCapSel:1;
                uint8_t b1224:1;
                uint8_t bCIE:1;
                uint8_t bUnused1:1;
                uint8_t bSR:1;
                uint8_t bStop:1;
                uint8_t bUnused2:1;
                uint8_t bExtTest:1;
        } control1;
        struct {
                uint8_t _COF:3;
                uint8_t bTF:1;
                uint8_t bHMI:1;
                uint8_t bMI:1;
                uint8_t bAF:1;
                uint8_t bAIE:1;
        } control2;
        struct {
                uint8_t _offset:7;
                uint8_t bMode:1;
        } offset;
        uint8_t bRAM;
};
struct RtcDateTimeData {
	struct {
		uint8_t _units:4;
		uint8_t _tens:3;
		uint8_t bOS:1;
	} seconds;
	struct {
		uint8_t _units:4;
		uint8_t _tens:3;
		uint8_t _unused:1;
	} minutes;
	union {
		struct {// 12 hours
			uint8_t _units12:4;
			uint8_t _tens12:1;
			uint8_t bAMPM:1;
			uint8_t _unused12:2;
		};
		struct {// 24 hours
			uint8_t _units:4;
			uint8_t _tens:2;
			uint8_t _unused:2;
		};
	} hours;
	struct {
	uint8_t _units:4;
	uint8_t _tens:2;
	uint8_t _unused:2;
	} days;
	struct {
		uint8_t _weekday:3;
		uint8_t _unused:5;
	} weekDays;
	struct {
		uint8_t _units:4;
		uint8_t _tens:1;
		uint8_t _unused:3;
	} months;
	struct {
		uint8_t _units:4;
		uint8_t _tens:4;
	} years;
	};
struct RtcAlarmData {
	struct {
		uint8_t _units:4;
		uint8_t _tens:3;
		uint8_t bAEnS:1;
	} secondAlarm;
	struct {
		uint8_t _units:4;
		uint8_t _tens:3;
		uint8_t bAEnM:1;
	} minuteAlarm;
	union {
		struct {//12 hours
			uint8_t _units12:4;
			uint8_t _tens12:2;
			uint8_t _unused12:1;
			uint8_t bAEnH12:1;
		};
		struct {//24 hours
			uint8_t _units:4;
			uint8_t _tens:1;
			uint8_t bAMPM:1;
			uint8_t _unused:1;
			uint8_t bAEnH:1;
		};
	} hourAlarm;
	struct {
		uint8_t _units:4;
		uint8_t _tens:2;
		uint8_t _unused:1;
		uint8_t bAEnD:1;
	} dayAlarm;
	struct {
		uint8_t _units:3;
		uint8_t _unused:4;
		uint8_t bAEnW:1;
	} weekDayAlarm;
};
struct RtcTimerData {
        uint8_t timerValue;
        struct {
                uint8_t bTiTp:1;
                uint8_t bTiE:1;
                uint8_t bTE:1;
                uint8_t _TCF:2;
        } timerMode;
};

struct RtcData {
        struct RtcControlData  prcdControl;
        struct RtcDateTimeData prdtdDateTime;
        struct RtcAlarmData    pradAlarm;
        struct RtcTimerData    prtdTimer;
};

void getRtcData(struct RtcData *data);
void setRtcData(struct RtcData *data);

void getRtcControlData(struct RtcControlData *control);
void setRtcControlData(struct RtcControlData *control);

void getRtcDateTimeData(struct RtcDateTimeData *time);
void setRtcDateTimeData(struct RtcDateTimeData *time);

void getRtcAlarmData(struct RtcAlarmData *alarm);
void setRtcAlarmData(struct RtcAlarmData *alarm);

void getRtcTimerData(struct RtcTimerData *timer);
void setRtcTimerData(struct RtcTimerData *timer);
#endif
