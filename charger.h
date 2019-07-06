/* 
 * File:   charger.h
 * Author: navado
 *
 * Created on 10 May 2018, 08:28
 */

#ifndef CHARGER_H
#define	CHARGER_H
#include "tbool.h"
#include "stdint.h"
#ifdef	__cplusplus
extern "C" {
#endif

#define BAT_STAT_PORT               PORTE
#define CHARGET_PG_NOT              0b00001000
#define CHARGET_STAT_1              0b00010000
#define CHARGET_STAT_2              0b00100000
#define CHARGER_STATE               (BAT_STAT_PORT & 0b00111000)
#define CHARGER_STATE_SHUTDOWN      0b00110000
#define CHARGER_STATE_STANDBY       0b00111000
#define CHARGER_STATE_TEST          0b00001000
#define CHARGER_STATE_PRECONDITION  0b00101000
#define CHARGER_STATE_TEMP_FAULT    0b00111000
#define CHARGER_STATE_FAST_CHARGE   0b00101000
#define CHARGER_STATE_TIMER_FAULT   0b00111000
#define CHARGER_STATE_CONST_VOLTAGE 0b00101000
#define CHARGER_STATE_COMPLETE      0b00011000

#define CHARGER_CHARGING            0x20
#define CHARGER_NOT_CONNECTED       0x30
#define CHARGER_DISCONNECTED        0x38
#define CHARGER_COMPLETE            0x10

    typedef enum {
        NotCharging,
        Charging,
        Complete
    } ChargerState_t;
    volatile ChargerState_t charger_state = NotCharging;
    ChargerState_t charger_display_state = NotCharging;
#define charger_state_changed   (charger_display_state != charger_state)
    void define_charger_state();
    char * charger_text_state();
    uint16_t battery_voltage_thresholds[] = {3930, 3850, 3760, 3660, 3420, 3350};
    uint16_t battery_mV = 0;
    uint16_t battery_min_mV = 4096;
#define battery_low         (battery_mV<battery_voltage_thresholds[5])
    uint8_t number_of_battery_bars();
    
#ifdef	__cplusplus
}
#endif

#endif	/* CHARGER_H */

