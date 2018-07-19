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
    /*
     * Battery charge is defined in mA-mS
     * Every time we're consuming electricity
     * we subtract some capacity of known consumers from the resource
     * Every time we're in charging state - we're adding capacity to the resource
     * Sometimes we're estimating capacity and updating measured value to be the full charge
     */

#define CONSUMPTION_FULLY_CHARGED   (UINT32_MAX-1000)
    uint32_t battery_charge = CONSUMPTION_FULLY_CHARGED/2;
    uint16_t backlight_consumption[] = {0, 80, 90, 130, 160, 190, 210, 240, 300, 350};
    uint32_t battery_level_thresholds[] = {0x1AAAAAAA, 0x35555555, 0x6FFFFFFF, 0xAAAAAAAA, 0xD5555554};
#define CONSUMPTION_BEEP_MA     210
#define CONSUME_BEEP(x)         {battery_charge-=x*CONSUMPTION_BEEP_MA;}
    // (x,y) -> duration,level
#define CONSUME_BACKLIGHT(x,y)  {battery_charge -= x * backlight_consumption[y];}
#define CONSUME_POWER_OFF(x)    {battery_charge -= x * 10; }
#define CONSUME_POWER_ON(x)     {battery_charge -= x*160; }
#define CONSUME_CHARGE_ADD(x)   {if(battery_charge<full_charge) battery_charge += x*10000; }
#define CONSUME_CHARGED_FULL    {battery_charge = full_charge;}
    uint32_t full_charge = CONSUMPTION_FULLY_CHARGED;
    uint8_t number_of_battery_bars();
    
#ifdef	__cplusplus
}
#endif

#endif	/* CHARGER_H */

