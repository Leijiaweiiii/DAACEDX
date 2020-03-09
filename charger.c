#include "charger.h"
#include "DAACED.h"

uint8_t number_of_battery_bars(){
    uint16_t avg = battery_average();
    if (avg > BATTERY_FULL_THR) return 5;
    if (avg > BATTERY_HALF_THR) return 3;
    return 0;
}

void define_charger_state() {
    switch (CHARGER_STATE) {
        case CHARGER_CHARGING:
            charger_state = Charging;
            break;
        case CHARGER_NOT_CONNECTED:
        case CHARGER_DISCONNECTED:
            charger_state = NotCharging;
            break;
        case CHARGER_COMPLETE:
            charger_state = Complete;
            break;
        default:
            break;
    }
}

uint16_t battery_average(){
    uint16_t res = 0;
    uint16_t max = 0, min = 4096;
    for(uint8_t i = 0;i<BAT_BUFFER_SIZE;i++){
        max = MAX(max,bat_samples[i]);
        min = MIN(min,bat_samples[i]);
    }
    for(uint8_t i = 0;i<BAT_BUFFER_SIZE;i++)
        res += bat_samples[i];
    res -= max;
    res -= min;
    res = (res >> BAT_BUFFER_SIZE_SHIFT);
    return res;
}
