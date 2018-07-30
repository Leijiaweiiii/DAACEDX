#include "charger.h"
#include "DAACED.h"

uint8_t number_of_battery_bars(){
    uint8_t res = 0;
    for(uint8_t i = 0;i<5;i++){
#ifdef TIMEBAT
        if(battery_level_thresholds[i]<battery_charge){
            res++;
        }
#else
        if(battery_voltage_thresholds[i]<battery_mV){
            res++;
        }
#endif
    }
    return res;
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

