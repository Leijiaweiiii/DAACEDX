#include "charger.h"
#include "DAACED.h"
#include "max17260.h"

uint8_t number_of_battery_bars(void){
    uint8_t rsoc = fg_get_rsoc();
    return (uint8_t) (rsoc / 10);
}

void define_charger_state(void) {
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