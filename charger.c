#include "charger.h"
#include "DAACED.h"

void define_charger_state(){
    ChargerState_t old_state = charger_state;
    switch(CHARGER_STATE){
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
    charger_state_changed |= (old_state != charger_state);
}

char * charger_text_state(){
    char msg[10];
    sprintf(msg," 0x%02X ",CHARGER_STATE);
    return msg;
    // off - 0xFB
    // connected - 0xE3
    // charging - 0xA3
//    switch(charger_state){
//        case Charging:
//            return "C";
//        case Complete:
//            return "F";
//        default:
//            return "N";
//    }
}
