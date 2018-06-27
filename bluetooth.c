#include "bluetooth.h"

void BT_init() {
    BT_hard_reset();
}

void BT_off(){
    BT_RESET_INV = 0;// Hold in reset when off
}

void BT_soft_reset() {

}

void BT_hard_reset() {
    BT_RESET_INV = 0;
    _delay(300);
    BT_RESET_INV = 1;
}

void BT_send_uart() {

}