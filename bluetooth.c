#include "bluetooth.h"
#include "uart.h"
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
void sendOneShot(uint8_t shot_number,shot_t * shot){
    char msg[16];
    int size;
    if(!BT_STATUS.connected) return; // Send nothing if not connected
    size = sprintf(msg,"%d,%c,%d",shot_number,shot->is_flags,shot->dt);
    uart_start_tx_string(msg,size);
}