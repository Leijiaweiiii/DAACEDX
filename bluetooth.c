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
//    if(!BT_STATUS.connected) return; // Send nothing if not connected
    size = sprintf(msg,"%d,%d,%3.2f\n",shot_number + 1,shot->is_flags,((float)shot->dt)/1000);
    uart_start_tx_string(msg,size);
}

void sendSignal(const char * name, uint16_t duration,uint24_t time_ms){
    char msg[16];
    int size;
//    if(!BT_STATUS.connected) return; // Send nothing if not connected
    size = sprintf(msg,"%s,%3.2f,%3.2f\n",name,((float)duration)/1000,((float)time_ms)/1000);
    uart_start_tx_string(msg,size);
}

void BT_define_action(){
    if(uart_rx_buffer[0]=='D' && uart_rx_buffer[1]=='A' && uart_rx_buffer[2]=='A'){
        switch(uart_rx_buffer[3]){
            case 1:
            case '1':
                BT_COMMAND = BT_StartTimer;
                break;
            case 6:
            case '6':
                BT_COMMAND = BT_GetLastString;
                break;
            case 5:
            case '5':
                BT_COMMAND = BT_GetLastString;
                break;
        }
        // DAA prefix - our commands
        uart_rx_handled();
    }
}