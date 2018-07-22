#include "bluetooth.h"
#include "uart.h"

TBool at_ok(){
    return (uart_rx_buffer[0]=='O' && uart_rx_buffer[1] == 'K');
}
void BT_send_comand(char * cmd, int length){
    uart_start_tx_string(cmd,length);
    uart_rx_handled();
    while(!uart_flags.tx_complete);
    Delay(100);
    asm(" nop");
}
void BT_init() {
    BT_hard_reset();
    Delay(100);
    BT_send_comand("AT",2);
    if(at_ok()){
        // Set high speed
        BT_send_comand("AT+BAUD4",16);
        uart_set_high_speed;
        BT_send_comand("AT",2);
        if(!at_ok()){
            uart_set_low_speed;
            BT_hard_reset();
        }
    }
    BT_send_comand("AT+NAMEDAA_RAZOR",16);
    BT_send_comand("AT+PWRM1",8);// Disable auto sleep when powered ON
    uart_rx_handled();
    BT_STATUS.initialized = 1;
}

void BT_off(){
    if(BT_STATUS.initialized){

        BT_send_comand("AT+PWRM0",8);
        Delay(45);
        BT_send_comand("AT+UART1",8);
        Delay(45);
        BT_send_comand("AT+SLEEP",8);
        BT_STATUS.initialized = 0;
        Delay(100);
        uart_disable();
    }
}

void BT_soft_reset(){
    BT_send_comand("AT+RESET",8);

}

void BT_hard_reset() {
    BT_RESET_INV = 0;
    _delay(100);
    BT_RESET_INV = 1;
}
void BT_set_high_speed(){
    
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
            case 2:
            case '2':
                BT_COMMAND = BT_GetLastString;
                break;
            case 5:
            case '5':
                BT_COMMAND = BT_GetConfig;
                break;
        }
        // DAA prefix - our commands
        uart_rx_handled();
    }
}