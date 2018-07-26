#include "bluetooth.h"

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
    // TODO: Try connect to last known device
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
    BT_send_comand("AT+NAMERAZOR",12);
    BT_send_comand("AT+PWRM1",8);// Disable auto sleep when powered ON
    uart_rx_handled();
    BT_STATUS.initialized = 1;
}

void BT_off(){
    if(BT_STATUS.initialized){
        BT_send_comand("AT",2);
        Delay(20);
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

void clear_args_buffer(){
    for (int i = 0; i < UART_RX_BUF_SIZE; i++) {
        bt_cmd_args_raw[i] = 0;
    }
}

void BT_define_action(){
    if(uart_rx_buffer[0]=='D' && uart_rx_buffer[1]=='A' && uart_rx_buffer[2]=='A'){
        int8_t cmd  = uart_rx_buffer[3] - '0';
        if(cmd >= 0){
            BT_COMMAND = (BT_COMMAND_T)cmd;
            clear_args_buffer();
            strmycpy(bt_cmd_args_raw,uart_rx_buffer+4);
        } else {
            BT_COMMAND = BT_None;
        }
        // DAA prefix - our commands
        uart_rx_handled();
    } else {
        uint8_t const_head = rx_head;
        UNUSED(const_head); // Don't optimize and memory barrier
        Delay(2); // 1ms is length of 1 character in 9600. in 115200 this is infinity
        if(rx_head==const_head)
            uart_rx_handled();
    }
}