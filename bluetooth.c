#include "bluetooth.h"

#define at_ok() ((uart_rx_buffer[0] == 'O' && uart_rx_buffer[1] == 'K'))

void BT_send_comand(const char * cmd, int length) {
    uart_start_tx_string(cmd, length);
    uart_rx_handled();
    while (!uart_flags.tx_complete);
    Delay(55);
    uart_tx_completed();
}

uint16_t get_mac_address(){
    uint16_t i = 0;
    do {
        BT_send_comand("AT91", AT_CMD_LEN);
        i++;
    } while (uart_rx_buffer[0] == 0);
    strncpy(mac_addr, uart_rx_buffer, 13);
    uart_rx_handled();
    return i;
}

uint16_t get_device_name(){
    uint16_t i = 0;
    do {
        BT_send_comand("AT92", AT_CMD_LEN);
        i++;
    } while(uart_rx_buffer[0] == 0);
    strncpy(device_name, uart_rx_buffer, 25);
    uart_rx_handled();
    return i;
}

void set_device_name(){
    int len;
    get_device_name();
    len = sprintf(device_name_cmd,"AT01RAZOR-%s-%u", device_id, FW_VERSION);
    BT_send_comand(device_name_cmd, len);
    while (! at_ok())Delay(1); // Waiting for "OK:xxx..x"
    uart_rx_handled();
    BT_soft_reset();
    get_device_name();
}

void BT_init() {
//    BT_hard_reset();
    BT_soft_reset();
    // After the battery reset the first boot will be long.
    if(0 == mac_addr[0]){
        get_mac_address();
        set_device_name();
    }
    BT_STATUS.connected = 0;
}

void BT_off() {
    BT_STATUS.connected = 0;
    uart_disable();
    BT_RESET_INV = 0; // Hold device in reset to mimic Power OFF
}

void sendOneShot(shot_t * shot) {
    char msg[16];
    int size;
    size = sprintf(msg, "%u,%u,%lu\n", shot->sn, shot->is_flags, 0x00FFFFFF & shot->dt);
    uart_start_tx_string(msg, size);
}

void sendSignal(const char * name, uint16_t duration, time_t time_ms) {
    char msg[16];
    int size;
    size = sprintf(msg, "%s,%u,%u\n", name, duration, time_ms);
    uart_start_tx_string(msg, size);
}

#define clear_args_buffer() { for (int i = 0; i < UART_RX_BUF_SIZE; i++) {bt_cmd_args_raw[i] = 0;}}

BT_COMMAND_T BT_define_action() {
    uint8_t cmd_len = 3;
    BT_COMMAND_T BT_COMMAND = BT_None;
    char tmp_b[8]; //TODO: allocate temporary short strings only once
    if (uart_rx_buffer[0] == 'D' && uart_rx_buffer[1] == 'A' && uart_rx_buffer[2] == 'A') {
        int cmd = atoi(uart_rx_buffer + 3);
        if (cmd >= 0) {
            cmd_len += sprintf(tmp_b, "%d", cmd);
            BT_COMMAND = (BT_COMMAND_T) cmd;
            clear_args_buffer();
            strncpy(bt_cmd_args_raw, uart_rx_buffer + cmd_len + 1, UART_RX_BUF_SIZE - cmd_len - 1); // 1 is for delimiter to use with commands with parameters
        } else {
            BT_COMMAND = BT_None;
        }
        // DAA prefix - our commands
        uart_rx_handled();
        BT_STATUS.connected = 1;
//    } else if (at_ok()) {
//        if (uart_rx_buffer[4] == 'O') { // CONN, LOST
//            if (uart_rx_buffer[3] == 'L' && uart_rx_buffer[5] == 'S') {
//                BT_STATUS.connected = 0;
//                uart_rx_handled();
//            } else if (uart_rx_buffer[3] == 'C' && uart_rx_buffer[5] == 'N') {
//                BT_STATUS.connected = 1;
//                uart_rx_handled();
//            }
//        }
    } else {
        uint8_t const_head = rx_head;
        UNUSED(const_head); // Don't optimize and memory barrier
        Delay(1); // 1ms is length of 1 character in 9600. in 115200 this is infinity
        if (rx_head == const_head)
            uart_rx_handled();
    }
    return BT_COMMAND;
}

void sendString(const char * x, size_t y)    {
    uart_rx_handled(); // TODO: Check if required
    uart_start_tx_string(x,y);
}