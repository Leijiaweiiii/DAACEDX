#include "bluetooth.h"

#define at_ok() ((uart_rx_buffer[0] == 'O' && uart_rx_buffer[1] == 'K'))

void BT_send_comand(const char * cmd, int length) {
    uart_start_tx_string(cmd, length);
    uart_rx_handled();
    while (!uart_flags.tx_complete);
    Delay(55);
    uart_tx_completed();
}

void get_mac_address(){
    do {
        BT_send_comand("AT91", 4);
        strncpy(mac_addr, uart_rx_buffer, 13);
    } while (0 == mac_addr[0]);
    uart_rx_handled();
}

void set_device_name(){
    int len;
    char * res[24]; // 20 for the name + 3 for OK: + 2 for NULL
    for(uint8_t i = 0;i<24;i++) res[i] = 0;
    len = sprintf(device_name_cmd,"AT01RAZOR-%s-%u", device_id, FW_VERSION);
    do {
        BT_send_comand(device_name_cmd, len);
        strncpy(res, uart_rx_buffer, 24);
    } while ((char)res[0] != 'O'); // Waiting for "OK:xxx..x"
    uart_rx_handled();
}

void BT_init() {
    BT_hard_reset();
    get_mac_address();
    set_device_name();
    BT_STATUS.connected = 0;
}

void BT_off() {
    BT_STATUS.connected = 0;
    uart_disable();
    BT_RESET_INV = 0; // Hold device in reset to mimic Power OFF
}

void sendOneShot(uint8_t shot_number, shot_t * shot) {
    char msg[16];
    int size;
    size = sprintf(msg, "%d,%d,%d\n", shot_number + 1, shot->is_flags, shot->dt);
    uart_start_tx_string(msg, size);
}

void sendSignal(const char * name, uint16_t duration, uint24_t time_ms) {
    char msg[16];
    int size;
    size = sprintf(msg, "%s,%d,%d\n", name, duration, time_ms);
    uart_start_tx_string(msg, size);
}

#define clear_args_buffer() { for (int i = 0; i < UART_RX_BUF_SIZE; i++) {bt_cmd_args_raw[i] = 0;}}

void BT_define_action() {
    if (uart_rx_buffer[0] == 'D' && uart_rx_buffer[1] == 'A' && uart_rx_buffer[2] == 'A') {
        int cmd = atoi(uart_rx_buffer + 3);
        if (cmd >= 0) {
            BT_COMMAND = (BT_COMMAND_T) cmd;
            clear_args_buffer();
            strncpy(bt_cmd_args_raw, uart_rx_buffer + 4, UART_RX_BUF_SIZE - 4);
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
        Delay(2); // 1ms is length of 1 character in 9600. in 115200 this is infinity
        if (rx_head == const_head)
            uart_rx_handled();
    }
}

void sendString(const char * x, size_t y)    {
    uart_rx_handled(); // TODO: Check if required
    uart_start_tx_string(x,y);
}