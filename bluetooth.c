#include "bluetooth.h"

#define at_ok() ((uart_rx_buffer[0] == 'O' && uart_rx_buffer[1] == 'K'))

void BT_send_comand(const char * cmd, int length) {
    uart_start_tx_string(cmd, length);
    uart_rx_handled();
    while (!uart_flags.tx_complete);
    Delay(20);
    asm(" nop");
}

void get_mac_address(){
    BT_send_comand("AT91", 4);
    // TODO: Tsake only 3 last bytes
    strncpy(mac_addr, uart_rx_buffer, 24);
}

void BT_init() {
    int len;
    BT_hard_reset();
    Delay(30);
    get_mac_address();
    len = sprintf(device_name_cmd,"AT01RAZOR:%s",mac_addr);
    BT_send_comand(device_name_cmd, len);
    uart_rx_handled();
    BT_STATUS.initialized = 1;
    BT_STATUS.connected = 0;
}

void BT_off() {
    if (BT_STATUS.initialized) {
        BT_send_comand("FUN_CMD_SLEEP_ENABLEID", 22);
        BT_STATUS.initialized = 0;
        BT_STATUS.connected = 0;
        Delay(5);
        uart_disable();
    }
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
        int8_t cmd = uart_rx_buffer[3] - '0';
        if (cmd >= 0) {
            BT_COMMAND = (BT_COMMAND_T) cmd;
            clear_args_buffer();
            strcpy(bt_cmd_args_raw, uart_rx_buffer + 4);
        } else {
            BT_COMMAND = BT_None;
        }
        // DAA prefix - our commands
        uart_rx_handled();
        BT_STATUS.connected = 1;
    } else if (at_ok()) {
        if (uart_rx_buffer[4] == 'O') { // CONN, LOST
            if (uart_rx_buffer[3] == 'L' && uart_rx_buffer[5] == 'S') {
                BT_STATUS.connected = 0;
                uart_rx_handled();
            } else if (uart_rx_buffer[3] == 'C' && uart_rx_buffer[5] == 'N') {
                BT_STATUS.connected = 1;
                uart_rx_handled();
            }
        }
    } else {
        uint8_t const_head = rx_head;
        UNUSED(const_head); // Don't optimize and memory barrier
        Delay(2); // 1ms is length of 1 character in 9600. in 115200 this is infinity
        if (rx_head == const_head)
            uart_rx_handled();
    }
}