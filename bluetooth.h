/*
 * File:   bluetooth.h
 * Author: navado
 *
 * Created on 23 May 2018, 10:49
 */

#ifndef BLUETOOTH_H
#define	BLUETOOTH_H
#include "DAACEDcommon.h"
#include "shot.h"
#include "uart.h"
#ifdef	__cplusplus
extern "C" {
#endif
#define BT_INSERT       PORTDbits.RD4
#define BT_RESET_INV    PORTDbits.RD3
#define BT_PRESENT      (BT_INSERT == 0)

    // <editor-fold defaultstate="collapsed" desc="HM-11 AT commands">
#define AT_RES_ON                  (1)
#define AT_RES_OFF                 (0)
#define AT_GET_OK_1_PARAM       "OK+Get:%1X"
#define AT_SET_OK_1_PARAM       "OK+Set:%1X"
#define AT_TEST                 "AT"
#define AT_TEST_OK_IDLE         "OK"
#define AT_TEST_OK_LOST         "OK+LOST"
#define AT_MODULE_ADDR          "AT+ADDR?"
#define AT_MODULE_ADDR_RES      "OK+ADDR:%X"
#define AT_GET_ADVI             "AT+ADVI?"
#define AT_GET_ADVI_RES         AT_GET_OK_1_PARAM
#define AT_SET_ADVI             "AT+ADVI%1X"
#define AT_SET_ADVI_RES         AT_SET_OK_1_PARAM
#define AT_ADVI_PARAM_100ms     (0x0)
#define AT_ADVI_PARAM_125ms     (0x1)
#define AT_ADVI_PARAM_211ms     (0x2)
#define AT_ADVI_PARAM_318ms     (0x3)
#define AT_ADVI_PARAM_417ms     (0x4)
#define AT_ADVI_PARAM_546ms     (0x5)
#define AT_ADVI_PARAM_760ms     (0x6)
#define AT_ADVI_PARAM_852ms     (0x7)
#define AT_ADVI_PARAM_1022ms    (0x8)
#define AT_ADVI_PARAM_1285ms    (0x9)
#define AT_ADVI_PARAM_2000ms    (0xA)
#define AT_ADVI_PARAM_3000ms    (0xB)
#define AT_ADVI_PARAM_4000ms    (0xC)
#define AT_ADVI_PARAM_5000ms    (0xD)
#define AT_ADVI_PARAM_6000ms    (0xE)
#define AT_ADVI_PARAM_7000ms    (0xF)
#define AT_GET_ADTY             "AT+ADTY?"
#define AT_GET_ADTY_RES         AT_GET_OK_1_PARAM
#define AT_SET_ADTY             "AT+ADTY%1X"
#define AT_SET_ADTY_RES         AT_SET_OK_1_PARAM
    /*
    Para: 0 ~ 3
    0: Advertising
    ScanResponse,
    Connectable
    1: Only allow last device
    connect in 1.28 seconds
    2: Only allow Advertising
    and ScanResponse.
    3: Only allow Advertising
    Default: 0
     */
#define AT_ADTY_PARAM_AD_SCAN_CON   (0x0)
#define AT_ADTY_PARAM_CONNECT_LAST  (0x1)
#define AT_ADTY_PARAM_AD_SCAN       (0x2)
#define AT_ADTY_PARAM_AD            (0x3)
    /*
    Note1: This command added in V524.
    Note2: Please send AT+RESET to restart module if you set value 1.
    Note3: Must execute AT+TYPE3 first.
     */
#define AT_GET_ANCS                 "AT+ANCS?"
#define AT_GET_ANCD_RES             AT_GET_OK_1_PARAM
#define AT_SET_ANCS                 "AT+ANCS%d"
#define AT_ANCS_ON                  AT_RES_ON
#define AT_ANCS_OFF                 AT_RES_OFF
#define AT_GET_ALLO                 "AT+ALLO?"
#define AT_GET_ALLO_RES             AT_GET_OK_1_PARAM
#define AT_SET_ALLO                 "AT+ALLO%1d"
#define AT_SET_ALLO_RES             AT_SET_OK_1_PARAM
    /*
    Note1: This command added in V523.
    Note2: Whitelist allow three mac address link to module. Please use AT+AD
    command set whitelist mac address.
     */
#define AT_ALLO_ON                  AT_RES_ON
#define AT_ALLO_OFF                 AT_RES_OFF
#define AT_RESET                    "AT+RESET"
#define AT_RESET_RES                "OK+RESET"
    // TODO: Continue
    // </editor-fold>

    char mac_addr[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0}; // 12 is the length + 1 for null
    char device_name_cmd[32];
#define AT_CMD_LEN      4
#define device_name (device_name_cmd + AT_CMD_LEN)
#define device_id (mac_addr + 6)
    union {
        unsigned status_byte : 8;

        struct {
            unsigned card_present   : 1;
            unsigned initialized    : 1;
            unsigned connected      : 1;
        };
    } BT_STATUS;

    typedef enum {
        BT_SendVersion = 0,
        BT_StartTimer = 1, // Same as to press "start" button in "Autostart" mode
        BT_GetLastString = 2,
        BT_SetPar = 3,
        BT_SetMode = 4,
        BT_ClearHistory = 5,
        BT_DefaultSettings = 6,
        BT_Find = 7,
        BT_GetPars = 8,
        BT_SetDelay = 9,
        BT_GetBatteryMV = 10,
        BT_GetStats = 11,
        BT_SetCustom = 12,
        BT_GetCustomSequence = 13,
        BT_None = 127
    } BT_COMMAND_T;

    char bt_cmd_args_raw[UART_RX_BUF_SIZE];
    /*
     Examples of commands
     DAA+CMD1
     */
    void BT_init();
    void BT_off();

#define BT_soft_reset() {BT_RESET_INV = 0; Delay(50); BT_RESET_INV = 1; BT_STATUS.connected = 0;}
#define BT_hard_reset() {LATEbits.LATE0 = 0; Delay(50); LATEbits.LATE0 = 0; BT_STATUS.connected = 0;}

    void sendOneShot(shot_t * shot);
    void sendSignal(const char * name, uint16_t duration, time_t time_ms);
    BT_COMMAND_T BT_define_action();
    void sendString(const char * x, size_t y);
#define DAA_MSG_OK       {sendString("OK\n",3);}
#define DAA_MSG_ERROR    {sendString("ERROR\n",6);}
#define DAA_MSG_EMPTY    {sendString("EMPTY\n",6);}
#define DAA_MSG_WAIT     {sendString("WAIT\n",5);}
#define DAA_MSG_LISTEN   {sendString("LISTEN\n",7);}
#define DAA_MSG_NOT_SUPPORTED   {sendString("NOT SUPPORTED\n",14);}
#ifdef	__cplusplus
}
#endif

#endif	/* BLUETOOTH_H */

