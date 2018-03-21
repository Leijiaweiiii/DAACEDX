/*
 * File:   ui.h
 * Author: navado
 *
 * Created on 21 March 2018, 14:16
 */

#ifndef UI_H
#define	UI_H

#include "rtc.h"
#include "lcd.h"

#ifdef	__cplusplus
extern "C" {
#endif

#define LONG_PRESS_THRESHOLD_SEC    2
#define STICKY_THRESHOLD_SEC        5
    void handle_ui();

    typedef enum {
        PowerOff = 0,
        TimerIdle,
        TimerCounting,
        ReviewScreen,
        SettingsScreen
    } UiState;

    UiState ui_state = TimerIdle;
    typedef enum {
        None = 0,
        StartShort,
        StartLong,
        ReviewShort,
        ReviewLong,
        UpShort,
        UpLong,
        DownShort,
        DownLong,
        BackShort,
        BackLong
    } ButtonCommand;
    ButtonCommand comandToHandle = None;
    TBool Autostart = false;
    
    char ScreenTitle[16];
    void set_screen_title(char * value);
    
#ifdef	__cplusplus
}
#endif

#endif	/* UI_H */

