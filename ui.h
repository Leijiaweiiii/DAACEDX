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
        TimerListening,
        TimerCountdown,
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
        CountdownExpired,
        UpShort,
        UpLong,
        DownShort,
        DownLong,
        BackShort,
        BackLong
    } ButtonCommand;
    volatile ButtonCommand comandToHandle = None;
    
    typedef enum TimerEvent{
        NoEvent = 0,TimerTimeout,ParEvent
    } TimerEvent;
    TimerEvent timerEventToHandle = NoEvent;
    
    char ScreenTitle[16];
    void set_screen_title(char * value);
    void print_big_time_label(time_t t);
    
#define UI_COUNTER_START_LINE   22
#define UI_COUNTER_START_PIXEL  0
#define UI_FOOTER_START_LINE    71
#define UI_FOOTER_GRID_H_CELLS  3
#define UI_FOOTER_GRID_V_CELLS  2
    
#define UI_FOOTER_GRID_WIDTH    (LCD_WIDTH/UI_FOOTER_GRID_H_CELLS)
#define UI_FOOTER_GRID_HEIGH    ((LCD_HEIGHT-UI_FOOTER_START_LINE)/UI_FOOTER_GRID_V_CELLS)
#define UI_FOOTER_GRID_X(x)     (x*(UI_FOOTER_GRID_WIDTH)+2)
#define UI_FOOTER_GRID_Y(x)     (UI_FOOTER_START_LINE+x*(UI_FOOTER_GRID_HEIGH)+2)
    
#ifdef	__cplusplus
}
#endif

#endif	/* UI_H */

