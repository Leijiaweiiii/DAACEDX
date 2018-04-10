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
        BackLong,
        OkShort,
        OkLong
    } ButtonCommand;
    volatile ButtonCommand comandToHandle = None;

    typedef enum TimerEvent{
        NoEvent = 0,TimerTimeout,ParEvent
    } TimerEvent;
    TimerEvent timerEventToHandle = NoEvent;

    char ScreenTitle[32];
    void set_screen_title(char * value);
    void print_big_time_label(time_t t);
    void print_line_with_shots_and_split(uint8_t shot_no,time_t split);
    void handle_settings_screen();
    void handle_review_screen();
    void define_input_action();
    void handle_ui();
    void PowerOffTimer();
    void StopTimer();
#define STATE_HANDLE_POWER_OFF          {ui_state = PowerOff;lcd_clear();PowerOffTimer();}
#define STATE_HANDLE_TIMER_IDLE         {ui_state = TimerIdle;lcd_clear();StopTimer();}
#define STATE_HANDLE_REVIEW_SCREEN      {ui_state = ReviewScreen;lcd_clear();DoReview();}
#define STATE_HANDLE_SETTINGS_SCREEN    {ui_state = SettingsScreen;lcd_clear();DoSettings();}
#define STATE_HANDLE_COUNTDOWN          {ui_state = TimerCountdown;lcd_clear();StartTimer();}
#ifdef	__cplusplus
}
#endif

#endif	/* UI_H */

