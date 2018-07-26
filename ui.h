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

#define LONG_PRESS_THRESHOLD_SEC    1300
#define STICKY_THRESHOLD_SEC        1600

    typedef enum {
        PowerOff = 0,
        ChargerScreen,
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
        OkLong,
        ChargerEvent,
        TimeChanged
    } ButtonCommand;
    volatile ButtonCommand comandToHandle = None;

    typedef enum TimerEvent {
        NoEvent = 0,
        TimerTimeout,
        ParEvent,
        ShotDetected
    } TimerEvent;
    TimerEvent timerEventToHandle = NoEvent;

    char ScreenTitle[32];
#define set_screen_title(x) {strmycpy(ScreenTitle, x);}
#define clear_screen_title {for(uint8_t i=0;i<32;i++){ScreenTitle[i] = ' ';}}

    void print_big_time_label(uint24_t t);
    void print_line_with_shots_and_split(uint8_t shot_no, time_t split);
    void handle_settings_screen();
    void handle_review_screen();
    void define_input_action();
    void handle_timer_idle_shutdown();
    void handle_ui();
    void StopTimer();
#define STATE_HANDLE_POWER_OFF          {ui_state = PowerOff;set_backlight(0);lcd_clear();}
#define STATE_HANDLE_POWER_ON           {ui_state = TimerIdle;DoPowerOn();StopTimer();}
#define STATE_HANDLE_TIMER_IDLE         {ui_state = TimerIdle;StopTimer();}
#define STATE_HANDLE_REVIEW_SCREEN      {ui_state = ReviewScreen;lcd_clear();}
#define STATE_HANDLE_SETTINGS_SCREEN    {ui_state = SettingsScreen;lcd_clear();}
#define STATE_HANDLE_COUNTDOWN          {ui_state = TimerCountdown;lcd_clear();StartCountdownTimer();}
#define STATE_HANDLE_CHARGING           {ui_state = ChargerScreen;set_backlight(0);}
#ifdef	__cplusplus
}
#endif

#endif	/* UI_H */

