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

#define LONG_PRESS_THRESHOLD_SEC    1000
#define STICKY_THRESHOLD            1200

    typedef enum {
        PowerOff = 0,
        TimerIdle,
        TimerListening,
        TimerCountdown,
        ReviewScreen,
        SettingsScreen,
        TimerCharging
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
        TimeChanged,
        ChargerConnected,
        OpenCountdown
    } ButtonCommand;
    volatile ButtonCommand comandToHandle = None;

    typedef enum TimerEvent {
        NoEvent = 0,
        TimerTimeout,
        ParEvent,
        AutoParEvent,
        AutoParCompletedEvent,
        RepetitiveParEvent,
        BianchiParEvent,
        ShotDetected
    } TimerEvent;
    TimerEvent timerEventToHandle = NoEvent;

    char ScreenTitle[32];
    uint8_t old_time_str_len = 0;
#define set_screen_title(x) {sprintf(ScreenTitle," %s ", x);}
#define clear_screen_title {for(uint8_t i=0;i<32;i++){ScreenTitle[i] = ' ';}}
#define clear_timer_area { lcd_clear_block(0, UI_HEADER_END_LINE, 0, BigFont->height + MediumFont->height); }
    
    void print_big_time_label(uint24_t t);
    void print_line_with_shots_and_split(uint8_t shot_no, time_t split);
    void handle_settings_screen();
    void handle_review_screen();
    void define_input_action();
    void handle_timer_idle_shutdown();
    void handle_ui();
    void StopTimer();

    void STATE_HANDLE_POWER_OFF();
    void STATE_HANDLE_POWER_ON();
    void STATE_HANDLE_TIMER_IDLE();
    void STATE_HANDLE_REVIEW_SCREEN();
    void STATE_HANDLE_SETTINGS_SCREEN();
    void STATE_HANDLE_COUNTDOWN();
    void STATE_HANDLE_CHARGER();
#ifdef	__cplusplus
}
#endif

#endif	/* UI_H */

