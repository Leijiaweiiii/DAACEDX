#include "ui.h"
#include "DAACED.h"

void print_line_with_shots_and_split(uint8_t shot_no, time_t split) {
    char message[20];
    double s;
    uint8_t x_pos = 0;
    uint8_t y_pos = UI_HEADER_END_LINE + BigFont->height;
    sprintf(message, "#%03d", shot_no);
    lcd_write_string_d(message, x_pos, y_pos, SmallFont, BLACK_OVER_WHITE);

    x_pos = lcd_string_lenght(message, SmallFont) + 7;
    s = 0.001 * split;
    sprintf(message, "Split %.2f", s);
    
    lcd_write_string_d(
            message,
            x_pos,
            y_pos, SmallFont,
            BLACK_OVER_WHITE
            );
}

void print_big_time_label(time_t t) {
    char message[16];
    time_t sec = t/1000;
    time_t ms = sec * 1000;
    ms = t - ms;
    sprintf(message, "%.02f ", 0.001 * ms + sec);
    lcd_write_string_d(message, 0, UI_HEADER_END_LINE, BigFont, BLACK_OVER_WHITE);
}

void update_countdown_time_on_screen() {
    time_t reminder = DelayTime - get_corrected_time_msec() + countdown_start_time;
    print_big_time_label(reminder /100 * 100);
}

void set_screen_title(char * value) {
    strcpy(ScreenTitle, value);
}

void PowerOffTimer() {
    set_screen_title("Power Off");
    DoPowerOff();
}

void StartTimer() {
    lcd_clear_data_ram();
    set_screen_title("Timer Run");
    CurPar_idx = 0;
    StartParTimer();
    StartCountdownTimer();
    DoMain();
}

void StopTimer() {
    set_screen_title("Timer Idle");
    lcd_clear();
    print_header();
    print_footer();
    update_shot_time_on_screen();
}

void NextReviewItem() {
    set_screen_title("Review-N");
}

void handle_power_off() {
    switch (comandToHandle) {
        case StartLong:
            ui_state = TimerIdle;
            DoPowerOn();
            StopTimer();
            break;
        default:
            //do nothing - we're sleeping;
            break;
    }
    comandToHandle = None;
}

void handle_timer_idle() {
    update_shot_time_on_screen();
    print_header();
    print_footer();
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF;break;
        case StartShort:STATE_HANDLE_COUNTDOWN;break;
        case ReviewShort:STATE_HANDLE_REVIEW_SCREEN;break;
        case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN;break;
        default:
            //All the rest ignoring
            break;
    }
    comandToHandle = None;
}

void HandleTimerEvents() {
    switch (timerEventToHandle) {
        case TimerTimeout:
            ui_state = TimerIdle;
            StopTimer();            
            break;
        case ParEvent:
            StartParTimer();
            break;
            // By default do nothing
    }
    timerEventToHandle = NoEvent;
}

void handle_timer_listening() {
    update_shot_time_on_screen();
    print_header();
    print_footer();
    switch (comandToHandle) {
        case StartLong:
            ui_state = PowerOff;
            PowerOffTimer();
            break;
        case StartShort:
            if (AutoStart) {
                ui_state = TimerListening;
                StartTimer();                
            }
            break;
        case ReviewShort:
            ui_state = TimerIdle;
            StopTimer();
            break;
        default:
            // All the rest keys handled inside the next handler.
            // As well as shoot events
            HandleTimerEvents();       
            break;
    }
    comandToHandle = None;
}

void handle_review_screen() {
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF;break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;break;
        case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN;break;
        default:
            //All the rest ignoring
            break;
    }
    comandToHandle = None;
}

void handle_settings_screen() {
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF;break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;break;
        case ReviewShort:STATE_HANDLE_REVIEW_SCREEN;break;
        default:
            //All the rest ignoring
            
            break;
    }
    comandToHandle = None;
}

void handle_countdown() {
    print_footer();
    print_header();
    update_countdown_time_on_screen();
    switch (comandToHandle) {
        case StartLong:
            ui_state = PowerOff;
            PowerOffTimer();
            break;
        case StartShort:
            if (AutoStart) {
                ui_state = TimerListening;
                StartTimer();                
            }
            break;
        case ReviewShort:
            ui_state = TimerIdle;
            StopTimer();            
            break;
        case CountdownExpired:
            ui_state = TimerListening;
            PlayStartSound();
            break;
        default:
            // All the rest keys handled inside the next handler.
            // As well as shoot events
            break;
    }
    comandToHandle = None;
}

TBool is_long_press() {
    //taking lower byte to save memory...
    uint8_t press_time = LSB(rtc_time_sec);
    uint8_t duration = 0;
    do {
        duration = rtc_time_sec - press_time;
        //        delay_rtc_ms(100);
        if (duration > STICKY_THRESHOLD_SEC)
            return duration >= LONG_PRESS_THRESHOLD_SEC;
    } while (Keypressed);
    KeyReleased = true; // Mark key released only here to avoid double sensing of key press
    return duration >= LONG_PRESS_THRESHOLD_SEC;
}
TBool is_long_press_repeatable() {
    //taking lower byte to save memory...
    uint8_t press_time = LSB(rtc_time_sec);
    uint8_t duration = 0;
    KeyReleased = false; // Mark key released only here to avoid double sensing of key press
    do {
        duration = rtc_time_sec - press_time;
        //        delay_rtc_ms(100);
        if (duration > STICKY_THRESHOLD_SEC)
            return duration >= LONG_PRESS_THRESHOLD_SEC;
    } while (Keypressed);
    
    return duration >= LONG_PRESS_THRESHOLD_SEC;
}

void define_input_action() {
    if (KeyReleased && Keypressed) {
        KeyReleased = false;
        switch (Key) {
            case KeyRw:
                if (is_long_press())
                    comandToHandle = ReviewLong;
                else
                    comandToHandle = ReviewShort;
                break;
            case KeySt:
                if (is_long_press())
                    comandToHandle = StartLong;
                else
                    comandToHandle = StartShort;
                break;
            case KeyBk:
                if (is_long_press())
                    comandToHandle = BackLong;
                else
                    comandToHandle = BackShort;
                break;
            case KeyDw:
                if (is_long_press_repeatable())
                    comandToHandle = DownLong;
                else
                    comandToHandle = DownShort;
                break;
            case KeyUp:
                if (is_long_press_repeatable())
                    comandToHandle = UpLong;
                else
                    comandToHandle = UpShort;
                break;
            case KeyIn:
                if (is_long_press())
                    comandToHandle = OkLong;
                else
                    comandToHandle = OkShort;
                break;
            case KeyInDw:
                break;
            case KeyInUp:
                break;
            default:
                //user can press anything, but we can handle only specific gestures
                break;
        }
    }
}

void handle_ui() {
    define_input_action();
    switch (ui_state) {
        case PowerOff:
            handle_power_off();
            break;
        case TimerIdle:
            handle_timer_idle();
            break;
        case TimerCountdown:
            handle_countdown();
            break;
        case TimerListening:
            handle_timer_listening();
            break;
        case ReviewScreen:
            handle_review_screen();
            break;
        case SettingsScreen:
            handle_settings_screen();
            break;
        default:
            //We should never get here, nothing to do.
            break;
    }
}