#include "ui.h"
#include "DAACED.h"

void print_line_with_shots_and_split(uint8_t shot_no, time_t split) {
    char message[20];
    double s;
    uint8_t x_pos = 0;
    uint8_t y_pos = UI_COUNTER_START_LINE + BigFont->height;
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
    if(orientation == ORIENTATION_NORMAL)
        lcd_write_string_d(message, 0, UI_COUNTER_START_LINE, BigFont, BLACK_OVER_WHITE);
    else
        lcd_write_string_d(message, LCD_WIDTH, LCD_HEIGHT - UI_COUNTER_START_LINE, BigFont, BLACK_OVER_WHITE);
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
    lcd_clear();
    set_screen_title("Timer Run");
    CurPar_idx = 0;
    StartParTimer();
    StartCountdownTimer();
    DoMain();
}

void StartReviewScreen() {
    set_screen_title("Review");
    DoReview();
}

void StartSettingsMenuScreen() {
    set_screen_title("Settings");
    DoSettings();
}

void StopTimer() {
    set_screen_title("Timer Idle");
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
}

void handle_timer_idle() {
    switch (comandToHandle) {
        case StartLong:
            ui_state = PowerOff;
            PowerOffTimer();
            break;
        case StartShort:
            ui_state = TimerCountdown;
            StartTimer();            
            break;
        case ReviewShort:
            ui_state = ReviewScreen;
            StartReviewScreen();            
            break;
        case ReviewLong:
            ui_state = SettingsScreen;
            StartSettingsMenuScreen();
            break;
        default:
            //All the rest ignoring
            break;
    }
    update_shot_time_on_screen();
    print_header();
    print_footer();
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
    update_shot_time_on_screen();
    print_header();
    print_footer();
}

void handle_review_screen() {
    switch (comandToHandle) {
        case StartLong:
            ui_state = PowerOff;
            PowerOffTimer();
            break;
        case StartShort:
            ui_state = TimerIdle;
            StopTimer();
            break;
        case ReviewShort:
            ui_state = ReviewScreen;
            NextReviewItem();
            break;
        case ReviewLong:
            ui_state = SettingsScreen;
            StartSettingsMenuScreen();
            break;
        default:
            //All the rest ignoring
            break;
    }
}

void handle_settings_screen() {
    switch (comandToHandle) {
        case StartLong:
            ui_state = PowerOff;
            PowerOffTimer();
            break;
        case StartShort:
            ui_state = TimerIdle;
            StopTimer();            
            break;
        case ReviewShort:
            ui_state = ReviewScreen;
            StartReviewScreen();            
            break;
        default:
            //All the rest ignoring
            
            break;
    }
}

void handle_countdown() {
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
            update_countdown_time_on_screen();
            break;
    }
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
                break;
            case KeyDw:
                break;
            case KeyUp:
                break;
            case KeyIn:
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
    comandToHandle = None;
}