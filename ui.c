#include "ui.h"
#include "DAACED.h"

void print_line_with_shots_and_split(uint8_t shot_no, time_t split) {
    char message[20];
    double s;
    uint8_t x_offset;
    sprintf(message, "#%03d", shot_no);
    lcd_write_string(message, 0, UI_COUNTER_START_LINE + BigFont->height, MediumFont, BLACK_OVER_WHITE);

    x_offset = lcd_string_lenght(message, MediumFont)+7;
    s = 0.001 * split;
    sprintf(message, "Split %.2f", s);
    
    lcd_write_string(
            message,
            x_offset,
            UI_COUNTER_START_LINE + BigFont->height, MediumFont,
            BLACK_OVER_WHITE
            );
}

void print_big_time_label(time_t t) {
    char message[7];
    sprintf(message, "%3.2f", ((float) t)/ 1000);
    lcd_write_string(message, 0, UI_COUNTER_START_LINE, BigFont, BLACK_OVER_WHITE);
}

void update_countdown_time_on_screen() {
    print_big_time_label(DelayTime);
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
            DoPowerOn();
            StopTimer();
            ui_state = TimerIdle;
            break;
        default:
            //do nothing - we're sleeping;
            break;
    }
}

void handle_timer_idle() {
    switch (comandToHandle) {
        case StartLong:
            PowerOffTimer();
            ui_state = PowerOff;
            break;
        case StartShort:
            StartTimer();
            ui_state = TimerCountdown;
            break;
        case ReviewShort:
            StartReviewScreen();
            ui_state = ReviewScreen;
            break;
        case ReviewLong:
            StartSettingsMenuScreen();
            ui_state = SettingsScreen;
            break;
        default:
            //All the rest ignoring
            break;
    }
    update_shot_time_on_screen();
}

void HandleTimerEvents() {
    switch (timerEventToHandle) {
        case TimerTimeout:
            StopTimer();
            ui_state = TimerIdle;
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
            PowerOffTimer();
            ui_state = PowerOff;
            break;
        case StartShort:
            if (AutoStart) {
                StartTimer();
                ui_state = TimerListening;
            }
            break;
        case ReviewShort:
            StopTimer();
            ui_state = TimerIdle;
            break;
        default:
            // All the rest keys handled inside the next handler.
            // As well as shoot events
            HandleTimerEvents();
            
            break;
    }
    update_shot_time_on_screen();
}

void handle_review_screen() {
    switch (comandToHandle) {
        case StartLong:
            PowerOffTimer();
            ui_state = PowerOff;
            break;
        case StartShort:
            StopTimer();
            ui_state = TimerIdle;
            break;
        case ReviewShort:
            NextReviewItem();
            ui_state = ReviewScreen;
            break;
        case ReviewLong:
            StartSettingsMenuScreen();
            ui_state = SettingsScreen;
            break;
        default:
            //All the rest ignoring
            break;
    }
}

void handle_settings_screen() {
    switch (comandToHandle) {
        case StartLong:
            PowerOffTimer();
            ui_state = PowerOff;
            break;
        case StartShort:
            StopTimer();
            ui_state = TimerIdle;
            break;
        case ReviewShort:
            StartReviewScreen();
            ui_state = ReviewScreen;
            break;
        default:
            //All the rest ignoring
            break;
    }
}

void handle_countdown() {
    switch (comandToHandle) {
        case StartLong:
            PowerOffTimer();
            ui_state = PowerOff;
            break;
        case StartShort:
            if (AutoStart) {
                StartTimer();
                ui_state = TimerListening;
            }
            break;
        case ReviewShort:
            StopTimer();
            ui_state = TimerIdle;
            break;
        case CountdownExpired:
            PlayStartSound();
            ui_state = TimerListening;
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