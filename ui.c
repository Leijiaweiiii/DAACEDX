#include "ui.h"
#include "DAACED.h"

void set_screen_title(char * value){
    strcpy(ScreenTitle,value);
}
void PowerOffTimer() {
    set_screen_title("Power Off");
    DoPowerOff();
}

void handle_shooting_events(){
    
}

void StartTimer() {
    set_screen_title("Timer Run");
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
    DoPowerOn();
}

void NextReviewItem() {
    set_screen_title("Review-N");
}

void handle_power_off() {
    switch (comandToHandle) {
        case StartLong:
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
            ui_state = TimerCounting;
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
}



void handle_timer_counting() {
    switch (comandToHandle) {
        case StartLong:
            PowerOffTimer();
            ui_state = PowerOff;
            break;
        case StartShort:
            if (Autostart) {
                StartTimer();
                ui_state = TimerCounting;
            }
            break;
        case ReviewShort:
            StopTimer();
            ui_state = TimerIdle;
            break;
        default:
            // All the rest keys handled inside the next handler.
            // As well as shoot events
            handle_shooting_events();
            break;
    }
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
TBool is_long_press(){
    //taking lower byte to save memory...
    uint8_t press_time = LSB(rtc_time_sec);
    uint8_t duration = press_time;
    do {
        duration = rtc_time_sec-press_time;
        delay_rtc_ms(100);
        if(duration > STICKY_THRESHOLD_SEC)
            return duration >= LONG_PRESS_THRESHOLD_SEC;
    } while (Keypressed);
    KeyReleasedBefore = true;
    return duration >= LONG_PRESS_THRESHOLD_SEC;
}

void define_input_action() {
    if (KeyReleasedBefore && Keypressed) {
        KeyReleasedBefore = false;
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
            case  KeyBk:
                break;
            case  KeyDw:
                break;
            case  KeyUp:
                break;
            case  KeyIn:
                break;
            case  KeyInDw:
                break;
            case  KeyInUp:
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
        case TimerCounting:
            handle_timer_counting();
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