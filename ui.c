#include "ui.h"
#include "DAACED.h"

void print_line_with_shots_and_split(uint8_t shot_no, time_t split) {
    char message[20];
    uint8_t x_pos = 0;
    uint8_t y_pos = UI_HEADER_END_LINE + BigFont->height;
    sprintf(message, "#%03d ", shot_no);
    lcd_write_string(message, x_pos, y_pos, SmallFont, BLACK_OVER_WHITE);

    sprintf(message, "  Split %3.2f", (float)split/1000);
    x_pos = LCD_WIDTH - lcd_string_lenght(message, SmallFont);
    lcd_write_string(
            message,
            x_pos,
            y_pos, SmallFont,
            BLACK_OVER_WHITE
            );
}

uint8_t old_time_str_len = 0;

void print_big_time_label(time_t t) {
    char message[16];
    sprintf(message, "%3.02f", ((float) t) / 1000);
    uint8_t len = lcd_string_lenght(message, BigFont);
    if (len < old_time_str_len)
        lcd_clear_block(LCD_WIDTH-old_time_str_len, UI_HEADER_END_LINE, LCD_WIDTH, BigFont->height + UI_HEADER_END_LINE);
    old_time_str_len = len;
    lcd_write_string(message, LCD_WIDTH - len, UI_HEADER_END_LINE, BigFont, BLACK_OVER_WHITE);
}
void update_countdown_time_on_screen() {
    time_t reminder = DelayTime - rtc_time.unix_time_ms + countdown_start_time;
    print_big_time_label(reminder / 100 * 100);
}

void PowerOffTimer() {
    set_screen_title("Power Off");
    DoPowerOff();
}

void StartTimer() {
    lcd_clear_data_ram();
    CurPar_idx = 0;
    StartCountdownTimer();
    DoMain();
}

void StopTimer() {
    lcd_clear();
    CurPar_idx = 0;
    print_header();
    print_footer();
    update_shot_time_on_screen();
    timer_idle_last_action_time = rtc_time.unix_time_ms;
}

void handle_charger_connected() {
    switch (comandToHandle) {
        case ReviewLong:
        case OkLong:
        case StartLong:STATE_HANDLE_POWER_ON;
            break;
        case ChargerEvent:
            DoCharging();
            break;
        default:
            break;
    }
}

void handle_power_off() {
    switch (comandToHandle) {
        case ReviewLong:
        case OkLong:
        case StartLong:STATE_HANDLE_POWER_ON;
            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            //do nothing - we're sleeping;
            break;
    }
    comandToHandle = None;
}

void handle_timer_idle_shutdown() {
    if (comandToHandle != None) {
        timer_idle_last_action_time = rtc_time.unix_time_ms;
    } else if (rtc_time.unix_time_ms - timer_idle_last_action_time >= timer_idle_shutdown_timeout) {
        comandToHandle = StartLong;
    }
}

void handle_timer_idle() {
    set_screen_title("Timer Idle");
    update_shot_time_on_screen();
    print_header();
    print_footer();
    handle_timer_idle_shutdown();
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_COUNTDOWN;
            break;
        case ReviewShort:STATE_HANDLE_REVIEW_SCREEN;
            break;
        case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN;
            break;
        case UpLong:
        case UpShort:
            lcd_increase_contrast();
            break;
        case DownLong:
        case DownShort:
            lcd_decrease_contrast();
            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            //All the rest ignoring
            break;
    }
    comandToHandle = None;
}

void HandleTimerEvents() {
    switch (timerEventToHandle) {
        case TimerTimeout:
            saveShootString();
            STATE_HANDLE_TIMER_IDLE;
            break;
        case ParEvent:
            if (TotPar > 0)
                StartParTimer();
            PlayParSound();
            break;
            // By default do nothing
    }
    timerEventToHandle = NoEvent;
}

void handle_timer_listening() {
    set_screen_title("Listening");
    update_shot_time_on_screen();

    print_header();
    //    DoAdcGraph();
    print_footer();
    switch (comandToHandle) {
        case StartLong:
            saveShootString();
            STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:
            if (AutoStart) {
                STATE_HANDLE_COUNTDOWN;
            } else saveShootString();
            break;
        case ReviewLong:
        case ReviewShort:
            saveShootString();
            STATE_HANDLE_REVIEW_SCREEN;
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
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
        case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN;
            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            DoReview();
            break;
    }
    comandToHandle = None;
}

void handle_settings_screen() {
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
        case ReviewShort:STATE_HANDLE_REVIEW_SCREEN;
            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            //All the rest ignoring

            break;
    }
    comandToHandle = None;
}

void handle_countdown() {
    set_screen_title("All Set");
    print_footer();
    print_header();
    update_countdown_time_on_screen();
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:
            if (AutoStart)
                STATE_HANDLE_TIMER_IDLE;
            break;
        case ReviewShort:STATE_HANDLE_TIMER_IDLE;
            break;
        case CountdownExpired:
            ui_state = TimerListening;
            update_shot_time_on_screen();

            if (TotPar > 0)
                StartParTimer();
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
    //TODO: try save memory taking lower byte to save memory...
    time_t press_time = rtc_time.unix_time_ms;
    time_t duration = 0;
    do {
        duration = rtc_time.unix_time_ms - press_time;
        if (duration > STICKY_THRESHOLD_SEC)
            return duration >= LONG_PRESS_THRESHOLD_SEC;
    } while (Keypressed);
    KeyReleased = true; // Mark key released only here to avoid double sensing of key press
    return duration >= LONG_PRESS_THRESHOLD_SEC;
}

TBool is_long_press_repeatable() {
    time_t press_time = rtc_time.unix_time_ms;
    time_t duration = 0;
    do {
        duration = rtc_time.unix_time_ms - press_time;
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
                // TODO: Remove when device fixed or for production
                if (is_long_press())
                    comandToHandle = StartLong;
                else
                    comandToHandle = StartShort;
                break;
            case KeyInUp:

                break;
            default:

                //user can press anything, but we can handle only specific gestures
                break;
        }
    } else {
        define_charger_state();
        if (charger_state_changed)
            comandToHandle = ChargerEvent;
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
        case ChargerScreen:
            handle_charger_connected();
            break;
        default:
            //We should never get here, nothing to do.
            break;
    }
}