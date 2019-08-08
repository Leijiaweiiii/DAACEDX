#include "ui.h"
#include "DAACED.h"

// These may be macros but moved here for space optimization
void STATE_HANDLE_POWER_OFF()          {if(ui_state != PowerOff || ui_state != TimerCharging) PowerOffSound(); ui_state = PowerOff;}
void STATE_HANDLE_POWER_ON()           {ui_state = TimerIdle;DoPowerOn();StopTimer();}
void STATE_HANDLE_TIMER_IDLE()         {ui_state = TimerIdle;StopTimer();}
void STATE_HANDLE_REVIEW_SCREEN()      {ui_state = ReviewScreen;lcd_clear();}
void STATE_HANDLE_SETTINGS_SCREEN()    {ui_state = SettingsScreen;lcd_clear();}
void STATE_HANDLE_COUNTDOWN()          {ui_state = TimerCountdown;lcd_clear();StartCountdownTimer();}
void STATE_HANDLE_CHARGER()            {ui_state = TimerCharging;set_backlight(0);lcd_clear();}


void print_line_with_shots_and_split(uint8_t shot_no, time_t split) {
    char message[20];
    uint8_t x_pos = 0;
    uint8_t y_pos = UI_HEADER_END_LINE + BigFont->height;
    sprintf(message, "#%03d ", shot_no);
    lcd_write_string(message, x_pos, y_pos, SmallFont, BLACK_OVER_WHITE);

    sprintf(message, "  Split %3.2f", (float) split / 1000);
    x_pos = LCD_WIDTH - lcd_string_lenght(message, SmallFont);
    lcd_write_string(
            message,
            x_pos,
            y_pos, SmallFont,
            BLACK_OVER_WHITE
            );
}

void print_shot_origin(const shot_t * s) {
    uint8_t y_pos = UI_HEADER_END_LINE + LCD_PAGE_HEIGTH;
    if (s->is_mic) return;
    if (s->is_a) {
        lcd_write_string(
                "A",
                3,
                y_pos,
                MediumFont,
                BLACK_OVER_WHITE
                );
        lcd_clear_block(3, y_pos + MediumFont->height, 35, y_pos + 2 * MediumFont->height);
        return;
    }
    if (s->is_b) {
        lcd_clear_block(3, y_pos, 35, y_pos + MediumFont->height);
        lcd_write_string(
                "B",
                3,
                y_pos + MediumFont->height,
                MediumFont,
                BLACK_OVER_WHITE
                );
        return;
    }
}

void update_shot_time_on_screen() {
    uint24_t t = 0;
    uint8_t index = get_shot_index_in_arr(top_shot_index());
    t = ShootString.shots[index].dt;
    print_shot_origin(&(ShootString.shots[index]));
    print_big_time_label(t);
}

void print_big_time_label(const uint24_t t) {
    char message[16];
    float tf;
    uint8_t len, spaceholder, height;
    height = BigFont->height;
    spaceholder = (Settings.InputType != INPUT_TYPE_Microphone) ? 40 : 0;
    if (t > MAX_MEASUREMENT_TIME)
        tf = 999.0;
    else
        tf = ((float) t) / 1000;
    sprintf(message, "%3.02f", tf);
    len = lcd_string_lenght(message, BigFont);
    if (len < old_time_str_len)
        lcd_clear_block(LCD_WIDTH - old_time_str_len, UI_HEADER_END_LINE, LCD_WIDTH, height + UI_HEADER_END_LINE);
    old_time_str_len = len;
    if (len < LCD_WIDTH - spaceholder) {
        lcd_write_string(message, LCD_WIDTH - len, UI_HEADER_END_LINE, BigFont, BLACK_OVER_WHITE);
    } else {
        lcd_clear_block(LCD_WIDTH - len, UI_HEADER_END_LINE, LCD_WIDTH, height + UI_HEADER_END_LINE);
        len = lcd_string_lenght(message, MediumFont);
        lcd_write_string(message, LCD_WIDTH - len, UI_HEADER_END_LINE + 16, MediumFont, BLACK_OVER_WHITE);
    }
}

void update_countdown_time_on_screen() {
    update_rtc_time();
    uint24_t reminder = runtimeDelayTime - unix_time_ms + countdown_start_time;
    print_big_time_label(reminder);
}

void StopTimer() {
    lcd_clear();
    InputFlags.FOOTER_CHANGED = True;
    ParFlags.ParNowCounting = False;
    ParFlags.AutoParOverDetect = False;
}

void handle_charger_connected() {
    if (comandToHandle == StartLong) STATE_HANDLE_POWER_ON();
    else DoCharging();
}

void handle_power_off() {
    switch(comandToHandle){
        case StartLong:
            STATE_HANDLE_POWER_ON();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            DoPowerOff();
            break;
    }
    comandToHandle = None;
}

void handle_timer_idle_shutdown() {
    if (!Settings.AR_IS.AutoPowerOff) return;
    update_rtc_time();
    time_t inactive_time;
    if (comandToHandle != None || ui_state == TimerListening) {
        timer_idle_last_action_time = unix_time_ms_sec;
        set_backlight(Settings.BackLightLevel);
        return;
    }
    inactive_time = unix_time_ms_sec - timer_idle_last_action_time;
    if (inactive_time > timer_idle_shutdown_timeout) {
        STATE_HANDLE_POWER_OFF();
        return;
    }
    if (inactive_time > timer_idle_dim_timeout) {
        set_backlight(0);
    }
}

void handle_timer_idle() {
    if (Settings.ParMode == ParMode_Regular) {
        switch (Settings.InputType) {
            case INPUT_TYPE_Microphone:
                set_screen_title("      Mic ");
                break;
            case INPUT_TYPE_A_and_B_single:
                set_screen_title("A+B single");
                break;
            case INPUT_TYPE_A_or_B_multiple:
                set_screen_title("A - B multi");
                break;
        }
    } else {
        set_screen_title(par_mode_header_names[Settings.ParMode]);
    }

    update_shot_time_on_screen();
    print_header(false);
    print_footer();
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:
            if(Settings.ParMode == ParMode_AutoPar) CurPar_idx = 0;
            STATE_HANDLE_COUNTDOWN();
            break;
        case ReviewShort:STATE_HANDLE_REVIEW_SCREEN();
            break;
        case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN();
            break;
        case UpLong:
        case UpShort:
            switch (Settings.ParMode){
                case ParMode_Regular:
                case ParMode_AutoPar:
                case ParMode_Repetitive:
                case ParMode_Silent:
                case ParMode_Spy:
                    // Don't increment par for these modes
                    break;
                default:
                    increment_par();
                    break;
            }
            break;
        case DownLong:
        case DownShort:
            switch (Settings.ParMode){
                case ParMode_Regular:
                case ParMode_AutoPar:
                case ParMode_Repetitive:
                case ParMode_Silent:
                case ParMode_Spy:
                    // Don't decrement par for these modes
                    break;
                default:
                    decrement_par();
                    break;
            }
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        case OpenCountdown:
            STATE_HANDLE_SETTINGS_SCREEN();
            SetCountDown();
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
            STATE_HANDLE_TIMER_IDLE();
            break;
        case ParEvent:
            // turn light ON on PAR sound
            update_rtc_time();
            timer_idle_last_action_time = unix_time_ms_sec;
            StartPlayParSound();
            sendSignal("PAR", Settings.BuzzerParDuration, (long) (Settings.ParTime[CurPar_idx] * 1000));
            next_par_ms = (long) (Settings.ParTime[CurPar_idx] * 1000);
            CurPar_idx++;
            if (Settings.TotPar > CurPar_idx)
                StartParTimer();
            break;
        case AutoParEvent:
            StartPlayParSound();
            sendSignal("PAR", Settings.BuzzerParDuration, (long) (Settings.AutoPar[CurPar_idx].par * 1000));
            next_par_ms = AUTO_PAR_OVER_DETECT_MS;
            ParFlags.AutoParOverDetect = True;
            ParFlags.ParNowCounting = False;
            break;
        case AutoParCompletedEvent:
            CurPar_idx++;
            saveShootString();
            if (Settings.TotAutoPar > CurPar_idx) {
                STATE_HANDLE_COUNTDOWN();
            } else {
                STATE_HANDLE_TIMER_IDLE();
                NOP();
            }
            break;
        case RepetitiveParEvent:
            StartPlayParSound();
            sendSignal("PAR", Settings.BuzzerParDuration, repetitive_state == Face ? Settings.RepetitiveEdgeTime : Settings.RepetitiveFaceTime);
            if(repetitive_state == Face){
                repetitive_state = Edge;
                next_par_ms = Settings.RepetitiveEdgeTime;
            } else {
                repetitive_state = Face;
                next_par_ms = Settings.RepetitiveFaceTime;
                repetitive_counter++;
            }
            if(repetitive_counter < Settings.RepetitiveRepeat)
                StartParTimer();
            break;
        case BianchiParEvent:
            StartPlayParSound();
            sendSignal("PAR", Settings.BuzzerParDuration, (long) (Settings.ParTime[CurPar_idx] * 1000));
            next_par_ms = (long)Settings.ParTime[CurPar_idx] * 1000;
            increment_par();
            break;
    }
    timerEventToHandle = NoEvent;
}

void handle_timer_listening() {
    update_shot_time_on_screen();
    print_header(false);
    print_footer();

    switch (comandToHandle) {
        case StartLong:
            saveShootString();
            STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:
            if (AutoStart) {
                saveShootString();
                if(Settings.ParMode == ParMode_AutoPar) CurPar_idx = 0;
                STATE_HANDLE_COUNTDOWN();
            }
            break;
        case ReviewLong:
        case ReviewShort:
            saveShootString();
            STATE_HANDLE_REVIEW_SCREEN();
            break;
        case ChargerConnected:
            saveShootString();
            STATE_HANDLE_CHARGER();
            break;
        default:
            // All the rest keys handled inside the next handler.
            // As well as shoot events
            check_timer_max_time();
            HandleTimerEvents();
            break;
    }
    comandToHandle = None;
}

void handle_review_screen() {
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE();
            break;
        case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            DoReview();
            break;
    }
    comandToHandle = None;
}

void handle_settings_screen() {
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE();
            break;
        case ReviewShort:STATE_HANDLE_REVIEW_SCREEN();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            DoSettings();
            break;
    }
    comandToHandle = None;
}

void handle_countdown() {
    print_footer();
    print_header(false);
    update_countdown_time_on_screen();
    check_countdown_expired();
    switch (comandToHandle) {
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:
            if(Settings.ParMode == ParMode_AutoPar) CurPar_idx = 0;
            STATE_HANDLE_COUNTDOWN();
            break;
        case ReviewShort:
            getShootString(0);
            STATE_HANDLE_TIMER_IDLE();
            break;
        case CountdownExpired:
            ui_state = TimerListening;
            StartListenShots();
            update_shot_time_on_screen();
            switch(Settings.ParMode){
                case ParMode_Repetitive:
                    StartParTimer();
                    break;
                case ParMode_AutoPar:
                    next_par_ms = (long)Settings.AutoPar[CurPar_idx].par * 1000;
                    StartParTimer();
                    break;
                default:
                    if (Settings.TotPar > 0)
                        StartParTimer();
                    break;
            }
            StartPlayStartSound();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            // All the rest keys handled inside the next handler.
            // As well as shoot events
            break;
    }
    Delay(1);
    comandToHandle = None;
}

TBool is_long_press(TBool repeatable) {
    uint16_t duration = 0;
    do {
        if (duration > STICKY_THRESHOLD - LongPressCount){
            LongPressCount += (LongPressCount <= STICKY_THRESHOLD - 100)? 300:0;
            InputFlags.KEY_RELEASED = repeatable;
            return true;
        }
        Delay(10);
        duration += 10;
    } while (Keypressed);
    InputFlags.KEY_RELEASED = repeatable;
    return duration >= LONG_PRESS_THRESHOLD_SEC;
}

void define_input_action() {
    if (InputFlags.KEY_RELEASED  && Keypressed) {
        InputFlags.KEY_RELEASED = False;
        switch (Key) {
            case KeyRw:
                if (is_long_press(False))
                    comandToHandle = ReviewLong;
                else
                    comandToHandle = ReviewShort;
                break;
            case KeySt:
                if (is_long_press(False))
                    comandToHandle = StartLong;
                else
                    comandToHandle = StartShort;
                break;
            case KeyBk:
                if (is_long_press(False))
                    comandToHandle = BackLong;
                else
                    comandToHandle = BackShort;
                break;
            case KeyDw:
                if (is_long_press(True))
                    comandToHandle = DownLong;
                else
                    comandToHandle = DownShort;
                break;
            case KeyUp:
                if (is_long_press(True))
                    comandToHandle = UpLong;
                else
                    comandToHandle = UpShort;
                break;
            case KeyIn:
                if (is_long_press(False))
                    comandToHandle = OkLong;
                else
                    comandToHandle = OkShort;
                break;
            case KeyInDw:

                break;
            case KeyInUp:
                comandToHandle = OpenCountdown;
                break;
            default:
                //user can press anything, but we can handle only specific gestures
                break;
        }
    }
    define_charger_state();
    if(charger_state != NotCharging)
        comandToHandle = ChargerConnected;
    else
        handle_timer_idle_shutdown();
}

void handle_low_battery() {
    if(!battery_low) return;
    if (ui_state != PowerOff) {
        char msg[16];
        STATE_HANDLE_POWER_OFF();
        lcd_clear();
        sprintf(msg, "Low Battery");
        lcd_write_string(msg, 40, UI_CHARGING_LBL_Y - MediumFont->height/2, MediumFont, BLACK_OVER_WHITE);
        sprintf(msg, "Please Charge");
        lcd_write_string(msg, 30, UI_CHARGING_LBL_Y + MediumFont->height/2, MediumFont, BLACK_OVER_WHITE);
        Stats.LowPower++;
        saveStats();
        Delay(3000);
    }
}

void handle_ui() {
    define_input_action();
    if(ui_state == PowerOff){
        handle_power_off();
        return;
    }
    handle_bt_commands();
    handle_low_battery();
    switch (ui_state) {
        case PowerOff:
            // Handled separately
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
        case TimerCharging:
            DoCharging();
            break;
        default:
            //We should never get here, nothing to do.
            break;
    }
}