#include "menu.h"
#include "lcd.h"
#include "ui.h"
#include "DAACED.h"
uint8_t old_label_len = 0;

void display_big_font_label(const char * msg) {
    uint8_t len = 0;
    len = lcd_string_lenght(msg, BigFont);
    if (len != old_label_len) {
        lcd_clear_block(
                (LCD_WIDTH - old_label_len) / 2,
                UI_HEADER_END_LINE + 24,
                (LCD_WIDTH - old_label_len) / 2 + old_label_len,
                UI_HEADER_END_LINE + 24 + BigFont->height);
        old_label_len = len;
    }
    lcd_write_string(msg, (LCD_WIDTH - len) / 2, UI_HEADER_END_LINE + 24, BigFont, BLACK_OVER_WHITE);
}

void DisplayTime(TimeSelection_t * t) {
    char msg[16];
    set_screen_title(t->MenuTitle);
    print_header();
    sprintf(msg, "%02d:%02d", t->hour, t->minute);
    display_big_font_label(msg);
}

void DisplayDouble(NumberSelection_t* s) {
    char msg[16];
    set_screen_title(s->MenuTitle);
    print_header();
    sprintf(msg, s->format, s->fvalue);
    display_big_font_label(msg);
}

void DisplayInteger(NumberSelection_t* s) {
    char msg[16];

    set_screen_title(s->MenuTitle);
    print_header();
    sprintf(msg, s->format, s->value);
    display_big_font_label(msg);
}

void DisplaySettings(SettingsMenu_t* sm) {
    uint8_t p, lineh;
    uint16_t i;
    set_screen_title(sm->MenuTitle);
    print_header();
    p = UI_HEADER_END_LINE;
    lineh = SmallFont->height;
    if (sm->redraw) {
        lcd_clear();
        sm->redraw = False;
    }

    // TODO: Move page calculations here to handle menu changes during menu operation
    for (i = MENU_PAGE_SIZE * sm->page; i < min(sm->TotalMenuItems, (MENU_PAGE_SIZE * (sm->page + 1))); i++) {
        if (sm->menu == i) {
            lcd_write_string(sm->MenuItem[i], 3, p, SmallFont, WHITE_OVER_BLACK);
        } else {
            lcd_write_string(sm->MenuItem[i], 3, p, SmallFont, BLACK_OVER_WHITE);
        }
        p += lineh;
    }
}

void decrement_menu_index(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu > 0) {
        s->menu--;
        s->page = ItemToPage(s->menu);
    } else Beep();
    s->redraw |= (oldPage != s->page);

}

void increment_menu_index(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu < s->TotalMenuItems - 1) {
        s->menu++;
        s->page = ItemToPage(s->menu);
    } else Beep();
    s->redraw |= (oldPage != s->page);
}

void decrement_menu_index_inf(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu > 0) {
        s->menu--;
    } else {
        s->menu = s->TotalMenuItems - 1;
    }
    s->page = ItemToPage(s->menu);
    s->redraw |= (oldPage != s->page);

}

void increment_menu_index_inf(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu < s->TotalMenuItems - 1) {
        s->menu++;
    } else {
        s->menu = 0;
    }
    s->page = ItemToPage(s->menu);
    s->redraw |= (oldPage != s->page);
}

void SelectMenuItem(SettingsMenu_t* s) {
    define_input_action();
    switch (comandToHandle) {
        case UpShort:
        case UpLong:
            decrement_menu_index(s);
            break;
        case ReviewShort:
        case DownShort:
        case DownLong:
            increment_menu_index(s);
            break;
        case OkShort:
        case OkLong:
            s->done = True;
            s->selected = True;
            break;
        case BackShort:
        case BackLong:
            s->selected = False;
            s->done = True;
            break;
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
            //        case ReviewShort:ui_state = ReviewScreen;
            //            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectBinaryMenuItem(SettingsMenu_t* s) {
    define_input_action();
    switch (comandToHandle) {
        case UpShort:
        case UpLong:
            decrement_menu_index(s);
            break;
        case ReviewShort:
        case DownShort:
        case DownLong:
            increment_menu_index(s);
            break;
        case OkShort:
        case OkLong:
            s->selected = True;
            s->done = False;
            break;
        case BackShort:
        case BackLong:
            s->selected = False;
            s->done = True;
            break;
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
            //        case ReviewShort:ui_state = ReviewScreen;
            //            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectMenuItemCircular(SettingsMenu_t* s) {
    define_input_action();
    switch (comandToHandle) {
        case UpShort:
        case UpLong:
            decrement_menu_index_inf(s);
            break;
        case ReviewShort:
        case DownShort:
        case DownLong:
            increment_menu_index_inf(s);
            break;
        case OkShort:
        case OkLong:
            s->selected = True;
            s->done = False;
            break;
        case BackShort:
        case BackLong:
            s->selected = False;
            s->done = True;
            break;
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
            //        case ReviewShort:ui_state = ReviewScreen;
            //            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectInteger(NumberSelection_t* sm) {
    define_input_action();
    switch (comandToHandle) {
        case UpShort:
            if (sm->value < sm->max) {
                sm->value += sm->step;
            } else Beep();
            break;
        case UpLong:
            if (sm->value < sm->max) {
                sm->value += sm->step;
                sm->value += sm->step;
            } else Beep();
            break;
        case DownShort:
            if (sm->value > sm->min) {
                sm->value -= sm->step;
            } else Beep();
            break;
        case DownLong:
            if (sm->value > sm->min) {
                sm->value -= sm->step;
                sm->value -= sm->step;
            } else Beep();
            break;
        case BackShort:
        case BackLong:
            sm->selected = False;
            sm->done = True;
            break;
        case OkShort:
        case OkLong:
            sm->done = True;
            sm->selected = True;
            break;
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
            //        case ReviewShort:ui_state = ReviewScreen;
            //            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectDouble(NumberSelection_t* sm) {
    define_input_action();
    switch (comandToHandle) {
        case UpShort:
            if (sm->fvalue < sm->fmax) {
                sm->fvalue += sm->fstep;
            } else Beep();
            break;
        case UpLong:
            for (uint8_t i = 0; i < 5; i++) {
                if (sm->fvalue < sm->fmax) {
                    sm->fvalue += sm->fstep;
                } else Beep();
            }
            break;
        case DownShort:
            if (sm->fvalue > sm->fmin) {
                sm->fvalue -= sm->fstep;
            } else Beep();
            break;
        case DownLong:
            for (uint8_t i = 0; i < 5; i++) {
                if (sm->fvalue > sm->fmin) {
                    sm->fvalue -= sm->fstep;
                } else Beep();
            }
            break;
        case OkShort:
        case OkLong:
            sm->done = True;
            sm->selected = True;
            break;
        case BackShort:
        case BackLong:
            sm->selected = False;
            sm->done = True;
            sm->fvalue = sm->fold_value;
            // Intentional failover to the next stage
            break;
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
            //        case ReviewShort:ui_state = ReviewScreen;
            //            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectTime(TimeSelection_t* ts) {
    define_input_action();
    switch (comandToHandle) {
        case UpShort:
            ts->minute += 1;
            if (ts->minute > 59) {
                ts->minute = ts->minute % 59;
                if (ts->hour < 23) ts->hour++;
                else (ts->hour = 0);
            }
            break;
        case UpLong:
            ts->minute += 5;
            if (ts->minute > 59) {
                ts->minute = ts->minute % 59;
                if (ts->hour < 23) ts->hour++;
                else (ts->hour = 0);
            }
            break;
        case DownShort:
            ts->minute -= 1;
            if (ts->minute < 0) {
                ts->minute = 60 - ts->minute;
                if (ts->hour > 0) ts->hour--;
                else {
                    ts->hour = 23;
                };
            }
            break;
        case DownLong:
            ts->minute -= 5;
            if (ts->minute < 0) {
                ts->minute = 60 - ts->minute;
                if (ts->hour > 0) ts->hour--;
                else {
                    ts->hour = 23;
                };
            }
            break;
        case BackShort:
        case BackLong:
            ts->minute = ts->old_minute;
            ts->hour = ts->old_hour;
            ts->selected = False;
            ts->done = True;
            break;
        case OkShort:
        case OkLong:
            ts->selected = True;
            ts->done = True;
            break;
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
            //        case ReviewShort:ui_state = ReviewScreen;
            //            break;
        case ChargerEvent:STATE_HANDLE_CHARGING;
            break;
        default:
            break;
    }
    comandToHandle = None;
}
