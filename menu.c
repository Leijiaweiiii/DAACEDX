#include "menu.h"
#include "lcd.h"
#include "ui.h"
#include "DAACED.h"

void DisplayTime(TimeSelection_t * t) {
    char msg[10];
    set_screen_title(t->MenuTitle);
    print_header();
    sprintf(msg, "%02d:%02d", t->hour, t->minute);
    lcd_write_string(msg, 3, UI_HEADER_END_LINE, BigFont, BLACK_OVER_WHITE);
}

void DisplayDouble(NumberSelection_t* s) {
    char msg[10];
    set_screen_title(s->MenuTitle);
    print_header();
    sprintf(msg, s->format, s->fvalue);
    lcd_write_string(msg, 3, UI_HEADER_END_LINE, BigFont, BLACK_OVER_WHITE);
}

void DisplayInteger(NumberSelection_t* s) {
    char msg[10];
    set_screen_title(s->MenuTitle);
    print_header();
    sprintf(msg, s->format, s->value);
    lcd_write_string(msg, 3, UI_HEADER_END_LINE, BigFont, BLACK_OVER_WHITE);
}

void DisplaySettings(SettingsMenu_t* sm) {
    uint8_t p, lineh;
    uint16_t i;
    set_screen_title(sm->MenuTitle);
    print_header();
    p = UI_HEADER_END_LINE;
    lineh = MediumFont->height;
    if(sm->redraw){
        lcd_clear();
        sm->redraw = False;
    }

    for (i = MENU_PAGE_SIZE * sm->page; i < min(sm->TotalMenuItems, (MENU_PAGE_SIZE * (sm->page + 1))); i++) {
        if (sm->menu == i) {
            lcd_write_string(sm->MenuItem[i], 3, p, MediumFont, WHITE_OVER_BLACK);
        } else {
            lcd_write_string(sm->MenuItem[i], 3, p, MediumFont, BLACK_OVER_WHITE);
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

void SelectMenuItem(SettingsMenu_t* s) {
    define_input_action();
    switch (comandToHandle) {
        case UpShort:
        case UpLong:
            decrement_menu_index(s);
            break;
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
        case ReviewShort:ui_state = ReviewScreen;
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
        case ReviewShort:ui_state = ReviewScreen;
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
        case UpLong:
            if (sm->value < sm->max) {
                sm->value += sm->step;
            } else Beep();
            break;
        case DownShort:
        case DownLong:
            if (sm->value > sm->min) {
                sm->value -= sm->step;
            } else Beep();
            break;
        case BackShort:
        case BackLong:
            sm->value = sm->old_value;
            sm->selected = True;
            sm->done = True;
            break;
        case OkShort:
        case OkLong:
            sm->done = True;
            sm->selected = False;
            break;
        case StartLong:STATE_HANDLE_POWER_OFF;
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE;
            break;
        case ReviewShort:ui_state = ReviewScreen;
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
        case UpLong:
            if (sm->fvalue < sm->fmax) {
                sm->fvalue += sm->fstep;
            } else Beep();
            break;
        case DownShort:
        case DownLong:
            if (sm->fvalue > sm->fmin) {
                sm->fvalue -= sm->fstep;
            } else Beep();
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
        case ReviewShort:ui_state = ReviewScreen;
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
        case ReviewShort:ui_state = ReviewScreen;
            break;
        default:
            break;
    }
    comandToHandle = None;
}

uint8_t PopMsg(const char* msg, uint16_t wait) {
    //    uint16_t t = 0;
    //    uint8_t Yo1 = PopY1 + ((PopY2 - (PopY1 + MediumFont->height)) >> 1);
    //    uint8_t Xo1 = PopX1 + ((PopX2 - (PopX1 + lcd_string_lenght(msg, MediumFont))) >> 1);
    //    lcd_fill_block(PopX1, PopY1, PopX2, PopY2);
    //    lcd_write_string(msg, Xo1, Yo1, MediumFont, WHITE_OVER_BLACK);
    //    //    if (wait == 0) {
    //    //        while (!Keypressed);
    //    //        return Key;
    //    //    } else {
    //    while (t < wait) {
    //        __delay_ms(1);
    //        t++;
    //        //            if (Keypressed) return Key;
    //    }
    //        return Yo1;
    //    }
    // TODO: Handle popup with selection
    return 0;
}
