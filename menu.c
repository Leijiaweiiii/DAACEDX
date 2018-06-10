#include "menu.h"
#include "lcd.h"
#include "ui.h"
#include "DAACED.h"
uint8_t old_label_len = 0;

void display_big_font_label(const char * msg) {
    uint8_t len = 0;
    len = lcd_string_lenght(msg, BigFont);
    if (len != old_label_len) {
        uint8_t block_start = (LCD_WIDTH - old_label_len) / 2;
        lcd_clear_block(
                block_start,
                UI_HEADER_END_LINE + 24,
                block_start + old_label_len,
                UI_HEADER_END_LINE + 24 + BigFont->height);
        old_label_len = len;
    }
    lcd_write_string(msg, (LCD_WIDTH - len) / 2, UI_HEADER_END_LINE + 24, BigFont, BLACK_OVER_WHITE);
}

void DisplayTime(NumberSelection_t * t) {
    char msg[16];
    uint8_t hour,minute;
    hour = t->value/60;
    hour %= 12;
    hour = (hour==0)?12:hour;
    minute = t->value % 60;
    set_screen_title(t->MenuTitle);
    print_header();
    sprintf(msg, "%02d:%02d", hour,minute);
    display_big_font_label(msg);
    if (rtc_time.msec <500 || 
            (rtc_time.msec > 1000 && rtc_time.msec<1500)) {
        uint8_t block_start, block_end,w;
        w = BigFont->char_descriptors[':' - BigFont->char_start].width;
        if (t->state == 0) {
            block_start = (LCD_WIDTH - old_label_len) / 2;
            block_end = block_start;
            block_end += (old_label_len) / 2-w;
        } else {
            block_start = (LCD_WIDTH - old_label_len) / 2;
            block_end = LCD_WIDTH - block_start;
            block_start += (old_label_len) / 2;
        }
        lcd_clear_block(
                block_start,
                UI_HEADER_END_LINE + 24,
                block_end,
                UI_HEADER_END_LINE + 24 + BigFont->height);
    }
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

