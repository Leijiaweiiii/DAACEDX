#include "menu.h"
#include "lcd.h"
#include "ui.h"
#include "DAACED.h"
uint8_t old_label_len = 0;

void display_big_font_label(const char * msg) {
    uint16_t len = 0;
    FONT_INFO * font = BigFont;

    len = lcd_string_lenght(msg, font);
    if (len > LCD_WIDTH) {
        font = MediumFont;
        len = lcd_string_lenght(msg, font);
    }
    if (len != old_label_len) {
        uint8_t block_start = (LCD_WIDTH - old_label_len) / 2;
        lcd_clear_block(
                block_start,
                UI_HEADER_END_LINE + 24,
                block_start + old_label_len,
                UI_HEADER_END_LINE + 24 + BigFont->height);
        old_label_len = len;
    }
    lcd_write_string(msg, (LCD_WIDTH - len) / 2, UI_HEADER_END_LINE + 24, font, BLACK_OVER_WHITE);
}

void DisplayTime(uint8_t hour, uint8_t minute, uint8_t state) {
    char msg[16];
    print_header(true);
    sprintf(msg, "%02d:%02d", hour, minute);
    display_big_font_label(msg);
    update_rtc_time();
    if (rtc_time.msec < 500 ||
            (rtc_time.msec > 1000 && rtc_time.msec < 1500)) {
        uint8_t block_start, block_end, margin;
        margin = (LCD_WIDTH - old_label_len) / 2;
        if (state == 0) {
            block_start = margin;
            sprintf(msg, "%02d", hour);
            block_end = block_start + lcd_string_lenght(msg, BigFont);
        } else {
            block_end = LCD_WIDTH - margin;
            sprintf(msg, "%02d", minute);
            block_start = block_end - lcd_string_lenght(msg, BigFont);
            block_start -= 1;
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
    print_header(true);
    sprintf(msg, s->format, s->fvalue);
    display_big_font_label(msg);
}

void DisplayInteger(NumberSelection_t* s) {
    char msg[16];

    set_screen_title(s->MenuTitle);
    print_header(true);
    sprintf(msg, s->format, s->value);
    display_big_font_label(msg);
}

void DisplaySettings(SettingsMenu_t* sm) {
    uint8_t y_pos, lineh, x_pos,x1_pos, polarity, delim_index;
    uint16_t i;
    char str[MAXItemLenght];
    if (! sm->changed) return;
    sm->changed = False;
    set_screen_title(sm->MenuTitle);
    print_header(true);
    y_pos = UI_HEADER_END_LINE;
    lineh = SmallFont->height;

    // TODO: Move page calculations here to handle menu changes during menu operation
    for (i = MENU_PAGE_SIZE * sm->page; i < min(sm->TotalMenuItems, (MENU_PAGE_SIZE * (sm->page + 1))); i++) {
        strcpy(str, sm->MenuItem[i]);
        polarity = (sm->menu != i);
        x1_pos = LCD_WIDTH;
        // Highlight the line if required
        lcd_send_block_d(0, y_pos, LCD_WIDTH, y_pos + lineh, !polarity);
        // Find split of the menu. Split by "|"
        delim_index = 0;
        while ( str[delim_index] != '|'
                 && delim_index < MAXItemLenght) delim_index++;
        // Print the value part of the menu
        if(delim_index > 0 && delim_index < MAXItemLenght){
            str[delim_index] = 0; // Mark the delimiter 
            delim_index ++;
            x1_pos -= lcd_string_lenght(str + delim_index, SmallFont);
            x1_pos -= SmallFont->character_spacing;
            lcd_write_string(str + delim_index, x1_pos, y_pos, SmallFont, polarity);
        }
        // Print the nemy name part of the menu
        x_pos = SmallFont->character_spacing;
        lcd_write_string(str, x_pos, y_pos, SmallFont, polarity);
        y_pos += lineh;
    }
}

void decrement_menu_index(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu > 0) {
        s->menu--;
        s->page = ItemToPage(s->menu);
    } else Beep();
    s->changed = True;
}

void increment_menu_index(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu < s->TotalMenuItems - 1) {
        s->menu++;
        s->page = ItemToPage(s->menu);
    } else Beep();
    s->changed = True;
}

void decrement_menu_index_inf(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu > 0) {
        s->menu--;
    } else {
        s->menu = s->TotalMenuItems - 1;
    }
    s->page = ItemToPage(s->menu);
    s->changed = True;

}

void increment_menu_index_inf(SettingsMenu_t * s) {
    uint8_t oldPage = s->page;
    if (s->menu < s->TotalMenuItems - 1) {
        s->menu++;
    } else {
        s->menu = 0;
    }
    s->page = ItemToPage(s->menu);
    s->changed = True;
}

void SelectMenuItem(SettingsMenu_t* s) {
    define_input_action();
    BT_define_action();
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
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectBinaryMenuItem(SettingsMenu_t* s) {
    define_input_action();
    BT_define_action();
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
        case StartLong:
            s->selected = True;
            s->done = True;
            STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:
            s->selected = True;
            s->done = True;
            STATE_HANDLE_TIMER_IDLE();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectMenuItemCircular(SettingsMenu_t* s) {
    define_input_action();
    BT_define_action();
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
            s->done = True;
            break;
        case BackShort:
        case BackLong:
            s->selected = False;
            s->done = True;
            break;
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectIntegerCircular(NumberSelection_t* sm) {
    define_input_action();
    BT_define_action();
    switch (comandToHandle) {
        case UpLong:
        case UpShort:
            if (sm->value < sm->max) {
                sm->value += sm->step;
            } else {
                sm->value = sm->min;
            }
            break;
        case DownLong:
        case DownShort:
            if (sm->value > sm->min) {
                sm->value -= sm->step;
            } else {
                sm->value = sm->max;
            }
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
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectInteger(NumberSelection_t* sm) {
    define_input_action();
    BT_define_action();
    switch (comandToHandle) {
        case UpLong:
        case UpShort:
            if (sm->value < sm->max) {
                sm->value += sm->step;
            } else Beep();
            break;
        case DownLong:
        case DownShort:
            if (sm->value > sm->min) {
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
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:STATE_HANDLE_TIMER_IDLE();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            break;
    }
    comandToHandle = None;
}

void SelectDouble(NumberSelection_t* sm) {
    define_input_action();
    BT_define_action();
    switch (comandToHandle) {
        case UpLong:
        case UpShort:
            if (sm->fvalue < sm->fmax) {
                sm->fvalue += sm->fstep;
            } else Beep();
            break;
        case DownLong:
        case DownShort:
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
        case StartLong:STATE_HANDLE_POWER_OFF();
            break;
        case StartShort:
            STATE_HANDLE_TIMER_IDLE();
            break;
        case ChargerConnected:
            STATE_HANDLE_CHARGER();
            break;
        default:
            break;
    }
    comandToHandle = None;
}
