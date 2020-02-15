#include "DAACEDcommon.h"
#include "ui.h"
/* 
 * File:   menu.h
 * Author: navado
 *
 * Created on 12 April 2018, 18:20
 */

#ifndef MENU_H
#define	MENU_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifndef SETTINGS_NUM_ELEMENTS
    #define MAXMenuItems        17
#else
    #define MAXMenuItems        SETTINGS_NUM_ELEMENTS + 2
#endif

#define MAXItemLenght       25
#define MAXMenuTitleLength  25

#define MENU_PAGE_SIZE  5

    typedef struct {
        char MenuItem[MAXMenuItems][MAXItemLenght];
        uint8_t TotalMenuItems;
        char MenuTitle[MAXMenuTitleLength];
        uint8_t menu;
        uint8_t page;

        union {
            unsigned flags : 8;

            struct {
                unsigned done           : 1;
                unsigned changed        : 1;
                unsigned selected       : 1;
                unsigned page_changed   : 1;
                unsigned UNUSED         : 5;
            };
        };
    } SettingsMenu_t;
#define InitSettingsMenuDefaults(m)     {m->done = False;m->menu = 0;m->page = 0;m->selected = False; m->changed = True;}
#define InitSettingsNumberDefaults(m)   {m->done = False;m->selected = False; m->redraw = True;}
#define ItemToPage(x)                   (x/MENU_PAGE_SIZE)
#define SettingsNotDone(x)              ((!x->done) && ui_state == SettingsScreen)    

    typedef struct {
        char MenuTitle[MAXMenuTitleLength];
        char * format;

        union {
            int24_t min;
            float fmin;
        };

        union {
            int24_t max;
            float fmax;
        };

        union {
            int24_t value;
            float fvalue;
        };

        union {
            int24_t old_value;
            float fold_value;
        };

        union {
            int24_t step;
            float fstep;
        };
         union {
            unsigned flags : 8;

            struct {
                unsigned done           : 1;
                unsigned redraw         : 1;
                unsigned selected       : 1;
                unsigned state          : 5; // Any state that controls know to manage
            };
        };
    } NumberSelection_t;

    // Service variables
    uint8_t old_label_start = 0;
    uint8_t old_label_end = 0;
    
    // Function definitions
    uint8_t PopMsg(const char* msg, uint16_t wait);
    void SelectMenuItem(SettingsMenu_t* s);
    void SelectBinaryMenuItem(SettingsMenu_t* s);
    void SelectMenuItemCircular(SettingsMenu_t* s);
    void DisplaySettings(SettingsMenu_t* s);
    void SelectInteger(NumberSelection_t* s);
    void SelectIntegerCircular(NumberSelection_t* s);
    void SelectDouble(NumberSelection_t* s);
    void DisplayDouble(NumberSelection_t* s);
    void DisplayInteger(NumberSelection_t* s);
    void DisplayTime(uint8_t hour, uint8_t minute, uint8_t state);
    extern uint8_t print_header(TBool hide_time); // implemented in DAACED.c
    void display_big_font_label(const char * msg);
#ifdef	__cplusplus
}
#endif

#endif	/* MENU_H */

