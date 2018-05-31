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


#define MAXMenuItems        15
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
                unsigned redraw         : 1;
                unsigned selected       : 1;
                unsigned UNUSED         : 6;
            };
        };
    } SettingsMenu_t;
#define InitSettingsMenuDefaults(m)     {m->done = False;m->menu = 0;m->page = 0;m->selected = False;}
#define InitSettingsNumberDefaults(m)   {m->done = False;m->selected = False;}
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
                unsigned UNUSED         : 6;
            };
        };
    } NumberSelection_t;

    typedef struct {
        char MenuTitle[MAXMenuTitleLength];
        int8_t hour, minute, old_hour, old_minute;
         union {
            unsigned flags : 8;

            struct {
                unsigned done           : 1;
                unsigned redraw         : 1;
                unsigned selected       : 1;
                unsigned UNUSED         : 6;
            };
        };
    } TimeSelection_t;


    uint8_t PopMsg(const char* msg, uint16_t wait);
    void SelectMenuItem(SettingsMenu_t* s);
    void SelectBinaryMenuItem(SettingsMenu_t* s);
    void SelectMenuItemCircular(SettingsMenu_t* s);
    void DisplaySettings(SettingsMenu_t* s);
    void SelectInteger(NumberSelection_t* s);
    void SelectDouble(NumberSelection_t* s);
    void DisplayDouble(NumberSelection_t* s);
    void DisplayInteger(NumberSelection_t* s);
    void SelectTime(TimeSelection_t * t);
    void DisplayTime(TimeSelection_t * t);
    extern uint8_t print_header(); // implemented in DAACED.c
#ifdef	__cplusplus
}
#endif

#endif	/* MENU_H */

