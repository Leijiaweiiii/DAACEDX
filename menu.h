#include "DAACEDcommon.h"
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

#define MENU_PAGE_SIZE  6
    typedef struct {
        char MenuItem[MAXMenuItems][MAXItemLenght];
        uint8_t TotMenuItems;
        char MenuTitle[MAXMenuTitleLength];
        uint8_t menu;
        uint8_t selected;
        uint8_t prev;
        uint8_t page;
        TBool done;
    } SettingsMenu_t;

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
        TBool done;
    } NumberSelection_t;

    typedef struct {
        char MenuTitle[MAXMenuTitleLength];
        int8_t hour, minute, old_hour, old_minute;
        TBool done;
    } TimeSelection_t;


    uint8_t PopMsg(const char* msg, uint16_t wait);
    void SelectMenuItem(SettingsMenu_t* s);
    void SelectBinaryMenuItem(SettingsMenu_t* s);
    void DisplaySettings(SettingsMenu_t* s);
    void SelectInteger(NumberSelection_t* s);
    void SelectDouble(NumberSelection_t* s);
    void DisplayDouble(NumberSelection_t* s);
    void DisplayInteger(NumberSelection_t* s);
    void SelectTime(TimeSelection_t * t);
    void DisplayTime(TimeSelection_t * t);

#ifdef	__cplusplus
}
#endif

#endif	/* MENU_H */

