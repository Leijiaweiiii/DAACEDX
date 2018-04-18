// How to generate font binary data? [theDotFactory.exe]
// Note : Font height MUST be in bit. (Configure under settings.)
// Note : Add one more parameter "character_spacing" for adding spaces between character as per font.

#ifndef _FONT_NEW_H_
#define _FONT_NEW_H_

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

typedef uint8_t uint_8;


typedef struct FONT_CHAR_INFO {
    uint_8 width;                   // Character width in pixel/bit.
    uint16_t offset;                // Offset of this character in bitmap.
}FONT_CHAR_INFO;

typedef struct FONT_INFO
{
    uint_8 height;                  // Character height in pixel, all characters have same height
    uint8_t character_spacing;      // Space in pixels between characters.
    char char_start;                // First character
    char char_end;                  // Last character
    const FONT_CHAR_INFO* char_descriptors; // descriptor for each character
    const uint_8 *bitmap;           // Character bitmap
} FONT_INFO;



/* Font data for Times New Roman 11pt */
extern const uint_8 timesNewRoman_11ptBitmaps[];
extern const FONT_INFO timesNewRoman_11ptFontInfo;
extern const FONT_CHAR_INFO timesNewRoman_11ptDescriptors[];

/* Font data for Tahoma 8pt */
extern const uint_8 tahoma_8ptBitmaps[];
extern const FONT_INFO tahoma_8ptFontInfo;
extern const FONT_CHAR_INFO tahoma_8ptDescriptors[];


// Font information for Microsoft Sans Serif 72pt
extern const uint_8 microsoftSansSerif_72ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_72ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_72ptDescriptors[];

// Font information for Microsoft Sans Serif 36pt
extern const uint_8 microsoftSansSerif_36ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_36ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_36ptDescriptors[];


#endif // _FONT_NEW_H_