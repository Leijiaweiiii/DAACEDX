/*
 * TODO : Add license.
 */

// How to generate font binary data? [theDotFactory.exe]
// Note : Font height MUST be in bit. (Configure under settings.)
// Note : Add one more parameter "character_spacing" for adding spaces between character as per font.

#ifndef _FONT_H_
#define _FONT_H_
#include <stdint.h>

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

/* Font data for Tahoma 8pt */
extern const uint_8 tahoma_8ptBitmaps[];
extern const FONT_INFO tahoma_8ptFontInfo;
extern const FONT_CHAR_INFO tahoma_8ptDescriptors[];

// Font data for Times New Roman 9pt
extern const uint_8 timesNewRoman_9ptBitmaps[];
extern const FONT_INFO timesNewRoman_9ptFontInfo;
extern const FONT_CHAR_INFO timesNewRoman_9ptDescriptors[];

/* Font data for Times New Roman 11pt */
extern const uint_8 timesNewRoman_11ptBitmaps[];
extern const FONT_INFO timesNewRoman_11ptFontInfo;
extern const FONT_CHAR_INFO timesNewRoman_11ptDescriptors[];

// Font data for Microsoft Sans Serif 36pt
extern const uint_8 microsoftSansSerif_36ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_36ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_36ptDescriptors[];

// Font data for Microsoft Sans Serif 42pt
extern const uint_8 microsoftSansSerif_42ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_42ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_42ptDescriptors[];

// Font data for Microsoft Sans Serif 48pt
extern const uint_8 microsoftSansSerif_48ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_48ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_48ptDescriptors[];

#endif // _FONT_H_