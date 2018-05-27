// How to generate font binary data? [theDotFactory.exe]
// Note : Font height MUST be in bit. (Configure under settings.)
// Note : Add one more parameter "character_spacing" for adding spaces between character as per font.

#ifndef _FONT_H_
#define _FONT_H_
#include <stdint.h>

typedef uint8_t uint_8;

typedef struct FONT_CHAR_INFO {
    uint_8 width; // Character width in pixel/bit.
    uint16_t offset; // Offset of this character in bitmap.
} FONT_CHAR_INFO;

typedef struct FONT_INFO {
    uint_8 height; // Character height in pixel, all characters have same height
    uint_8 character_spacing; // Space in pixels between characters.
    char char_start; // First character
    char char_end; // Last character
    const FONT_CHAR_INFO* char_descriptors; // descriptor for each character
    const uint_8 *bitmap; // Character bitmap
    uint_8 truncate;
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

// Font data for Roboto Condensed 20pt
extern const uint_8 robotoCondensed_20ptBitmaps[];
extern const FONT_INFO robotoCondensed_20ptFontInfo;
extern const FONT_CHAR_INFO robotoCondensed_20ptDescriptors[];

// Font data for DejaVu Sans Mono 11pt
extern const uint8_t dejaVuSansMono_11ptBitmaps[];
extern const FONT_INFO dejaVuSansMono_11ptFontInfo;
extern const FONT_CHAR_INFO dejaVuSansMono_11ptDescriptors[];

/* Font data for Microsoft Sans Serif 42pt */
extern const uint_8 microsoftSansSerif_42ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_42ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_42ptDescriptors[];

/* Font data for Roboto Condensed 11pt */
extern const uint_8 robotoCondensed_11ptBitmaps[];
extern const FONT_INFO robotoCondensed_11ptFontInfo;
extern const FONT_CHAR_INFO robotoCondensed_11ptDescriptors[];

/* Font data for Roboto Condensed 16pt */
extern const uint_8 robotoCondensed_16ptBitmaps[];
extern const FONT_INFO robotoCondensed_16ptFontInfo;
extern const FONT_CHAR_INFO robotoCondensed_16ptDescriptors[];

// Font data for Roboto Condensed 20pt
extern const uint_8 robotoCondensed_20ptBitmaps[];
extern const FONT_INFO robotoCondensed_20ptFontInfo;
extern const FONT_CHAR_INFO robotoCondensed_20ptDescriptors[];

/* Font data for Roboto Condensed 44pt */
extern const uint_8 robotoCondensed_44ptBitmaps[];
extern const FONT_INFO robotoCondensed_44ptFontInfo;
extern const FONT_CHAR_INFO robotoCondensed_44ptDescriptors[];

/* Font data for Roboto Condensed 45pt */
extern const uint_8 robotoCondensed_45ptBitmaps[];
extern const FONT_INFO robotoCondensed_45ptFontInfo;
extern const FONT_CHAR_INFO robotoCondensed_45ptDescriptors[];

/* Font data for Tahoma 8pt */
extern const uint_8 tahoma_8ptBitmaps[];
extern const FONT_INFO tahoma_8ptFontInfo;
extern const FONT_CHAR_INFO tahoma_8ptDescriptors[];

/* Font data for Times New Roman 12pt */
extern const uint_8 timesNewRoman_12ptBitmaps[];
extern const FONT_INFO timesNewRoman_12ptFontInfo;
extern const FONT_CHAR_INFO timesNewRoman_12ptDescriptors[];

/* Font data for Myriad Pro 18pt */
extern const uint_8 myriadPro_18ptBitmaps[];
extern const FONT_INFO myriadPro_18ptFontInfo;
extern const FONT_CHAR_INFO myriadPro_18ptDescriptors[];

/* Font data for Myriad Pro 24pt */
extern const uint_8 myriadPro_24ptBitmaps[];
extern const FONT_INFO myriadPro_24ptFontInfo;
extern const FONT_CHAR_INFO myriadPro_24ptDescriptors[];

/* Font data for Myriad Pro 62pt */
extern const uint_8 myriadPro_62ptBitmaps[];
extern const FONT_INFO myriadPro_62ptFontInfo;
extern const FONT_CHAR_INFO myriadPro_62ptDescriptors[];

/* Font data for Squada One 20pt */
extern const uint_8 squadaOne_20ptBitmaps[];
extern const FONT_INFO squadaOne_20ptFontInfo;
extern const FONT_CHAR_INFO squadaOne_20ptDescriptors[];

/* Font data for Squada One 26pt */
extern const uint_8 squadaOne_26ptBitmaps[];
extern const FONT_INFO squadaOne_26ptFontInfo;
extern const FONT_CHAR_INFO squadaOne_26ptDescriptors[];

/* Font data for Squada One 62pt */
extern const uint_8 squadaOne_62ptBitmaps[];
extern const FONT_INFO squadaOne_62ptFontInfo;
extern const FONT_CHAR_INFO squadaOne_62ptDescriptors[];

/* Font data for Squada One 72pt */
extern const uint_8 squadaOne_72ptBitmaps[];
extern const FONT_INFO squadaOne_72ptFontInfo;
extern const FONT_CHAR_INFO squadaOne_72ptDescriptors[];

#endif // _FONT_H_