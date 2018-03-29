
#ifndef LCD_H
#define	LCD_H

#include <xc.h> // include processor files - each processor file is guarded.
#include "DAACEDcommon.h"
#include "DAACEDbitmap.h"


// <editor-fold defaultstate="collapsed" desc="LCD parameters and definitions">
#define SMALL_LCD
#define LCD_WIDTH               (160)
#define LCD_HEIGHT              (115)
#define PAGE_HEIGTH              (8)
#define LCD_MAX_PAGES           (LCD_HEIGHT/PAGE_HEIGTH)
#define PAGE(x)                 x/PAGE_HEIGTH
#define Y_OFFSET                (6)
#define BLACK_OVER_WHITE        (0x01)
#define WHITE_OVER_BLACK        (0x00)

#define LCD_MODE_DATA()         (LATFbits.LF4 = 1)
#define LCD_MODE_COMMAND()      (LATFbits.LF4 = 0)

#define LCD_RESET_EN()          (LATFbits.LF6 = 0)
#define LCD_RESET_DIS()         (LATFbits.LF6 = 1)

#define LCD_BACKLIGHT_ON()      (LATEbits.LE6 = 0)
#define LCD_BACKLIGHT_OFF()     (LATEbits.LE6 = 1)

int16_t cursor_x, cursor_y;

typedef union {
    uint8_t PAGE : 8;

    struct {
        unsigned p0 : 1;
        unsigned p1 : 1;
        unsigned p2 : 1;
        unsigned p3 : 1;
        unsigned p4 : 1;
        unsigned p5 : 1;
        unsigned p6 : 1;
        unsigned p7 : 1;
    };
} LCDPage;
LCDPage lcd_buffer[LCD_MAX_PAGES][LCD_WIDTH];

typedef struct {
    uint8_t min_x, max_x, min_y, max_y;
    TBool changed;
} UpdateBoundary;

UpdateBoundary full_screen_update_boundary = {0, LCD_WIDTH - 1, 0, LCD_HEIGHT - 1};

#define ORIENTATION_NORMAL              0
#define ORIENTATION_INVERTED            1
#define ORIENTATION_INVERSE_THRESHOLD   400
TBool orientation = ORIENTATION_NORMAL;
TBool orientation_change_enabled = true;

#define topSpace     20
#define botSpace     10

#define PopX1       LCD_WIDTH/8
#define PopY1       LCD_HEIGHT/4
#define PopX2       LCD_WIDTH*7/8
#define PopY2       LCD_HEIGHT*3/4

const FONT_INFO *SmallFont = &tahoma_8ptFontInfo;
const FONT_INFO *MediumFont = &timesNewRoman_11ptFontInfo;
const FONT_INFO *BigFont = &microsoftSansSerif_42ptFontInfo;


volatile uint8_t frames_count = 0;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ST75256 COMMANDS">
#define CMD_EXTENSION_1         (0x30)
#define CMD_EXTENSION_2         (0x31)
#define CMD_EXTENSION_3         (0x38)
#define CMD_EXTENSION_4         (0x39)

/* Commands under EXTENSION1 */
#define CMD_NOP                 (0x25)

#define CMD_DISPLAY_OFF         (0xAE)
#define CMD_DISPLAY_ON          (0xAF)

#define CMD_INVERSION_OFF       (0xA6)
#define CMD_INVERSION_ON        (0xA7)

#define CMD_ALL_PIXEL_OFF       (0x22)
#define CMD_ALL_PIXEL_ON        (0x23)

#define CMD_DISPLAY_CONTROL     (0xCA)

#define CMD_SLEEP_OUT           (0x94)
#define CMD_SLEEP_IN            (0x95)

#define CMD_ROW_ADD             (0x75)
#define CMD_PAGE_ADD            (0x75)
#define CMD_COL_ADD             (0x15)

#define CMD_DATASCAN_DIR        (0xBC)

#define CMD_WRITE_DATA          (0x5C)
#define CMD_READ_DATA           (0x5D)

#define CMD_PARTIAL_IN          (0xA8)
#define CMD_PARTIAL_OUT         (0xA9)

#define CMD_RWM_IN              (0xE0)
#define CMD_RWM_OUT             (0xEE)

#define CMD_SET_SCROLL_AREA     (0xAA)
#define CMD_SET_START_LINE      (0xAB)

#define CMD_INT_OSC_ON          (0xD1)
#define CMD_INT_OSC_OFF         (0xD2)

#define CMD_POWER_CON           (0x20)

#define CMD_SET_VOP             (0x81)
#define CMD_VOP_CON_INC_VOP     (0xD6)
#define CMD_VOP_CON_DEC_VOP     (0xD7)

#define CMD_READ_REG_MODE_VPRL  (0x7C)
#define CMD_READ_REG_MODE_VPRH  (0x7D)

#define CMD_DATA_FORMAT_LSB     (0x0C)
#define CMD_DATA_FORMAT_MSB     (0x08)

#define CMD_DISPLAY_MODE        (0xF0)

#define CMD_ICON_DISABLE        (0x76)
#define CMD_ICON_ENABLE         (0x77)

#define CMD_SET_MODE_MASTER     (0x6E)
#define CMD_SET_MODE_SLAVE      (0x6F)

/* Commands under EXTENSION2 */
#define CMD_SET_GRAY_LEVEL      (0x20)

#define CMD_ANALOG_CKT          (0x32)

#define CMD_BOOSTER_LVL         (0x51)

#define CMD_PWR_SRC_INT         (0x40)
#define CMD_PWR_SRC_EXT         (0x41)

#define CMD_HIGH_PWR_MODE       (0x49)
#define CMD_HIGH_PWR_MODE_NOR   (0x48)

#define CMD_AUTO_READ           (0xD7)

#define CMD_OTP_RD_WR_CON       (0xE0)
#define CMD_OTP_CONTROL_OUT     (0xE1)
#define CMD_OTP_WRITE           (0xE2)
#define CMD_OTP_READ            (0xE3)
#define CMD_OTP_PROG_SET        (0xE5)

#define CMD_SET_FRAME_RATE      (0xF0)

#define CMD_TEMP_RANGE          (0xF2)
#define CMD_TEMP_GRAD           (0xF4)

/* Commands under EXTENSION3 */
#define CMD_SET_ID              (0xD5)
#define CMD_READ_ID_EN          (0x7F)
#define CMD_READ_ID_DIS         (0x7E)

/* Commands under EXTENSION4 */
#define ENABLE_OTP              (0xD6)

#define LCD_BIAS_12             (0x02)
#define LCD_BIAS_13             (0x01)

#define LCD_DUTY_CICLE_128      (0x7F)
#define LCD_DUTY_CICLE_160      (0x0F)


// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="SPI functions definitions">
uint8_t spi_write(uint8_t data);
void spi_init();
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="LCD functions definitions">
void lcd_refresh(UpdateBoundary * box);
uint8_t lcd_string_lenght(const char* str_ptr, const FONT_INFO *font);
void lcd_init();
void lcd_clear();
void lcd_draw_line(uint8_t x0_pos, uint8_t y0_pos, uint8_t x1_pos, uint8_t y1_pos, uint8_t polarity);
void lcd_draw_hline(uint8_t x0_pos, uint8_t x1_pos, uint8_t y_pos, uint8_t polarity);
void lcd_write_char(unsigned int c, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity);
void lcd_write_string(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity);
void lcd_draw_bitmap(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data);
void lcd_battery_info(uint8_t x_pos, uint8_t y_pos, uint8_t battery_percentage);
void lcd_fill_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos);
void lcd_clear_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos);
void lcd_clear_data_ram();

// </editor-fold>
#endif	/* LCD_H */

