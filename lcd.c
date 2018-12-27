#include "lcd.h"
#include "rtc.h"
#include "DAACED.h"

// <editor-fold defaultstate="collapsed" desc="SPI">

void spi_init() {
    RC4PPS = 0x1A; // data-output
    SSP1DATPPS = 0x1D; // PPS to unused PIN.
    RC3PPS = 0x19; // clock output

    SSP1STAT &= 0x3F; // Power on state
    SSP1STATbits.CKE = 1; // Data transmission on rising edge
    SSP1STATbits.SMP = 0; // Data sampled/latched at middle of clock.
    SSP1CON1 = 0x21; // Enable synchronous serial port , CKL ,FOSC_DIV_16 page 394
    //    SSP1CON1bits.SSPM = 0x01;               // SPI Clock FOSC_DIV_16
    //    SSP1CON1bits.SSPEN = 1;                 // Enable synchronous serial port
    PIE3bits.SSP1IE = 0; // Disable interrupt.
}

uint8_t spi_write(uint8_t data) {
    LCD_CS_SELECT();
    unsigned char temp_var = SSP1BUF; // Clear buffer.
    PIR3bits.SSP1IF = 0; // clear interrupt flag bit
    SSP1CON1bits.WCOL = 0; // clear write collision bit if any collision occurs

    SSP1BUF = data; // transmit data
    while (!PIR3bits.SSP1IF); // waiting for the process to complete
    PIR3bits.SSP1IF = 0; // clear interrupt flag bit
    LCD_CS_DESELECT();
    return (SSP1BUF); // return receive data
}
//SPI End
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LCD helper functions">

void lcd_reset(void) {
    __delay_ms(5);
    LCD_RESET_EN();
    __delay_ms(10);
    LCD_RESET_DIS();
    __delay_ms(100);
}

void lcd_send_command(uint8_t command) {
    LCD_MODE_COMMAND();
    spi_write(command);
}

void lcd_send_data(uint8_t data) {
    LCD_MODE_DATA();
    spi_write(data);
}

void lcd_send_command_data(uint8_t command, uint8_t data) {
    LCD_MODE_COMMAND();
    spi_write(command);
    LCD_MODE_DATA();
    spi_write(data);
}

void lcd_send_command_data_array(uint8_t command, uint8_t *data, size_t no_of_bytes) {
    LCD_MODE_COMMAND();
    spi_write(command);
    LCD_MODE_DATA();
    for (uint8_t index = 0; index < no_of_bytes; index++) {
        spi_write(data[index]);
    }
}

// rows in pages

void lcd_prepare_send_data(uint8_t c1, uint8_t p1, uint8_t c2, uint8_t p2) {
    lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
    lcd_send_command(CMD_COL_ADD); // Column address.
    lcd_send_data(c1 + x_offset); // Start column address.
    lcd_send_data(c2 + x_offset); // End column address.

    lcd_send_command(CMD_PAGE_ADD); // Row address.
    lcd_send_data(p1 + y_offset); // Start row address.
    lcd_send_data(p2 + y_offset); // End row address.
    lcd_send_command(CMD_WRITE_DATA); // Write data.
}

void lcd_clear_data_ram() {
    lcd_prepare_send_data(0, 0, LCD_WIDTH, LCD_MAX_PAGES);
    for (uint16_t index = 0; index < LCD_MAX_PAGES * LCD_WIDTH; index++) {
        lcd_send_data(LCD_WHITE_PAGE);
    }
}

// write char directly to the screen

uint8_t lcd_write_char(unsigned int c,
        uint8_t x_pos,
        uint8_t y_pos,
        const FONT_INFO *font,
        uint8_t polarity) {
    uint8_t column, start_page, end_page, data, heigh_in_bytes;
    int8_t i;
    const uint8_t *bitmap;

    if ((c < font->char_start) || (c > font->char_end)) return 0;
    if (!START_OF_PAGE(y_pos)) return 0; // Don't draw not on the edge of the page
    c = c - font->char_start; // 'c' now become index to tables.
    heigh_in_bytes = (font->height % 8 == 0) ? font->height / 8 : font->height / 8 + 1;
    bitmap = font->bitmap + font->char_descriptors[c].offset;
    start_page = PAGE(y_pos);
    end_page = PAGE(y_pos) + heigh_in_bytes;
    /*
     *      for column between x and x + char width in bits
     *          set address for writing a column
     *          for index between PAGE(font heigh) and 0
     *              draw page (PAGE(Y),column,bitmap[index])
     */
    for (column = x_pos; column < x_pos + font->char_descriptors[c].width; column++) {
        lcd_prepare_send_data(column, start_page, column, end_page);
        // TODO: Calculate carefully
        for (i = heigh_in_bytes; i > 0; i = i - 1) {
            data = bitmap[i - 1];
            if (polarity == BLACK_OVER_WHITE)
                lcd_send_data(data);
            else
                lcd_send_data(~data);
        }
        bitmap += heigh_in_bytes;
    }

    return font->char_descriptors[c].width;
}

void lcd_send_block_d(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos, uint8_t polarity) {
    if (x1_pos > x2_pos) SWAP(x1_pos, x2_pos);
    if (y1_pos > y2_pos) SWAP(y1_pos, y2_pos);
    //don't fill blocks not 8 bit hight
    //    if (! (START_OF_PAGE(y1_pos) )) return;

    for (uint8_t column = x1_pos; column < x2_pos; column = column + 1) {
        lcd_prepare_send_data(column, PAGE(y1_pos), column, PAGE(y2_pos));
        for (uint8_t page = PAGE(y1_pos); page < PAGE(y2_pos); page = page + 1) {
            if (polarity == BLACK_OVER_WHITE)
                lcd_send_data(LCD_BLACK_PAGE);
            else
                lcd_send_data(LCD_WHITE_PAGE);
        }
    }
}

void lcd_fill_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_send_block_d(x1_pos, y1_pos, x2_pos, y2_pos, BLACK_OVER_WHITE);
}

void lcd_clear_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_send_block_d(x1_pos, y1_pos, x2_pos, y2_pos, WHITE_OVER_BLACK);
}

uint16_t lcd_string_lenght(const char* str_ptr, const FONT_INFO *font) {
    uint16_t strlng = 0;
    if (str_ptr == NULL) return 0;
    while (*str_ptr) {
        strlng += font->char_descriptors[*str_ptr - font->char_start].width;
        strlng += font->character_spacing;
        ++str_ptr;
    }
    return strlng;
}

void lcd_write_string(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    if (str_ptr == NULL) return;
    //TODO: There is a bug that prints bright pages at spacing when polarity is WHITE_OVER_BLACK
    while (*str_ptr) {
        x_pos += lcd_write_char(*str_ptr, x_pos, y_pos, font, polarity);
        ++str_ptr;
        if (*str_ptr) {
            if (polarity == WHITE_OVER_BLACK) {
                lcd_fill_block(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
            } else {
                lcd_clear_block(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
            }
            x_pos += font->character_spacing;
        }
        if (x_pos >= LCD_WIDTH) {
            y_pos += (font->height + 1);
            x_pos %= LCD_WIDTH;
            y_pos %= LCD_HEIGHT;
        }
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LCD functions implementations">

void lcd_init() {
    LCD_CS_DESELECT();

    __delay_ms(10);
    lcd_reset(); // Reset LCD.
    lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
    lcd_send_command(CMD_SET_MODE_MASTER); // Enable master mode.

    lcd_send_command(CMD_EXTENSION_2); // Extension2 command.
    lcd_send_command(CMD_AUTO_READ); // Disable auto read.
    lcd_send_data(0x9F);

    lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
    lcd_send_command(CMD_SLEEP_OUT); // Sleep out.
    lcd_send_command(CMD_DISPLAY_OFF); // Display OFF.
    lcd_send_command(CMD_INT_OSC_ON); // Internal oscillator ON.

    lcd_send_command(CMD_POWER_CON); // Power control.
    lcd_send_data(0x0B); // Regulator, Follower and Booster ON.
    lcd_send_command(CMD_SET_VOP); // Set Vop.
    lcd_send_data(contrast_value & 0x3F); // 16 Volts.
    lcd_send_data((contrast_value >> 6) & 0x07);

    lcd_send_command(CMD_EXTENSION_2); // Extension2 commands.
    lcd_send_command(CMD_ANALOG_CKT); // Analog Circuit set.
    lcd_send_data(0x00);
    lcd_send_data(0x01); // Booster efficiency 1.

    lcd_send_data(LCD_BIAS_12); // LCD bias 1/13
    lcd_send_command(CMD_BOOSTER_LVL); // Booster level.
    lcd_send_data(0xFB); // x10.

    lcd_send_command(CMD_EXTENSION_1); // Extension1 commands.
    lcd_send_command(CMD_DISPLAY_MODE); // Display mode.
    lcd_send_data(0x10); // Monochrome mode.
    lcd_send_command(CMD_DISPLAY_CONTROL); // Display control.
    lcd_send_data(0x00); // No clock division.
    lcd_send_data(LCD_DUTY_CICLE_160); // 1/160 duty.
    lcd_send_data(0x00); // ??

    lcd_send_command(CMD_DATASCAN_DIR); // data scan directon.
    lcd_send_data(LCD_ORIENTATION_NORMAL);

    lcd_send_command(CMD_DATA_FORMAT_LSB); // LSB First.

    lcd_send_command(CMD_INVERSION_OFF);
    lcd_send_command(CMD_EXTENSION_2); // Extension2 command.
    lcd_send_command(CMD_PWR_SRC_INT); // Use internel oscillator.

    lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
    lcd_send_command(CMD_ICON_DISABLE); // Disable ICON RAM.
    lcd_send_command(CMD_NOP);
    lcd_clear_data_ram(); // Clearing data RAM.

    lcd_send_command(CMD_DISPLAY_ON); // Turn ON display.
    x_offset = 0;
    y_offset = 0;
}

void lcd_clear() {
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_set_orientation();
}

void lcd_increase_contrast() {
    lcd_send_command(CMD_EXTENSION_1);
    lcd_send_command(CMD_DISPLAY_ON);
    lcd_send_command(CMD_VOP_CON_INC_VOP);
    contrast_value++;
}

void lcd_decrease_contrast() {
    lcd_send_command(CMD_EXTENSION_1);
    lcd_send_command(CMD_DISPLAY_ON);
    lcd_send_command(CMD_VOP_CON_DEC_VOP);
    contrast_value--;
}

void lcd_write_integer(const int Int, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    char msg[10];
    sprintf(msg, "%d", Int);
    lcd_write_string_d("       ", x_pos, y_pos, font, polarity);
    lcd_write_string_d(msg, x_pos, y_pos, font, polarity);
}
// TODO: Try generate bitmaps suitable for this method

void lcd_draw_bitmap_flat(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
    uint8_t min_page, max_page, min_col, max_col;
    min_page = PAGE(y_pos);
    max_page = PAGE(y_pos) + bitmap_data->heigth_in_bytes;
    min_col = x_pos;
    max_col = x_pos + bitmap_data->width_in_bits;
    lcd_prepare_send_data(min_col, min_page, max_col, max_page);
    for (uint16_t i = 0; i < bitmap_data->heigth_in_bytes * bitmap_data->width_in_bits; i++) {
        lcd_send_data(bitmap_data->image_data[i]);
    }
}

void lcd_draw_bitmap(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
    uint8_t column, page;
    uint16_t index;
    for (column = x_pos; column < x_pos + bitmap_data->width_in_bits; column++) {
        lcd_prepare_send_data(column, PAGE(y_pos), column, PAGE(y_pos) + bitmap_data->heigth_in_bytes);
        // TODO: Calculate carefully
        for (page = bitmap_data->heigth_in_bytes; page > 0; page--) {
            index = bitmap_data->heigth_in_bytes * (column - x_pos) + page - 1;
            lcd_send_data(bitmap_data->image_data[index]);
        }
    }
}
void inline setOrientationNormal(){
    lcd_send_data(LCD_ORIENTATION_NORMAL);
    lcd_send_command(CMD_DATA_FORMAT_LSB); // LSB First.
    x_offset = 0;
    y_offset = 0;
}

void inline setOrientationInverted(){
    lcd_send_data(LCD_ORIENTATION_INVERTED);
    lcd_send_command(CMD_DATA_FORMAT_MSB); // MSB First.
    x_offset = 15;
    y_offset = 1;    
}

void lcd_set_orientation() {
    lcd_send_command(CMD_EXTENSION_1);
    lcd_send_command(CMD_DATASCAN_DIR); // data scan directon.
    switch(Orientation){
        case ORIENTATION_NORMAL:
            setOrientationNormal();
            break;
        case ORIENTATION_INVERTED:
            setOrientationInverted();
            break;
        case ORIENTATION_AUTO:
            if(OrientationSensor == ORIENTATION_NORMAL)
                setOrientationNormal();
            else
                setOrientationInverted();
            break;
        default:
            // Do nothing, it's an error state
            break;
    }
}

void lcd_draw_fullsize_hline(uint8_t line, uint8_t data) {
    uint8_t x1_pos = 0;
    uint8_t x2_pos = LCD_WIDTH;
    uint8_t page = PAGE(line);
    for (uint8_t column = x1_pos; column < x2_pos; column++) {
        lcd_prepare_send_data(column, page, column, page);
        lcd_send_data(data);
    }
}

void lcd_send_page(uint8_t page, uint8_t col, uint8_t data) {
    lcd_prepare_send_data(col, page, col, page);
    lcd_send_data(data);
}

void display_message(const char * message) {
    lcd_fill_block(2, 98, 158, 112);
    lcd_write_string(message, 4, 101, BigFont, WHITE_OVER_BLACK);
}
