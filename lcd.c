#include "lcd.h"
#include "rtc.h"

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
    lcd_send_data(c1); // Start column address.
    lcd_send_data(c2); // End column address.

    lcd_send_command(CMD_PAGE_ADD); // Row address.
    lcd_send_data(p1); // Start row address.
    lcd_send_data(p2); // End row address.
    lcd_send_command(CMD_WRITE_DATA); // Write data.
}
#ifndef LCD_DIRECT_ACCESS

void lcd_update_boundingbox(UpdateBoundary * box, uint8_t x_min, uint8_t y_min, uint8_t x_max, uint8_t y_max) {
    box->changed |= box->min_x > x_min;
    box->changed |= box->max_x < x_max;
    box->changed |= box->min_y > y_min;
    box->changed |= box->max_y < y_max;
    if (x_min < box->min_x) box->min_x = x_min;
    if (x_max > box->max_x) box->max_x = x_max;
    if (y_min < box->min_y) box->min_y = y_min;
    if (y_max > box->max_y) box->max_y = y_max;
}

uint8_t update_page(LCDPage * page, uint8_t position, uint8_t polarity) {
    uint8_t p = page->PAGE;
    switch (position % LCD_PAGE_HEIGTH) {
#ifdef MSB_FIRST
        case 0:
            page->p0 = polarity;
            break;
        case 1:
            page->p1 = polarity;
            break;
        case 2:
            page->p2 = polarity;
            break;
        case 3:
            page->p3 = polarity;
            break;
        case 4:
            page->p4 = polarity;
            break;
        case 5:
            page->p5 = polarity;
            break;
        case 6:
            page->p6 = polarity;
            break;
        case 7:
            page->p7 = polarity;
            break;
#else
        case 0:
            page->p7 = polarity;
            break;
        case 1:
            page->p6 = polarity;
            break;
        case 2:
            page->p5 = polarity;
            break;
        case 3:
            page->p4 = polarity;
            break;
        case 4:
            page->p3 = polarity;
            break;
        case 5:
            page->p2 = polarity;
            break;
        case 6:
            page->p1 = polarity;
            break;
        case 7:
            page->p0 = polarity;
            break;
#endif
    }
    return p^page->PAGE;
}

void lcd_draw_pixel_b(uint8_t x_pos, uint8_t y_pos, uint8_t polarity) {
    if ((x_pos >= LCD_WIDTH) || (y_pos >= LCD_HEIGHT)) {
        return;
    }

    if (orientation == ORIENTATION_INVERTED && orientation_change_enabled) {
        y_pos = LCD_HEIGHT - y_pos;
        x_pos = LCD_WIDTH - x_pos;
    } else {
        y_pos += Y_OFFSET;
    }
    if (update_page(&lcd_buffer[PAGE(y_pos)][x_pos], y_pos, polarity))/* Update bounding box only if the bit actually changed the state*/
        lcd_update_boundingbox(&full_screen_update_boundary, x_pos, y_pos, x_pos, y_pos);
    // TODO: Calculate this in more generic way - now this will work only for the main screen
}

void lcd_set_pixel_b(uint8_t x_pos, uint8_t y_pos) {
    lcd_draw_pixel_b(x_pos, y_pos, BLACK_OVER_WHITE);
}

void lcd_clear_pixel_b(uint8_t x_pos, uint8_t y_pos) {
    lcd_draw_pixel_b(x_pos, y_pos, WHITE_OVER_BLACK);
}

void lcd_draw_line_b(uint8_t x0_pos, uint8_t y0_pos, uint8_t x1_pos, uint8_t y1_pos, uint8_t polarity) {
    uint8_t steep = abs(y1_pos - y0_pos) > abs(x1_pos - x0_pos);
    if (steep) {
        SWAP(x0_pos, y0_pos);
        SWAP(x1_pos, y1_pos);
    }

    if (x0_pos > x1_pos) {
        SWAP(x0_pos, x1_pos);
        SWAP(y0_pos, y1_pos);
    }

    //    lcd_update_boundingbox(&full_screen_update_boundary, x0_pos, y0_pos, x1_pos, y1_pos);

    uint8_t dx, dy;
    dx = x1_pos - x0_pos;
    dy = abs(y1_pos - y0_pos);

    int8_t error = dx / 2;
    int8_t y_step;

    if (y0_pos < y1_pos) y_step = 1;
    else y_step = -1;

    for (; x0_pos <= x1_pos; x0_pos = x0_pos + 1) {
        if (steep) {
            lcd_draw_pixel_b(y0_pos, x0_pos, polarity);
        } else {
            lcd_draw_pixel_b(x0_pos, y0_pos, polarity);
        }

        error -= dy;

        if (error < 0) {
            y0_pos += y_step;
            error += dx;
        }
    }
}

void lcd_draw_vline_b(uint8_t x_pos, uint8_t y0_pos, uint8_t y1_pos, uint8_t polarity) {
    lcd_draw_line_b(x_pos, y0_pos, x_pos, y1_pos, polarity);
}

void lcd_draw_hline_b(uint8_t x0_pos, uint8_t x1_pos, uint8_t y_pos, uint8_t polarity) {
    lcd_draw_line_b(x0_pos, y_pos, x1_pos, y_pos, polarity);
}
#endif

void lcd_clear_data_ram() {
    lcd_prepare_send_data(0, 0, LCD_WIDTH, LCD_MAX_PAGES);
    for (uint16_t index = 0; index < LCD_MAX_PAGES * LCD_WIDTH; index++) {
        lcd_send_data(LCD_WHITE_PAGE);
    }
}
#ifndef LCD_DIRECT_ACCESS

void lcd_refresh(UpdateBoundary * box) {
    uint8_t column, max_column, page;
    // Don't even try to update unchanged box
    if (!box->changed) return;

    for (page = box->min_y / LCD_PAGE_HEIGTH; page < box->max_y / LCD_PAGE_HEIGTH; page = page + 1) {
        lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
        lcd_send_command(CMD_PAGE_ADD); // Row address.
        lcd_send_data(page + Y_OFFSET); // Start row address.
        lcd_send_data(page + Y_OFFSET); // End row address.

        column = box->min_x;
        max_column = box->max_x;

        lcd_send_command(CMD_COL_ADD); // Column address.
        lcd_send_data(column); // Start column address.
        lcd_send_data(max_column); // End column address.

        lcd_send_command(CMD_WRITE_DATA); // Write data.

        for (; column <= max_column; column = column + 1) {
            lcd_send_data(lcd_buffer[page][column].PAGE); // Sending data from buffer.
        }
    }
    // min set to max, max to min - for box comparison when updated
    box->min_x = LCD_WIDTH - 1;
    box->max_x = 0;
    box->min_y = LCD_HEIGHT - 1;
    box->max_y = 0;
    box->changed = false;
    //    frames_count=frames_count+1;
}

uint8_t lcd_write_char_b(unsigned int c, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    uint8_t i, j, line;
    const uint8_t *bitmap;

    if ((c < font->char_start) || (c > font->char_end)) return 0;

    c = c - font->char_start; // 'c' now become index to tables.
    bitmap = font->bitmap + font->char_descriptors[c].offset;

    for (j = 0; j < font->height; j = j + 1) {
        for (i = 0; i < font->char_descriptors[c].width; i = i + 1) {
            if (i % 8 == 0) {
                line = bitmap[(font->char_descriptors[c].width + 7) / 8 * j + i / 8]; // line data
            }
            if (line & 0x80) {
                lcd_draw_pixel_b(x_pos + i, y_pos + j, polarity);
            } else {
                lcd_draw_pixel_b(x_pos + i, y_pos + j, !polarity);
            }
            line = line << 1;
        }

    }
    return (font->char_descriptors[c].width);
}
#endif

// write char directly to the screen

uint8_t lcd_write_char_d(unsigned int c,
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
#ifndef LCD_DIRECT_ACCESS

void lcd_fill_block_b(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    if (x1_pos > x2_pos) SWAP(x1_pos, x2_pos);
    if (y1_pos > y2_pos) SWAP(y1_pos, y2_pos);
    for (uint8_t x = x1_pos; x < x2_pos; x++) {
        for (uint8_t y = y1_pos; y < y2_pos; y++) {
            lcd_set_pixel_b(x, y);
        }
    }
}
#endif

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

void lcd_fill_block_d(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_send_block_d(x1_pos, y1_pos, x2_pos, y2_pos, BLACK_OVER_WHITE);
}

void lcd_clear_block_d(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_send_block_d(x1_pos, y1_pos, x2_pos, y2_pos, WHITE_OVER_BLACK);
}
#ifndef LCD_DIRECT_ACCESS

void lcd_clear_block_b(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    if (x1_pos > x2_pos) SWAP(x1_pos, x2_pos);
    if (y1_pos > y2_pos) SWAP(y1_pos, y2_pos);
    for (uint8_t x = x1_pos; x < x2_pos; x++) {
        for (uint8_t y = y1_pos; y < y2_pos; y++) {
            lcd_clear_pixel_b(x, y);
        }
    }
}
#endif

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

void lcd_write_string_d(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    if (str_ptr == NULL) return;
    //TODO: There is a bug that prints bright pages at spacing when polarity is WHITE_OVER_BLACK
    while (*str_ptr) {
        x_pos += lcd_write_char_d(*str_ptr, x_pos, y_pos, font, polarity);
        ++str_ptr;
        if (*str_ptr) {
            if (polarity == WHITE_OVER_BLACK) {
                lcd_fill_block_d(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
            } else {
                lcd_clear_block_d(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
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
#ifndef LCD_DIRECT_ACCESS

void lcd_write_string_b(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    if (str_ptr == NULL) return;

    while (*str_ptr) {
        x_pos += lcd_write_char_b(*str_ptr, x_pos, y_pos, font, polarity);
        ++str_ptr;
        if (*str_ptr) {
            if (polarity == WHITE_OVER_BLACK) {
                lcd_fill_block_b(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
            } else {
                lcd_clear_block_b(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
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

void lcd_draw_bitmap_b(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
    for (uint8_t y = 0; y < bitmap_data->image_height; y++) {
        for (uint8_t x = 0; x < (bitmap_data->image_width + 7) / 8; x++) {
            uint8_t data = *(bitmap_data->image_data + ((y * ((bitmap_data->image_width + 7) / 8) + x)));
            for (int8_t bit_index = 7; bit_index >= 0; bit_index--) {
                if (data & BIT(bit_index)) {
                    lcd_set_pixel_b((x * 8) + (7 - bit_index) + x_pos, y + y_pos);
                } else {
                    lcd_clear_pixel_b((x * 8) + (7 - bit_index) + x_pos, y + y_pos);
                }
            }
        }
    }
}
// TODO: Implement
//void lcd_draw_bitmap_d(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
//    for (uint8_t y = 0; y < bitmap_data->image_height; y++) {
//        for (uint8_t x = 0; x < (bitmap_data->image_width + 7) / 8; x++) {
//            uint8_t data = *(bitmap_data->image_data + ((y * ((bitmap_data->image_width + 7) / 8) + x)));
//
//        }
//    }
//}

void lcd_battery_info_b(uint8_t x_pos, uint8_t y_pos, uint8_t battery_percentage) {
    battery_percentage %= 100;
    lcd_draw_bitmap_b(x_pos, y_pos, &battery_bitmap_data);
    uint8_t no_of_bars = battery_percentage / 16;
    for (uint8_t bar = 0; bar <= no_of_bars; bar++) {
        lcd_draw_vline_b(x_pos + (bar * 2), y_pos + 2, y_pos + 3 + 2, BLACK_OVER_WHITE);
    }
}

void lcd_battery_info_d(uint8_t x_pos, uint8_t y_pos, uint8_t battery_percentage) {
    battery_percentage %= 100;
    lcd_draw_bitmap_b(x_pos, y_pos, &battery_bitmap_data);
    uint8_t no_of_bars = battery_percentage / 16;
    for (uint8_t bar = 0; bar <= no_of_bars; bar++) {
        lcd_draw_vline_b(x_pos + (bar * 2), y_pos + 2, y_pos + 3 + 2, BLACK_OVER_WHITE);
    }
}

#endif
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LCD functions implementations">

void lcd_old_init() {
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
    lcd_send_data(0x35); // 16 Volts.
    lcd_send_data(0x04);

    lcd_send_command(CMD_EXTENSION_2); // Extension2 commands.
    lcd_send_command(CMD_ANALOG_CKT); // Analog Circuit set.
    lcd_send_data(0x00);
    lcd_send_data(0x01); // Booster efficiency 1.
    lcd_send_data(0x02); // LCD bias 1/12
    //    lcd_send_data(0x01);                                    // LCD bias 1/13

    lcd_send_command(CMD_BOOSTER_LVL); // Booster level.
    lcd_send_data(0xFB); // x10.

    lcd_send_command(CMD_EXTENSION_1); // Extension1 commands.
    lcd_send_command(CMD_DISPLAY_MODE); // Display mode.
    lcd_send_data(0x10); // Monochrome mode.
    lcd_send_command(CMD_DISPLAY_CONTROL); // Display control.
    lcd_send_data(0x00); // No clock division.
    lcd_send_data(0x7F); // 1/128 duty.
    //    lcd_send_data(0b10011111);                              // 1/160 duty.
    lcd_send_data(0x00); // ??

    lcd_send_command(CMD_DATASCAN_DIR); // data scan directon.
    lcd_send_data(0x01);
    lcd_send_command(0xA6);

    lcd_send_command(CMD_DATA_FORMAT_LSB); // LSB First.

    lcd_send_command(CMD_INVERSION_OFF); // Pixel inversion OFF.

    lcd_send_command(CMD_EXTENSION_2); // Extension2 command.
    lcd_send_command(CMD_PWR_SRC_INT); // Use internel oscillator.

    lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
    lcd_send_command(CMD_ICON_DISABLE); // Disable ICON RAM.

    lcd_clear_data_ram(); // Clearing data RAM.

    lcd_send_command(CMD_DISPLAY_ON); // Turn ON display.
}

void lcd_init() {
#ifndef LCD_DIRECT_ACCESS
    memset(lcd_buffer, 0, sizeof (lcd_buffer)); // Clear LCD Buffer.
    lcd_update_boundingbox(&full_screen_update_boundary, 0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1); // Reset refresh window.
#endif
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
#ifdef SMALL_LCD
    lcd_send_data(LCD_BIAS_12); // LCD bias 1/12
#else
    lcd_send_data(LCD_BIAS_12); // LCD bias 1/13
#endif
    lcd_send_command(CMD_BOOSTER_LVL); // Booster level.
    lcd_send_data(0xFB); // x10.

    lcd_send_command(CMD_EXTENSION_1); // Extension1 commands.
    lcd_send_command(CMD_DISPLAY_MODE); // Display mode.
    lcd_send_data(0x10); // Monochrome mode.
    lcd_send_command(CMD_DISPLAY_CONTROL); // Display control.
    lcd_send_data(0x00); // No clock division.
#ifdef SMALL_LCD
    lcd_send_data(LCD_DUTY_CICLE_128); // 1/128 duty.
#else
    lcd_send_data(LCD_DUTY_CICLE_160); // 1/160 duty.
#endif
    lcd_send_data(0x00); // ??

    lcd_send_command(CMD_DATASCAN_DIR); // data scan directon.
    lcd_send_data(LCD_ORIENTATION_NORMAL);

#ifdef MSB_FIRST
    lcd_send_command(CMD_DATA_FORMAT_MSB); // MSB First.
#else
    lcd_send_command(CMD_DATA_FORMAT_LSB); // LSB First.
#endif

#ifdef SMALL_LCD
    lcd_send_command(CMD_INVERSION_OFF); // Pixel inversion OFF.
#else
    lcd_send_command(CMD_INVERSION_OFF);
#endif
    lcd_send_command(CMD_EXTENSION_2); // Extension2 command.
    lcd_send_command(CMD_PWR_SRC_INT); // Use internel oscillator.

    lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
    lcd_send_command(CMD_ICON_DISABLE); // Disable ICON RAM.
#ifndef SMALL_LCD
    //    lcd_send_command(CMD_SET_SCROLL_AREA);
    //    lcd_send_data(0);           //  Start at line 0
    //    lcd_send_data(LCD_MAX_PAGES);  //  End at LCD_HEGHT
    //    lcd_send_data(LCD_MAX_PAGES);  //  LCD_HEGHT lines to display
    //    lcd_send_data(0x11);        // Whole Mode
#endif
    lcd_send_command(CMD_NOP);
    lcd_clear_data_ram(); // Clearing data RAM.

    lcd_send_command(CMD_DISPLAY_ON); // Turn ON display.
}

void lcd_clear() {
    //    memset(lcd_buffer, 0, sizeof (lcd_buffer));
    //    lcd_update_boundingbox(&full_screen_update_boundary, 0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    //    lcd_refresh(&full_screen_update_boundary);
    lcd_clear_data_ram();
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

void lcd_write_char(unsigned int c, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    lcd_write_char_d(c, x_pos, y_pos, font, polarity);
}

void lcd_write_string(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    lcd_write_string_d(str_ptr, x_pos, y_pos, font, polarity);
}

void lcd_write_integer(const int Int, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    char msg[10];
    sprintf(msg, "%d", Int);
    lcd_write_string_d("       ", x_pos, y_pos, font, polarity);
    lcd_write_string_d(msg, x_pos, y_pos, font, polarity);
}

void lcd_draw_bitmap(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
    //    lcd_draw_bitmap_d(x_pos, y_pos, bitmap_data);
}
// TODO: Implement

void lcd_battery_info(uint8_t x_pos, uint8_t y_pos, uint8_t battery_percentage) {
    if (battery_percentage > 100) battery_percentage = 100;

}

void lcd_fill_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_fill_block_d(x1_pos, y1_pos, x2_pos, y2_pos);
}

void lcd_clear_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_clear_block_d(x1_pos, y1_pos, x2_pos, y2_pos);
}

void lcd_set_orientation() {
    lcd_send_command(CMD_DATASCAN_DIR); // data scan directon.
    if (orientation == ORIENTATION_NORMAL)
        lcd_send_data(LCD_ORIENTATION_NORMAL);
    else
        lcd_send_data(LCD_ORIENTATION_INVERTED);
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

TBool column_is_a_junction(uint8_t c) {
    switch (c) {
        case 0:
        case 1:
        case 60:
        case 61:
        case 120:
        case 121:
        case 180:
        case 181:
        case 238:
        case 239:
            return True;
            break;
        default:
            return False;
            break;
    }
}

TBool page_not_on_hline(uint8_t p){
    // TODO: fix
    switch(p){
        case 4:
        case 8:
        case 12:
        case 16:
        case 20:
            return False;
            break;
        default:
            return True;
            break;
    }
}

void lcd_draw_fullsize_hgridline(uint8_t line, uint8_t data) {
    uint8_t x1_pos = 0;
    uint8_t x2_pos = LCD_WIDTH;
    uint8_t page = PAGE(line);
    for (uint8_t column = x1_pos; column < x2_pos; column++) {
        lcd_prepare_send_data(column, page, column, page);
        if (column_is_a_junction(column))
            lcd_send_data(LCD_BLACK_PAGE);
        else
            lcd_send_data(data);
    }
}

void lcd_send_page(uint8_t page, uint8_t col, uint8_t data) {
    lcd_prepare_send_data(col, page, col, page);
    lcd_send_data(data);
}

void lcd_draw_vgrid_lines(uint8_t start_line) {
    for (uint8_t p = PAGE(start_line); p < LCD_MAX_PAGES; p++) {
        if (page_not_on_hline(p)) {
            for (uint8_t c = 0; c < LCD_WIDTH; c++) {
                if (column_is_a_junction(c))
                    lcd_send_page(p, c, LCD_BLACK_PAGE);
            }
        }
    }
}
//
//void lcd_draw_bit_graph_column(size_t column, uint16_t value) {
//    lcd_prepare_send_data(column, LCD_GRAPH_START_PAGE, column, PAGE(LCD_HEIGHT));
//    for (uint8_t page = 0; page < LCD_GRAPH_HEIGTH; page++) {
//        if ((value >> (2 * page)) & 0x3) lcd_send_data(LCD_BLACK_PAGE);
//        else lcd_send_data(LCD_WHITE_PAGE);
//    }
//}
//
//void lcd_draw_bit_mark_column(size_t column) {
//    lcd_prepare_send_data(column, LCD_GRAPH_START_PAGE, column, PAGE(LCD_HEIGHT));
//    for (uint8_t page = 0; page < LCD_GRAPH_HEIGTH; page++) {
//        if (page%2==0) lcd_send_data(LCD_BLACK_PAGE);
//        else lcd_send_data(LCD_WHITE_PAGE);
//    }
//}
//
//void lcd_draw_scope_column(size_t column, uint16_t value) {
//    uint8_t mark_page, mark_bit;
//    mark_bit = (value % LCD_GRAPH_HEIGTH) % PAGE_HEIGTH;
//    mark_page = value / LCD_GRAPH_HEIGTH;
//    lcd_prepare_send_data(column, LCD_GRAPH_START_PAGE, column, PAGE(LCD_HEIGHT));
//    for (uint8_t page = 0; page < LCD_GRAPH_HEIGTH; page++) {
//        if (page == mark_page)
//            lcd_send_data(mark_bit);
//        else
//            lcd_send_data(LCD_WHITE_PAGE);
//    }
//}
//
//void lcd_send_page_mark(uint8_t column, uint8_t page, uint8_t polarity) {
//    lcd_prepare_send_data(column, page, column, page);
//    if (polarity == BLACK_OVER_WHITE) lcd_send_data(LCD_BLACK_PAGE);
//    else lcd_send_data(LCD_WHITE_PAGE);
//}
//
//void lcd_send_page(uint8_t column, uint8_t page, uint8_t value, uint8_t polarity) {
//    lcd_prepare_send_data(column, page, column, page);
//    if (polarity == BLACK_OVER_WHITE) lcd_send_data(value);
//    else lcd_send_data(~value);
//}
// </editor-fold>

void display_message(const char * message) {
    lcd_fill_block(2, 98, 158, 112);
    lcd_write_string(message, 4, 101, BigFont, WHITE_OVER_BLACK);
}

void lcd_demo() {
    uint8_t d = LCD_BLACK_PAGE;
    for (int page = 0; page < 50; page++) {
        lcd_prepare_send_data(0, page, LCD_WIDTH, page);
        if (page % 2 == 0) d = LCD_WHITE_PAGE;
        else d = LCD_BLACK_PAGE;
        for (int col = 0; col < LCD_WIDTH; col++) {
            if (col % 8 == 0) d = ~d;
            lcd_send_data(d);
        }
    }
}

void lcd_demo1() {
    char message[32];
    lcd_fill_block(0, 0, 160, 114);
    display_message("lcd_fill_block()");
    __delay_ms(1000);

    lcd_clear_block(158, 112, 2, 2);
    display_message("lcd_clear_block()");
    __delay_ms(1000);

    for (uint8_t battery = 0; battery <= 100; battery += 10) {
        lcd_battery_info(4, 4, battery);
        sprintf(message, "Battery : %d%%", battery);
        display_message(message);
        __delay_ms(1);
    }
    //    lcd_clear_block(2, 2, 156, 98);
    //    display_message("lcd_draw_bitmap()");
    //    for(uint8_t y = 5; y < 96-demo_bitmap_data.image_height; y = y + 30) {
    //        for(uint8_t x = 5; x < 150 - demo_bitmap_data.image_width; x = x + 30) {
    //            lcd_draw_bitmap(x, y, &demo_bitmap_data);
    //            __delay_ms(300);
    //            lcd_clear_block(x,y, x+demo_bitmap_data.image_width, y+demo_bitmap_data.image_height);
    //        }
    //    }

    lcd_clear_block(2, 2, 158, 98);
    lcd_write_string("Hello..!!", 3, 3, MediumFont, BLACK_OVER_WHITE);
    display_message("Black On White");
    __delay_ms(1000);

    lcd_fill_block(2, 2, 158, 98);
    __delay_ms(1000);
    lcd_clear_block(2, 2, 158, 98);
    lcd_write_string(" Hello..!! ", 3, 3, MediumFont, WHITE_OVER_BLACK);
    display_message("White On Black");
    __delay_ms(1000);

    lcd_clear_block(2, 2, 158, 98);
    lcd_write_string("1234", 3, 3, BigFont, BLACK_OVER_WHITE);
    display_message("I am Big");
    __delay_ms(1000);

}

