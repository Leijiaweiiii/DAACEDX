#include "lcd.h"

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
    switch (position % PAGE_HEIGTH) {
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
    if(update_page(&lcd_buffer[PAGE(y_pos)][x_pos], y_pos, polarity))/* Update bounding box only if the bit actually changed the state*/
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

    for (; x0_pos <= x1_pos; x0_pos=x0_pos+1) {
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
    lcd_send_command(CMD_EXTENSION_1);
    lcd_send_command(CMD_PAGE_ADD); // Row address.
    lcd_send_data(0); // Start row address.
    lcd_send_data(LCD_MAX_ADDRESS); // End row address.

    lcd_send_command(CMD_COL_ADD); // Column address.
    lcd_send_data(0x00); // Start column address.
    lcd_send_data(LCD_WIDTH); // End column address.

    lcd_send_command(CMD_WRITE_DATA);

    // Clear only the area that we're using
    for (uint16_t index = 0; index < (50+PAGE(LCD_MAX_ADDRESS)) * LCD_WIDTH; index++) {
        lcd_send_data(LCD_WHITE_PAGE);
    }
}
#ifndef LCD_DIRECT_ACCESS
void lcd_refresh(UpdateBoundary * box) {
    uint8_t column, max_column, page;
    // Don't even try to update unchanged box
    if (!box->changed) return;

    for (page = box->min_y / PAGE_HEIGTH; page < box->max_y / PAGE_HEIGTH; page=page+1) {
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

        for (; column <= max_column; column=column+1) {
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

    for (j = 0; j < font->height; j=j+1) {
        for (i = 0; i < font->char_descriptors[c].width; i=i+1) {
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
    uint8_t column,start_page,end_page,data,heigh_in_bytes;
    int8_t i;
    const uint8_t *bitmap,*column_start;
    heigh_in_bytes = (font->height%8==0)?font->height/8:font->height/8+1;
    if ((c < font->char_start) || (c > font->char_end)) return 0;
    if( ! START_OF_PAGE(y_pos)) return 0; // Don't drow not on the edge of the page
    c = c - font->char_start; // 'c' now become index to tables.
    bitmap = font->bitmap + font->char_descriptors[c].offset;
    start_page = PAGE(y_pos) + Y_OFFSET;
    end_page = PAGE(y_pos+font->height) + Y_OFFSET;
    /*
     *      for column between x and x + char width in bits
     *          set address for writing a column
     *          for index between PAGE(font heigh) and 0
     *              draw page (PAGE(Y),column,bitmap[index])
     */
    for(column = x_pos;column < x_pos+font->char_descriptors[c].width;column++){
        lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
        lcd_send_command(CMD_PAGE_ADD); // Row address.
        lcd_send_data(start_page); // Start row address.
        lcd_send_data(end_page); // End row address.
        lcd_send_command(CMD_COL_ADD); // Column address.
        lcd_send_data(column); // Start column address.
        lcd_send_data(column); // End column address.
        lcd_send_command(CMD_WRITE_DATA); // Write data.
        // TODO: Calculate carefully
        for (i=heigh_in_bytes ;i>font->truncate;i=i-1){
            data = bitmap[i-1];
            if(polarity == BLACK_OVER_WHITE)
                lcd_send_data(data);
            else
                lcd_send_data(~data);
        }
        bitmap += heigh_in_bytes;
    }

    return (font->char_descriptors[c].width);
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

// TODO: Implement lcd_get_block_d
void lcd_send_block_d(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos,uint8_t polarity) {
    if (x1_pos > x2_pos) SWAP(x1_pos, x2_pos);
    if (y1_pos > y2_pos) SWAP(y1_pos, y2_pos);
    //don't fill blocks not 8 bit hight
//    if (! (START_OF_PAGE(y1_pos) && START_OF_PAGE(y2_pos + 1))) return;
    
    for (uint8_t column = x1_pos; column < x2_pos; column = column + 1) {
        lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
        lcd_send_command(CMD_COL_ADD); // Column address.
        lcd_send_data(column); // Start column address.
        lcd_send_data(column); // End column address.
        
        lcd_send_command(CMD_PAGE_ADD); // Row address.
        lcd_send_data(PAGE(y1_pos)+Y_OFFSET); // Start row address.
        lcd_send_data(PAGE(y2_pos+1)+Y_OFFSET); // End row address.
        lcd_send_command(CMD_WRITE_DATA); // Write data.
        for (uint8_t page = PAGE(y1_pos); page < PAGE(y2_pos+1); page = page + 1) {
            if(polarity==BLACK_OVER_WHITE)
                lcd_send_data(LCD_BLACK_PAGE);
            else
                lcd_send_data(LCD_WHITE_PAGE);
        }
    }
}

void lcd_fill_block_d(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_send_block_d(x1_pos,y1_pos,x2_pos,y2_pos,BLACK_OVER_WHITE);
}

void lcd_clear_block_d(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_send_block_d(x1_pos,y1_pos,x2_pos,y2_pos,WHITE_OVER_BLACK);
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

uint8_t lcd_string_lenght(const char* str_ptr, const FONT_INFO *font) {
    uint8_t strlng = 0;
    if (str_ptr == NULL) return 0;
    while (*str_ptr) {
        strlng += font->char_descriptors[*str_ptr].width;
        strlng += font->character_spacing;
        ++str_ptr;
    }
    return strlng;
}
void lcd_write_string_d(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    if (str_ptr == NULL) return;

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
    lcd_send_data(0x35); // 16 Volts.
    lcd_send_data(0x04);

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
    lcd_send_command(CMD_INVERSION_OFF); // Pixel inversion OFF.

    lcd_send_command(CMD_EXTENSION_2); // Extension2 command.
    lcd_send_command(CMD_PWR_SRC_INT); // Use internel oscillator.

    lcd_send_command(CMD_EXTENSION_1); // Extension1 command.
    lcd_send_command(CMD_ICON_DISABLE); // Disable ICON RAM.
    
//    lcd_send_command(CMD_SET_SCROLL_AREA);
//    lcd_send_data(100);           //  Start at line 0
//    lcd_send_data(LCD_HEIGHT);  //  End at LCD_HEGHT
//    lcd_send_data(LCD_HEIGHT);  //  LCD_HEGHT lines to display
//    lcd_send_data(0x01);        // Top mode

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
}

void lcd_decrease_contrast() {
    lcd_send_command(CMD_EXTENSION_1);
    lcd_send_command(CMD_DISPLAY_ON);
    lcd_send_command(CMD_VOP_CON_DEC_VOP);
}

#ifndef LCD_DIRECT_ACCESS
void lcd_set_pixel(uint8_t x_pos, uint8_t y_pos) {
    lcd_set_pixel_b(x_pos, y_pos);
}

void lcd_clear_pixel(uint8_t x_pos, uint8_t y_pos) {
    lcd_clear_pixel_b(x_pos, y_pos);
}

void lcd_draw_line(uint8_t x0_pos, uint8_t y0_pos, uint8_t x1_pos, uint8_t y1_pos, uint8_t polarity) {
    lcd_draw_line_b(x0_pos, y0_pos, x1_pos, y1_pos, polarity);
}

void lcd_draw_vline(uint8_t x_pos, uint8_t y0_pos, uint8_t y1_pos, uint8_t polarity) {
    lcd_draw_vline_b(x_pos, y0_pos, y1_pos, polarity);
}

void lcd_draw_hline(uint8_t x0_pos, uint8_t x1_pos, uint8_t y_pos, uint8_t polarity) {
    lcd_draw_hline_b(x0_pos, x1_pos, y_pos, polarity);
}
#endif
void lcd_write_char(unsigned int c, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    lcd_write_char_d(c, x_pos, y_pos, font, polarity);
}

void lcd_write_string(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    lcd_write_string_d(str_ptr, x_pos, y_pos, font, polarity);
//    lcd_write_string_b(str_ptr, x_pos, y_pos, font, polarity);
//    lcd_refresh(&full_screen_update_boundary);
}

void lcd_write_integer(const int Int, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    char msg[10];
    sprintf(msg, "%d", Int);
    lcd_write_string_d("       ", x_pos, y_pos, font, polarity);
    lcd_write_string_d(msg, x_pos, y_pos, font, polarity);
}
// TODO: Implement
void lcd_draw_bitmap(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
//    lcd_draw_bitmap_d(x_pos, y_pos, bitmap_data);
}
// TODO: Implement
void lcd_battery_info(uint8_t x_pos, uint8_t y_pos, uint8_t battery_percentage) {
    if (battery_percentage > 100) battery_percentage = 100;
 //   lcd_battery_info_d(x_pos, y_pos, battery_percentage);
//    lcd_refresh(&full_screen_update_boundary);
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
        lcd_send_command(LCD_ORIENTATION_INVERTED);
}
// </editor-fold>

