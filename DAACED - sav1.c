/* ===========================================================================
    Project : DAACED
    Version 1.0
 
 *  File:   DAACED.c
 *  Author: Eli
 *  Created on Sept 15, 2017
 
    Global R&D ltd. 04_9592201    ;  www.global_rd.com
    Eli Jacob 054_8010330         ;  eli@global_rd.com
   ===========================================================================*/
// <editor-fold defaultstate="collapsed" desc="Configuration">

// PIC18F65K40 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1L
 #pragma config FEXTOSC = LP     //  (LP (crystal oscillator) optimized for 32.768 kHz; PFM set to low power)
 #pragma config RSTOSC = HFINTOSC_64MHZ// Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
 #pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
 #pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
 #pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
 #pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
 #pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
 #pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
 #pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
 #pragma config BORV = VBOR_2P45 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
 #pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
 #pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
 #pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
 #pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
 #pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
 #pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
 #pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

// CONFIG3H
 #pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
 #pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
 #pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
 #pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
 #pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
 #pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG4H
 #pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
 #pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
 #pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
 #pragma config SCANE = OFF      // Scanner Enable bit (Scanner module is NOT available for use, SCANMD bit is ignored)
 #pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
 #pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
 #pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

// CONFIG5H

// CONFIG6L
 #pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
 #pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
 #pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
 #pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG6H
 #pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//Configuration End
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Includes">
#include <xc.h>
#include "DAACED.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC_init">
void PIC_init(void)
{
    // 0 = OUTPUT, 1 = INPUT
    // 0 = DIGITAL, 1 = ANALOG
    TRISA  = 0b11111111;        // ADC inputs 0..3
    ANSELA = 0b00001111;
    OSCENbits.ADOEN = 1;        // Enable ADC oscillator;
    
    TRISB  = 0b11111111;        // 
    ANSELB = 0b00000000;

    TRISC  = 0b11100101;        // C6 = TX, C7 RX
                                // C3 = DP_SCL(OP), C4 = DP_SDA(OP)

    TRISD  = 0b00110111;        // EEPROM SPI SDI=5 SCK=6 SDO=7
    ANSELD = 0b00000000;

    TRISE  = 0b10111010;        // E0 = POWER(OP), E6 = BL_EN(OP)
    ANSELE = 0b00000000;

    TRISF  = 0b00000000;        // F3 = DP_CS(OP), F4 = DP_A0(OP), F6 = DP_RST(OP)
    ANSELF = 0b00100000;        // DAC
    DAC1CON0=0b10100000;        // DAC enabled output on pin13 (RF5) with reference from VDD & VSS

    TRISG  = 0x00;              // CON8, Debug Header.
    ANSELG = 0x00;

}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="SPI">
void spi_init() {
    RC4PPS = 0x1A;                          // data-output
    SSP1DATPPS = 0x1D;                      // PPS to unused PIN.
    RC3PPS = 0x19;                          // clock output

    SSP1STAT &= 0x3F;                       // Power on state
    SSP1CON1 = 0x00;                        // Power on state
    SSP1CON1bits.SSPM = 0x01;               // SPI Clock FOSC_DIV_16
    SSP1STATbits.CKE = 1;                   // Data transmission on rising edge
    SSP1STATbits.SMP = 0;                   // Data sampled/latched at middle of clock.
    SSP1CON1bits.SSPEN = 1;                 // Enable synchronous serial port
    PIE3bits.SSP1IE = 0;                    // Disable interrupt.
}

uint8_t spi_write(uint8_t data) {
    LCD_CS_SELECT();
    unsigned char temp_var = SSP1BUF;        // Clear buffer.
    PIR3bits.SSP1IF = 0;                    // clear interrupt flag bit
    SSP1CON1bits.WCOL = 0;                  // clear write collision bit if any collision occurs

    SSP1BUF = data;                         // transmit data
    while(!PIR3bits.SSP1IF);                 // waiting for the process to complete
    PIR3bits.SSP1IF = 0;                    // clear interrupt flag bit
    LCD_CS_DESELECT();
    return(SSP1BUF);                        // return receive data
}
//SPI End
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Backlight PWM Function">
void initialize_backlight() {
    uint8_t PRvalue, prescalar;
    TRISEbits.TRISE6 = 0;       // Disable output.
    PWM6CON = 0;                // Clear register.
    find_optimal_PWM_settings(1000, &PRvalue, &prescalar);
    
    // Initialize Timer2.
    PIR5bits.TMR2IF = 0;        // Clear timer interrupt flag.
    T2CLKCON = 0b001;           // Timer2 clock source = Fosc/4;
    T2CONbits.CKPS = find_set_bit_position(prescalar);
    PR2 = PRvalue;
}
void set_backlight(uint8_t duty_cycle) {
    PWM6CONbits.EN = 0;
    if(duty_cycle == 0) {
        LATEbits.LATE6 = 1;
    } else if(duty_cycle > 0 && duty_cycle < 100) {
        uint16_t ON_value;
        uint8_t PR_value = PR2;
        ON_value = (duty_cycle * (PR_value + 1 ))/25;
        PWM6DCH = (ON_value >> 2);
        PWM6DCL = ((ON_value & 0x0003) << 6);
        T2CONbits.ON = 1;                   // Start timer.
        PWM6CONbits.EN = 1;                 // Enable PWM.
        LATEbits.LATE6 = 0;
        RE6PPS = 0x0A;                      // PPS setting.
        PWM6CONbits.POL = 1;                // Polarity.
    } else if(duty_cycle == 100) {
        LATEbits.LATE6 = 0;
    }
}

uint8_t find_optimal_PWM_settings(int32_t freq, uint8_t *selectedPRvalue, uint8_t *selectedPrescalar) {
    uint16_t tempPRvalue = 0;
    int32_t tempactualfreq = 0;
    int32_t freqdiffrence = INT32_MAX;

    *selectedPRvalue = 0;

    for (uint16_t prescaler = 1; prescaler <= 128; prescaler *= 2) {
        tempPRvalue = (((_XTAL_FREQ / (4 * prescaler)) / (freq)) - 1);

        if (tempPRvalue > 0xFF) continue;

        tempactualfreq = ((_XTAL_FREQ / (4 * prescaler)) / ((tempPRvalue + 1)));

        if (freqdiffrence > abs(tempactualfreq - freq)) {
            *selectedPRvalue = tempPRvalue;
            *selectedPrescalar = prescaler;
            freqdiffrence = abs(tempactualfreq - freq);
        }
    }

    tempactualfreq = ((_XTAL_FREQ / (4 * (*selectedPrescalar))) / ((*selectedPRvalue) + 1));
    int32_t lowestfreq = ((_XTAL_FREQ / (4 * (*selectedPrescalar))) / ((*selectedPRvalue) + 2));

    if (abs(lowestfreq - freq) < abs(tempactualfreq - freq)) {
        (*selectedPRvalue)++;
    }
    tempactualfreq = ((_XTAL_FREQ / (4 * (*selectedPrescalar))) / ((*selectedPRvalue) + 1));
    return 0;
}
uint8_t is_power_of2(uint8_t n) {
    return n && (!(n & (n-1)));
}
uint8_t find_set_bit_position(uint8_t n) {
    if (!is_power_of2(n))
        return 0;
    uint8_t i = 1, pos = 1;
    while (!(i & n)) {
        i = i << 1;
        ++pos;
    }
    return pos;
}
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
    for(uint8_t index = 0; index < no_of_bytes; index++) {
        spi_write(data[index]);
    }
}

void lcd_update_boundingbox(uint8_t x_min, uint8_t y_min, uint8_t x_max, uint8_t y_max) {
    if(x_min < x_update_min) x_update_min = x_min;
    if(x_max > x_update_max) x_update_max = x_max;
    if(y_min < y_update_min) y_update_min = y_min;
    if(y_max > y_update_max) y_update_max = y_max;
}

void lcd_draw_pixel_b(uint8_t x_pos, uint8_t y_pos, uint8_t polarity) {
    if((x_pos >= LCD_WIDTH) || (y_pos >= LCD_HEIGHT)) {
        return;
    }
    y_pos += Y_OFFSET;
    if(polarity == BLACK_OVER_WHITE) lcd_buffer[x_pos + ((y_pos)/8)*LCD_WIDTH] |= (BIT(7-(y_pos%8)));
    else lcd_buffer[x_pos + ((y_pos)/8)*LCD_WIDTH] &= ~(BIT(7-(y_pos%8)));
    lcd_update_boundingbox(x_pos, y_pos, x_pos, y_pos);
}

void lcd_set_pixel_b(uint8_t x_pos, uint8_t y_pos) {
    lcd_draw_pixel_b(x_pos, y_pos, BLACK_OVER_WHITE);
}

void lcd_clear_pixel_b(uint8_t x_pos, uint8_t y_pos) {
    lcd_draw_pixel_b(x_pos, y_pos, WHITE_OVER_BLACK);
}

void lcd_draw_line_b(uint8_t x0_pos, uint8_t y0_pos, uint8_t x1_pos, uint8_t y1_pos, uint8_t polarity) {
    uint8_t steep = abs(y1_pos - y0_pos) > abs(x1_pos - x0_pos);
    if(steep) {
        SWAP(x0_pos, y0_pos);
        SWAP(x1_pos, y1_pos);
    }

    if(x0_pos > x1_pos) {
        SWAP(x0_pos, x1_pos);
        SWAP(y0_pos, y1_pos);
    }

    lcd_update_boundingbox(x0_pos, y0_pos, x1_pos, y1_pos);

    uint8_t dx, dy;
    dx = x1_pos - x0_pos;
    dy = abs(y1_pos - y0_pos);

    int8_t error = dx/2;
    int8_t y_step;

    if(y0_pos < y1_pos) y_step = 1;
    else y_step = -1;

    for( ; x0_pos <= x1_pos; x0_pos++) {
        if(steep) {
            lcd_draw_pixel_b(y0_pos, x0_pos, polarity);
        }
        else {
            lcd_draw_pixel_b(x0_pos, y0_pos, polarity);
        }

        error -= dy;

        if(error < 0) {
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

void lcd_clear_data_ram() {
    lcd_send_command(CMD_EXTENSION_1);
    lcd_send_command(CMD_PAGE_ADD);                             // Row address.
    lcd_send_data(0x00);                                        // Start row address.
    lcd_send_data(20);                                          // End row address.

    lcd_send_command(CMD_COL_ADD);                              // Column address.
    lcd_send_data(0x00);                                        // Start column address.
    lcd_send_data(240);                                         // End column address.

    lcd_send_command(CMD_WRITE_DATA);

    for(uint16_t index = 0; index < 240*160; index++){
        lcd_send_data(0x00);
    }
}

void lcd_refresh() {
    uint8_t column, max_column, page;

    for(page = 0; page < LCD_MAX_PAGES; page++){
        if(y_update_min >= ((page+1)*8)) continue;
        if(y_update_max < page*8) break;

        lcd_send_command(CMD_EXTENSION_1);                      // Extension1 command.
        lcd_send_command(CMD_PAGE_ADD);                         // Row address.
        lcd_send_data(page+6);                                  // Start row address.
        lcd_send_data(page+6);                                  // End row address.

        column = x_update_min;
        max_column = x_update_max;

        lcd_send_command(CMD_COL_ADD);                          // Column address.
        lcd_send_data(column);                                  // Start column address.
        lcd_send_data(max_column);                              // End column address.

        lcd_send_command(CMD_WRITE_DATA);                       // Write data.

        for( ; column <= max_column; column++) {
            lcd_send_data(lcd_buffer[(LCD_WIDTH*page)+column]); // Sending data from buffer.
        }
    }

    x_update_min = LCD_WIDTH-1;
    x_update_max = 0;
    y_update_min = LCD_HEIGHT-1;
    y_update_max = 0;
}

uint8_t lcd_write_char_b(unsigned int c, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    uint8_t i, j, line;
    const uint8_t *bitmap;

    if ((c < font->char_start) || (c > font->char_end)) return 0;

    c = c - font->char_start;       // 'c' now become index to tables.
    bitmap = font->bitmap + font->char_descriptors[c].offset;

    for (j = 0; j < font->height; ++j) {
        for (i = 0; i < font->char_descriptors[c].width; ++i) {
            if (i % 8 == 0) {
                line = bitmap[(font->char_descriptors[c].width + 7) / 8 * j + i / 8]; // line data
            }
            if (line & 0x80) {
                lcd_draw_pixel_b(x_pos+i, y_pos+j, polarity);
            } else {
                lcd_draw_pixel_b(x_pos+i, y_pos+j, !polarity);
            }
            line = line << 1;
        }
        
    }
    return(font->char_descriptors[c].width);
}

void lcd_fill_block_b(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    if(x1_pos > x2_pos) SWAP(x1_pos, x2_pos);
    if(y1_pos > y2_pos) SWAP(y1_pos, y2_pos);
    for(uint8_t x = x1_pos; x < x2_pos; x++) {
        for(uint8_t y = y1_pos; y < y2_pos; y++) {
            lcd_set_pixel_b(x, y);
        }
    }
}

void lcd_clear_block_b(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    if(x1_pos > x2_pos) SWAP(x1_pos, x2_pos);
    if(y1_pos > y2_pos) SWAP(y1_pos, y2_pos);
    for(uint8_t x = x1_pos; x < x2_pos; x++) {
        for(uint8_t y = y1_pos; y < y2_pos; y++) {
            lcd_clear_pixel_b(x, y);
        }
    }
}

void lcd_write_string_b(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    if(str_ptr == NULL) return;
    
	while(*str_ptr) {
		x_pos += lcd_write_char_b(*str_ptr, x_pos, y_pos, font, polarity);
		++str_ptr;
        if(*str_ptr) {
			if(polarity == WHITE_OVER_BLACK) {
				lcd_fill_block_b(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
			} else {
				lcd_clear_block_b(x_pos, y_pos, x_pos + font->character_spacing, y_pos + font->height);
			}
            x_pos += font->character_spacing;
        }
        if(x_pos >= LCD_WIDTH) {
            y_pos += (font->height + 1);
            x_pos %= LCD_WIDTH;
            y_pos %= LCD_HEIGHT;
        }
	}
}
void lcd_draw_bitmap_b(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
    for(uint8_t y = 0; y < bitmap_data->image_height; y++) {
        for(uint8_t x = 0; x < (bitmap_data->image_width+7)/8; x++) {
            uint8_t data = *(bitmap_data->image_data+((y*((bitmap_data->image_width+7)/8)+x)));
            for(int8_t bit_index = 7; bit_index >= 0; bit_index--) {
                if(data & BIT(bit_index)) {
                    lcd_set_pixel_b((x*8) + (7-bit_index) + x_pos, y + y_pos);
                }else {
                    lcd_clear_pixel_b((x*8) + (7-bit_index) + x_pos, y + y_pos);
                }
            }
        }
    }
}
void lcd_battery_info_b(uint8_t x_pos, uint8_t y_pos, uint8_t battery_percentage) {
    battery_percentage %= 100;
    lcd_draw_bitmap_b(x_pos, y_pos, &battery_bitmap_data);
    uint8_t no_of_bars = battery_percentage/16;
    for(uint8_t bar = 0; bar <= no_of_bars; bar++){
        lcd_draw_vline_b(x_pos+(bar*2), y_pos+2, y_pos+3+2, BLACK_OVER_WHITE);
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LCD functions implementations">
void lcd_init() {
    memset(lcd_buffer, 0, sizeof(lcd_buffer));              // Clear LCD Buffer.
    lcd_update_boundingbox(0,0,LCD_WIDTH-1, LCD_HEIGHT-1);  // Reset refresh window.

    LCD_CS_DESELECT();

    __delay_ms(10);
    lcd_reset();                                            // Reset LCD.
    lcd_send_command(CMD_EXTENSION_1);                      // Extension1 command.
    lcd_send_command(CMD_SET_MODE_MASTER);                  // Enable master mode.

    lcd_send_command(CMD_EXTENSION_2);                      // Extension2 command.
    lcd_send_command(CMD_AUTO_READ);                        // Disable auto read.
    lcd_send_data(0x9F);

    lcd_send_command(CMD_EXTENSION_1);                      // Extension1 command.
    lcd_send_command(CMD_SLEEP_OUT);                        // Sleep out.
    lcd_send_command(CMD_DISPLAY_OFF);                      // Display OFF.
    lcd_send_command(CMD_INT_OSC_ON);                       // Internal oscillator ON.

    lcd_send_command(CMD_POWER_CON);                        // Power control.
    lcd_send_data(0x0B);                                    // Regulator, Follower and Booster ON.
    lcd_send_command(CMD_SET_VOP);                          // Set Vop.
    lcd_send_data(0x35);                                    // 16 Volts.
    lcd_send_data(0x04);

    lcd_send_command(CMD_EXTENSION_2);                      // Extension2 commands.
    lcd_send_command(CMD_ANALOG_CKT);                       // Analog Circuit set.
    lcd_send_data(0x00);
    lcd_send_data(0x01);                                    // Booster efficiency 1.
    lcd_send_data(0x02);                                    // LCD bias 1/12

    lcd_send_command(CMD_BOOSTER_LVL);                      // Booster level.
    lcd_send_data(0xFB);                                    // x10.

    lcd_send_command(CMD_EXTENSION_1);                      // Extension1 commands.
    lcd_send_command(CMD_DISPLAY_MODE);                     // Display mode.
    lcd_send_data(0x10);                                    // Monochrome mode.
    lcd_send_command(CMD_DISPLAY_CONTROL);                  // Display control.
    lcd_send_data(0x00);                                    // No clock division.
    lcd_send_data(0x7F);                                    // 1/128 duty.
    lcd_send_data(0x00);                                    // ??

    lcd_send_command(CMD_DATASCAN_DIR);                     // data scan directon.
    lcd_send_data(0x01);
    lcd_send_command(0xA6);

    lcd_send_command(CMD_DATA_FORMAT_LSB);                  // LSB First.

    lcd_send_command(CMD_INVERSION_OFF);                    // Pixel inversion OFF.

    lcd_send_command(CMD_EXTENSION_2);                      // Extension2 command.
    lcd_send_command(CMD_PWR_SRC_INT);                      // Use internel oscillator.

    lcd_send_command(CMD_EXTENSION_1);                      // Extension1 command.
    lcd_send_command(CMD_ICON_DISABLE);                     // Disable ICON RAM.

    lcd_clear_data_ram();                                   // Clearing data RAM.

    lcd_send_command(CMD_DISPLAY_ON);                       // Turn ON display.
}

void lcd_clear() {
    memset(lcd_buffer,0, sizeof(lcd_buffer));
    lcd_update_boundingbox(0,0,LCD_WIDTH-1, LCD_HEIGHT-1);
    lcd_refresh();
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

void lcd_set_pixel(uint8_t x_pos, uint8_t y_pos) {
    lcd_set_pixel_b(x_pos, y_pos);
    lcd_refresh();
}

void lcd_clear_pixel(uint8_t x_pos, uint8_t y_pos) {
    lcd_clear_pixel_b(x_pos, y_pos);
    lcd_refresh();
}

void lcd_draw_line(uint8_t x0_pos, uint8_t y0_pos, uint8_t x1_pos, uint8_t y1_pos, uint8_t polarity) {
    lcd_draw_line_b(x0_pos, y0_pos, x1_pos, y1_pos, polarity);
    lcd_refresh();
}

void lcd_draw_vline(uint8_t x_pos, uint8_t y0_pos, uint8_t y1_pos, uint8_t polarity) {
    lcd_draw_vline_b(x_pos, y0_pos, y1_pos, polarity);
    lcd_refresh();
}

void lcd_draw_hline(uint8_t x0_pos, uint8_t x1_pos, uint8_t y_pos, uint8_t polarity) {
    lcd_draw_hline_b(x0_pos, x1_pos, y_pos, polarity);
    lcd_refresh();
}

void lcd_write_char(unsigned int c, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    lcd_write_char_b(c, x_pos, y_pos, font, polarity);
    lcd_refresh();
}

void lcd_write_string(const char* str_ptr, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    lcd_write_string_b(str_ptr, x_pos, y_pos, font, polarity);
    lcd_refresh();
}

void lcd_write_integer(const int Int, uint8_t x_pos, uint8_t y_pos, const FONT_INFO *font, uint8_t polarity) {
    char msg[10];
    sprintf(msg,"%d",Int);
    lcd_write_string_b("       ", x_pos, y_pos, font, polarity);
    lcd_write_string_b(msg, x_pos, y_pos, font, polarity);
    lcd_refresh();
}


void lcd_draw_bitmap(uint8_t x_pos, uint8_t y_pos, const bitmap_data_t *bitmap_data) {
    lcd_draw_bitmap_b(x_pos, y_pos, bitmap_data);
    lcd_refresh();
}

void lcd_battery_info(uint8_t x_pos, uint8_t y_pos, uint8_t battery_percentage) {
    lcd_battery_info_b(x_pos, y_pos, battery_percentage);
    lcd_refresh();
}

void lcd_fill_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_fill_block_b(x1_pos, y1_pos, x2_pos, y2_pos);
    lcd_refresh();
}

void lcd_clear_block(uint8_t x1_pos, uint8_t y1_pos, uint8_t x2_pos, uint8_t y2_pos) {
    lcd_clear_block_b(x1_pos, y1_pos, x2_pos, y2_pos);
    lcd_refresh();
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM Functions">
// Helper functions.
void eeprom_spi_init() {
    EEPROM_CS_INIT();
    EEPROM_HOLD_INIT();
    EEPROM_WP_INIT();
    
    EEPROM_CS_DESELECT();
    EEPROM_HOLD_DIS();
    EEPROM_WP_DIS();
    
    RD7PPS = 0x1C;          // SPI2 dataout (RD7)
    SSP2DATPPS = 0x1D;      // SPI2 datain.
    RD6PPS = 0x1B;          // SPI2 clock.
    
    SSP2STAT &= 0x3F;
    SSP2CON1 = 0x00;        // power on state.
    SSP2CON1bits.SSPM = 0b0010;
    SSP2STATbits.SMP = 0;

    SSP2CON1bits.CKP = 1;
    SSP2STATbits.CKE = 0;
    SSP2CON1bits.SSPEN = 1;
    SSP2CON1bits.WCOL = 0;
}

uint8_t eeprom_spi_write(uint8_t data) {
    uint8_t temp_var = SSP2BUF;     // Clear buffer.
    UNUSED(temp_var);
    PIR3bits.SSP2IF = 0;            // Clear interrupt flag bit
    SSP2CON1bits.WCOL = 0;          // Clear write collision bit if any collision occurs
    SSP2BUF = data;
    while (SSP2STATbits.BF == 0);
    PIR3bits.SSP2IF = 0;            // clear interrupt flag bit
    return SSP2BUF;
}

void eeprom_init() {
    eeprom_spi_init();
}

void eeprom_write_data(uint16_t address, uint8_t data) {
    eeprom_busy_wait();
    
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_WRSR);
    eeprom_spi_write(0x02);     // Enable Write Latch.
    EEPROM_CS_DESELECT();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_WREN);
    EEPROM_CS_DESELECT();
    
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_WRITE);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    eeprom_spi_write(data);
    EEPROM_CS_DESELECT();    
}
void eeprom_write_wdata(uint16_t address, uint16_t data) {
    eeprom_write_data(address,data & 0xFF);
    eeprom_write_data(address+1,(data>>8) & 0xFF);
}

uint8_t eeprom_read_data(uint16_t address) {
    uint8_t read_data;
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_data = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return(read_data);
}

uint16_t eeprom_read_wdata(uint16_t address) {
    uint16_t read_word;
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_word = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    address++;
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_word += (eeprom_spi_write(0x00)<<8);
    EEPROM_CS_DESELECT();
    return(read_word);
}

uint16_t eeprom_read_array(uint16_t address, uint8_t *data, uint16_t no_of_bytes){
    uint16_t index;
    
    if(address > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();
    
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    for(index = 0; index < no_of_bytes; index++) {
        if(address+index > EEPROM_MAX_SIZE) return (index);
        data[index] = eeprom_spi_write(0x00);
    }
    return (index);
}
void eeprom_busy_wait() {
    while(eeprom_read_status_reg() & 0x01);
}
uint8_t eeprom_read_status_reg() {
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_RDSR);
    uint8_t status_reg = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return status_reg;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Sinus Generator">
uint8_t sinus_table[32] = { 0x10,               // 11.25
                            0x13,               // 22.50
                            0x16,               // 33.75
                            0x18,               // 45.00
                            0x1B,               // 56.25
                            0x1D,               // 68.50
                            0x1E,               // 79.75
                            0x1F,               // 90.00
                            0x20,               // 101.25
                            0x1F,               // 112.50
                            0x1E,               // 123.75
                            0x1D,               // 135.00
                            0x1B,               // 146.25
                            0x18,               // 157.50
                            0x16,               // 168.75
                            0x13,               // 180.00
                            0x10,               // 191.25
                            0x0C,               // 202.50
                            0x09,               // 213.75
                            0x07,               // 225.00
                            0x04,               // 236.25
                            0x02,               // 247.50
                            0x01,               // 258.75
                            0x00,               // 270.00
                            0x00,               // 281.25
                            0x00,               // 292.50
                            0x01,               // 303.75
                            0x02,               // 315.00
                            0x04,               // 326.25
                            0x07,               // 337.50
                            0x09,               // 348.75
                            0x0C};              // 360.00

void sinus_dac_init() {
    TRISFbits.TRISF5 = 0;
    ANSELFbits.ANSELF5 = 0;
    DAC1CON0 = 0x00;
    DAC1CON0bits.DAC1EN = 1;    // DAC enabled. = 0b10100000;       // DAC enabled output on pin13 (RF5) with reference from VDD & VSS
    DAC1CON0bits.OE1 = 1;
    DAC1CON0bits.PSS = 0;       // Vdd
    DAC1CON0bits.NSS = 0;       // Vss
    LATEbits.LATE2=1;           // Driver Enable
}
void generate_sinus(uint8_t amplitude, uint16_t frequency, int16_t duration) {
    float sinus_time_period_us = (1000000UL/(frequency*2));
    uint32_t sinus_sample_update_period_us = (uint32_t)(((sinus_time_period_us)/32.0)+0.5f);
    int32_t no_of_cycles = ((int32_t)duration*1000)/sinus_time_period_us;
    
    sinus_dac_init();

    while(no_of_cycles--) {
        for(uint8_t count = 0; count < 32; count++) {
            dac_value = (sinus_table[count] * amplitude)>>2;
            if(dac_value > 31) dac_value = 31;
            if(dac_value < 0) dac_value = 0;
            DAC1CON1 = dac_value;
            for(uint16_t delay = 0; delay < sinus_sample_update_period_us; delay++)
                __delay_us(1);
        }
    }
    stop_sinus();
}

void stop_sinus() {
    T0CON0bits.T0EN = 0;                                    // Timer OFF
    DAC1CON0bits.EN = 0;                                    // DAC Off
    LATEbits.LATE2=0;                                       // Driver OFF
}
void Beep(void)
{
    generate_sinus(1,1000,50);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="MicroController ADC Interface">

void ADC_init()
{
    TRISA  = 0b11111111;        // ADC inputs 0..3
    ANSELA = 0b00001111;
//  ADCON0 = 0b10000000;        // Enable ADC	 - single byte mode	   return ADRESH;
    ADCON0 = 0b10000100;        // Enable ADC	 - single 10 bit mode	return (ADRESH<<8)|ADRESL;   
    ADCON1 = 0b00000001;        // Select ADC Double Sample
    ADCON2 = 0b00001000;        // Normal ADC operation
    ADCON3 = 0b00001000;        // Normal ADC operation
    ADCLK  = 0b00100000;        // ADC CLK = OSC/64
    ADREF  = 0b00000011;        // ADC connected to FVR
    FVRCON = 0b11000010;        // FVR set to 2048mV
}   

uint16_t ADC_Read(char selectedADC)
{
   ADPCH  = selectedADC;   // Select ADC input
   ADCON0bits.ADGO=1;      // Start conversion
   while (GO_nDONE);
   return (ADRESH<<8)|ADRESL;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Diagnostics">
// <editor-fold defaultstate="collapsed" desc="LCD demo functions">
void display_message(const char * message) {
    lcd_fill_block(2, 98, 158, 112);
    lcd_write_string(message, 4, 101, &tahoma_8ptFontInfo, WHITE_OVER_BLACK);
}

void lcd_demo() {
    char message[32];
    lcd_fill_block(0, 0, 160, 114);
    display_message("lcd_fill_block()");
    __delay_ms(1000);
    
    lcd_clear_block(158, 112, 2, 2);
    display_message("lcd_clear_block()");
    __delay_ms(1000);
    
    for(uint8_t battery = 0; battery <= 100; battery+=10) {
        lcd_battery_info(4,4,battery);
        sprintf(message, "Battery : %d%%", battery);
        display_message(message);
        __delay_ms(1);
    }
    lcd_clear_block(2, 2, 156, 98);
    display_message("lcd_draw_bitmap()");
    for(uint8_t y = 5; y < 96-demo_bitmap_data.image_height; y = y + 30) {
        for(uint8_t x = 5; x < 150 - demo_bitmap_data.image_width; x = x + 30) {
            lcd_draw_bitmap(x, y, &demo_bitmap_data);
            __delay_ms(300);
            lcd_clear_block(x,y, x+demo_bitmap_data.image_width, y+demo_bitmap_data.image_height);
        }
    }

    lcd_clear_block(2, 2, 158, 98);
    lcd_write_string("Hello..!!", 3, 3, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    display_message("Black On White");
    __delay_ms(1000);
    
    lcd_fill_block(2, 2, 158, 98);
    __delay_ms(1000);
    lcd_clear_block(2, 2, 158, 98);
    lcd_write_string(" Hello..!! ", 3, 3, &timesNewRoman_11ptFontInfo, WHITE_OVER_BLACK);
    display_message("White On Black");
    __delay_ms(1000);
    
    lcd_clear_block(2, 2, 158, 98);
    lcd_write_string("1234", 3, 3, &microsoftSansSerif_48ptFontInfo, BLACK_OVER_WHITE);
    display_message("I am Big");
    __delay_ms(1000);
    
}
//LCD demo functions End
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Backlight test">
void Diag_Backlight(void)
{//PWM Backlite
    char msg[10];
    uint8_t i=1;
    uint8_t j=0;
    lcd_write_string(" Backlight test", 3, 3, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    lcd_write_string(" ^ - Backlight ON", 3, 40, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    lcd_write_string(" _ - Backlight OFF", 3, 60, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    while (!Exit)
    {
        lcd_write_string("    ",100,90, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((BackLightLevel+i)<99)     BackLightLevel+=i;
                           else BackLightLevel=99;
                           lcd_write_char('^',100,90, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
                break;
                case KeyDw: if ((BackLightLevel-i)>0)      BackLightLevel-=i;
                           else BackLightLevel=0;
                           lcd_write_char('_',100,90, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
                break;
                case KeyIn: BackLightLevel=10;
                break;
                default:   lcd_write_char('.',100,90, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            }
            set_backlight(BackLightLevel);
            sprintf(msg,"Level: %d",BackLightLevel);
            lcd_write_string(msg, 3, 90, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
        }
        __delay_ms(100);
        if (Keypressed) j++;
        if (j>5){ if (i<11) i++;j=0;}//if after 100mS still pressed go faster
        if (!Keypressed) { i=1;j=0;} //if key not pressed go slow
        lcd_write_integer(i,120,90, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Keypad test">
void Diag_Keypad(void)
{
    lcd_write_string(" Keypad test", 3, 3, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    while (!Exit)
    {
        lcd_clear_block(20, 30, 80,110);
        switch (Key)
        {
            case KeySt: 
                lcd_write_char('1', 20, 30, &microsoftSansSerif_48ptFontInfo, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyRw: 
                lcd_write_char('2', 20, 30, &microsoftSansSerif_48ptFontInfo, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyBk: 
                lcd_write_char('3', 20, 30, &microsoftSansSerif_48ptFontInfo, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyDw: 
                lcd_write_char('4', 20, 30, &microsoftSansSerif_48ptFontInfo, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyUp: 
                lcd_write_char('5', 20, 30, &microsoftSansSerif_48ptFontInfo, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyIn: 
                lcd_write_char('6', 20, 30, &microsoftSansSerif_48ptFontInfo, BLACK_OVER_WHITE);
                Delay(100);
            break;
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Buzzer Test Menu">
void Diag_Buzzer_display(uint8_t m, TBool selected)
{
    uint8_t color;
    if (selected) color=WHITE_OVER_BLACK;
    else          color=BLACK_OVER_WHITE;
    switch (m)
    {
        case 0:
            lcd_clear_block(0, 0, 160, 114);
            lcd_write_string(" Buzzer test ", 3, 1, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" 1KHz    10% ", 3, 15, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" 1KHz   100% ", 3, 29, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" 1.5KHz  10% ", 3, 43, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" 1.5KHz 100% ", 3, 57, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" 2KHz    10% ", 3, 71, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" 2KHz   100% ", 3, 85, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            break;
        case 1:
            lcd_write_string(" 1KHz    10% ", 3, 15, &timesNewRoman_11ptFontInfo, color);
            break;
        case 2:
            lcd_write_string(" 1KHz   100% ", 3, 29, &timesNewRoman_11ptFontInfo, color);
            break;
        case 3:
            lcd_write_string(" 1.5KHz  10% ", 3, 43, &timesNewRoman_11ptFontInfo, color);
            break;
        case 4:
            lcd_write_string(" 1.5KHz 100% ", 3, 57, &timesNewRoman_11ptFontInfo, color);
            break;
        case 5:
            lcd_write_string(" 2KHz    10% ", 3, 71, &timesNewRoman_11ptFontInfo, color);
            break;
        case 6:
            lcd_write_string(" 2KHz   100% ", 3, 85, &timesNewRoman_11ptFontInfo, color);
            break;
    }
}
void DoBeep(uint8_t mode)
{
    switch (mode)
    {
        case 1:
            generate_sinus(1,1000,200);
            break;
        case 2:
            generate_sinus(10,1000,50);
            break;
        case 3:
            generate_sinus(1,1500,200);
            break;
        case 4:
            generate_sinus(10,1500,50);
            break;
        case 5:
            generate_sinus(1,2000,200);
            break;
        case 6:
            generate_sinus(10,2000,50);
            break;
    }
}
void Diag_Buzzer()
{
    uint8_t menu,prevmenu;

    menu=1;
    prevmenu=0;
    while (!Exit)
    {    
        if (prevmenu!=menu)
        {
            Diag_Buzzer_display(prevmenu,false);    //if prevmenu=0 Display All
                                             //else repaint previous selection
            Diag_Buzzer_display(menu,true);              //Display selected
            prevmenu=menu;
        }
        if (Keypressed)
        {
            lcd_write_char(' ' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
            switch (Key)
            {
                case KeyUp:if (menu>1) menu--;
                          lcd_write_char('5' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                    break;
                case KeyDw:if (menu<6) menu++; 
                          lcd_write_char('4' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                   break;
                case KeyIn: //select
                          lcd_write_char('6' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                          DoBeep(menu);
                          prevmenu=0;//Redraw menu
                    break;
                case KeyBk: //escape
                          lcd_write_char('3' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                          prevmenu=0;//Redraw menu
                    break;  
                case KeySt:  
                          lcd_write_char('1' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                    break;
                case KeyRw:  
                          lcd_write_char('2' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                    break;
            }
            while (Keypressed); // wait here till key is released
            lcd_write_char(' '    ,140,15,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
            lcd_write_char(menu+48,140,15,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Battery Test">
void Diag_Battery()
{
    lcd_write_string(" Battery ",3,10,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
    while (!Exit)
    {
        char message[32];
        uint16_t battery = ADC_Read(BATTERY);
        uint16_t battery_mV = battery*BAT_divider;
        uint8_t battery_level = (battery_mV/10)-320; // ((battery_mV-3200)*100)/(4200-3200)
        sprintf(message, " Battery level   : %d%% ", battery_level);
        lcd_write_string(message,3,30,&tahoma_8ptFontInfo,WHITE_OVER_BLACK);
        sprintf(message, " Battery voltage : %dmV ", battery_mV);
        lcd_write_string(message,3,50,&tahoma_8ptFontInfo,WHITE_OVER_BLACK);
        sprintf(message, " Battery reading : %d ", battery);
        lcd_write_string(message,3,70,&tahoma_8ptFontInfo,WHITE_OVER_BLACK);
        lcd_battery_info(130,10,battery_level);
        Delay(50);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="ADC test Menu">
void Diag_ADC_display(uint8_t m, TBool selected)
{
    uint8_t color;
    if (selected) color=WHITE_OVER_BLACK;
    else          color=BLACK_OVER_WHITE;
    switch (m)
    {
        case 0:
            lcd_clear_block(0, 0, 160, 114);
            lcd_write_string(" ADC test ", 3, 1, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" Accelerometer ", 3, 15, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" Microphone slow", 3, 29, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" Microphone fast", 3, 43, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            lcd_write_string(" Envelope ", 3, 57, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            break;
        case 1:
            lcd_write_string(" Accelerometer ", 3, 15, &timesNewRoman_11ptFontInfo, color);
            break;
        case 2:
            lcd_write_string(" Microphone slow", 3, 29, &timesNewRoman_11ptFontInfo, color);
            break;
        case 3:
            lcd_write_string(" Microphone fast", 3, 43, &timesNewRoman_11ptFontInfo, color);
            break;
        case 4:
            lcd_write_string(" Envelope ", 3, 57, &timesNewRoman_11ptFontInfo, color);
            break;
    }
    
}

void displayGraph(const char* title, const char*  x_axis, const char*  y_axis)
{
    lcd_clear_block(160, 114, 0, 0);
    lcd_write_string(title, 40, 3, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    lcd_write_string(x_axis, 3, 18, &tahoma_8ptFontInfo, BLACK_OVER_WHITE);
    lcd_write_string(y_axis, 135, 102, &tahoma_8ptFontInfo, BLACK_OVER_WHITE);
    lcd_fill_block(3, 30, 4, 101);
    lcd_fill_block(3, 100, 150, 101);
}

uint16_t PrevyValue,PrevxValue;
void DoADC(uint8_t mode)
{
    uint16_t yValue,xValue;
    uint16_t ADCvalue,Bias;
    char ADCstr[30];
    xValue=3;
   
    ADC_init();
    switch (mode)
    {
        case 1:
            displayGraph(" Accelerometer "," [mS] "," [mV] ");
            Bias=ADC_Read(ACCELEROMETER);
            for (uint8_t i=0 ; i<64; i++)
            {
                Bias+=ADC_Read(ACCELEROMETER);
            }
            Bias=(Bias>>6)+60;
            PrevxValue=xValue;
            PrevyValue=Bias;
            while (!Exit)
            {
                xValue++;
                ADCvalue=ADC_Read(ACCELEROMETER);
                sprintf(ADCstr,"%d",ADCvalue);
                lcd_write_string(ADCstr, 30, 18, &tahoma_8ptFontInfo, BLACK_OVER_WHITE);
                yValue=Bias-ADCvalue;
                if (yValue<2) yValue=2;
                if (yValue>(LCD_HEIGHT-2)) yValue=LCD_HEIGHT-2;
                //lcd_set_pixel(xValue,yValue);
                lcd_draw_line(PrevxValue,PrevyValue,xValue,yValue,BLACK_OVER_WHITE);
                __delay_ms(10);
                if (xValue>155)
                {
                    xValue=3;
                    displayGraph(" Accelerometer "," [mS] "," [mV] ");
                }
                PrevxValue=xValue;
                PrevyValue=yValue;
            }
            break;
        case 2:
        case 3:
            displayGraph(" Microphone "," [mS] "," [mV] ");
            Bias=ADC_Read(MICROPHONE);
            for (uint8_t i=0 ; i<64; i++)
            {
                Bias+=ADC_Read(MICROPHONE);
            }
            Bias=(Bias>>6)+60;
            PrevxValue=xValue;
            PrevyValue=Bias;
            while (!Exit)
            {
                xValue++;
                ADCvalue=ADC_Read(MICROPHONE);
                sprintf(ADCstr,"%d",ADCvalue);
                lcd_write_string(ADCstr, 30, 18, &tahoma_8ptFontInfo, BLACK_OVER_WHITE);
                yValue=Bias-ADCvalue;
                if (yValue<2) yValue=2;
                if (yValue>(LCD_HEIGHT-2)) yValue=LCD_HEIGHT-2;
                //lcd_set_pixel(xValue,yValue);
                lcd_draw_line(PrevxValue,PrevyValue,xValue,yValue,BLACK_OVER_WHITE);
                if (mode==2) __delay_ms(50);
                if (xValue>155)
                {
                    xValue=3;
                    displayGraph(" Microphone "," [mS] "," [mV] ");
                }
                PrevxValue=xValue;
                PrevyValue=yValue;
            }
            break;
        case 4:
            displayGraph(" Envelope "," [mS] "," [mV] ");
            Bias=ADC_Read(ENVELOPE);
            for (uint8_t i=0 ; i<64; i++)
            {
                Bias+=ADC_Read(ENVELOPE);
            }
            Bias=(Bias>>6)+60;
            PrevxValue=xValue;
            PrevyValue=Bias;
            while (!Exit)
            {
                xValue++;
                ADCvalue=ADC_Read(ENVELOPE);
                sprintf(ADCstr,"%d",ADCvalue);
                lcd_write_string(ADCstr, 30, 18, &tahoma_8ptFontInfo, BLACK_OVER_WHITE);
                //yValue=128-(ADCvalue>>3);
                yValue=Bias-ADCvalue;
                if (yValue<2) yValue=2;
                if (yValue>(LCD_HEIGHT-2)) yValue=LCD_HEIGHT-2;
                //lcd_set_pixel(xValue,yValue);
                lcd_draw_line(PrevxValue,PrevyValue,xValue,yValue,BLACK_OVER_WHITE);
                __delay_ms(50);
                if (xValue>155)
                {
                    xValue=3;
                    displayGraph(" Envelope "," [mS] "," [mV] ");
                }
                PrevxValue=xValue;
                PrevyValue=yValue;
            }
            break;
    }
}
void Diag_ADC(void)
{
    uint8_t menu,prevmenu;

    menu=1;
    prevmenu=0;
    while (!Exit)
    {    
        if (prevmenu!=menu)
        {
            Diag_ADC_display(prevmenu,false);    //if prevmenu=0 Display All
                                             //else repaint previous selection
            Diag_ADC_display(menu,true);              //Display selected
            prevmenu=menu;
        }
        if (Keypressed)
        {
            lcd_write_char(' ' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
            switch (Key)
            {
                case KeyUp:if (menu>1) menu--;
                          lcd_write_char('5' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                    break;
                case KeyDw:if (menu<4) menu++; 
                          lcd_write_char('4' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                   break;
                case KeyIn: //select
                          lcd_write_char('6' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                          DoADC(menu);
                          prevmenu=0;//Redraw menu
                    break;
                case KeyBk: //escape
                          lcd_write_char('3' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                          prevmenu=0;//Redraw menu
                    break;  
                case KeySt:  
                          lcd_write_char('1' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                    break;
                case KeyRw:  
                          lcd_write_char('2' ,140,3 ,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
                    break;
            }
            while (Keypressed); // wait here till key is released
            lcd_write_char(' '    ,140,15,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
            lcd_write_char(menu+48,140,15,&tahoma_8ptFontInfo,BLACK_OVER_WHITE);
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="EEPROM test">
void eeprom_test() 
{
    char EE_data[4];
    lcd_write_string(" EEPROM diagnostics ", 3, 1, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
    lcd_write_string(" Writing ", 3, 20, &tahoma_8ptFontInfo, BLACK_OVER_WHITE);
    uint8_t temp_data[8] = {1,2,4,8,16,32,64,128};//,0xAA,0xCE,0x55,0x31,0xFF,0x45,0xEC};
    uint8_t x=37;
    
    while (!Exit)
    {
        for(uint8_t i = 0; i < sizeof(temp_data); i++) {
            sprintf(EE_data, "%02x", temp_data[i]);
            lcd_write_string(EE_data, (i+1)*18,35,&tahoma_8ptFontInfo,WHITE_OVER_BLACK);
            eeprom_write_data(i,temp_data[i]);
        }
        lcd_write_string(" Reading ", 3, 50, &tahoma_8ptFontInfo, BLACK_OVER_WHITE);
        for(uint8_t i = 0; i < sizeof(temp_data); i++) {
            sprintf(EE_data, "%02x", eeprom_read_data(i));
            lcd_write_string(EE_data, (i+1)*18,65,&tahoma_8ptFontInfo,WHITE_OVER_BLACK);//((i%8)*15)+
        }
        while(Keypressed);   //Wait till the key is released
        while(!Keypressed);  //Wait till a key is pressed
        for(uint8_t i = 0; i < sizeof(temp_data); i++) {
            temp_data[i]=5*x+1;
            x=temp_data[i];
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Auxiliar Input test">
void aux_test()
{
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_write_string(" TBD ", 3, 40, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Pseudo Data">
void PseudoData()
{
    uint8_t x = 10;
    uint8_t y = 17;
    uint16_t add=ShootStringStartAddress;
    uint16_t data=111;
    char msg[10];
    
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_write_string(" Sure to overwrite? ", 3, 40, &timesNewRoman_11ptFontInfo, WHITE_OVER_BLACK);
    while (Keypressed);
    while (!Keypressed);
    if (Select)
    {
        __delay_ms(30);//debounce
        lcd_write_string(" Realy !? ", 30, 60, &timesNewRoman_11ptFontInfo, WHITE_OVER_BLACK);
        while (Keypressed);
        while (!Keypressed);
        if (Select)
        {
            lcd_write_string(" Writting... ", 30, 60, &timesNewRoman_11ptFontInfo, BLACK_OVER_WHITE);
            for (uint8_t s=0;s<30;s++)
            {
                x = 5*x+1;
                if (x>99) x = x%99;
                if (x<5)  x = 5*x+1;
                for (uint8_t i=0;i<x;i++)
                {
                    y = 5*y+1;
                    if (y>100) y = y%100;
                    if (y<10)  y = 5*y+1;
                    data+=y;
                    if (data>0xFFFF) data=0xFFFF;
                    eeprom_write_wdata(add,data);
                    if (Keypressed)
                    {
                        sprintf(msg,"String :%d  Shoot:%d",s,i);
                        lcd_write_string(msg,10,80, &timesNewRoman_9ptFontInfo, BLACK_OVER_WHITE);
                        sprintf(msg,"Address:%d  Data :%d",add,data);
                        lcd_write_string(msg,10,95, &timesNewRoman_9ptFontInfo, BLACK_OVER_WHITE);
                    }
                }
            }
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Diagnostic Menu">
void Diag_display(uint8_t menu, uint8_t top, uint8_t pos)
{
    uint8_t color,p,lineh,height;
    
    p=top;
    lineh=timesNewRoman_11ptFontInfo.height+1; 
    height=LCD_HEIGHT-lineh;
    lcd_clear_block(0, top, LCD_WIDTH, LCD_HEIGHT);
    
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==1)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" Backlight ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==2)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" Display ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==3)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" Keypad ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==4)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" Buzzer ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==5)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" Battery ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==6)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" ADC ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==7)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" EEPROM ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==8)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" Auxiliary ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
    if ((p>=(pos+top)) && (p<=(height+pos))) {
        if (menu==9)     color=WHITE_OVER_BLACK;
        else          color=BLACK_OVER_WHITE;
        lcd_write_string(" Pseudo Data ", 3, p-pos, &timesNewRoman_11ptFontInfo, color);
    }
    p+=lineh;
}
void Diagnose(uint8_t menu)  
{
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    switch (menu)
    {
        case 1:Diag_Backlight();
            break;
        case 2:lcd_demo();
            break;
        case 3:Diag_Keypad();
            break;
        case 4:Diag_Buzzer();
            break;
        case 5:Diag_Battery();
            break;
        case 6:Diag_ADC();
            break;
        case 7:eeprom_test();
            break;
        case 8:aux_test();
            break;
        case 9:PseudoData();
            break;
    }
}
void Diagnostics(void)
{
    uint8_t menu,top,pos,lineh,height, battery_level;
    uint16_t battery;

    menu=1;
    pos=0;
    top=0;
    lcd_clear_block(0,0,LCD_WIDTH,LCD_HEIGHT);
    lcd_write_string("Settings",0,0,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
    lineh=timesNewRoman_11ptFontInfo.height+1; 
    battery_level = (ADC_Read(BATTERY)/10)-320; // ((battery_mV-3200)*100)/(4200-3200)
    //Top Line
    lcd_write_string("Settings ",0,top,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
    if (BT) lcd_write_string("BT",123,top,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
    lcd_battery_info(LCD_WIDTH-20,top,battery_level);
    top +=(timesNewRoman_11ptFontInfo.height+1);
    lcd_draw_hline(0,LCD_WIDTH,top,BLACK_OVER_WHITE);
    top +=2;
    height=LCD_HEIGHT-(lineh+top);
    //Main Screen
    Diag_display(menu,top,pos);
    
    while (!Exit)
    {
        if (Keypressed)
        {
            switch (Key)
            {
                case KeyUp:if (menu>2) {
                               menu--;
                               if (((menu-1)*lineh)<pos) pos -=lineh ;
                               Diag_display(menu,top,pos);
                          }
                          else Beep();
                    break;
                case KeyDw:if (menu<9) {
                               menu++; 
                               if (((menu-1)*lineh)>(height+pos)) pos +=lineh ;
                               Diag_display(menu,top,pos);
                          }
                          else Beep();
                   break;
                case KeyIn: //select
                          Diagnose(menu);
                    break;
                case KeyBk: //escape
                    break;  
            }
            while (Keypressed); // wait here till key is released
        }
    }
}
// </editor-fold>
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Nominal">

void saveShootString(uint16_t ShootStrNum)
{
    //The maximum shoots per string is MAXSHOOT=99, each shoot 2bytes therefore 
    uint16_t Address=ShootStringStartAddress+((ShootStrNum % 30)*MAXSHOOT*2);
    for (uint8_t i=0; i<ShootString.TotShoots; i++)
    {
        eeprom_write_wdata(Address,ShootString.ShootTime[i]);
        Address+=2;
    }
}
void getShootString(uint16_t ShootStrNum)
{
    //The maximum shoots per string is MAXSHOOT=99, each shoot 2bytes therefore 
    uint16_t Address=ShootStringStartAddress+((ShootStrNum % 30)*MAXSHOOT*2);
    for (uint8_t i=0; i<ShootString.TotShoots; i++)
    {
        ShootString.ShootTime[i] = eeprom_read_wdata(Address);
        Address+=2;
    }
}
// <editor-fold defaultstate="collapsed" desc="SettingsMenu">
void SettingsDisplay(uint8_t battery,uint8_t MenuItem,uint8_t SubMenuItem,TBool BT) 
{
   char message[60];
   uint8_t line = 0;
   uint8_t i,l,m;
           
   lcd_clear_block(0,0,LCD_WIDTH,LCD_HEIGHT);
   //Top Line
   lcd_write_string("Settings ",0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   if (BT) lcd_write_string("BT",123,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   lcd_battery_info(LCD_WIDTH-20,line,battery);
   line +=(timesNewRoman_11ptFontInfo.height+1);
   lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
   line +=2;
  //Main Screen
    for (i=0;i<5;i++)
    {
        m=MenuItem-3+i;
        l=line+(m*15);
        if ((m>0) && (m<5))
        {
            if (m==MenuItem) lcd_write_string(Menu[m],0,l,&timesNewRoman_11ptFontInfo,WHITE_OVER_BLACK);
            else             lcd_write_string(Menu[m],0,l,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
        }
    }

}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ReviewMenu">
void ReviewDisplay(uint8_t battery,uint8_t CurShoot, uint8_t CurShootStringDisp, TBool FullRedraw) 
{
   char message[60];
   uint8_t line=0;
           
   if (!FullRedraw)  line +=((timesNewRoman_11ptFontInfo.height*2)+23);
   lcd_clear_block(0,line,LCD_WIDTH,LCD_HEIGHT);
   if (FullRedraw)
   {
        //Top Line
        lcd_write_string("REVIEW",0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
        if (BT) lcd_write_string("BT",123,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
        lcd_battery_info(LCD_WIDTH-20,line,battery);
        line +=(timesNewRoman_11ptFontInfo.height+1);
        lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
        line +=2;
        //String line 
        lcd_write_char('^',0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
        line +=7;
        sprintf(message, " String:%02d/%02d %6.2f ", CurShootStringDisp,ShootString.TotShoots,(float)ShootString.ShootTime[ShootString.TotShoots]/100);
        lcd_write_string(message,12,line,&timesNewRoman_11ptFontInfo,WHITE_OVER_BLACK);
        line +=7;
        lcd_write_char('_',0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
        line +=(timesNewRoman_11ptFontInfo.height+1);
        lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
        line +=5;
   }
   //Shoot lines
   //1st ShootNumber 01, before it ShootNumber 00 time=0
   sprintf(message, " %02d:%06.2f ", CurShoot-1,(float)ShootString.ShootTime[CurShoot-1]/100);
   lcd_write_string(message,0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   lcd_write_char('^',LCD_WIDTH-10,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   line +=7;
   sprintf(message, "}%06.2f",(float)(ShootString.ShootTime[CurShoot]-ShootString.ShootTime[CurShoot-1])/100);
   lcd_write_string(message,85,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   line +=7;
   sprintf(message, " %02d:%06.2f ", CurShoot,(float)ShootString.ShootTime[CurShoot]/100);
   lcd_write_string(message,0,line,&timesNewRoman_11ptFontInfo,WHITE_OVER_BLACK);
   line +=7;
   if (CurShoot<ShootString.TotShoots)
   {
        sprintf(message, "}%06.2f",(float)(ShootString.ShootTime[CurShoot+1]-ShootString.ShootTime[CurShoot])/100);
        lcd_write_string(message,85,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   }
   line +=7;
   if (CurShoot<ShootString.TotShoots)
   {
        sprintf(message, " %02d:%06.2f ", CurShoot+1,(float)ShootString.ShootTime[CurShoot+1]/100);
        lcd_write_string(message,0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   }
   lcd_write_char('_',LCD_WIDTH-10,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
}
void DoReview()
{
    uint16_t battery_level;
    CurShootString=1;
    CurShoot=1;
    battery_level = (ADC_Read(BATTERY)*BAT_divider/10)-320; // ((battery_mV-3200)*100)/(4200-3200)
    getShootString(CurShootString);
    ReviewDisplay(battery_level,CurShoot,(LastStringNumber-CurShootString)+1,True);
    while (!Exit)
    {
        if (Keypressed)
        {
            switch (Key)
            {
                case KeyUp: CurShoot--;
                break;
                case KeyDw: CurShoot++;
                break;
            }
            ReviewDisplay(battery_level,CurShoot,(LastStringNumber-CurShootString)+1,False);
        }
        while (Keypressed); //wait here till key is released
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Main Menu">
/*
void MainDisplay(uint8_t battery, uint16_t time, uint16_t time1shoot,uint8_t ShootNumber, uint16_t par, TdelTy delTy, TBool Aux, uint8_t hour, uint8_t minute, TBool A, TBool B, TBool BT) 
                    //percentage, 10mS, 10mS, num, 10mS, TdelTy, TBool
{
   char message[60];
   char clock[6];
   lcd_fill_block(0,0,LCD_WIDTH,LCD_HEIGHT);
   lcd_clear_block(1,1,LCD_WIDTH-1,Bot);
   lcd_clear_block(1,Bot+1,LCD_WIDTH-1,Bot+14);
   lcd_clear_block(1,Bot+15,LCD_WIDTH-1,Bot+28);
   lcd_battery_info(LCD_WIDTH-20,3,battery);
   sprintf(message, "%05.2f", (float)time/100);
   lcd_write_string(message,6,30,&microsoftSansSerif_48ptFontInfo,BLACK_OVER_WHITE);
   sprintf(message, "1~Shot:%05.2f|Shot:%02d|Par:%05.2f",(float)time1shoot/100,ShootNumber, (float)par/100);
   lcd_write_string(message,2,Bot+1,&timesNewRoman_9ptFontInfo, BLACK_OVER_WHITE);
   switch (delTy)
   {
       case Instant:sprintf(message,"Delay:None  |");
       break;        
       case Fixed:  sprintf(message,"Delay:Fixed |");
       break;        
       case Random: sprintf(message,"Delay:Random|");
       break;        
       case Custom: sprintf(message,"Delay:Custom|");
       break;        
   }
   if (Aux) strcat(message,"Aux:On |");
   else     strcat(message,"Aux:Off|"); 
   sprintf(clock," %02d:%02d",hour,minute);
   strcat(message,clock);
   lcd_write_string(message,2,Bot+15,&timesNewRoman_9ptFontInfo, BLACK_OVER_WHITE);
   lcd_draw_vline(LCD_WIDTH-1,1,LCD_HEIGHT-1, BLACK_OVER_WHITE);
   if (A) lcd_write_string(" A ",10,3,&timesNewRoman_11ptFontInfo, WHITE_OVER_BLACK);
   if (B) lcd_write_string(" B ",30,3,&timesNewRoman_11ptFontInfo, WHITE_OVER_BLACK);
   if (BT) lcd_write_string(" BT ",50,3,&timesNewRoman_11ptFontInfo, WHITE_OVER_BLACK);
}
*/
void MainDisplay(uint8_t battery, uint16_t time, uint16_t time1shoot,uint8_t ShootNumber, uint8_t par, uint16_t parTime, TdelTy delTy, TBool Aux, TBool A, TBool B) 
                    //percentage, 10mS, 10mS, num, 10mS, TdelTy, TBool
{
   char message[60];
   uint8_t line=0;
           
   lcd_clear_block(0,0,LCD_WIDTH,LCD_HEIGHT);
   //Top Line
   sprintf(message," %02d:%02d",hour,minute);
   lcd_write_string(message,0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   if (Aux) lcd_write_string("Aux:On ",45,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   else     lcd_write_string("Aux:Off",45,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   if (A) lcd_write_string("A",95,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   if (B) lcd_write_string("B",110,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   if (BT) lcd_write_string("BT",123,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   lcd_battery_info(LCD_WIDTH-20,line,battery);
   line +=timesNewRoman_11ptFontInfo.height;
   line++;
   lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
   line +=5;
   //Main line 
   sprintf(message, "%06.2f", (float)time/100);
   lcd_write_string(message,0,line,&microsoftSansSerif_36ptFontInfo,BLACK_OVER_WHITE);
   line +=microsoftSansSerif_36ptFontInfo.height;
   line +=5;
   sprintf(message, "1~:%06.2f",(float)time1shoot/100);
   lcd_write_string(message,0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   sprintf(message, "Shoot:%02d",ShootNumber);
   lcd_write_string(message,LCD_WIDTH-62,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   line +=timesNewRoman_11ptFontInfo.height;
   line++;
   lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
   line +=3;
   // bottom lines
   switch (delTy)
   {
       case Instant:sprintf(message,"Delay:None");
       break;        
       case Fixed:  sprintf(message,"Delay:Fixed");
       break;        
       case Random: sprintf(message,"Delay:Random");
       break;        
       case Custom: sprintf(message,"Delay:Custom");
       break;        
   }
   lcd_write_string(message,0,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   sprintf(message, "Mic:%d",Sensitivity);
   lcd_write_string(message,LCD_WIDTH-32,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   line +=timesNewRoman_9ptFontInfo.height;
   line++;
   sprintf(message, "Par%02d:%06.2f",par,(float)parTime/100);
   lcd_write_string(message,0,line,&timesNewRoman_11ptFontInfo,BLACK_OVER_WHITE);
   sprintf(message, "Buz:%d",BuzzerLevel);
   lcd_write_string(message,LCD_WIDTH-32,line,&timesNewRoman_9ptFontInfo,BLACK_OVER_WHITE);
   }
// </editor-fold>
// </editor-fold>

void main(void) {
// <editor-fold defaultstate="collapsed" desc="Initialization">
    PIC_init();
    PowerON;
    initialize_backlight();
    set_backlight(90);
    spi_init();
    lcd_init();
    ADC_init();
    eeprom_init();
    set_backlight(BackLightLevel);
 // Initialization End
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Main">
    uint16_t battery = ADC_Read(BATTERY);
    uint16_t battery_mV = battery*BAT_divider;
    uint8_t  battery_level = (battery_mV/10)-320; // ((battery_mV-3200)*100)/(4200-3200)
/*            ShootString.ShootTime[0]=145;
            ShootString.ShootTime[1]=278;
            ShootString.ShootTime[2]=312;
            ShootString.ShootTime[3]=431;
            ShootString.ShootTime[4]=545;
            ShootString.ShootTime[5]=678;
            ShootString.ShootTime[6]=712;
            ShootString.ShootTime[7]=812;
            ShootString.ShootTime[8]=945;
            ShootString.ShootTime[9]=1078;
            ShootString.TotShoots=10;*/
    
    MainDisplay(battery_level,1234,345,3,4,4537,Fixed,False,True,True);
    while(True) 
    {
        battery = ADC_Read(BATTERY);
        battery_mV = battery*BAT_divider;
        battery_level = (battery_mV/10)-320; // ((battery_mV-3200)*100)/(4200-3200)
        if (Keypressed)
        {
            switch (Key)
            {
                case KeyIn:Diagnostics();
                          MainDisplay(battery_level,1234,345,3,4,4537,Fixed,False,True,True);
                break;
                case KeyDw:MainDisplay(battery_level,1234,345,3,4,4537,Fixed,False,True,True);
                break;
                case KeyUp:ReviewDisplay(battery_level,7, 3,True);
                break;
            }
        }
        while (Keypressed); // wait here till key is released
    }
// </editor-fold>
}
