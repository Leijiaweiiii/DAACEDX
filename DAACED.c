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
#include "measurements.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC_init">
void PIC_init(void)
{
    // 0 = OUTPUT, 1 = INPUT
    // 0 = DIGITAL, 1 = ANALOG
    
    OSCFRQ = 0b00001000;        // 64 MHz Fosc.
    
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
    SSP1STATbits.CKE = 1;                   // Data transmission on rising edge
    SSP1STATbits.SMP = 0;                   // Data sampled/latched at middle of clock.
    SSP1CON1 = 0x21;                        // Enable synchronous serial port , CKL ,FOSC_DIV_16 page 394
//    SSP1CON1bits.SSPM = 0x01;               // SPI Clock FOSC_DIV_16
//    SSP1CON1bits.SSPEN = 1;                 // Enable synchronous serial port
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
    if(!refresh_lcd) return;
    refresh_lcd = false;
    
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

uint8_t lcd_string_lenght(const char* str_ptr, const FONT_INFO *font)
{
    uint8_t strlng=0;
    if(str_ptr == NULL) return 0;
	while(*str_ptr) 
    {   
        strlng += font->char_descriptors[*str_ptr].width;
        strlng += font->character_spacing;
        ++str_ptr;
    }    
    return strlng;
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
    if (battery_percentage>100) battery_percentage=100;
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

void eeprom_write_tdata(uint16_t address, uint24_t data) {
    eeprom_write_data(address,data & 0xFF);
    eeprom_write_data(address+1,(data>>8) & 0xFF);
    eeprom_write_data(address+2,(data>>16) & 0xFF);
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
    EEPROM_CS_DESELECT();
    return (index);
}

uint16_t eeprom_read_wdata(uint16_t address) {
    uint8_t read_least,read_most;
    if(address+1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();
    
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_least = eeprom_spi_write(0x00);
    read_most  = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_most<<8)+read_least;
}

uint24_t eeprom_read_tdata(uint16_t address) {
    uint8_t read_least,read_mid,read_most;
    if(address+1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();
    
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_least = eeprom_spi_write(0x00);
    read_mid   = eeprom_spi_write(0x00);
    read_most  = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_most<<16)+(read_mid<<8)+read_least;
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
    uint32_t sinus_sample_update_period_us = (uint32_t)(sinus_time_period_us/57.0);///32.0)+0.5f);
    int32_t no_of_cycles = ((int32_t)duration*580)/sinus_time_period_us; //1000
    
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

// <editor-fold defaultstate="collapsed" desc="Small Routines">
void  TestBattery(void)
{
    uint16_t battery = ADC_Read(BATTERY);
    uint16_t battery_mV = battery*BAT_divider;
    battery_level = (battery_mV/8)-320; // "/10" ((battery_mV-3200)*100)/(3900-3200)
    if (battery_level>99)  battery_level=99;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Settings Display">
uint8_t SettingsTitle(void)
{
    uint8_t top;
    top=0;
    lcd_clear_block(0,0,LCD_WIDTH,LCD_HEIGHT);
    TestBattery();
    lcd_write_string(SettingsMenu.MenuTitle,0,0,MediumFont,BLACK_OVER_WHITE);
    if (BT) lcd_write_string("BT",123,0,SmallFont,BLACK_OVER_WHITE);
    lcd_battery_info(LCD_WIDTH-20,0,battery_level);
    top +=(MediumFont_height+1);
    lcd_draw_hline(0,LCD_WIDTH,top,BLACK_OVER_WHITE);
    top +=2;
    return top;
}
void SettingsDisplay(void)
{
    uint8_t i,color,p,lineh,height,mpos;
    char msg[10];
    p=Menu.top;
    lineh=MediumFont_height+1; 
    height=LCD_HEIGHT-lineh;
    Menu.PageSize=6;

    if (Menu.menu>(Menu.PageSize*Menu.page))
    {
        //Inc Page
         Menu.page++;
         Menu.refresh=True;
    }
    else if (Menu.menu<=(Menu.PageSize*(Menu.page-1)))
    {
        //Dec Page
         Menu.page--;
         Menu.refresh=True;
    }

    if (Menu.refresh)
    {
        lcd_clear_block(0, Menu.top, LCD_WIDTH, LCD_HEIGHT);
        for (i=(Menu.PageSize*(Menu.page-1));i<SettingsMenu.TotMenuItems;i++)
        {
            if ((p>=(Menu.pos+Menu.top)) && (p<=(height+Menu.pos))) {
                if (Menu.menu==i+1)     color=WHITE_OVER_BLACK;
                else                    color=BLACK_OVER_WHITE;
                lcd_write_string(SettingsMenu.MenuItem[i], 3, p-Menu.pos, MediumFont, color);
            }
            p+=lineh;
        }
    }
    else
    {
        for (i=(Menu.PageSize*(Menu.page-1));i<SettingsMenu.TotMenuItems;i++)
        {
            if (Menu.menu==i+1) lcd_write_string(SettingsMenu.MenuItem[i], 3, p-Menu.pos, MediumFont, WHITE_OVER_BLACK);
            if (Menu.prev==i+1) lcd_write_string(SettingsMenu.MenuItem[i], 3, p-Menu.pos, MediumFont, BLACK_OVER_WHITE);
            p+=lineh;
        }
    }
}

void MenuSelection(void)
{
    TBool Done=False;
    while (!Done)
    {
        if (Keypressed)
        {
            
            switch (Key)
            {
                case KeyUp:if (Menu.menu>1) {
                               Menu.prev=Menu.menu;
                               Menu.menu--;
                               Menu.refresh=False;
                               SettingsDisplay();
                          }
                          else Beep();
                    break;
                case KeyDw:if (Menu.menu<(SettingsMenu.TotMenuItems)) {
                               Menu.prev=Menu.menu;
                               Menu.menu++; 
                               Menu.refresh=False;
                               SettingsDisplay();
                          }
                          else Beep();
                    break;
                case KeyIn:
                    Done=True;
                    break;
                case KeyBk:
                    Menu.menu=0;
                    Done=True;
                    break;
            }
            while (Keypressed); // wait here till key is released
        }
    }
}
uint8_t PopMsg(const char* msg, uint16_t wait)
{
    uint16_t t=0;
    uint8_t Yo1=PopY1+((PopY2-(PopY1+MediumFont_height))>>1);
    uint8_t Xo1=PopX1+((PopX2-(PopX1+lcd_string_lenght(msg,MediumFont)))>>1);
    lcd_fill_block(PopX1,PopY1,PopX2,PopY2);
    lcd_write_string(msg,Xo1,Yo1,MediumFont,WHITE_OVER_BLACK);
    if (wait==0)
    {
        while(!Keypressed);
        return Key;
    }
    else
    {
        while (t<wait)
        {
            __delay_ms(1);
            t++;
            if (Keypressed) return Key;
        }
        return Yo1;
    }
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
//LCD demo functions End
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Keypad test">
void Diag_Keypad(void)
{
    lcd_write_string(" Keypad test", 3, 3, MediumFont, BLACK_OVER_WHITE);
    while (!Exit)
    {
        lcd_clear_block(20, 30, 80,110);
        switch (Key)
        {
            case KeySt: 
                lcd_write_char('1', 20, 30, BigFont, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyRw: 
                lcd_write_char('2', 20, 30, BigFont, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyBk: 
                lcd_write_char('3', 20, 30, BigFont, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyDw: 
                lcd_write_char('4', 20, 30, BigFont, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyUp: 
                lcd_write_char('5', 20, 30, BigFont, BLACK_OVER_WHITE);
                Delay(100);
            break;
            case KeyIn: 
                lcd_write_char('6', 20, 30, BigFont, BLACK_OVER_WHITE);
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
            lcd_write_string(" Buzzer test ", 3, 1, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" 1KHz    10% ", 3, 15, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" 1KHz   100% ", 3, 29, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" 1.5KHz  10% ", 3, 43, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" 1.5KHz 100% ", 3, 57, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" 2KHz    10% ", 3, 71, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" 2KHz   100% ", 3, 85, MediumFont, BLACK_OVER_WHITE);
            break;
        case 1:
            lcd_write_string(" 1KHz    10% ", 3, 15, MediumFont, color);
            break;
        case 2:
            lcd_write_string(" 1KHz   100% ", 3, 29, MediumFont, color);
            break;
        case 3:
            lcd_write_string(" 1.5KHz  10% ", 3, 43, MediumFont, color);
            break;
        case 4:
            lcd_write_string(" 1.5KHz 100% ", 3, 57, MediumFont, color);
            break;
        case 5:
            lcd_write_string(" 2KHz    10% ", 3, 71, MediumFont, color);
            break;
        case 6:
            lcd_write_string(" 2KHz   100% ", 3, 85, MediumFont, color);
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
        battery_level = (battery_mV/10)-320; // ((battery_mV-3200)*100)/(4200-3200)
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
            lcd_write_string(" ADC test ", 3, 1, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" Accelerometer ", 3, 15, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" Microphone slow", 3, 29, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" Microphone fast", 3, 43, MediumFont, BLACK_OVER_WHITE);
            lcd_write_string(" Envelope ", 3, 57, MediumFont, BLACK_OVER_WHITE);
            break;
        case 1:
            lcd_write_string(" Accelerometer ", 3, 15, MediumFont, color);
            break;
        case 2:
            lcd_write_string(" Microphone slow", 3, 29, MediumFont, color);
            break;
        case 3:
            lcd_write_string(" Microphone fast", 3, 43, MediumFont, color);
            break;
        case 4:
            lcd_write_string(" Envelope ", 3, 57, MediumFont, color);
            break;
    }
    
}

void displayGraph(const char* title, const char*  x_axis, const char*  y_axis)
{
    lcd_clear_block(160, 114, 0, 0);
    lcd_write_string(title, 40, 3, MediumFont, BLACK_OVER_WHITE);
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
    lcd_write_string(" EEPROM diagnostics ", 3, 1, MediumFont, BLACK_OVER_WHITE);
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
    lcd_write_string(" TBD0 ", 3, 40, MediumFont, BLACK_OVER_WHITE);
    __delay_ms(500);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Pseudo Data">
void ReadBackEEPROMdata()
{
    uint16_t add,data;
    uint8_t s,i,j,h;
    char msg[20];
    
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    for (s=0;s<30;s++)
    {
        add=ShootStringStartAddress+(s*Size_of_ShootString);
        h=eeprom_read_data(add+1);
        sprintf(msg,"String %1d, Sh=%2d",eeprom_read_data(add),h);
        lcd_write_string(msg, 3, 0, SmallFont, WHITE_OVER_BLACK);
        j=SmallFont_height;
        add +=2;
        for (i=0;i<h;i++)
        {
            data=eeprom_read_wdata(add);
            sprintf(msg,"Shoot %2d:%5.2f   ",i,(float)data/100);
            add +=2;
            lcd_write_string(msg, 3,j , SmallFont, BLACK_OVER_WHITE);
            j+=SmallFont_height;
            if (j>LCD_HEIGHT-SmallFont_height)
            {
                while (Keypressed);
                while (!Keypressed);
                if (Exit) 
                {
                   lcd_clear();
                   PopMsg("Exit",200);
                   return;
                }
                __delay_ms(30);
                j=SmallFont_height;
                lcd_clear_block(0,SmallFont_height,LCD_WIDTH-1,LCD_HEIGHT-1);
            }
        }
        while (Keypressed);
        while (!Keypressed);
        if (Exit) 
        {
           lcd_clear();
           PopMsg("Exit",200);
           return;
        }
    }
    PopMsg("Done",1000);
}
void PseudoData()
{
    uint8_t x = 27;
    uint8_t s,i,j;
    uint16_t add=ShootStringStartAddress;
    uint16_t data=60;//0.6sec
    
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_write_string("Sure to overwrite?", 3, 40, MediumFont, WHITE_OVER_BLACK);
    while (Keypressed);
    while (!Keypressed);
    if (Select)
    {
        __delay_ms(30);//debounce
        lcd_write_string(" Really !? ", 30, 60, MediumFont, WHITE_OVER_BLACK);
        while (Keypressed);
        while (!Keypressed);
        if (Select)
        {
            lcd_write_string(" Writing... ", 30, 60, MediumFont, BLACK_OVER_WHITE);
            for (s=0;s<30;s++)
            {
                add=ShootStringStartAddress+(s*Size_of_ShootString);
                data=60+x;
                if (s==0) eeprom_write_data(add,1);
                else      eeprom_write_data(add,0);
                add++;
                j=(j*5+1)%99;
                eeprom_write_data(add,j);
                add++;
                for (i=0;i<j;i++)
                {
                    x = 5*x+1;
                    if (x>200) x = x%200;
                    if (x<20)  x = 27;
                    if (data>(0xFFFF-x)) data=0xFFFF;
                    else  data+=x;
                    eeprom_write_wdata(add,data);
                    add +=2;;
                }
            }
            PopMsg("Done",1000);
        }
    }
    lcd_clear();
    ReadBackEEPROMdata();
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Diagnostic Menu">
void Diagnose()  
{
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    switch (Menu.menu)
    {
        case 1:lcd_demo();
            break;
        case 2:Diag_Keypad();
            break;
        case 3:Diag_Buzzer();
            break;
        case 4:Diag_Battery();
            break;
        case 5:Diag_ADC();
            break;
        case 6:eeprom_test();
            break;
        case 7:aux_test();
            break;
        case 8:PseudoData();
            break;
    }
    Menu.refresh=True;
}
void Diagnostics(void)
{
    Menu.menu=1;
    Menu.pos=0;
    Menu.top=0;
    SettingsMenu.TotMenuItems=8;
    strcpy(SettingsMenu.MenuTitle,"Diagnostics ");
    strcpy(SettingsMenu.MenuItem[0]," Display ");
    strcpy(SettingsMenu.MenuItem[1]," Keypad ");
    strcpy(SettingsMenu.MenuItem[2]," Buzzer ");
    strcpy(SettingsMenu.MenuItem[3]," Battery ");
    strcpy(SettingsMenu.MenuItem[4]," ADC ");
    strcpy(SettingsMenu.MenuItem[5]," EEPROM ");
    strcpy(SettingsMenu.MenuItem[6]," Auxiliary ");
    strcpy(SettingsMenu.MenuItem[7]," Pseudo Data ");
    //SettingsMenu.MenuItem={" Backlight "," Display "," Keypad "," Buzzer "," Battery "," ADC "," EEPROM "," Auxiliary "," Pseudo Data "};

    Menu.lineh=MediumFont_height+1; 
    //Top Line
    Menu.top = SettingsTitle();
    Menu.height=LCD_HEIGHT-(Menu.lineh+Menu.top);
    //Main Screen
    Menu.refresh=True;
    Menu.page=0;//Force refresh
    
    do {
      SettingsDisplay();
      MenuSelection();
      if (Menu.menu>0)  Diagnose();
    } while (Menu.menu>0) ;
}
// </editor-fold>
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Nominal">
// <editor-fold defaultstate="collapsed" desc="Save and retrive DATA">
void saveSettings()
{
    eeprom_write_wdata(Sensitivity_Address,Sensitivity);
    eeprom_write_wdata(Filter_Address,Filter);
    eeprom_write_wdata(AutoStart_Address,AutoStart);
    eeprom_write_wdata(AR_IS_Address,AR_IS);
    eeprom_write_wdata(BuzzerFrequency_Address,BuzzerFrequency);
    eeprom_write_wdata(BuzzerParDuration_Address,BuzzerParDuration);
    eeprom_write_wdata(BuzzerStartDuration_Address,BuzzerStartDuration);
    eeprom_write_wdata(BuzzerLevel_Address,BuzzerLevel);
    eeprom_write_wdata(CustomCDtime_Address,CustomCDtime);
    eeprom_write_wdata(BT_Address,BT);
    eeprom_write_wdata(Delay_Address,DelayMode);
    eeprom_write_wdata(DelayTime_Address,DelayTime);
    eeprom_write_wdata(BackLightLevel_Address,BackLightLevel);
}
void getSettings()
{
    Sensitivity=eeprom_read_wdata(Sensitivity_Address);
    Filter=eeprom_read_wdata(Filter_Address);
    AutoStart=eeprom_read_wdata(AutoStart_Address);
    AR_IS=eeprom_read_wdata(AR_IS_Address);
    BuzzerFrequency=eeprom_read_wdata(BuzzerFrequency_Address);
    BuzzerParDuration=eeprom_read_wdata(BuzzerParDuration_Address);
    BuzzerStartDuration=eeprom_read_wdata(BuzzerStartDuration_Address);
    BuzzerLevel=eeprom_read_wdata(BuzzerLevel_Address);
    CustomCDtime=eeprom_read_wdata(CustomCDtime_Address);
    BT=eeprom_read_wdata(BT_Address);
    DelayMode=eeprom_read_wdata(Delay_Address);
    DelayTime=eeprom_read_wdata(DelayTime_Address);
    BackLightLevel=eeprom_read_wdata(BackLightLevel_Address);
}
void savePar(uint8_t i)
{
        eeprom_write_tdata(ParAddress+(i*3)+1,ParTime[i]);
}
void saveTotPar()
{
        eeprom_write_data(ParAddress, TotPar);
}
void getPar()
{
    TotPar= eeprom_read_data(ParAddress);
    for (uint8_t i=0;i<TotPar;i++)
        ParTime[i]=eeprom_read_tdata(ParAddress+(i*3)+1);
}

uint16_t findCurrStringAddress()
{
    uint16_t add=ShootStringStartAddress; 
    CurrStringStartAddress=0;
    uint8_t data;
    do {
       data=eeprom_read_data(add);
       if (data==1)
           CurrStringStartAddress=add;
       else add += sizeof(ShootString);
    } while ((data!=1) && (add<(ShootStringStartAddress+30*sizeof(ShootString))));
    return CurrStringStartAddress;
}

void saveShootString(void)
{
    uint16_t Address;
    // current string is overwritten over the older string
    
    findCurrStringAddress(); // the address of stored shoot string 0
    if (ShootStringStartAddress==CurrStringStartAddress)
         Address= ShootStringStartAddress+(29*Size_of_ShootString);
    else Address= (CurrStringStartAddress-Size_of_ShootString);
    eeprom_write_data(CurrStringStartAddress,0); //No longer current
    eeprom_write_data(Address,1); //Mark current string
    Address++;
    eeprom_write_data(Address,ShootString.TotShoots);
    Address++;
    for (uint8_t i=0; i<ShootString.TotShoots; i++)
    {
        eeprom_write_tdata(Address,ShootString.ShootTime[i]);
        Address+=3;
    }
}

TBool getShootString(uint8_t ShootStrNum)
{
    uint16_t Address;
    
    findCurrStringAddress(); // the address of shoot string 0
    uint16_t StrBeforeCurr= ((CurrStringStartAddress-ShootStringStartAddress)/Size_of_ShootString);
    if ((30-StrBeforeCurr)>ShootStrNum) Address=CurrStringStartAddress+ (ShootStrNum*Size_of_ShootString);
    else                                Address=ShootStringStartAddress+ (((ShootStrNum+StrBeforeCurr)-30)*Size_of_ShootString);
    uint8_t mark= eeprom_read_data(Address);
    Address++; 
    ShootString.TotShoots= eeprom_read_data(Address);
    Address++;
    for (uint8_t i=0; i<ShootString.TotShoots; i++)
    {
        ShootString.ShootTime[i] = eeprom_read_tdata(Address);
        Address+=3;
    }
    return (((ShootStrNum==0) && (mark==1)) || ((ShootStrNum>0) && ((mark==0))));
}        
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Settings">

// <editor-fold defaultstate="collapsed" desc="Delay Settings">
void SetCustomDelay(uint8_t top)
{
    TBool Done;
    char msg[20];
    
    if (DelayTime<10) DelayTime=10;
    strcpy(SettingsMenu.MenuTitle,"Settings: ");
    top = SettingsTitle();
    top+=3;
    lcd_write_string("Custom Delay",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    top+=topSpace;
    sprintf(msg,"%3.1f",(float)DelayTime/10);
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=1;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((DelayTime+i)<99)     DelayTime+=i;
                            else DelayTime=99;
                            SaveToEEPROM=True; 
                break;
                case KeyDw: if ((DelayTime-i)>10)      DelayTime-=i;
                            else DelayTime=10;
                            SaveToEEPROM=True; 
                break;
                default:   Done=True;
            }
            sprintf(msg,"%3.1f",(float)DelayTime/10);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i<11)   i=i*2;  j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=1;j=0;}                 //if key not pressed go slow (again))
    }
}
void SetDelay()
{
    Menu.top=0;
    Menu.pos=0;
    Menu.menu=1;
    SettingsMenu.TotMenuItems=4;
    strcpy(SettingsMenu.MenuTitle,"Settings: Delay");
    strcpy(SettingsMenu.MenuItem[0]," Instant ");
    strcpy(SettingsMenu.MenuItem[1]," Fixed 3sec. ");
    strcpy(SettingsMenu.MenuItem[2]," Random");
    strcpy(SettingsMenu.MenuItem[3]," Custom ");
    
    switch (DelayMode)
    {
        case Instant: Menu.menu=1; break;
        case Fixed  : Menu.menu=2; break;
        case Random : Menu.menu=3; break;
        case Custom : Menu.menu=4; break;
        default     : Menu.menu=2; break;
    }

    Menu.lineh=MediumFont_height+1; 
    //Top Line
    Menu.top = SettingsTitle();
    Menu.height=LCD_HEIGHT-(Menu.lineh+Menu.top);
    Menu.refresh=True;
    //Main Screen
    SettingsDisplay();
    MenuSelection();
   
    switch (Menu.menu)
    {
        case 1: DelayMode=Instant; SaveToEEPROM=True; break;
        case 2: DelayMode=Fixed; DelayTime=30; SaveToEEPROM=True; break;
        case 3: DelayMode=Random; SaveToEEPROM=True; break;
        case 4: DelayMode=Custom; 
                SetCustomDelay(Menu.top);
                break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Par">
TBool DeletePar(uint8_t Par_i)
{
    char msg[20];
    sprintf(msg,"Delete Par%d ?",Par_i+1);
    while (Keypressed);
    if (PopMsg(msg,0)==KeyIn)
    {
        for (uint8_t i=Par_i;i<TotPar;i++)
        {
            ParTime[i]=ParTime[i+1];
            savePar(i);
        }
        TotPar--;
        saveTotPar();
        sprintf(msg,"Par%d Deleted",Par_i+1);
        PopMsg(msg,200);
        return True;
    }
    else
        return False;
}

void SetPar()
{
    uint8_t Mtop;
    char msg[15];
    TBool Done;
    TBool changed;
    uint8_t redraw;
    uint8_t i,j,k;
     
    Menu.lineh=MediumFont_height+1; 
    Menu.pos=0;
    //Main Screen
    Menu.menu=1;
    redraw=2;
    changed=False;
    while (Menu.menu>0)
    {
        if (redraw>0)
        {
            Mtop=0;
            i=0;
            TotPar=0;
            do 
            {
                if ((ParTime[i]>0) && (ParTime[i]<100000))
                {
                    sprintf(msg," Par %d: %5.1f",i+1,(float)ParTime[i]/100);//unit is 10mS
                    strcpy(SettingsMenu.MenuItem[i],msg); 
                    TotPar++;
                }
                i++;
            } while ((ParTime[i]>0) && (ParTime[i]<100000) && (i<=MAXPAR));

            sprintf(SettingsMenu.MenuTitle,"Total Par=%d",TotPar);
            sprintf(SettingsMenu.MenuItem[TotPar]," Par %d: Off",TotPar+1);
            SettingsMenu.TotMenuItems=TotPar+1;

            //Top Line
            Mtop = SettingsTitle();
            Menu.height=LCD_HEIGHT-(Menu.lineh+Mtop);
            redraw=0;
        } 
        Menu.top=Mtop;
        Menu.refresh=True;
        SettingsDisplay();
        MenuSelection();
        if (Menu.menu>0)
        {
            if (Menu.menu==TotPar+1)
            {
                if (TotPar>0)
                {
                    ParTime[Menu.menu-1]=ParTime[Menu.menu-2]+(BuzzerParDuration/10)+10;
                    TotPar++;
                    changed = True;
                    redraw=1;
                }
                else
                {
                    ParTime[0]=(BuzzerStartDuration/10)+10;
                    TotPar=1;
                    changed = True;
                    redraw=1;
                }
            }
            Menu.top = SettingsTitle();
            Menu.top+=3;
            sprintf(msg," Par %d",Menu.menu);
            lcd_write_string(msg,30,Menu.top,MediumFont,BLACK_OVER_WHITE);
            lcd_write_char('^',0,Menu.top,MediumFont,BLACK_OVER_WHITE);
            Menu.top+=topSpace;
            sprintf(msg,"%5.1f",(float)ParTime[Menu.menu-1]/100);//unit is 10mS
            lcd_write_string(msg, 30, Menu.top, BigFont, BLACK_OVER_WHITE);
            lcd_write_char('_',0,Menu.top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
            Done=False;
            i=10;
            j=0;
            k=0;
            while (!Done)
            {
                if (Keypressed) 
                {
                    switch (Key)
                    {
                        case KeyUp: if ((ParTime[Menu.menu-1]+i)<=((ParTime[Menu.menu])-((BuzzerParDuration/10)+10)))
                                    {
                                        ParTime[Menu.menu-1]+=i;
                                        changed=True;
                                    }
                                    else 
                                    {
                                        redraw=2;
                                        DeletePar(Menu.menu-1); //DeletePar will save if needed no need change
                                        Done=True;
                                    }
                            break;
                        case KeyDw: if (TotPar>1) 
                                    {
                                        if ((ParTime[Menu.menu-1]-i)>=((ParTime[Menu.menu-2])+(BuzzerParDuration/10)+10))     
                                        {
                                            ParTime[Menu.menu-1]-=i;
                                            changed=True;
                                        }
                                        else 
                                        {
                                            redraw=2;
                                            DeletePar(Menu.menu-1);
                                            Done=True;
                                        }
                                    }
                                    else if (TotPar==1) 
                                    {
                                        if (ParTime[0]>=(((BuzzerStartDuration/10)+10)+i))
                                        {
                                            ParTime[0] -=i;
                                            changed=True;
                                        }
                                        else 
                                        {
                                            redraw=2;
                                            DeletePar(0);
                                            Done=True;
                                        }
                                    }
                            break;
                        default:    Done=True;
                                   
                    }
                    if (redraw<2) //do not draw for delate
                    {
                        sprintf(msg,"%5.1f",(float)ParTime[Menu.menu-1]/100);//unit is 10mS
                        lcd_write_string(msg, 30, Menu.top, BigFont, BLACK_OVER_WHITE);
                        sprintf(msg," Par %d: %5.1f",Menu.menu,(float)ParTime[Menu.menu-1]/100);//unit is 10mS
                        strcpy(SettingsMenu.MenuItem[Menu.menu-1],msg);        
                    }
                }
                while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
                if (Keypressed)   j++;   
                if (j>1)         { if (i*2<255)   i=i*2; else i=255;   j=0;   } //if after 500mS still pressed go faster
                if (!Keypressed) { i=10;j=0;}                  //if key not pressed go slow (again))
            }
        }    
    }
    if (changed)
    {
        saveTotPar();
        for (i=0;i<TotPar;i++) 
            savePar(i);
        PopMsg("Par Saved",200);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Backlight">
void SetBacklight()
{//PWM Backlite
    uint8_t top;
    TBool Done;
    char msg[20];
    
    if (BackLightLevel>99) BackLightLevel=50;
    strcpy(SettingsMenu.MenuTitle,"Settings: ");
    top = SettingsTitle();
    top+=3;
    lcd_write_string("Backlight",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    top+=topSpace;
    sprintf(msg,"%2d",BackLightLevel);
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=1;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((BackLightLevel+i)<99)     BackLightLevel+=i;
                            else BackLightLevel=99;
                            SaveToEEPROM=True; 
                break;
                case KeyDw: if ((BackLightLevel-i)>1)      BackLightLevel-=i;
                            else BackLightLevel=1;
                            SaveToEEPROM=True; 
                break;
                default:   Done=True;
            }
            set_backlight(BackLightLevel);
            sprintf(msg,"%2d",BackLightLevel);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i<40)   i=i*2;  j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=1;j=0;}                 //if key not pressed go slow (again))
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Buzzer Settings">
void SetBeepFreq(uint8_t top)
{
    TBool Done;
    char msg[20];
    
    if (BuzzerFrequency<800) BuzzerFrequency=800;
    if (BuzzerFrequency>3000) BuzzerFrequency=3000;
    strcpy(SettingsMenu.MenuTitle,"Settings: Beep");
    top = SettingsTitle();
    top+=3;
    lcd_write_string(" Frequency ",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    top+=topSpace;
    sprintf(msg,"%3.1f",(float)BuzzerFrequency/1000);
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint16_t i=100;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((BuzzerFrequency+i)<3000)     BuzzerFrequency+=i;
                            else BuzzerFrequency=3000;
                            SaveToEEPROM=True; 
                break;
                case KeyDw: if ((BuzzerFrequency-i)>800)      BuzzerFrequency-=i;
                            else BuzzerFrequency=800;
                            SaveToEEPROM=True; 
                break;
                default:   Done=True;
            }
            sprintf(msg,"%3.1f",(float)BuzzerFrequency/1000);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i<1000)   i=i*2;  j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=100;j=0;}                    //if key not pressed go slow (again))
    }
}
void SetBeepLevel(uint8_t top)
{
    TBool Done;
    char msg[20];
    
    strcpy(SettingsMenu.MenuTitle,"Settings: Beep");
    top = SettingsTitle();
    top+=3;
    lcd_write_string(" Loudness ",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    top+=topSpace;
    sprintf(msg,"%2d",BuzzerLevel);
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=1;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((BuzzerLevel+i)<10)     BuzzerLevel+=i;
                            else BuzzerLevel=10;
                            SaveToEEPROM=True; 
                            generate_sinus(BuzzerLevel,BuzzerFrequency,100);
                break;
                case KeyDw: if ((BuzzerLevel-i)>0)      BuzzerLevel-=i;
                            else BuzzerLevel=0;
                            SaveToEEPROM=True; 
                            generate_sinus(BuzzerLevel,BuzzerFrequency,100);
                break;
                default:   Done=True;
            }
            sprintf(msg,"%2d",BuzzerLevel);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }        
        while (Keypressed);
    }
}
void SetBeepTime(uint8_t top,TBool Par)
{
    TBool Done;
    char msg[20];
    uint16_t duration;
    
    if (Par) duration=BuzzerParDuration;
    else     duration=BuzzerStartDuration;
    if (duration<50) duration=50;
    if (duration>1000) duration=1000;
    strcpy(SettingsMenu.MenuTitle,"Settings: Beep ");
    top = SettingsTitle();
    top+=3;
    if (Par)   lcd_write_string("Par Duration",30,top,MediumFont,BLACK_OVER_WHITE);
    else       lcd_write_string("Start Duration",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    top+=topSpace;
    sprintf(msg,"%3.1fsec",(float)duration/1000);
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=50;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((duration+i)<1000)     duration+=i;
                            else duration=1000;
                            SaveToEEPROM=True; 
                break;
                case KeyDw: if ((duration-i)>50)      duration-=i;
                            else duration=50;
                            SaveToEEPROM=True; 
                break;
                default:   Done=True;
            }
            sprintf(msg,"%3.1fsec",(float)duration/1000);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i*2<255)   i=i*2; else i=255; j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=50;j=0;}                   //if key not pressed go slow (again))
    }
    if (Par) BuzzerParDuration=duration;
    else     BuzzerStartDuration=duration;
}
void SetBeep()
{
    TBool Done;
     
    Menu.menu=1;
    Menu.top=0;
    Menu.pos=0;
    Done=False;
    SettingsMenu.TotMenuItems=5;
    strcpy(SettingsMenu.MenuTitle,"Settings: Beep");
    strcpy(SettingsMenu.MenuItem[0]," Frequency ");
    strcpy(SettingsMenu.MenuItem[1]," Loudness ");
    strcpy(SettingsMenu.MenuItem[2]," Par Duration ");
    strcpy(SettingsMenu.MenuItem[3]," Start Duration ");
    strcpy(SettingsMenu.MenuItem[4]," Test Beep ");
    
    Menu.lineh=MediumFont_height+1; 
    //Top Line
    Menu.top = SettingsTitle();
    Menu.height=LCD_HEIGHT-(Menu.lineh+Menu.top);
    //Main Screen
    Menu.refresh=True;
    SettingsDisplay();
    
    while (!Done)
    {
        if (Keypressed)
        {
            switch (Key)
            {
                case KeyUp: if (Menu.menu>1) {
                                Menu.prev=Menu.menu;
                                Menu.refresh=False;
                                Menu.menu--;
                                SettingsDisplay();
                            }
                            else Beep();
                    break;
                case KeyDw: if (Menu.menu<(SettingsMenu.TotMenuItems)) {
                                Menu.prev=Menu.menu;
                                Menu.refresh=False;
                                Menu.menu++; 
                                SettingsDisplay();
                            }
                            else Beep();
                    break;
                case KeyIn: switch (Menu.menu)
                            {
                                case 1: SetBeepFreq(Menu.top); break;
                                case 2: SetBeepLevel(Menu.top); break;
                                case 3: SetBeepTime(Menu.top,True); break;
                                case 4: SetBeepTime(Menu.top,False);break;
                                case 5: generate_sinus(BuzzerLevel,BuzzerFrequency,BuzzerParDuration);break;
                            }
                            SettingsTitle();
                            Menu.refresh=True;
                            SettingsDisplay();
                    break;
                default :   Done=True;
                    break;
            }
            while (Keypressed); // wait here till key is released
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Sensitivity">
void SetSens()
{//Sensitivity

    TBool Done;
    char msg[20];
    uint8_t top;
    
    if (Sensitivity>10) Sensitivity=10;
    strcpy(SettingsMenu.MenuTitle,"Settings: ");
    top = SettingsTitle();
    top+=3;
    lcd_write_string("Sensitivity",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    top+=topSpace;
    sprintf(msg,"%2d",Sensitivity);
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=1;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((Sensitivity+i)<10)     Sensitivity+=i;
                            else Sensitivity=10;
                            SaveToEEPROM=True; 
                break;
                case KeyDw: if ((Sensitivity-i)>1)      Sensitivity-=i;
                            else Sensitivity=1;
                            SaveToEEPROM=True; 
                break;
                default:   Done=True;
            }
            sprintf(msg,"%2d",Sensitivity);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i<4)   i=i*2;  j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=1;j=0;}                 //if key not pressed go slow (again))
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Filter">
void SetFilter()
{//Filter

    TBool Done;
    char msg[20];
    uint8_t top;
    
    if (Filter>10) Filter=10;
    strcpy(SettingsMenu.MenuTitle,"Settings: ");
    top = SettingsTitle();
    top+=3;
    lcd_write_string("Filter",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    top+=topSpace;
    sprintf(msg,"%2d",Filter);
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=1;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((Filter+i)<10)     Filter+=i;
                            else Filter=10;
                            SaveToEEPROM=True; 
                break;
                case KeyDw: if ((Filter-i)>1)      Filter-=i;
                            else Filter=1;
                            SaveToEEPROM=True; 
                break;
                default:   Done=True;
            }
            sprintf(msg,"%2d",Filter);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i<4)   i=i*2;  j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=1;j=0;}                 //if key not pressed go slow (again))
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="AutoStart">
void SetAutoStart()
{
    TBool orgset; 
    TBool Done=False;
    Menu.top=0;
    Menu.pos=0;
    Menu.menu=1;
    SettingsMenu.TotMenuItems=1;
    strcpy(SettingsMenu.MenuTitle,"Settings: Tilt");
    if (AutoStart)   strcpy(SettingsMenu.MenuItem[0]," Auto Start ON ");
    else             strcpy(SettingsMenu.MenuItem[0]," Auto Start OFF ");
    
    Menu.top = SettingsTitle();
    Menu.refresh=True;

    SettingsDisplay();    
    orgset= AutoStart;
    while (!Done)
    {
        if (Keypressed)
        {
            switch (Key)
            {
                case KeyIn: AutoStart = !AutoStart ; 
                            SettingsTitle();
                            Menu.refresh=True;
                            if (AutoStart)   strcpy(SettingsMenu.MenuItem[0]," Auto Start ON ");
                            else             strcpy(SettingsMenu.MenuItem[0]," Auto Start OFF ");
                            SettingsDisplay();
                    break;
                default :   Done=True;
                    break;
            }
            while (Keypressed); // wait here till key is released
        }
    }
    
    SaveToEEPROM= (AutoStart!=orgset); 
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="TimerMode">
void SetMode()
{
    TBool orgset; 
     
    Menu.menu=1;
    Menu.top=0;
    Menu.pos=0;
    SettingsMenu.TotMenuItems=8;
    strcpy(SettingsMenu.MenuTitle,"Settings: Mode");
    strcpy(SettingsMenu.MenuItem[0]," Timer Mode ");
    strcpy(SettingsMenu.MenuItem[1]," Biancchi ");
    strcpy(SettingsMenu.MenuItem[2]," Barricade ");
    strcpy(SettingsMenu.MenuItem[3]," Falling Plate ");
    strcpy(SettingsMenu.MenuItem[4]," NRA-PPC A ");
    strcpy(SettingsMenu.MenuItem[5]," NRA-PPC B ");
    strcpy(SettingsMenu.MenuItem[6]," NRA-PPC C ");
    strcpy(SettingsMenu.MenuItem[7]," NRA-PPC D ");

    Menu.lineh=MediumFont_height+1; 
    Menu.top = SettingsTitle();
    Menu.height=LCD_HEIGHT-(Menu.lineh+Menu.top);
    Menu.refresh=True;
    //Main Screen
    SettingsDisplay();
    
    MenuSelection();
    switch (Menu.menu)
    {
        case 1://TBC
            break;
        case 2:
            break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Clock">
void SetClock()
{
    uint8_t menu,top,pos,height;
    uint8_t hour = get_hour();
    uint8_t minute = get_minute();
    TBool Done;
     
    menu=1;
    top=0;
    pos=0;
    Done=False;
    SettingsMenu.TotMenuItems=9;
    strcpy(SettingsMenu.MenuTitle,"Settings: Clock");
    top = SettingsTitle();
    //Main Screen
    
    TBool Done;
    char msg[20];
    
    if (hour>23) hour=23;
    if (minute>59) minute=59;

    top+=3;
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    sprintf(msg,"%02d:%02d",hour,minute);
    top+=topSpace;
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0, top+BigFont_height+botSpace ,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=1;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if (minute+i<59)     minute+=i;
                            else {
                                minute=0;
                                if (hour<23)  hour++; 
                                else (hour=0);
                            }
                            SaveToEEPROM=True;
                break;
                case KeyDw: if (minute-i>0)     minute-=i;
                            else {
                                minute=60-i;
                                if   (hour>0)  hour--; 
                                else (hour=23);
                            }
                            SaveToEEPROM=True;
                break;
                default:   Done=True;
            }
            if (hour>23) hour=23;
            if (minute>59) minute=59;
            sprintf(msg,"%02d:%02d",hour,minute);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i<60)   i=i*2;  j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=1;j=0;}                  //if key not pressed go slow (again))
    }
    set_time(hour,minute,0);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="CountDown">
void CountDownMode(uint8_t cdt)
{
    uint8_t top,prev_cdtime,tcount;
    uint16_t cdtime;
    TBool Run;
    char msg[20];
    
    prev_cdtime=0;
    strcpy(SettingsMenu.MenuTitle,"Cntdwn");
    top = SettingsTitle();
    while (!Exit)
    {
        cdtime=cdt*10;
        Run=False;
        tcount=0;
        while (cdtime>0)
        {
            if (cdtime!=prev_cdtime)
            {
                sprintf(msg,"%02d:%02d",cdtime/60,cdtime%60);
                lcd_write_string(msg, 10, top+30, BigFont, BLACK_OVER_WHITE);
                prev_cdtime=cdtime;
            }
            switch (Key)
            {
                case KeySt: Run=True; break;
                case KeyRw: Run=False; break;
                case KeyBk: Run=False; cdtime=0; break; //Exit no sound
                case KeyIn: Run=True; cdtime=0; break;  //Exit with sound
            }
            if (Run && (tcount>98)) 
            {
                tcount=0;
                cdtime--;
            }
            __delay_ms(10);
            if (Run) tcount++;
        }
        if (Run) 
        {
            for (char i=0;i<5;i++)
            {
                for (char j=0;j<3;j++)
                {
                    generate_sinus(1,BuzzerFrequency,50);
                    for (char t=0;t<100;t++)
                    {
                        if (Keypressed) return;
                        __delay_ms(1);
                    }
                }
                for (char t=0;t<200;t++)
                {
                    if (Keypressed) return;
                    __delay_ms(2);
                }
            }
        }
    }
}
void SetCustomCountDown(uint8_t top)
{
    TBool Done;
    char msg[20];
    
    strcpy(SettingsMenu.MenuTitle,"Settings: ");
    top = SettingsTitle();
    top +=3;
    lcd_write_string("Countdown",30,top,MediumFont,BLACK_OVER_WHITE);
    lcd_write_char('^',0,top,MediumFont,BLACK_OVER_WHITE);
    sprintf(msg,"%02d:%02d",(CustomCDtime*10)/60,(CustomCDtime*10)%60);
    top +=topSpace;
    lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
    lcd_write_char('_',0,top+BigFont_height+botSpace,MediumFont,BLACK_OVER_WHITE);
    Done=False;
    uint8_t i=1;
    uint8_t j=0;
    uint8_t k=0;
    
    while (!Done)
    {
        if (Keypressed) 
        {
            switch (Key)
            {
                case KeyUp: if ((CustomCDtime+i)<99)     CustomCDtime+=i;
                            else CustomCDtime=99;
                            SaveToEEPROM=True;
                break;
                case KeyDw: if ((CustomCDtime-i)>1)      CustomCDtime-=i;
                            else CustomCDtime=1;
                            SaveToEEPROM=True;
                break;
                default:   Done=True;
            }
            sprintf(msg,"%02d:%02d",(CustomCDtime*10)/60,(CustomCDtime*10)%60);
            lcd_write_string(msg, 30, top, BigFont, BLACK_OVER_WHITE);
        }
        
        while ((Keypressed) && (k<250)) { __delay_ms(1); k++; } k=0;
        if (Keypressed)   j++;   
        if (j>1)         { if (i<11)   i=i*2;  j=0;   }//if after 500mS still pressed go faster
        if (!Keypressed) { i=1;j=0;}                 //if key not pressed go slow (again))
    }
}
void SetCountDown()
{
    TBool Done;
     
    Menu.menu=1;
    Menu.top=0;
    Menu.pos=0;
    Done=False;
    SettingsMenu.TotMenuItems=4;
    strcpy(SettingsMenu.MenuTitle,"Settings:Countdown");
    strcpy(SettingsMenu.MenuItem[0]," Off ");
    strcpy(SettingsMenu.MenuItem[1]," 3 minutes ");
    strcpy(SettingsMenu.MenuItem[2]," 5 minutes ");
    strcpy(SettingsMenu.MenuItem[3]," Custom ");
    
    Menu.lineh=MediumFont_height+1; 
    //Top Line
    Menu.top = SettingsTitle();
    Menu.height=LCD_HEIGHT-(Menu.lineh+Menu.top);
    Menu.refresh=True;
    //Main Screen
    SettingsDisplay();
    
    MenuSelection();
    switch (Menu.menu)
    {
        case 2: CountDownMode(18); break;
        case 3: CountDownMode(30); break;
        case 4: SetCustomCountDown(Menu.top);CountDownMode(CustomCDtime); break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Tilt">
void SetTilt()
{
    uint8_t orgset; 
    TBool Done=False;
     
    Menu.top=0;
    Menu.pos=0;
    Menu.menu=1;
    SettingsMenu.TotMenuItems=1;
    strcpy(SettingsMenu.MenuTitle,"Settings: Tilt");
    if ((AR_IS&1)==1)   strcpy(SettingsMenu.MenuItem[0]," Auto Rotate ON ");
    else                strcpy(SettingsMenu.MenuItem[0]," Auto Rotate OFF ");
    
    Menu.top = SettingsTitle();
    Menu.refresh=True;

    SettingsDisplay();    
    orgset= AR_IS;

    while (!Done)
    {
        if (Keypressed)
        {
            switch (Key)
            {
                case KeyIn: AR_IS = AR_IS ^ 1 ; 
                            SettingsTitle();
                            Menu.refresh=True;
                            if ((AR_IS&1)==1)   strcpy(SettingsMenu.MenuItem[0]," Auto Rotate ON ");
                            else                strcpy(SettingsMenu.MenuItem[0]," Auto Rotate OFF ");
                            SettingsDisplay();
                    break;
                default :   Done=True;
                    break;
            }
            while (Keypressed); // wait here till key is released
        }
    }
    SaveToEEPROM= (AR_IS!=orgset); 
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Input">
void UpdateIS(void)
{
    if ((AR_IS&2)==2)   strcpy(SettingsMenu.MenuItem[0]," Microphone: ON  ");
    else                strcpy(SettingsMenu.MenuItem[0]," Microphone: OFF ");
    if ((AR_IS&4)==4)   strcpy(SettingsMenu.MenuItem[1]," A: ON  ");
    else                strcpy(SettingsMenu.MenuItem[1]," A: OFF ");
    if ((AR_IS&8)==8)   strcpy(SettingsMenu.MenuItem[2]," B: ON  ");
    else                strcpy(SettingsMenu.MenuItem[2]," B: OFF ");
}

void SetInput()
{
    uint8_t orgset; 
    TBool Done=False;
     
    Menu.top=0;
    Menu.pos=0;
    Menu.menu=1;
    SettingsMenu.TotMenuItems=3;
    strcpy(SettingsMenu.MenuTitle,"Settings: Input");
    Menu.top = SettingsTitle();
    Menu.refresh=True;
    UpdateIS();
    SettingsDisplay();    
    orgset= AR_IS;
    
    while (!Done)
    {
        if (Keypressed)
        {
            switch (Key)
            {
                case KeyUp: if (Menu.menu>1) {
                                Menu.prev=Menu.menu;
                                Menu.refresh=False;
                                Menu.menu--;
                                SettingsDisplay();
                            }
                            else Beep();
                    break;
                case KeyDw: if (Menu.menu<(SettingsMenu.TotMenuItems)) {
                                Menu.prev=Menu.menu;
                                Menu.refresh=False;
                                Menu.menu++; 
                                SettingsDisplay();
                            }
                            else Beep();
                    break;
                case KeyIn: switch (Menu.menu)
                            {
                                case 1: AR_IS = AR_IS ^ 2 ; break;
                                case 2: AR_IS = AR_IS ^ 4 ; break;
                                case 3: AR_IS = AR_IS ^ 8 ; break;
                            }
                            SettingsTitle();
                            Menu.refresh=True;
                            UpdateIS();
                            SettingsDisplay();
                    break;
                default :   Done=True;
                    break;
            }
            while (Keypressed); // wait here till key is released
        }
    }
 
    SaveToEEPROM= (AR_IS!=orgset); 
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="BlueTooth">
void BlueTooth()
{
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_write_string(" TBD9 ", 3, 40, MediumFont, BLACK_OVER_WHITE);
    __delay_ms(500);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Settings Menu">
void DoSet(void)  
{
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    switch (Menu.menu)
    {
        case 1:SetDelay();
            break;
        case 2:SetPar();
            break;
        case 3:SetBeep();
            break;
        case 4:SetAutoStart();
            break;
        case 5:SetMode();
            break;
        case 6:SetClock();
            break;
        case 7:SetCountDown();
            break;
        case 8:SetTilt();
            break;
        case 9:SetBacklight();
            break;
        case 10:SetSens();
            break;
        case 11:SetFilter();
            break;
        case 12:SetInput();
            break;
        case 13:BlueTooth();
            break;
        case 14:
            saveSettings();
            PopMsg("Saved",1000);
            break;
        case 15:Diagnostics();
            break;
    }
    Menu.refresh=True;
}
void SetSettingsMenu()
{
//{"Delay","Par","Beep","Auto","Mode","Clock","CountDown","Tilt","Bklight","Input","BT","Diag"};
    SettingsMenu.TotMenuItems=13;
    strcpy(SettingsMenu.MenuTitle,"Settings ");
    strcpy(SettingsMenu.MenuItem[0]," Delay ");
    strcpy(SettingsMenu.MenuItem[1]," Par ");
    strcpy(SettingsMenu.MenuItem[2]," Buzzer ");
    strcpy(SettingsMenu.MenuItem[3]," Auto Start ");
    strcpy(SettingsMenu.MenuItem[4]," Timer Mode ");
    strcpy(SettingsMenu.MenuItem[5]," Clock ");
    strcpy(SettingsMenu.MenuItem[6]," Countdown ");
    strcpy(SettingsMenu.MenuItem[7]," Tilt ");
    strcpy(SettingsMenu.MenuItem[8]," Backlight ");
    strcpy(SettingsMenu.MenuItem[9]," Sensitivity ");
    strcpy(SettingsMenu.MenuItem[10]," Filter ");
    strcpy(SettingsMenu.MenuItem[11]," Input ");
    strcpy(SettingsMenu.MenuItem[12]," Bluetooth ");
    strcpy(SettingsMenu.MenuItem[13]," SaveSettings ");
    strcpy(SettingsMenu.MenuItem[14]," Diagnostics ");
}
void Settings(void)
{
    Menu.menu=1;
    Menu.pos=0;
    Menu.top=0;
    
    Menu.lineh=MediumFont_height+1; 
    SetSettingsMenu();
    //Top Line
    Menu.top = SettingsTitle();
    Menu.height=LCD_HEIGHT-(Menu.lineh+Menu.top);
    //Main Screen
    SaveToEEPROM=False;
    Menu.refresh=True;
    Menu.page=0;//Force refresh
    
    do {
        SetSettingsMenu();
        SettingsTitle();
        SettingsDisplay();
        MenuSelection();
        if (Menu.menu>0)  DoSet();
    } while (Menu.menu>0);
    if (SaveToEEPROM)
    {
        saveSettings();
        PopMsg("Saved",200);
        return;
    }
}
// </editor-fold>
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="ReviewMenu">
void ReviewDisplay(uint8_t battery,uint8_t CurShoot, uint8_t CurShootStringDisp, TBool FullRedraw) 
{
   char message[60];
   uint8_t line=0;
   uint8_t halfline=((MediumFont_height/2)+1);
           
   if (!FullRedraw)  line =((MediumFont_height*3)+16);
   lcd_clear_block(0,line,LCD_WIDTH,LCD_HEIGHT);
   if (FullRedraw)
   {
        //Top Line
        lcd_write_string("REVIEW",0,line,MediumFont,BLACK_OVER_WHITE);
        if (BT) lcd_write_string("BT",123,line,SmallFont,BLACK_OVER_WHITE);
        lcd_battery_info(LCD_WIDTH-20,line,battery);
        line +=MediumFont_height;
        lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
        line ++;
        //String line 
        lcd_write_char('^',0,line,MediumFont,BLACK_OVER_WHITE);
        line +=halfline;
        sprintf(message, " Str:%2d/%2d %6.2f ", CurShootStringDisp,ShootString.TotShoots,(float)ShootString.ShootTime[ShootString.TotShoots]/100);
        lcd_write_string(message,12,line,MediumFont,WHITE_OVER_BLACK);
        line +=halfline;
        lcd_write_char('_',0,line,MediumFont,BLACK_OVER_WHITE);
        line +=MediumFont_height;
        lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
        line +=15;
   }
   //Shoot lines
   //1st ShootNumber 01, before it ShootNumber 00 time=0
   if (ShootString.ShootTime[CurShoot-1]>0)
   {
        sprintf(message, " %2d:%6.2f ", CurShoot-1,(float)ShootString.ShootTime[CurShoot-1]/100);
        lcd_write_string(message,0,line,MediumFont,BLACK_OVER_WHITE);
        lcd_write_char('^',LCD_WIDTH-10,line,MediumFont,BLACK_OVER_WHITE);
        line +=halfline;
        sprintf(message, "}%6.2f",(float)(ShootString.ShootTime[CurShoot]-ShootString.ShootTime[CurShoot-1])/100);
        lcd_write_string(message,89,line,MediumFont,BLACK_OVER_WHITE);
        line +=halfline;
   }
   
   if (ShootString.ShootTime[CurShoot]>0)
   {
        sprintf(message, " %2d:%6.2f ", CurShoot,(float)ShootString.ShootTime[CurShoot]/100);
        lcd_write_string(message,0,line,MediumFont,WHITE_OVER_BLACK);
        line +=halfline;
        if (CurShoot<ShootString.TotShoots)
        {
             sprintf(message, "}%6.2f",(float)(ShootString.ShootTime[CurShoot+1]-ShootString.ShootTime[CurShoot])/100);
             lcd_write_string(message,89,line,MediumFont,BLACK_OVER_WHITE);
        }
        line +=halfline;
   }
   if (CurShoot<ShootString.TotShoots)
   {
        if (ShootString.ShootTime[CurShoot+1]>0)
        {
            sprintf(message, " %2d:%6.2f ", CurShoot+1,(float)ShootString.ShootTime[CurShoot+1]/100);
            lcd_write_string(message,0,line,MediumFont,BLACK_OVER_WHITE);
        }
   }
   lcd_write_char('_',LCD_WIDTH-10,line,MediumFont,BLACK_OVER_WHITE);
}
void DoReview()
{
    TestBattery();
    CurShootString=0;
    CurShoot=2;
    //getShootString(CurShootString);
    ReviewDisplay(battery_level,CurShoot,CurShootString+1,True);
    while (!Exit)
    {
        TestBattery();
        if (Keypressed)
        {
            switch (Key)
            {
                case KeySt:   return;
           
                case KeyUp:   if (CurShoot>1) {
                                  CurShoot--;
                                  ReviewDisplay(battery_level,CurShoot,CurShootString+1,False);
                              }
                              else Beep();
                              while (Key==KeyUp);
                              break;
                case KeyDw:   if (CurShoot<ShootString.TotShoots) {
                                  CurShoot++;
                                  ReviewDisplay(battery_level,CurShoot,CurShootString+1,False);
                              }
                              else Beep();
                              while (Key==KeyDw);
                              break;
                case KeyInDw: if (CurShootString>0) {
                                 CurShootString--;
                                 getShootString(CurShootString);
                                 ReviewDisplay(battery_level,CurShoot,CurShootString+1,True);
                              }
                              else Beep();
                              while (Key==KeyInDw);
                              break;
                case KeyInUp: if (CurShootString<30) {
                                 CurShootString++;
                                 getShootString(CurShootString);
                                 ReviewDisplay(battery_level,CurShoot,CurShootString+1,True);
                              }
                              else Beep();
                              while (Key==KeyInUp);
                              break;
            }           
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Main Menu">

void DetectInit(void)
{
    uint16_t Mean=0;
    uint16_t Peak=0;
    uint16_t ADCvalue;

    for (uint8_t i=0;i<64;i++)
    {
        ADCvalue=ADC_Read(ENVELOPE);
        Mean+=ADCvalue;
        if (Peak<ADCvalue) Peak=ADCvalue;
    }
    Mean=Mean>>6;
    switch (Sensitivity)
    {
        case 10 :   DetectThreshold=Mean+33;break;
        case  9 :   DetectThreshold=Mean+36;break;
        case  8 :   DetectThreshold=Mean+43;break;
        case  7 :   DetectThreshold=Mean+51;break;
        case  6 :   DetectThreshold=Mean+61;break;
        case  5 :   DetectThreshold=Mean+73;break;
        case  4 :   DetectThreshold=Mean+87;break;
        case  3 :   DetectThreshold=Mean+104;break;
        case  2 :   DetectThreshold=Mean+124;break;
        case  1 :   DetectThreshold=Mean+148;break;
    }    
}
TBool Detect(void)
{
    switch (DetectMode)
    {
        case Mic:  return ADC_Read(ENVELOPE)>DetectThreshold;
        case AuxA: return False;//TBC
        case AuxB: return False;//TBC
        default: return False;
    }
}

uint8_t print_time(uint8_t line,uint8_t pos){
    char message[10];
    lcd_clear_block(pos,line,45,MediumFont->height);
    //Top Line
    sprintf(message,"%02d%s%02d",get_hour(),(rtc_time_sec%2)?":":".",get_minute());
 //   sprintf(message,"%d",timers_diff);    
    lcd_write_string(message,pos,line,MediumFont,BLACK_OVER_WHITE);
}
uint8_t print_header_line(){
    uint8_t line = 0;
    TBool Aux = false;
    char message[10];
    lcd_clear_block(line,0,LCD_WIDTH,MediumFont->height+6);
    print_time(line,0);
    if (Aux) lcd_write_string("Aux:On ",45,line,SmallFont,BLACK_OVER_WHITE);
    else     lcd_write_string("Aux:Off",45,line,SmallFont,BLACK_OVER_WHITE);
    if ((AR_IS&4)==4) lcd_write_string("A",95,line,SmallFont,BLACK_OVER_WHITE);
    if ((AR_IS&8)==8)  lcd_write_string("B",110,line,SmallFont,BLACK_OVER_WHITE);
    if (BT) lcd_write_string("BT",123,line,SmallFont,BLACK_OVER_WHITE);
    else lcd_write_char(' ',20,line,MediumFont,WHITE_OVER_BLACK);
    lcd_battery_info(LCD_WIDTH-20,line,battery_level);
    lcd_draw_hline(0,LCD_WIDTH,MediumFont->height+1,BLACK_OVER_WHITE);
    return MediumFont->height + 7;
}
uint8_t MainDisplay(uint8_t ShootNumber, uint8_t par, uint8_t voffset) 
                    //percentage, 10mS, 10mS, num, 10mS, TdelTy, TBool
{
    char message[60];
    uint8_t line=voffset;
    uint8_t pos=0;

    lcd_clear_block(voffset,0,LCD_WIDTH,LCD_HEIGHT);
    pos=line;
    //Main line 
    sprintf(message, "%6.2f", (float)ShootString.ShootTime[ShootNumber]/100);
    lcd_write_string(message,0,line,BigFont,BLACK_OVER_WHITE);
    line +=BigFont_height;
    line +=5;
    sprintf(message, "%6.2f",(float)ShootString.ShootTime[0]/100);
    lcd_write_string(message,0,line,MediumFont,BLACK_OVER_WHITE);
    sprintf(message, "Shoot:%2d",ShootNumber);
    lcd_write_string(message,LCD_WIDTH-62,line,MediumFont,BLACK_OVER_WHITE);
    line +=MediumFont_height;
    line++;
    lcd_draw_hline(0,LCD_WIDTH,line,BLACK_OVER_WHITE);
    line +=3;
    // bottom lines
    switch (DelayMode)
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
    lcd_write_string(message,0,line,SmallFont,BLACK_OVER_WHITE);
    if ((AR_IS&2)==2) sprintf(message, "Mic:%d",Sensitivity);
    else             sprintf(message, "Mic:%d",0);
    lcd_write_string(message,LCD_WIDTH-32,line,SmallFont,BLACK_OVER_WHITE);
    line +=SmallFont_height;
    line++;
    if (ParTime[par]>0)
    {
        sprintf(message, "Par%2d:%5.1f",par+1,(float)ParTime[par]/100);
        lcd_write_string(message,0,line,MediumFont,BLACK_OVER_WHITE);
    }
    sprintf(message, "Buz:%d",BuzzerLevel);
    lcd_write_string(message,LCD_WIDTH-32,line,SmallFont,BLACK_OVER_WHITE);
    return pos;
}
void DoMain(void)
{
    char message[10];
    TestBattery();
    uint24_t FirstTime=0;
    uint8_t Shoot=0;
    uint8_t pos;
    ShootString.ShootTime[Shoot]=0;
    getPar();
    uint8_t Par=0;
    TBool Done=False;
    pos=MainDisplay(Shoot,Par,print_header_line());
    
    switch (DelayMode)
    {
        case Instant:DelayTime=0;
        break;        
        case Fixed:  DelayTime=30;
        break;        
        case Random: DelayTime=(5*DelayTime)%99;
        break;        
        case Custom: DelayTime=eeprom_read_wdata(DelayTime_Address);
             //Read again in case was changed from other mode
        break;        
    }
    uint16_t t=DelayTime;
    DetectMode=Mic;
    DetectInit();
    t=DelayTime*10;
    while (t>8)
    {
        if (t%2==0) 
        {
            sprintf(message, "%6.2f",(float)t/100);
            lcd_write_string(message,0,pos,BigFont,BLACK_OVER_WHITE);
        }
        t-=9;
    }
    lcd_write_string("  0.00",0,pos,BigFont,BLACK_OVER_WHITE);
    generate_sinus(BuzzerLevel,BuzzerFrequency,BuzzerStartDuration);
    delay_rtc(150);
    t=0;
    while (!Done)
    {
        TestBattery();
        if (Detect()) 
        {
            Shoot++;
            ShootString.TotShoots=Shoot;
            ShootString.ShootTime[Shoot]=t; 
            if (Shoot==1) FirstTime=t;
            MainDisplay(Shoot,Par,print_header_line());
        }
        __delay_ms(10);
        t++;
        if ((t>ParTime[Par]) && (Par<TotPar)) 
        { 
            Par++;
            MainDisplay(Shoot,Par,print_header_line());
            generate_sinus(BuzzerLevel,BuzzerFrequency,BuzzerParDuration); 
            __delay_ms(150);
        } 
        if (t>99900) 
        {
            if (Shoot>0) saveShootString();
            DoReview();
            return;
        }
        if (Keypressed)
            switch (Key)
            {
                case KeyRw: {
                                t=0;
                                if (Shoot>0) saveShootString();
                                DoReview();
                                return;
                            }
                case KeySt: if (AutoStart)
                                {
                                    t=0;
                                    if (Shoot>0) saveShootString();
                                    Shoot=0;
                                }
            }
    }
}
// </editor-fold>
// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="RTC functions">

static void interrupt isr(void) {
    if (RTC_TIMER_IF) {
        RTC_TIMER_IF = 0;   // Clear Interrupt flag.
        handle_preceise_time();
    }
//    if (SHOOT_IF){
//        SHOOT_IF = 0;       //Clear interrupt        
//        save_shoot_time();
//    }
//    if (ACCELEROMETR_IF){
//        ACCELEROMETR_IF = 0;
//        if(orientation_changed()){
//            mark_to_flip_screen();
//        }
//    }
//    if(IOCBFbits.IOCBF1) {
//        if(PORTBbits.RB1 == 1) {
//            button_up_time = get_rtc_time();
//            button_down_time = 0;
//        } else {
//            button_down_time = get_rtc_time();
//            if((button_down_time - button_up_time) >= 3) {
//                if(system_operation_mode == SYS_MODE_SLEEP) {
//                    system_operation_mode = SYS_MODE_NORMAL;
//                    init_10ms_timer0();
//                } else {
//                    system_operation_mode = SYS_MODE_SLEEP;
//                    T0CON0bits.T0EN = 0;        // Stop timer.
//                }
//            }
//        }
//        IOCBFbits.IOCBF1 = 0;
//    }
    if(PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        rtc_time_msec++;
        refresh_lcd = (rtc_time_msec%25==0);
        init_1ms_timer0();
    }
}
// </editor-fold>

void main(void) {
    // <editor-fold defaultstate="collapsed" desc="Initialization">
    PIC_init();
    PowerON
    initialize_backlight();
    set_backlight(90);
    spi_init();
    lcd_init();
    ADC_init();
    eeprom_init();
    getSettings();
    getPar();
    set_backlight(BackLightLevel);
    SmallFont_height = tahoma_8ptFontInfo.height;
    MediumFont_height = timesNewRoman_11ptFontInfo.height;
    BigFont_height = microsoftSansSerif_42ptFontInfo.height;
    initialize_rtc_timer();
    init_1ms_timer0();
    // Initialization End
    // </editor-fold>
    // <editor-fold defaultstate="collapsed" desc="Main">
    uint8_t to = 0;
    TestBattery();
    MainDisplay( 0, 0, print_header_line());
    while (True) {
        if (Powered) {
            TestBattery();
            print_header_line();
            if (Keypressed) {

                switch (Key) {
                    case KeyUp:
                    case KeyDw:Settings();
                        break;
                    case KeyRw:DoReview(); // ReviewDisplay(battery_level,7, 3,True);
                        break;
                    case KeySt:
                        to = 0;
                        while (Keypressed) {
                            __delay_ms(20);
                            to++;
                            if (to > Timeoff) {
                                PowerOFF;
                                set_backlight(0);
                                lcd_clear();
                                //                                 OSCFRQ = 0b00000000;        // 1 MHz Fosc.
                            }
                        }
                        if (Powered) DoMain(); //MainTimer display
                        break;
                }
                if (Powered) {
                    MainDisplay( 0, 0, print_header_line());
                   
                }
            }
        } else {
            to = 0;
            while (Keypressed) {
                __delay_ms(20);
                to++;
                if (to > Timeoff) {
                    //                    OSCFRQ = 0b00001000;        // 64 MHz Fosc.

                    PowerON;
                    set_backlight(BackLightLevel);
                    MainDisplay(0, 0, print_header_line());
                }
            }
        }
        while (Keypressed); //Wait till key is released
        lcd_refresh();
    }
    // </editor-fold>
}
