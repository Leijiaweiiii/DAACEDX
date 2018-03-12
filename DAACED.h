/* ===========================================================================
    Project : DAACED
    Version 1.0
 
 *  File:   DAACED.h
 *  Author: Eli
 *  Created on Sept 15, 2017
 
    Global R&D ltd. 04_9592201    ;  www.global_rd.com
    Eli Jacob 054_8010330         ;  eli@global_rd.com
   ===========================================================================*/

// <editor-fold defaultstate="collapsed" desc="Includes">
#ifndef _DAACED_H_
#define _DAACED_H_

#pragma warning disable 373
#pragma warning disable 520

#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "DAACEDbitmap.h"
#include "DAACEDfont.h"

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="General">
#define _XTAL_FREQ            (64000000UL) //64MHz

#define	True 1
#define	False 0
#define	On 1
#define	Off 0
#define	Pos 1
#define	Neg 0
typedef enum {
    false=0, true=1
} TBool;

#define HEX2DEC(x)            (x > '9') ? (x - 'a')+10 : x-'0'
#define DEC(x)                (x-'0')
#define Delay(t)           {for (int w=0 ; w<t ; w++)  __delay_ms(1);}

#define IO_AS_OUTPUT            (0)
#define IO_AS_INPUT             (1)
#define BIT(bit_position)       (1<<bit_position)
#define SWAP(x,y)               { x = x + y; y = x - y; x = x - y;}
#define UNUSED(x)               (void)(x)

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="LCD parameters and definitions">
#define LCD_WIDTH               (160)
#define LCD_HEIGHT              (115)
#define LCD_MAX_PAGES           (0x0F)
#define Y_OFFSET                (6)
#define BLACK_OVER_WHITE        (0x00)
#define WHITE_OVER_BLACK        (0x01)

#define LCD_CS_SELECT()         (LATFbits.LF3 = 0)
#define LCD_CS_DESELECT()       (LATFbits.LF3 = 1)

#define LCD_MODE_DATA()         (LATFbits.LF4 = 1)
#define LCD_MODE_COMMAND()      (LATFbits.LF4 = 0)

#define LCD_RESET_EN()          (LATFbits.LF6 = 0)
#define LCD_RESET_DIS()         (LATFbits.LF6 = 1)

#define LCD_BACKLIGHT_ON()      (LATEbits.LE6 = 0)
#define LCD_BACKLIGHT_OFF()     (LATEbits.LE6 = 1)

int16_t cursor_x, cursor_y;
uint8_t lcd_buffer[2600];
static uint8_t x_update_min, x_update_max, y_update_min, y_update_max;

#define topSpace     20
#define botSpace     10

#define PopX1       LCD_WIDTH/8 
#define PopY1       LCD_HEIGHT/4
#define PopX2       LCD_WIDTH*7/8
#define PopY2       LCD_HEIGHT*3/4

const FONT_INFO *SmallFont = &tahoma_8ptFontInfo;
uint8_t SmallFont_height;
const FONT_INFO *MediumFont = &timesNewRoman_11ptFontInfo;
uint8_t MediumFont_height;
const FONT_INFO *BigFont = &microsoftSansSerif_42ptFontInfo;
uint8_t BigFont_height;

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
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="uC-ADC Definitions">
#define	ACCELEROMETER  0b000000          // AN0
#define	MICROPHONE     0b000001          // AN1
#define	ENVELOPE       0b000010          // AN2
#define	BATTERY        0b000011          // AN3
#define BAT_divider    4   // resistors relationship, will be changed to 4.96 (12.4K/10.0K)*4 to allow measure of 4.5V
TBool BatteryLow = False;
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Sinus Generator">
extern uint8_t sinus_table[32];
volatile uint8_t current_sample_index = 0;
volatile uint16_t timer_reload_value;
volatile uint8_t amplitude_value;
volatile int16_t dac_value;
void sinus_dac_init();
void generate_sinus(uint8_t amplitude, uint16_t frequency, int16_t duration);
void start_sinus(uint16_t frequency);
void stop_sinus();
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM">
// TODO Remove

// <editor-fold defaultstate="collapsed" desc="EEPROM Specificaion">
/* EEPROM specification */
#define EEPROM_PAGE_SIZE    (64)
#define EEPROM_MAX_SIZE     (0x8000)

/* Instructions */
#define CMD_WREN            (0x06)
#define CMD_WRDI            (0x04)
#define CMD_RDSR            (0x05)
#define CMD_WRSR            (0x01)
#define CMD_READ            (0x03)
#define CMD_WRITE           (0x02)
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM Hardware Interface">
#define EEPROM_CS_INIT()        (TRISFbits.TRISF0 = 0)

#define EEPROM_CS_SELECT()      (LATFbits.LATF0 = 0)
#define EEPROM_CS_DESELECT()    (LATFbits.LATF0 = 1)

#define EEPROM_HOLD_INIT()      (TRISFbits.TRISF2 = 0)

#define EEPROM_HOLD_EN()        (LATFbits.LATF2 = 0)
#define EEPROM_HOLD_DIS()       (LATFbits.LATF2 = 1)

#define EEPROM_WP_INIT()        (TRISFbits.TRISF1 = 0)

#define EEPROM_WP_EN()          (LATFbits.LATF1 = 0)
#define EEPROM_WP_DIS()         (LATFbits.LATF1 = 1)
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM Helper Functions">
void eeprom_spi_init();
uint8_t eeprom_spi_write(uint8_t data);
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="EEPROM Functions">
void eeprom_init();
void eeprom_write_data(uint16_t address, uint8_t data);
uint8_t eeprom_read_data(uint16_t address);
uint16_t eeprom_read_array(uint16_t address, uint8_t *data, uint16_t no_of_bytes);
void eeprom_busy_wait();
uint8_t eeprom_read_status_reg();
//</editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Backlight PWM functions">
void initialize_backlight();
void set_backlight(uint8_t duty_cycle);
uint8_t find_optimal_PWM_settings(int32_t freq, uint8_t *selectedPRvalue, uint8_t *selectedPrescalar);
uint8_t is_power_of2(uint8_t n);
uint8_t find_set_bit_position(uint8_t n);

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="RTC timer functions">
#define RTC_TIMER_IE    (PIE5bits.TMR1IE)
#define RTC_TIMER_IF    (PIR5bits.TMR1IF)

#define SYS_MODE_NORMAL (0)
#define SYS_MODE_SLEEP  (1)

volatile uint32_t rtc_time;
volatile uint32_t button_down_time, button_up_time;
volatile uint8_t system_operation_mode = SYS_MODE_NORMAL;
void initialize_rtc_timer();
void set_rtc_time(uint32_t time);
uint32_t get_rtc_time();
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Wakeup and Sleep button.">
#define SLEEP_BUTTON_TRIS       (TRISEbits.TRISE1)

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disp&Keys">
#define Bot                  LCD_HEIGHT-30
#define Key                  (PORTB & 0x3F)
#define KeySt                 0x02  //Start
#define KeyRw                 0x01  //Review
#define KeyBk                 0x04  //Back
#define KeyDw                 0x08  //v
#define KeyUp                 0x10  //^
#define KeyIn                 0x20  //Enter
#define KeyInDw               0x28  //Enter+v
#define KeyInUp               0x30  //Enter+^

#define Keypressed           (Key>0)
#define Exit                 (Key==KeyBk)
#define Select               (Key==KeyIn)
#define Start                (Key==KeySt)
#define Stop                 (Key==KeyRw)
// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Ports">
unsigned char PortE_Data;
#define BackLightON          {PortE_Data = PORTE ; PortE_Data &= 0xBF ; PORTE = PortE_Data;}
#define BackLightOFF         {PortE_Data = PORTE ; PortE_Data |= 0x40 ; PORTE = PortE_Data;}

TBool Powered;
#define PowerON              {LATEbits.LATE0 = 1; Powered=True;}
#define PowerOFF             {LATEbits.LATE0 = 0; Powered=False;}

#define BuzzerPeriod          50            //[10uSec]  = 10/Freq
#define BuzzerCycles          300           //Cycles
#define DACdata               DAC1CON1

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Nominal">
#define Timeoff                      150 //150*20= 3sec

uint8_t AR_IS=2;  // 1= AutoRotate 2=Mic 4=A 8=B
#define AR_IS_Address                100
uint16_t BuzzerFrequency=2000;
uint16_t BuzzerParDuration=200;
uint16_t BuzzerStartDuration=200;
uint8_t BuzzerLevel=2;
#define BuzzerStartDuration_Address  102
#define BuzzerFrequency_Address      104
#define BuzzerParDuration_Address    106
#define BuzzerLevel_Address          108
uint8_t CustomCDtime=18;
#define CustomCDtime_Address         110
uint8_t hour=12;
uint8_t minute=32;
TBool   BT=False;
TBool   SaveToEEPROM;
#define BT_Address                   112

#define MAXSHOOT    50//100
struct tShoot 
{
    uint24_t ShootTime[MAXSHOOT]; //in 10mS unit
    uint8_t  ShootStringMark;     //The first ShootString is marked 1 , other are 0 
                                  //the order is from the one matked 1 up till last address 
                                  //then the next will be at starting address (cyclic)
    uint8_t TotShoots;            //Total shoots in current string 
} ShootString;
//ShootStringMark,TotShoots,ShootTime[0],ShootTime[1],ShootTime[n]...,ShootTime[TotShoots-1]

uint8_t  CurShoot;                //The current shoot of the displayed string
                
uint16_t CurShootString,          //Currently displayed string number 0..29
         CurrStringStartAddress; 
#define  Size_of_ShootString        302  // sizeof(ShootString)  did not work

typedef enum {
    Instant, Fixed, Random, Custom
} TdelTy;

typedef enum {
    Mic, AuxA, AuxB
} TdetTy;

TdelTy  DelayMode;
TdetTy  DetectMode;
uint16_t Threshold;
TBool   DetectAutoThreshold=True;
uint16_t DetectThreshold;
uint8_t DelayTime;
#define Delay_Address                118
#define DelayTime_Address            120

uint8_t Sensitivity=10;
#define Sensitivity_Address          122
TBool   AutoStart=False;
#define AutoStart_Address            124

uint8_t BackLightLevel=10;
#define BackLightLevel_Address       126
uint8_t Filter=1;
#define Filter_Address               128

#define ParAddress                   200
#define MAXPAR 10
uint8_t TotPar;
uint24_t ParTime[MAXPAR]; //in 10mS unit
uint8_t CurPar; //The par value active


#define ShootStringStartAddress     1000

#define MAXMenuItems   15
#define MAXItemLenght  18

typedef struct DispTy 
{
    uint8_t menu,pos,top,lineh,height,prev,page,PageSize;
    TBool refresh;
}SetMenuTy;
struct MenuTy 
{
    char MenuItem[MAXMenuItems][MAXItemLenght];
    uint8_t TotMenuItems;
    char MenuTitle[25];
}SettingsMenu;
SetMenuTy Menu;
uint8_t  battery_level;
// </editor-fold> 

#endif
