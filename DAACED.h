/* ===========================================================================
    Project : DAACED
    Version 1.0

 *  File:   DAACED.h
 *  Author: Eli
 *  Created on Sept 15, 2017

    Global R&D ltd. 04_9592201    ;  www.global_rd.com
    Eli Jacob 054_8010330         ;  eli@global_rd.com
   ===========================================================================*/
#define ASYNC_DETECT
// <editor-fold defaultstate="collapsed" desc="Includes">
#ifndef _DAACED_H_
#define _DAACED_H_

#pragma warning disable 373
#pragma warning disable 520

#include <xc.h>
#include "DAACEDcommon.h"
#include "DAACEDfont.h"
#include "rtc.h"
#include "lcd.h"
#include "ui.h"
#include "adc.h"
#include "charger.h"
#include "bluetooth.h"
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

// <editor-fold defaultstate="collapsed" desc="Wakeup and Sleep button.">
#define SLEEP_BUTTON_TRIS       (TRISEbits.TRISE1)

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Aux.">
#define AUX_detect          (PORTDbits.RD0)
#define AUX_A       (PORTDbits.RD1)
#define AUX_B       (PORTDbits.RD2)

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disp&Keys">
#define Bot                  LCD_HEIGHT-30
//#define SWAP_KEYS
#ifndef SWAP_KEYS
#define Key                  (PORTB & 0x3F)
#define KeySt                 0x02  //Start
#else
#define Key                  (PORTB & 0x3D)
#define KeySt                 0x80  //Start
#endif
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

#define BuzzerPeriod          50            //[10uSec]  = 10/Freq
#define BuzzerCycles          300           //Cycles
#define DACdata               DAC1CON1

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Data Model">

typedef union {
    uint8_t AR_IS;

    struct {
        unsigned AutoRotate : 1;
        unsigned Mic : 1;
        unsigned A_or_B_multiple : 1;
        unsigned A_and_B_single : 1;
        unsigned Autostart : 1;
        unsigned BT : 1;
        unsigned Aux : 1;
    };

} AR_IS_T;
AR_IS_T AR_IS;

union {
    uint8_t byte;

    struct {
        unsigned A_RELEASED : 1;
        unsigned B_RELEASED : 1;
        unsigned KEY_RELEASED : 1;
    };
} InputFlags;

typedef enum {
    Mic = 0,
    A = 1,
    B = 2
} ShotInput_t;

typedef enum {
    Microphone = 0,
    A_or_B_multiple = 1,
    A_and_B_single = 2
} InputMode_t;
InputMode_t InputType = Microphone;

typedef struct {
    InputMode_t input;
    TBool detected;
} DetectionResult_t;

#define AutoStart AR_IS.Autostart
#define Autorotate AR_IS.AutoRotate
#define AR_IS_Address                100
uint16_t BuzzerFrequency = 2000;
uint16_t BuzzerParDuration = 200;
uint16_t BuzzerStartDuration = 200;
uint8_t BuzzerLevel = 2;
// Automatic shutdown after timeout 20 minutes
time_t timer_idle_shutdown_timeout = 1200000;
time_t timer_idle_last_action_time;
#define BuzzerStartDuration_Address  102
#define BuzzerFrequency_Address      104
#define BuzzerParDuration_Address    106
#define BuzzerLevel_Address          108
time_t CustomCDtime = 5 * 60 * 1000; // 5 min in ms
// TODO: Review storage size for all the data
#define CustomCDtime_Address         110


TBool SaveToEEPROM;
#define BT_Address                   112

#define MAXSHOOTSTRINGS    30
#define MAXSHOOT    100

struct tShoot {
    time_t ShootTime[MAXSHOOT]; //in 1mS unit
    uint8_t ShootStringMark; //The first ShootString is marked 1 , other are 0
    //the order is from the one matked 1 up till last address
    //then the next will be at starting address (cyclic)
    uint8_t TotShoots; //Total shoots in current string
} ShootString;

//ShootStringMark,TotShoots,ShootTime[0],ShootTime[1],ShootTime[n]...,ShootTime[TotShoots-1]
time_t measurement_start_time_msec; // Reference time for counting shppt.
// Should be set to RTC before beep starts
#define MAX_MEASUREMENT_TIME    999000
uint8_t CurShoot; //The current shoot of the displayed string
uint16_t CurShootString, //Currently displayed string number 0..29
CurrStringStartAddress;
#define  Size_of_ShootString        336  // sizeof(ShootString)  did not work
#define SHOTS_ON_REVIEW_SCREEN      3

typedef enum {
    Instant, Fixed, Random, Custom
} TdelTy;

TdelTy DelayMode = Fixed;
uint16_t DetectThreshold;
time_t DelayTime = 3000; // mS
time_t countdown_start_time;
#define Delay_Address                118
#define DelayTime_Address            120

uint8_t Sensitivity;
#define Sensitivity_Address          122

#define AutoStart_Address            124

uint8_t BackLightLevel = 10;
#define BackLightLevel_Address       126
uint8_t Filter;
#define Filter_Address               128

#define ParAddress                   200
#define MAXPAR 12
uint8_t TotPar = 0; // 1 based
uint24_t ParTime[MAXPAR]; //in 1mS unit
volatile uint8_t CurPar_idx = 0; //The par index
volatile TBool ParNowCounting = false;
time_t parStartTime_ms;

typedef enum {
    Regular = 0,
    Practical = 1,
    Barricade = 2,
    FallingPlate = 3,
    NRA_PPC_A = 4,
    NRA_PPC_B = 5,
    NRA_PPC_C = 6,
    NRA_PPC_D = 7
} ParMode_t;
ParMode_t ParMode = Regular;
#define ShootStringStartAddress     1000

#include "menu.h"
SettingsMenu_t ma; // Submenu for second level menu
SettingsMenu_t mx; // Submenu for third level menu
SettingsMenu_t SettingsMenu;

uint16_t battery_level;
// </editor-fold>

void DoSettings();
void DoMain();
void DoReview();
void DoPowerOff();
void DoPowerOn();
void DoCharging();
void update_shot_time_on_screen();
void PlayParSound();
void StartParTimer();
void PlayStartSound();
void StartCountdownTimer();
uint8_t print_header();
void print_footer();
uint8_t print_time();
void handle_rotation();
void UpdateShootNow(ShotInput_t input);
void DoAdcGraph();
void DoDiagnostics();
void print_label_at_footer_grid(const char* msg, const uint8_t grid_x, const uint8_t grid_y);
void saveShootString(void);
#endif /*  _DAACED_H_ */