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
#include "eeprom.h"
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

union {
    uint8_t byte;

    struct {
        unsigned A_RELEASED     :1;
        unsigned B_RELEASED     :1;
        unsigned KEY_RELEASED   :1;
        unsigned TiltChanged    :1;
        unsigned orientation    :1;
    };
} InputFlags;

typedef enum {
    Mic = 0b0001,
    A = 0b0010,
    B = 0b0100
} ShotInput_t;

#define INPUT_TYPE_Microphone       0
#define INPUT_TYPE_A_or_B_multiple  1
#define INPUT_TYPE_A_and_B_single   2


typedef struct {
    uint8_t input;
    TBool detected;
} DetectionResult_t;

#define AutoStart Settings.AR_IS.Autostart
#define Autorotate Settings.AR_IS.AutoRotate
// Automatic shutdown after timeout 20 minutes
#define timer_idle_shutdown_timeout 1200000
// Dim light after 45 seconds
#define timer_idle_dim_timeout      45000
time_t timer_idle_last_action_time;

// This should be changed carefully.
// Saving to EEPROM strongly depends on these values
#define MAXSHOOTSTRINGS              (30)
#define MAXSHOOT                     (100)
#define Size_of_ShootString         (402)

typedef struct {
    union {
        uint8_t is_flags;

        struct {
            unsigned is_mic : 1;
            unsigned is_a : 1;
            unsigned is_b : 1;
            unsigned unused : 5;
        };
    };
    uint24_t dt;
} shot_t;

typedef union {
    uint8_t data[Size_of_ShootString];

    struct {
        uint8_t latest;
        uint8_t TotShoots; //Total shoots in current string

        shot_t shots[MAXSHOOT]; //in 1mS unit
    };
} ShootString_t;
ShootString_t ShootString;
time_t ShootString_start_time;

#define MAX_MEASUREMENT_TIME    999000
uint8_t CurShoot; //The current shoot of the displayed string
uint8_t CurShootString; //Currently displayed string number 0..29
uint16_t CurrStringStartAddress;

#define SHOTS_ON_REVIEW_SCREEN      3


#define DELAY_MODE_Instant 0
#define DELAY_MODE_Fixed 1
#define DELAY_MODE_Random 2
#define DELAY_MODE_Custom 3

#define DETECT_THRESHOLD_LEVELS 13
#define DEFAULT_SENSITIVITY     73
const uint8_t threshold_offsets[DETECT_THRESHOLD_LEVELS] = { 220, 190, 160, 150, 140, 125, 104, 73, 61, 51, 42, 33, 15};

uint16_t DetectThreshold;
time_t countdown_start_time;

#define MAXPAR 12
volatile uint8_t CurPar_idx = 0; //The par index
volatile TBool ParNowCounting = false;
time_t parStartTime_ms;

#define ParMode_Regular 0
#define ParMode_Practical 1
#define ParMode_Barricade 2
#define ParMode_FallingPlate 3
#define ParMode_NRA_PPC_A 4
#define ParMode_NRA_PPC_B 5
#define ParMode_NRA_PPC_C 6
#define ParMode_NRA_PPC_D 7
//uint8_t ParMode = ParMode_Regular;

#define ShootStringStartAddress     (0x0B80)
#define SettingsStartAddress        (0x0000)
#define SettingsDataSize            (104)
#define SettingsOffsetOfField(s,f)  (&(f)-&(s))
#define SettingAddress(s,f)         (SettingsStartAddress + SettingsOffsetOfField(s,f))

typedef union {
    uint8_t data[SettingsDataSize];

    struct {
        uint8_t version;
        AR_IS_T AR_IS;
        uint8_t DelayMode;
        uint8_t BuzzerLevel;
        uint8_t BackLightLevel;
        uint8_t Filter;
        uint8_t ParMode;
        uint8_t TotPar; // 1 based
        uint8_t InputType;
        uint24_t Sensitivity;
        uint16_t BuzzerFrequency;
        uint16_t BuzzerParDuration;
        uint16_t BuzzerStartDuration;
        time_t DelayTime; // mS
        time_t CustomCDtime; // sec TODO: Reduce storage type
        uint24_t ParTime[MAXPAR]; //in 1mS unit
    };
} Settings_t;
volatile Settings_t Settings;

#include "menu.h"
SettingsMenu_t ma; // Submenu for second level menu
SettingsMenu_t mx; // Submenu for third level menu
SettingsMenu_t SettingsMenu;

uint16_t battery_level;
// </editor-fold>

void DoSettings();
void StartListenShots();
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
void UpdateShot(time_t now, ShotInput_t input);
#define UpdateShootNow(x) {UpdateShot(rtc_time.unix_time_ms, x);}
void DoAdcGraph();
void DoDiagnostics();
void print_label_at_footer_grid(const char* msg, const uint8_t grid_x, const uint8_t grid_y);
void saveShootString(void);
#endif /*  _DAACED_H_ */