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

#include "DAACEDcommon.h"
#include "DAACEDfont.h"
#include "rtc.h"
#include "lcd.h"
#include "adc.h"
#include "charger.h"
#include "bluetooth.h"
#include "eeprom.h"
#include "random.h"
#include "ui.h"
#include "sinus.h"
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
#define KeySt                 0x01  //Start
#else
#define Key                  (PORTB & 0x3D)
#define KeySt                 0x80  //Start
#endif
#define KeyRw                 0x02  //Review
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
// <editor-fold defaultstate="collapsed" desc="Attenuator">
/*
  0dBm Q1A 0 Q1b 0
-10dBm Q1A 1 Q1b 0
-20dBm Q1A 0 Q1b 1
-30dBm Q1A 1 Q1b 1
 */
#define ATTENUATOR_00_DBm   0b00
#define ATTENUATOR_10_DBm   0b01
#define ATTENUATOR_20_DBm   0b10
#define ATTENUATOR_30_DBM   0b11
#define SetAttenuator(x)    { LATGbits.LATG0 = x & 1; LATGbits.LATG1 = (x & 2) >> 1; }
#define InitAttenuator()    { TRISG = 0b11111100; LATG = 0;}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Data Model">

typedef union {
    uint8_t AR_IS;

    struct {
        unsigned AutoRotate     : 1;
        unsigned BuzRef         : 1;
        unsigned Autostart      : 1;
        unsigned BT             : 1;
        unsigned MIC_SRC        : 1;
        unsigned AutoPowerOff   : 1;
        unsigned Clock24h       : 1;
        unsigned StartSound     : 1;
    };

} AR_IS_T;

union {
    uint8_t byte;

    struct {
        unsigned A_RELEASED     : 1;
        unsigned B_RELEASED     : 1;
        unsigned KEY_RELEASED   : 1;
        union{
            struct{
                unsigned FOOTER_CHANGED : 1;
                unsigned NEW_SHOT_D     : 1;
                unsigned NEW_SHOT_S     : 1;
            };
            unsigned NEW_SHOT : 3;
        };
        unsigned INITIALIZED    : 1;
        unsigned BEEP_GUARD     : 1;
    };
} InputFlags;

uint16_t LongPressCount = 0;
typedef enum {
    Mic = 0b0001,
    A = 0b0010,
    B = 0b0100
} ShotInput_t;

enum{
    INPUT_TYPE_Microphone       = 0,
    INPUT_TYPE_A_or_B_multiple,
    INPUT_TYPE_A_and_B_single,
    NUM_INPUT_TYPES
};

typedef struct {
    uint8_t input;
    TBool detected;
} DetectionResult_t;

#define AutoStart Settings.AR_IS.Autostart
// Manual orientation settings
#define Orientation Settings.AR_IS.AutoRotate
// Automatic orientation
#define OrientationSensor PORTAbits.RA0
// Automatic shutdown after timeout 20 minutes
#define timer_idle_shutdown_timeout 1200000L
// Dim light after 46 seconds 
#define timer_idle_dim_timeout      45000L
time_t timer_idle_last_action_time;

// This should be changed carefully.
// Saving to EEPROM strongly depends on these values

#include "shot.h"
ShootString_t ShootString;
time_t ShootString_start_time;
uint8_t last_sent_index; // last shot sent to BT 
ShootString_t ReviewString;
#define MAX_MEASUREMENT_TIME    999991
uint8_t TopShotIndex; //The current shoot of the displayed string
uint8_t CurShootString; //Currently displayed string number 0..29
uint8_t ReviewTopShotDefault;
#define SHOTS_ON_REVIEW_SCREEN      3

enum {
    DELAY_MODE_Instant = 0,
    DELAY_MODE_Fixed,
    DELAY_MODE_Random,
    DELAY_MODE_Custom,
    DELAY_MODE_Other,
    NUM_DELAY_MODES
};

enum {
    SENS_MAX = 0,
    SENS_HIGH,
    SENS_MED_HIGH,
    SENS_MED,
    SENS_MED_LOW,
    SENS_LOW,
    SENS_MIN,
    NUM_SENS
};

#define DEFAULT_SENSITIVITY     4

typedef struct {
    uint8_t att;
    uint8_t thr;
} detection_setting_t;

enum {
    PRESET_OUTDOOR = 0,
    PRESET_INDOOR,
    PRESET_AIRSOFT,
    PRESETS_NUM
};

const detection_setting_t detection_presets[PRESETS_NUM][NUM_SENS] = {
    { 
        {1, 15}, 
        {1, 30}, 
        {1, 40}, 
        {1, 50}, 
        {2, 35}, 
        {2, 40}, 
        {2, 50}
     }, // OUTDOOR
    {
        {2, 30},
        {3, 30},
        {2, 50},
        {3, 40},
        {3, 50},
        {3, 70},
        {3, 100}
     }, // INDOOR
    {
        {0, 5}, 
        {0, 10}, 
        {0, 15}, 
        {0, 20}, 
        {0, 30}, 
        {0, 40}, 
        {0, 50}
     },  // AIRSOFT
};

time_t countdown_start_time;

// Should not be more than we can display in menu items
#define MAXPAR 14
volatile int8_t CurPar_idx = 0; //The par index
volatile struct{
    unsigned ParNowCounting     : 1;
    unsigned AutoParOverDetect  : 1;
    unsigned UNUSED             : 6;
} ParFlags;
#define AUTO_PAR_OVER_DETECT_MS         500
#define AUTO_PAR_LAST_DETECTION_TIME    999000;
time_t parStartTime_ms;

#define ParMode_Regular         0
#define ParMode_Spy             1
#define ParMode_Repetitive      2
#define ParMode_AutoPar         3
//#define ParMode_Practical       4
//#define ParMode_Barricade       5
//#define ParMode_FallingPlate    6
//#define ParMode_MovingTarget    7
//#define ParMode_NRA_PPC_A       8
//#define ParMode_NRA_PPC_B       9
//#define ParMode_NRA_PPC_C       10
//#define ParMode_NRA_PPC_D       11
#define ParMode_CUSTOM          4

#define TOT_PAR_MODES           5
extern const char * par_mode_menu_names[];
extern const char * par_mode_header_names[];

#define SettingsStartAddress        (0x0000)
// 270 bytes
#define SettingsDataSize            (sizeof(Settings_t))
#define SettingsOffsetOfField(s,f)  (&(f)-&(s))
#define SettingAddress(s,f)         (SettingsStartAddress + SettingsOffsetOfField(s,f))

typedef struct {
    float delay;
    float par;
} AutoPar_t;

typedef struct {
    uint8_t version; // 0
    AR_IS_T AR_IS;
    uint8_t DelayMode; // 1
    uint8_t Volume; // 2
    uint8_t BackLightLevel; // 3
    uint8_t Filter; // 4
    uint8_t ParMode; // 5
    uint8_t TotPar; // 1 based      // 6
    uint8_t TotCustomPar;
    uint8_t TotAutoPar;
    uint8_t InputType; // 7
    uint8_t RangeType;
    uint8_t Sensitivity_idx[PRESETS_NUM]; // 8
    uint16_t ContrastValue;
    uint16_t BuzzerFrequency; // 9
    uint16_t BuzzerParDuration; // A
    uint16_t BuzzerStartDuration; // B
    uint16_t RepetitiveFaceTime;
    uint16_t RepetitiveEdgeTime;
    uint8_t RepetitiveRepeat;
    time_t CUstomDelayTime; // mS         // C
    time_t CustomCDtime; // sec TODO: Reduce storage type
    float ParTime[MAXPAR]; //in decimal seconds unit
    float CustomPar[MAXPAR]; //in 1mS unit
    AutoPar_t AutoPar[MAXPAR];
} Settings_t;
volatile Settings_t Settings;


uint24_t runtimeDelayTime = 2500;
uint8_t repetitive_counter = 0;
time_t next_par_ms;
enum {Face = 0, Edge = 1} repetitive_state;

#include "menu.h"
SettingsMenu_t ma; // Submenu for second level menu
SettingsMenu_t mx; // Submenu for third level menu
SettingsMenu_t SettingsMenu;
#define SETTINGS_INDEX_DELAY        0
#define SETTINGS_INDEX_PAR          1
#define SETTINGS_INDEX_BUZZER       2
#define SETTINGS_INDEX_MIC          3
#define SETTINGS_INDEX_MODE         4
#define SETTINGS_INDEX_DISPLAY      5
#define SETTINDS_INDEX_COUNTDOWN    6
#define SETTINDS_INDEX_AUTOSTART    7
#define SETTINDS_INDEX_CLOCK        8
#define SETTINDS_INDEX_INPUT        9
#define SETTINDS_INDEX_BLUETOOTH    10
#define SETTINDS_INDEX_AUTO_POWER   11
#define SETTINDS_INDEX_CLEAR        12
#define SETTINDS_INDEX_RESET        13
#define SETTINDS_INDEX_VERSION      14

#define ROM_GUARD_INTERVAL  32
#define StatsStartAddress   (SettingsStartAddress + sizeof(Settings_t) + ROM_GUARD_INTERVAL)
typedef struct{
    uint32_t PowerOn;
    uint32_t Signal;
    uint32_t Review;
    uint16_t Settings;
    uint16_t Charging;
    uint16_t Charged;
    uint16_t LowPower;
    uint16_t InputModes[3];
    uint16_t Modes[TOT_PAR_MODES];
    uint16_t Menu[MAXMenuItems];
} Stats_t;
#define StatsDataSize   (sizeof(Stats_t))
Stats_t Stats;

#define ShootStringStartAddress     (StatsStartAddress + StatsDataSize + ROM_GUARD_INTERVAL)

// </editor-fold>

void DoSettings();
void StartListenShots();
void DoReview();
void DoPowerOff();
void DoPowerOn();
void DoCharging();
void update_shot_time_on_screen();
void StartPlayParSound();
void StartParTimer();
void StartPlayStartSound();
void StartCountdownTimer();
uint8_t print_header(TBool hide_time);
void print_footer();
uint8_t print_title(TBool hide_time);
void handle_rotation();
void DoAdcGraph();
void DoDiagnostics();
void print_label_at_footer_grid(const char* msg, const uint8_t grid_x, const uint8_t grid_y);
void saveShootString(void);
void getShootString(uint8_t offset);
void save_shots_if_required();
void check_countdown_expired();
void check_par_expired();
void check_timer_max_time();
void DetectMicShot();
void handle_bt_commands();
void set_par_mode(int m);
void increment_par();
void decrement_par();
uint8_t top_shot_index();
void SetCountDown();
void saveStatsField(void * f, size_t l);
void saveStats();
void PowerOffSound();

#endif /*  _DAACED_H_ */