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

// <editor-fold defaultstate="collapsed" desc="Ports">

#define BuzzerPeriod          50            //[10uSec]  = 10/Freq
#define BuzzerCycles          300           //Cycles
#define DACdata               DAC1CON1

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Data Model">

typedef union {
    uint8_t AR_IS;

    struct {
        unsigned AutoRotate     : 2;
        unsigned Autostart      : 1;
        unsigned BT             : 1;
        unsigned Aux            : 1;
        unsigned AutoPowerOff   : 1;
        unsigned unused         : 2;
    };

} AR_IS_T;

union {
    uint8_t byte;

    struct {
        unsigned A_RELEASED     : 1;
        unsigned B_RELEASED     : 1;
        unsigned KEY_RELEASED   : 1;
        unsigned FOOTER_CHANGED : 1;
        unsigned ADC_DETECTED   : 1;
        unsigned INITIALIZED    : 1;
        unsigned UNUSED         : 2;
    };
} InputFlags;

uint16_t LongPressCount = 0;
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
// Manual orientation settings
#define Orientation Settings.AR_IS.AutoRotate
// Automatic orientation
#define OrientationSensor PORTAbits.RA0
// Automatic shutdown after timeout 20 minutes (in 2 sec intervals))
#define timer_idle_shutdown_timeout 600L
// Dim light after 46 seconds (in 2 sec intervals)
#define timer_idle_dim_timeout      22L
time_t timer_idle_last_action_time;

// This should be changed carefully.
// Saving to EEPROM strongly depends on these values

#include "shot.h"
ShootString_t ShootString;
time_t ShootString_start_time;

ShootString_t ReviewString;
#define MAX_MEASUREMENT_TIME    999000
uint8_t TopShotIndex; //The current shoot of the displayed string
uint8_t CurShootString; //Currently displayed string number 0..29
uint8_t ReviewTopShotDefault;
#define SHOTS_ON_REVIEW_SCREEN      3


#define DELAY_MODE_Instant 0
#define DELAY_MODE_Fixed 1
#define DELAY_MODE_Random 2
#define DELAY_MODE_Custom 3

#define DETECT_THRESHOLD_LEVELS 13
#define DEFAULT_SENSITIVITY     590
#define DEFAULT_SLOPE           200
const uint8_t threshold_offsets[DETECT_THRESHOLD_LEVELS] = {220, 190, 160, 150, 140, 125, 104, 73, 61, 51, 42, 33, 15};

uint16_t DetectThreshold;
int16_t DetectSlopeThreshold;
time_t countdown_start_time;

#define MAXPAR 20
volatile int8_t CurPar_idx = 0; //The par index
volatile TBool ParNowCounting = false;
time_t parStartTime_ms;

#define ParMode_Regular         0
#define ParMode_Silent          1
#define ParMode_Spy             2
#define ParMode_Repetitive      3
#define ParMode_CUSTOM          4
#define ParMode_Practical       5
#define ParMode_Barricade       6
#define ParMode_FallingPlate    7
#define ParMode_MovingTarget    8
#define ParMode_NRA_PPC_A       9
#define ParMode_NRA_PPC_B       10
#define ParMode_NRA_PPC_C       11
#define ParMode_NRA_PPC_D       12
#define ParMode_AutoPar         13
#define TOT_PAR_MODES           14
extern const char * par_mode_menu_names[];
extern const char * par_mode_header_names[];

#define ShootStringStartAddress     (0x0B80)
#define SettingsStartAddress        (0x0000)
// size of Settings fields + custom par + auto par
#define SettingsDataSize            (0x108 + 2 + 5)
#define SettingsOffsetOfField(s,f)  (&(f)-&(s))
#define SettingAddress(s,f)         (SettingsStartAddress + SettingsOffsetOfField(s,f))

typedef struct {
    uint24_t delay;
    uint24_t par;
} AutoPar_t;

typedef union {
    uint8_t data[SettingsDataSize];

    struct {
        uint8_t version; // 0
        AR_IS_T AR_IS;
        uint8_t DelayMode; // 1
        uint8_t Volume; // 2
        uint8_t BackLightLevel; // 3
        uint8_t Filter; // 4
        uint8_t ParMode; // 5
        uint8_t TotPar; // 1 based      // 6
        uint8_t TotParBak;
        uint8_t InputType; // 7
        uint16_t Sensitivity; // 8
        uint16_t Slope;
        uint16_t BuzzerFrequency; // 9
        uint16_t BuzzerParDuration; // A
        uint16_t BuzzerStartDuration; // B
        uint16_t RepetitiveFaceTime;
        uint16_t RepetitiveEdgeTime;
        uint8_t RepetitiveRepeat;
        time_t DelayTime; // mS         // C
        time_t CustomCDtime; // sec TODO: Reduce storage type
        float ParTime[MAXPAR]; //in decimal seconds unit
        uint24_t ParBackup[MAXPAR]; //in 1mS unit
        AutoPar_t AutoPar[MAXPAR];
    };
} Settings_t;
volatile Settings_t Settings;

#include "menu.h"
SettingsMenu_t ma; // Submenu for second level menu
SettingsMenu_t mx; // Submenu for third level menu
SettingsMenu_t SettingsMenu;

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
uint8_t top_shot_index();

#endif /*  _DAACED_H_ */