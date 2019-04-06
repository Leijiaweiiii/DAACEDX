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
#include "ui.h"
#include "uart.h"
#include "random.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="PIC_init">

void PIC_init(void) {
    // 0 = OUTPUT, 1 = INPUT
    // 0 = DIGITAL, 1 = ANALOG
    OSCFRQ = 0b00001000; // 64 MHz Fosc.
    // fix settings of RTC oscillator
    OSCENbits.SOSCEN = 1;
    OSCENbits.EXTOEN = 0;
    OSCENbits.LFOEN = 0;
    TRISA = 0b11111111;
    ANSELA = 0b00001110; // ADC inputs 1..3
    OSCENbits.ADOEN = 1; // Enable ADC oscillator;

    TRISB = 0b11111111; //
    ANSELB = 0b00000000;

    TRISC = 0b11100101; // C6 = TX, C7 RX
    // C3 = DP_SCL(OP), C4 = DP_SDA(OP)

    TRISD = 0b00110000; // EEPROM SPI SDI=5 SCK=6 SDO=7
    ANSELD = 0b00000000;

    TRISE = 0b10111010; // E0 = POWER(OP), E6 = BL_EN(OP)
    ANSELE = 0b00000000;

    TRISF = 0b00000000; // F3 = DP_CS(OP), F4 = DP_A0(OP), F6 = DP_RST(OP)
    ANSELF = 0b00100000; // DAC
    DAC1CON0 = 0b10100000; // DAC enabled output on pin13 (RF5) with reference from VDD & VSS

    TRISG = 0x00; // CON8, Debug Header.
    ANSELG = 0x00;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Backlight PWM Function">

void initialize_backlight() {
    uint8_t PRvalue, prescalar;
    TRISEbits.TRISE6 = 0; // Disable output.
    PWM6CON = 0; // Clear register.
    find_optimal_PWM_settings(1000, &PRvalue, &prescalar);

    // Initialize Timer2.
    PIR5bits.TMR2IF = 0; // Clear timer interrupt flag.
    T2CLKCON = 0b001; // Timer2 clock source = Fosc/4;
    T2CONbits.CKPS = find_set_bit_position(prescalar);
    PR2 = PRvalue;
}

void set_backlight(uint8_t level) {
    uint8_t duty_cycle = (level == 0) ? 0 : level * 10 - 1;
    PWM6CONbits.EN = 0;
    if (duty_cycle == 0) {
        LATEbits.LATE6 = 1;
    } else if (duty_cycle > 0 && duty_cycle < 100) {
        uint16_t ON_value;
        uint8_t PR_value = PR2;
        ON_value = (duty_cycle * (PR_value + 1)) / 25;
        PWM6DCH = (ON_value >> 2);
        PWM6DCL = ((ON_value & 0x0003) << 6);
        T2CONbits.ON = 1; // Start timer.
        PWM6CONbits.EN = 1; // Enable PWM.
        LATEbits.LATE6 = 0;
        RE6PPS = 0x0A; // PPS setting.
        PWM6CONbits.POL = 1; // Polarity.
    } else if (duty_cycle == 100) {
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

        if (freqdiffrence > ABS(tempactualfreq - freq)) {
            *selectedPRvalue = tempPRvalue;
            *selectedPrescalar = prescaler;
            freqdiffrence = ABS(tempactualfreq - freq);
        }
    }

    tempactualfreq = ((_XTAL_FREQ / (4 * (*selectedPrescalar))) / ((*selectedPRvalue) + 1));
    int32_t lowestfreq = ((_XTAL_FREQ / (4 * (*selectedPrescalar))) / ((*selectedPRvalue) + 2));

    if (ABS(lowestfreq - freq) < ABS(tempactualfreq - freq)) {
        (*selectedPRvalue)++;
    }
    tempactualfreq = ((_XTAL_FREQ / (4 * (*selectedPrescalar))) / ((*selectedPRvalue) + 1));
    return 0;
}

#define is_power_of2(n) (n && (!(n & (n - 1))))

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

// <editor-fold defaultstate="collapsed" desc="Sinus Generator">

void generate_sinus(uint8_t amplitude, uint16_t frequency, int16_t duration) {
    uint8_t findex = frequency/100;
    // Don't beep ever in silent modes
    if (Settings.ParMode == ParMode_Silent) return;
    if (Settings.ParMode == ParMode_Spy) return;
    if (amplitude == 0) return;
    amplitude_index = amplitude - 1;
    sinus_dac_init();
    sinus_duration_timer_init(duration);
    sinus_value_timer_init(findex);
    beep_start = rtc_time.unix_time_ms;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Save and retrive DATA">

void clearHistory() {
    lcd_clear();
    lcd_write_string("Please wait", UI_CHARGING_LBL_X - 20, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
    eeprom_clear_block_bulk(ShootStringStartAddress, EEPROM_MAX_SIZE - ShootStringStartAddress);
}

void saveSettings() {
    eeprom_write_array_bulk(SettingsStartAddress, Settings.data, SettingsDataSize);
}

void getSettings() {
    eeprom_read_array(SettingsStartAddress, Settings.data, SettingsDataSize);
}

void getDefaultSettings() {
    Settings.version = FW_VERSION;
    Settings.Sensitivity = DEFAULT_SENSITIVITY;
    Settings.Slope = DEFAULT_SLOPE;
    Settings.Filter = 80; // ms
    Settings.AR_IS.Autostart = 1; // on
    Settings.AR_IS.AutoRotate = 0; // Off
    Settings.AR_IS.BT = 0; // Off by default
    Settings.AR_IS.AutoPowerOff = 1; // ON by default
    Settings.InputType = INPUT_TYPE_Microphone;
    Settings.BuzzerFrequency = 2000; // Hz
    Settings.BuzzerParDuration = 300; // ms
    Settings.BuzzerStartDuration = 500; // ms
    Settings.Volume = 2; // Middle strength sound.
    Settings.CustomCDtime = 240; // 4 minutes in sec
    Settings.DelayMode = DELAY_MODE_Fixed;
    Settings.DelayTime = 3000; // ms before start signal
    Settings.BackLightLevel = 1; // Most dimmed visible
    Settings.TotPar = 0; // Par Off
    Settings.ParMode = ParMode_Regular;
    CurPar_idx = MAXPAR;
    while (CurPar_idx > 0) {
        Settings.ParTime[CurPar_idx--] = 0.0;
    }
    // TODO: Define proper defaults
    Settings.RepetitiveEdgeTime = 5000;
    Settings.RepetitiveFaceTime = 4000;
    Settings.RepetitiveRepeat = 5;
    repetitive_time = Settings.RepetitiveFaceTime;
    repetitive_state = Face;
    repetitive_counter = 0;
}
void restoreSettingsField(Settings_t * s, void * f, size_t l){
    int offset = f - s;
    eeprom_read_array(SettingsStartAddress + offset, f, l);
}
void saveSettingsField(Settings_t * s, void * f, size_t l) {
    int offset = f - s;
    eeprom_write_array_bulk(SettingsStartAddress + offset, f, l);
}

void savePar(uint8_t par_index) {
    int offset = &(Settings.ParTime) - (&Settings) + par_index;
    eeprom_write_array_bulk(SettingsStartAddress + offset, Settings.data + offset, sizeof(uint24_t));
}

void restorePar() {
    int offset = &(Settings.ParTime) - (&Settings);
    eeprom_read_array(SettingsStartAddress + offset, Settings.ParTime, MAXPAR);
    offset = (&(Settings.TotPar))-(&Settings);
    Settings.TotPar = eeprom_read_data(SettingsStartAddress + offset);
}

uint16_t findStringAddress(uint8_t index_in_eeprom) {
    return ShootStringStartAddress + index_in_eeprom*Size_of_ShootString;
}

uint8_t findCurStringIndex() {
    uint16_t addr;
    uint8_t counts[MAXSHOOTSTRINGS];
    uint8_t labels[MAXSHOOTSTRINGS];
    for (uint8_t i = MAXSHOOTSTRINGS; i > 0; i--) {
        addr = findStringAddress(i - 1);
        labels[i - 1] = eeprom_read_data(addr);
        counts[i - 1] = eeprom_read_data(addr + 1);
    }
    for (uint8_t i = 0; i < MAXSHOOTSTRINGS; i++) {
        if (labels[i] == 1) {
            return i;
        }
    }
    return 0;
}

// increments the string position, based on the current mark

void saveShootString(void) {
    uint8_t index;
    uint16_t addr;
    // Don't save empty strings
    if (ShootString.TotShoots == 0)
        return;
    index = findCurStringIndex();
    addr = findStringAddress(index);
    eeprom_write_data(addr, 0);
    index++;
    if (index >= MAXSHOOTSTRINGS)
        index = 0;
    ShootString.latest = 1;
    addr = findStringAddress(index);
    eeprom_write_array_bulk(addr, ShootString.data, Size_of_ShootString);
    eeprom_write_data(addr, 1);
}

void saveOneShot(uint8_t shot_number) {
    uint8_t index;
    uint16_t addr;
//    shot_t test0;
    index = findCurStringIndex();
    addr = findStringAddress(index);
    if (shot_number == 0) {
        eeprom_write_data(addr, 0); // Mark old sot not last
        index++;
        if (index >= MAXSHOOTSTRINGS)
            index = 0;
        addr = findStringAddress(index);
        eeprom_write_data(addr, 1); // Matk this shot last
    }
    addr++;
    eeprom_write_data(addr, shot_number + 1);
    addr++;
    addr += shot_number * SIZE_OF_SHOT_T;
    eeprom_write_array_bulk(addr, &(ShootString.shots[shot_number]), SIZE_OF_SHOT_T);
}

void send_all_shots() {
    for (uint8_t shot = 0; shot < ShootString.TotShoots; shot++) {
        sendOneShot(shot, &(ShootString.shots[shot]));
    }
}

void getShootString(uint8_t offset) {
    uint16_t addr;
    int8_t index;
    index = findCurStringIndex();
    if (index >= offset)
        index -= offset;
    else
        index = MAXSHOOTSTRINGS - offset + index;
    addr = findStringAddress(index);
    eeprom_read_array(addr, ShootString.data, Size_of_ShootString);    
    ReviewString.shots[0] = ShootString.shots[0];
    ReviewString.TotShoots = ShootString.TotShoots;

    if(ReviewString.TotShoots < MAX_SAVED_SHOTS){
        ReviewTopShotDefault = ReviewString.TotShoots - 1;
        for(uint8_t i = 1; i < ReviewString.TotShoots; i++){
            ReviewString.shots[i] = ShootString.shots[i];
        }
    } else {
        ReviewTopShotDefault = MAX_SAVED_SHOTS - 1;
        uint8_t totShots = ReviewString.TotShoots;
        for(uint8_t i = 1; i < MAX_SAVED_SHOTS; i++){
            index = totShots - ShootString.shots[i].sn;
            index = MAX_SAVED_SHOTS - index;
            index--;
            ReviewString.shots[index] = ShootString.shots[i];
        }
    }
}

TBool checkShotStringEmpty(uint8_t offset) {
    uint16_t addr;
    int8_t index;
    uint8_t t;
    index = findCurStringIndex();
    if (index >= offset)
        index -= offset;
    else
        index = MAXSHOOTSTRINGS - offset + index;
    addr = findStringAddress(index) + 1; // offset of TotShots
    t = eeprom_read_data(addr);
    t = eeprom_read_data(addr);
    return (t == 0);
}
// </editor-fold>

void print_delay(char * str, const char * prefix){
    switch (Settings.DelayMode) {
        case DELAY_MODE_Instant: sprintf(str, "%sINST ", prefix);
            break;
        case DELAY_MODE_Fixed: sprintf(str, "%s3.0 ", prefix);
            break;
        case DELAY_MODE_Random: sprintf(str, "%sRND ", prefix);
            break;
        case DELAY_MODE_Custom: sprintf(str, "%s%1.1f ", prefix, (float) (Settings.DelayTime) / 1000);
            break;
    }
}

uint8_t top_shot_index(){
    if ( ShootString.TotShoots < MAX_REGISTERED_SHOTS) 
        return ShootString.TotShoots - 1;
    return ShootString.TotShoots;
}

uint8_t get_shot_index_in_arr(uint8_t x){
    while (x > MAX_SAVED_SHOTS - 1){
        x++;
        x -= MAX_SAVED_SHOTS;
    }
    return x;
}

uint8_t get_rev_shot_index_in_arr(uint8_t x, uint8_t totShots){
    if(totShots < MAX_SAVED_SHOTS){
        if(x < totShots) return x;
        return x - totShots;
    }
    switch(x){
        case 0:
        case MAX_SAVED_SHOTS:
            return 0;
        case MAX_SAVED_SHOTS + 1:
            return 1;
    }
    
    return get_shot_index_in_arr(x);
}

// <editor-fold defaultstate="collapsed" desc="Settings">
// <editor-fold defaultstate="collapsed" desc="Delay">

void SetCustomDelay() {
    NumberSelection_t n;
    strcpy(n.MenuTitle, "Custom Delay");
    InitSettingsNumberDefaults((&n));
    n.fmin = 0.1;
    n.fmax = 10.0;
    n.fstep = 0.1;
    n.fvalue = (float) Settings.DelayTime / 1000;
    n.fold_value = n.fvalue;
    n.format = " %2.1fs ";
    lcd_clear();
    do {
        DisplayDouble(&n);
        SelectDouble(&n);
    } while (SettingsNotDone((&n)));
    Settings.DelayTime = (time_t) (n.fvalue * 1000);
    if (n.fold_value != n.fvalue) {
        saveSettingsField(&Settings, &(Settings.DelayTime), 4);
    }
}

void SetDelay() {
    uint8_t oldValue = Settings.DelayMode;
    InitSettingsMenuDefaults((&ma));
    strcpy(ma.MenuTitle, "Delay");
    strcpy(ma.MenuItem[DELAY_MODE_Instant], " Instant ");
    strcpy(ma.MenuItem[DELAY_MODE_Fixed], " Fixed 3.0 sec. ");
    strcpy(ma.MenuItem[DELAY_MODE_Random], " Random");
    sprintf(ma.MenuItem[DELAY_MODE_Custom], " Custom %3.1f sec. ",((double)Settings.DelayTime)/1000);
    ma.TotalMenuItems = 4;
    ma.menu = Settings.DelayMode;

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
        if (ma.done && ma.selected && ma.menu == DELAY_MODE_Custom) {
            SetCustomDelay();
            lcd_clear();
            sprintf(ma.MenuItem[DELAY_MODE_Custom], " Custom %3.1f sec. ",((double)Settings.DelayTime)/1000);
            ma.done = False;
            Settings.DelayMode = ma.menu;
        }
    } while (SettingsNotDone((&ma)));
    if (ma.selected) {
        Settings.DelayMode = ma.menu;
        if (Settings.DelayMode != oldValue) {
            saveSettingsField(&Settings, &(Settings.DelayMode), 1);
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Par">

TBool EditPar(uint8_t par_index) {
    NumberSelection_t b;
    b.fmin = 0.1;
    b.fmax = 99.9;
    b.fvalue = Settings.ParTime[par_index];
    b.fold_value = b.fvalue;
    sprintf(b.MenuTitle, "Par %d Settings ", par_index);
    b.fstep = 0.01;
    b.format = "%3.2fs";
    b.done = False;
    lcd_clear();
    do {
        DisplayDouble(&b);
        SelectDouble(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.ParTime[par_index] = b.fvalue;
        savePar(par_index);
    }
    return b.selected;
}

void FillParSettings(SettingsMenu_t * m) {
    uint8_t i = 0;
    for (i = 0; i < Settings.TotPar; i++) {
        sprintf(m->MenuItem[i], "Par %d: %3.2fs  ", i + 1, Settings.ParTime[i]);
    }
    if (i < MAXPAR)
        strcpy(m->MenuItem[i], "Add ");
    else
        strcpy(m->MenuItem[i], "Max Par reached ");
    strcpy(m->MenuItem[++i], "Delete Last ");
    strcpy(m->MenuItem[++i], "Delete All ");
    m->TotalMenuItems = i + 1;
}

void clear_par() {
    while (0 < Settings.TotPar) {
        Settings.ParTime[Settings.TotPar--] = 0.0;
    }
}

void HandleParMenuSelection(SettingsMenu_t * m) {
    if (m->selected) {
        m->selected = False;
        if (m->menu < (m->TotalMenuItems - 3)) {
            EditPar(m->menu);
        } else if (m->menu == (m->TotalMenuItems - 3)) {
            // Add new par
            if (m->menu < MAXPAR) {
                TBool res = False;

                Settings.ParTime[Settings.TotPar] = 1.0; // Default setting 1 second
                res = EditPar(Settings.TotPar);
                if (res) { // Roll back if not selected
                    Settings.TotPar++;
                    m->menu++;
                } else {
                    Settings.ParTime[Settings.TotPar] = 0.0;
                }
            }
        } else if (m->menu == (m->TotalMenuItems - 2)) {
            // Delete last PAR
            if (Settings.TotPar > 0) {
                Settings.ParTime[Settings.TotPar--] = 0.0;
                m->menu--;
            }
        } else if (m->menu == (m->TotalMenuItems - 1)) {
            // Clear PAR
            clear_par();
            m->menu = 0;
            m->page = 0;
        }
        lcd_clear();
    }
}

void SetPar(SettingsMenu_t * m) {
    uint8_t oldTotPar = Settings.TotPar;
    InitSettingsMenuDefaults(m);
    strcpy(m->MenuTitle, "Par Settings ");
    do {
        FillParSettings(m);
        DisplaySettings(m);
        SelectBinaryMenuItem(m);
        HandleParMenuSelection(m);
    } while (SettingsNotDone(m));
    if (Settings.TotPar != oldTotPar) {
        saveSettingsField(&Settings, &(Settings.TotPar), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Backlight">

void SetBacklight() {//PWM Backlight
    NumberSelection_t b;
    uint8_t tmpVal = Settings.BackLightLevel;
    strcpy(b.MenuTitle, "Backlight ");
    b.max = 9;
    b.min = 0;
    b.step = 1;
    b.value = tmpVal;
    b.old_value = b.value;
    b.format = "%u";
    b.done = False;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
        if(b.value - Settings.BackLightLevel){
            set_backlight(b.value);
            Settings.BackLightLevel = b.value;
        }
    } while (SettingsNotDone((&b)));

    if (b.selected && b.value != b.old_value) {
        Settings.BackLightLevel = b.value;
        saveSettingsField(&Settings, &(Settings.BackLightLevel), 1);
    } else {
        Settings.BackLightLevel = b.old_value;
    }
    set_backlight(Settings.BackLightLevel);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Buzzer">
void SetPre() {
    uint24_t tmp;
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b))
    b.min = 0;
    b.max = 5;
    b.value = 2;
    b.old_value = b.value;
    tmp = b.value;
    strcpy(b.MenuTitle, "Set Prescaler");
    b.step = 1;
    b.format = "%d";
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
        if (b.value != tmp) {
            generate_sinus(1, 0, 1000);
            tmp = b.value;
            sinus_s[0].PRE = tmp;
        }
    } while (SettingsNotDone((&b)));
}

void SetPos() {
    uint24_t tmp;
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b))
    b.min = 1;
    b.max = 16;
    b.value = 5;
    b.old_value = b.value;
    tmp = b.value;
    strcpy(b.MenuTitle, "Set Postscale");
    b.step = 1;
    b.format = "%d";
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
        if (b.value != tmp) {
            generate_sinus(1, 0, 1000);
            tmp = b.value;
            sinus_s[0].POS = tmp;
        }
    } while (SettingsNotDone((&b)));
}

void SetPR() {
    uint24_t tmp;
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b))
    b.min = 1;
    b.max = 255;
    b.value = 150;
    b.old_value = b.value;
    tmp = b.value;
    strcpy(b.MenuTitle, "Set PR");
    b.step = 1;
    b.format = "%d";
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
        if (b.value != tmp) {
            generate_sinus(1, 0, 1000);
            tmp = b.value;
            sinus_s[0].PR = tmp;
        }
    } while (SettingsNotDone((&b)));
}

void SetBeepFreq() {
    uint24_t tmp;
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b))
    b.min = 800;
    b.max = 3000;
    b.value = Settings.BuzzerFrequency;
    b.old_value = b.value;
    tmp = b.value;
    strcpy(b.MenuTitle, "Set Frequency");
    b.step = 100;
    b.format = "%dHz";
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
        if (b.value != tmp) {
            generate_sinus(1, b.value, 50);
            tmp = b.value;
        }
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.BuzzerFrequency = b.value;
        if (b.fvalue != b.fold_value) {
            saveSettingsField(&Settings, &(Settings.BuzzerFrequency), 2);
        }
    }
}

void SetVolume() {
    uint8_t tmp;
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b));
    if (Settings.Volume > 3) Settings.Volume = 3;
    strcpy(b.MenuTitle, "Set Volume");
    b.min = 0;
    b.max = 3;
    b.step = 1;
    b.format = " %2d ";
    b.value = Settings.Volume;
    b.old_value = b.value;
    tmp = b.value;
    do {
        DisplayInteger(&b);
        SelectIntegerCircular(&b);
        if (b.value != tmp) {
            generate_sinus(b.value, Settings.BuzzerFrequency, 50);
            tmp = b.value;
        }
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.Volume = b.value;
        if (b.value != b.old_value) {
            saveSettingsField(&Settings, &(Settings.Volume), 1);
        }
    }
}
#define SEC_FIELD_DISPLAY_FORMAT    " %1.2fs "
#define SEC_FIELD_DISPLAY_FORMAT_SHORT    " %1.1fs "
void SetBeepTime(TBool Par) {
    NumberSelection_t d;
    // Space optimization
    float fv = (float) (Settings.BuzzerParDuration) / 1000;
    InitSettingsNumberDefaults((&d));
    if (Par) {
        d.fvalue = fv;
        strcpy(d.MenuTitle, "Par Duration ");
    } else {
        d.fvalue = fv;
        strcpy(d.MenuTitle, "Start Duration ");
    }
    d.fmin = 0.050;
    d.fmax = 1.0;
    d.fstep = 0.050;
    d.fold_value = d.fvalue;
    d.done = False;
    d.format = SEC_FIELD_DISPLAY_FORMAT;
    do {
        DisplayDouble(&d);
        SelectDouble(&d);
    } while (SettingsNotDone((&d)));

    if (d.selected) {
        if (d.fvalue != d.fold_value) {
            // Code space optimization
            uint16_t duration = (int) (d.fvalue * 1000);
            if (Par) {
                Settings.BuzzerParDuration = duration;
                saveSettingsField(&Settings, &(Settings.BuzzerParDuration), 2);
            } else {
                Settings.BuzzerStartDuration = duration;
                saveSettingsField(&Settings, &(Settings.BuzzerStartDuration), 2);
            }
        }
    }
}

void setBuzzerMenu(){
    sprintf(ma.MenuItem[0], " Frequency - %dHz ", Settings.BuzzerFrequency);
    sprintf(ma.MenuItem[1], " Volume - %d ", Settings.Volume);
    sprintf(ma.MenuItem[2], " Par Duration - %1.1fs ", (float) (Settings.BuzzerParDuration) / 1000);
    strcpy(ma.MenuItem[3], " Test Beep ");
    ma.TotalMenuItems = 4;
}

void SetBeep() {
    InitSettingsMenuDefaults((&ma));
    strcpy(ma.MenuTitle, "Buzzer ");

    do {
        setBuzzerMenu();
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
        if (ma.selected) {
            lcd_clear();
            switch (ma.menu) {
                case 0:
                    SetBeepFreq();
                    break;
                case 1:
                    SetVolume();
                    break;
                case 2:
                    SetBeepTime(True);
                    break;
                case 3:
                    generate_sinus(Settings.Volume, Settings.BuzzerFrequency, Settings.BuzzerStartDuration);
                    break;
            }
            // Here we want it done only when back pressed
            // i.e. not selected and done
            ma.done = False;
            ma.selected = False;
            lcd_clear();
        }
    } while (SettingsNotDone((&ma)));
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Sensitivity">

void SetSens() {//Sensitivity
    NumberSelection_t s;
    InitSettingsNumberDefaults((&s));
    //    if (Settings.Sensitivity > DETECT_THRESHOLD_LEVELS) Settings.Sensitivity = DETECT_THRESHOLD_LEVELS;
    strcpy(s.MenuTitle, "Set Sensitivity");
    //    s.max = DETECT_THRESHOLD_LEVELS;
    //    s.min = 1;
    s.max = 1024;
    s.min = 10;
    s.value = Settings.Sensitivity;
    s.old_value = Settings.Sensitivity;
    s.step = 1;
    s.format = "%d";
    do {
        DisplayInteger(&s);
        SelectInteger(&s);
    } while (SettingsNotDone((&s)));
    if (s.selected) {
        Settings.Sensitivity = s.value;
        if (s.value != s.old_value) {
            saveSettingsField(&Settings, &(Settings.Sensitivity), 2);
        }
    }
}

void SetSlope() {//Sensitivity
    NumberSelection_t s;
    InitSettingsNumberDefaults((&s));
    //    if (Settings.Sensitivity > DETECT_THRESHOLD_LEVELS) Settings.Sensitivity = DETECT_THRESHOLD_LEVELS;
    strcpy(s.MenuTitle, "Set Slope");
    //    s.max = DETECT_THRESHOLD_LEVELS;
    //    s.min = 1;
    s.max = 1024;
    s.min = 10;
    s.value = Settings.Slope;
    s.old_value = Settings.Slope;
    s.step = 1;
    s.format = "%d";
    do {
        DisplayInteger(&s);
        SelectInteger(&s);
    } while (SettingsNotDone((&s)));
    if (s.selected) {
        Settings.Slope = s.value;
        if (s.value != s.old_value) {
            saveSettingsField(&Settings, &(Settings.Slope), 2);
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Filter">

void SetFilter() {
    if (Settings.Filter > 100) Settings.Filter = 100;
    NumberSelection_t f;
    strcpy(f.MenuTitle, "Set Filter");
    f.fmin = 0.02;
    f.fmax = 0.12;
    f.fstep = 0.01;
    f.fvalue = (float) (Settings.Filter) / 1000;
    f.fold_value = f.value;
    f.done = False;
    f.format = " %1.2fs ";
    do {
        DisplayDouble(&f);
        SelectDouble(&f);
    } while (SettingsNotDone((&f)));
    Settings.Filter = (uint8_t) (f.fvalue * 1000);
    if (f.fvalue != f.fold_value) {
        saveSettingsField(&Settings, &(Settings.Filter), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="AutoStart">

void SetAutoStart() {
    TBool orgset;
    InitSettingsMenuDefaults((&ma));
    ma.TotalMenuItems = 2;
    strcpy(ma.MenuTitle, "Auto Start");
    strcpy(ma.MenuItem[0], " Auto Start OFF ");
    strcpy(ma.MenuItem[1], " Auto Start ON ");
    orgset = AutoStart;
    ma.menu = AutoStart;
    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    if (ma.selected) {
        AutoStart = ma.menu;
        if (AutoStart != orgset) {
            saveSettingsField(&Settings, &(Settings.AR_IS), 1);
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="TimerMode">
// <editor-fold defaultstate="collapsed" desc="Bianchi modes">

void fill_par_bianci() {
    Settings.TotPar = 12;
    Settings.ParTime[0] = 3.000;
    Settings.ParTime[1] = 4.000;
    Settings.ParTime[2] = 8.000;
    Settings.ParTime[3] = 4.000;
    Settings.ParTime[4] = 5.000;
    Settings.ParTime[5] = 6.000;
    Settings.ParTime[6] = 5.000;
    Settings.ParTime[7] = 6.000;
    Settings.ParTime[8] = 7.000;
    Settings.ParTime[9] = 7.000;
    Settings.ParTime[10] = 10.000;
    Settings.ParTime[11] = 15.000;
}

void fill_par_barricade() {
    Settings.TotPar = 8;
    Settings.ParTime[0] = 5.000;
    Settings.ParTime[1] = 5.000;
    Settings.ParTime[2] = 6.000;
    Settings.ParTime[3] = 6.000;
    Settings.ParTime[4] = 7.000;
    Settings.ParTime[5] = 7.000;
    Settings.ParTime[6] = 8.000;
    Settings.ParTime[7] = 8.000;
}

void fill_par_falling_plate() {
    Settings.TotPar = 8;
    Settings.ParTime[0] = 6.000;
    Settings.ParTime[1] = 6.000;
    Settings.ParTime[2] = 7.000;
    Settings.ParTime[3] = 7.000;
    Settings.ParTime[4] = 8.000;
    Settings.ParTime[5] = 8.000;
    Settings.ParTime[6] = 9.000;
    Settings.ParTime[7] = 9.000;
}

int fill_par_moving_target() {
    for (uint8_t i = 0; i < 12; i++) {
        Settings.ParTime[i] = 6.000;
    }
}

void fill_par_nra_ppc_a() {
    Settings.TotPar = 4;
    Settings.ParTime[0] = 20.000;
    Settings.ParTime[1] = 90.000;
    Settings.ParTime[2] = 16.5000;
    Settings.ParTime[3] = 12.000;
}

void fill_par_nra_ppc_b() {
    Settings.TotPar = 5;
    Settings.ParTime[0] = 20.000;
    Settings.ParTime[1] = 12.000;
    Settings.ParTime[2] = 90.000;
    Settings.ParTime[3] = 12.000;
    Settings.ParTime[4] = 12.0000;
}

void fill_par_nra_ppc_c() {
    Settings.TotPar = 4;
    Settings.ParTime[0] = 20.000;
    Settings.ParTime[1] = 90.000;
    Settings.ParTime[2] = 12.000;
    Settings.ParTime[3] = 16.5000;
}

void fill_par_nra_ppc_d() {
    Settings.TotPar = 4;
    Settings.ParTime[0] = 8.000;
    Settings.ParTime[1] = 20.000;
    Settings.ParTime[2] = 20.000;
    Settings.ParTime[3] = 90.000;
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Par Mode">
void set_par_mode(int m) {
    // Draw back from destructive modes
    restoreSettingsField(&Settings, &(Settings.Volume), 1);
    restoreSettingsField(&Settings, &(Settings.Sensitivity), 2);
    switch (m) {
        case ParMode_Silent:
            Settings.Volume = 0; // Intentional fall-through
        case ParMode_Regular:
            CurPar_idx = 0;
            restorePar();
            break;
        case ParMode_Spy:
            Settings.DelayMode = DELAY_MODE_Instant;
            Settings.Volume = 0;
            Settings.Sensitivity = Settings.Sensitivity>0?Settings.Sensitivity-1:0;
            break;
        case ParMode_Repetitive:
            break;
        case ParMode_CUSTOM:
            restorePar();
            break;
        case ParMode_Practical:
            fill_par_bianci();
            break;
        case ParMode_Barricade:
            fill_par_barricade();
            break;
        case ParMode_FallingPlate:
            fill_par_falling_plate();
            break;
        case ParMode_MovingTarget:
            fill_par_moving_target();
            break;
        case ParMode_NRA_PPC_A:
            fill_par_nra_ppc_a();
            break;
        case ParMode_NRA_PPC_B:
            fill_par_nra_ppc_b();
            break;
        case ParMode_NRA_PPC_C:
            fill_par_nra_ppc_c();
            break;
        case ParMode_NRA_PPC_D:
            fill_par_nra_ppc_d();
            break;
        default:
            // How can we get here?
            break;
    }
}

const char * par_mode_menu_names[TOT_PAR_MODES] = {
    " Normal Timer ",
    " Silent Mode ",
    " Spy Mode ",
    " Repetitive Mode ",
    " Custom ",
    " Bianchi: Practical ",
    " Bianchi: Barricade ",
    " Bianchi: Falling Plate ",
    " Bianchi: Moving Targ. ",
    " NRA-PPC: A ",
    " NRA-PPC: B ",
    " NRA-PPC: C ",
    " NRA-PPC: D ",
    " Auto Par "
};

const char * par_mode_header_names[TOT_PAR_MODES] = {
    "Timer",
    "Silent",
    "Spy Mode",
    "Repetitive",
    "Custom",
    "Practical",
    "Barricade",
    "Falling Plate",
    "Moving Targ",
    "NRA-PPC A",
    "NRA-PPC B",
    "NRA-PPC C",
    "NRA-PPC D",
    "Auto Par"
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Repetitive Mode">
void SetFaceTime() {
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b));
    strcpy(b.MenuTitle, "Set Face");
    b.fmin = 0.1;
    b.fmax = 99.9;
    b.fstep = 0.01;
    b.format = SEC_FIELD_DISPLAY_FORMAT_SHORT;
    b.fvalue = (float) Settings.RepetitiveFaceTime / 1000;
    b.fold_value = b.fvalue;
    do {
        DisplayInteger(&b);
        SelectDouble(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.RepetitiveFaceTime = (uint16_t)(b.fvalue * 1000);
        if (b.fvalue != b.fold_value) {
            saveSettingsField(&Settings, &(Settings.RepetitiveFaceTime), 2);
        }
    }
}

void SetEdgeTime() {
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b));
    strcpy(b.MenuTitle, "Set Edge Time");
    b.fmin = 0.1;
    b.fmax = 99.9;
    b.fstep = 0.1;
    b.format = SEC_FIELD_DISPLAY_FORMAT_SHORT;
    b.fvalue = (float) Settings.RepetitiveEdgeTime / 1000;
    b.fold_value = b.fvalue;
    do {
        DisplayInteger(&b);
        SelectDouble(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.RepetitiveEdgeTime = (uint16_t)(b.fvalue * 1000);
        if (b.fvalue != b.fold_value) {
            saveSettingsField(&Settings, &(Settings.RepetitiveEdgeTime), 2);
        }
    }
}

void SetRepeat() {
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b));
    strcpy(b.MenuTitle, "Set Repeat");
    b.min = 1;
    b.max = 10;
    b.step = 1;
    b.format = " %u ";
    b.value = Settings.RepetitiveRepeat;
    b.old_value = b.value;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.RepetitiveRepeat = b.value;
        if (b.value != b.old_value) {
            saveSettingsField(&Settings, &(Settings.RepetitiveRepeat), 1);
        }
    }
}

#define FACE_IDX        0
#define EDGE_IDX        1
#define REPEAT_IDX      2
void setRepetitiveMenu(){
    strcpy(mx.MenuTitle, "Repetitive Mode ");
    mx.TotalMenuItems = 3;
    sprintf(mx.MenuItem[FACE_IDX]," Face - %1.2fs ", (float) Settings.RepetitiveFaceTime / 1000);
    sprintf(mx.MenuItem[EDGE_IDX]," Edge - %1.2fs ", (float) Settings.RepetitiveEdgeTime / 1000);
    sprintf(mx.MenuItem[REPEAT_IDX]," Repeat - %d ", Settings.RepetitiveRepeat);
}
void SetRepetitiveMode(){
    InitSettingsMenuDefaults((&mx));
    do {
        setRepetitiveMenu();
        DisplaySettings((&mx));
        SelectMenuItem((&mx));
        if (mx.selected) {
            lcd_clear();
            switch (mx.menu) {
                case FACE_IDX:
                    SetFaceTime();
                    break;
                case EDGE_IDX:
                    SetEdgeTime();
                    break;
                case REPEAT_IDX:
                    SetRepeat();
                    break;
                case 3:
                    generate_sinus(Settings.Volume, Settings.BuzzerFrequency, Settings.BuzzerStartDuration);
                    break;
            }
            // Here we want it done only when back pressed
            // i.e. not selected and done
            mx.done = False;
            mx.selected = False;
            lcd_clear();
        }
    } while (SettingsNotDone((&mx)));
}
// </editor-fold>
void SetMode() {
    uint8_t oldPar = Settings.ParMode;
    InitSettingsMenuDefaults((&ma));
    ma.TotalMenuItems = TOT_PAR_MODES;
    strcpy(ma.MenuTitle, "Select Mode");
    for (uint8_t i = 0; i < TOT_PAR_MODES; i++) {
        strcpy(ma.MenuItem[i], par_mode_menu_names[i]);
    }

    ma.menu = Settings.ParMode;
    ma.page = ItemToPage(ma.menu);
    //Main Screen
    do {
        DisplaySettings((&ma));
        SelectMenuItemCircular((&ma));
        if(ma.selected && ma.done){
            switch (ma.menu){
                case ParMode_CUSTOM:
                    restorePar();
                    lcd_clear();
                    SetPar(&mx);
                    break;
                case ParMode_Repetitive:
                    lcd_clear();
                    SetRepetitiveMode();
                    break;
            }
        }
    } while (SettingsNotDone((&ma)));
    if (ma.selected) {
        Settings.ParMode = ma.menu;
        if (oldPar != Settings.ParMode) {
            saveSettingsField(&Settings, &(Settings.ParMode), 1);
        }
        STATE_HANDLE_TIMER_IDLE();
    }
    set_par_mode(Settings.ParMode);
    CurPar_idx = 0;
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Clock">

void SetClock() {
    NumberSelection_t ts;
    uint8_t h = get_hour(), m = get_minute();
    InitSettingsNumberDefaults((&ts));
    ts.min = 0;
    ts.max = 23;
    ts.value = h;
    ts.old_value = ts.value;
    ts.step = 1;
    strcpy(ts.MenuTitle, "Set Clock");
    ts.state = 0; // 0 - hour, 1 - Minute. DisplayTime knows to handle this
    set_screen_title(ts.MenuTitle);
    do {
        DisplayTime(ts.value, m, ts.state);
        SelectIntegerCircular(&ts);
    } while (SettingsNotDone((&ts)));
    if (ts.selected && ts.state == 0) {
        ts.state = 1;
        ts.selected = False;
        ts.done = False;
        ts.max = 59;
        h = ts.value;
        ts.value = m;
        do {
            DisplayTime(h, ts.value, ts.state);
            SelectIntegerCircular(&ts);
        } while (SettingsNotDone((&ts)));
    }
    if (ts.selected && (h != get_hour() || ts.value != m)) {
        set_time(h, ts.value);
        comandToHandle = TimeChanged;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="CountDown">

void countdown_expired_signal() {
    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            generate_sinus(
                    Settings.Volume,
                    Settings.BuzzerFrequency,
                    50
                    );
            Delay(50);
        }
        if (Keypressed)
            break;
        Delay(400);
    }
}

void CountDownMode(time_t countdown) {
    char msg[16];
    time_t reminder = countdown * 1000;
    time_t stop_time = rtc_time.unix_time_ms + reminder + 1;
    uint8_t minute, second;
    TBool done = False;
    lcd_clear();
    do {
        print_header(true);
        update_rtc_time();
        reminder = (stop_time - rtc_time.unix_time_ms) / 1000;
        minute = reminder / 60;
        second = reminder % 60;
        sprintf(msg,
                "%2d:%02d",
                minute,
                second
                );
        display_big_font_label(msg);
        define_input_action();
        switch (comandToHandle) {
            case StartLong:STATE_HANDLE_POWER_OFF();
                break;
            case StartShort:STATE_HANDLE_TIMER_IDLE();
                break;
            case None:
                break;
            default:
                done = True;
                break;
        }
        if (minute == 0) {
            if (second == 0) {
                done = True;
            }
        }
    } while (!done && ui_state == SettingsScreen);
    if (done && minute == 0 && second == 0) {
        countdown_expired_signal();
        STATE_HANDLE_TIMER_IDLE();
    }
}

TBool SetCustomCountDown() {
    NumberSelection_t ts;
    uint8_t minute, second;
    char msg[16];
    InitSettingsNumberDefaults((&ts));
    strcpy(ts.MenuTitle, "Set Time");

    // in seconds
    ts.value = Settings.CustomCDtime;
    ts.old_value = ts.value;
    ts.max = 3600;
    ts.min = 0;
    ts.step = 1;
    lcd_clear();
    do {
        print_header(true);
        minute = ts.value / 60;
        second = ts.value % 60;
        sprintf(msg,
                "%2d:%02d",
                minute,
                second
                );
        display_big_font_label(msg);
        SelectInteger(&ts);
    } while (SettingsNotDone((&ts)));
    if (ts.selected) {
        Settings.CustomCDtime = ts.value;
        if (ts.value != ts.old_value) {
            saveSettingsField(&Settings, &(Settings.CustomCDtime), 4);
        }
        return True;
    }
    return False;
}

void SetCountDown() {
    InitSettingsMenuDefaults((&ma));
    ma.TotalMenuItems = 3;
    strcpy(ma.MenuTitle, "Set Countdown");
    strcpy(ma.MenuItem[0], " 3 minutes ");
    strcpy(ma.MenuItem[1], " 5 minutes ");
    strcpy(ma.MenuItem[2], " Custom ");

    //Main Screen
    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    if(ma.selected){
        switch (ma.menu) {
            case 0: CountDownMode(180);
                break;
            case 1: CountDownMode(300);
                break;
            case 2:
                if (SetCustomCountDown())
                    CountDownMode(Settings.CustomCDtime);
                break;
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Tilt">

void SetOrientation() {
    InitSettingsMenuDefaults((&ma));
    ma.TotalMenuItems = 2;
    strcpy(ma.MenuTitle, "Orientation");
    strcpy(ma.MenuItem[ORIENTATION_NORMAL], "Upright");
    strcpy(ma.MenuItem[ORIENTATION_INVERTED], "Upside-down");
    ma.menu = Orientation;

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    if (ma.selected && Orientation != ma.menu) {
        Orientation = ma.menu;
        saveSettingsField(&Settings, &(Settings.AR_IS), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Input">

void SetInput() {
    uint8_t orgset;
    InitSettingsMenuDefaults((&ma));
    strcpy(ma.MenuTitle, "Select Source");
    strcpy(ma.MenuItem[INPUT_TYPE_Microphone], " Microphone ");
    strcpy(ma.MenuItem[INPUT_TYPE_A_or_B_multiple], " A or B (multiple) ");
    strcpy(ma.MenuItem[INPUT_TYPE_A_and_B_single], " A and B (single) ");
    ma.TotalMenuItems = 3;
    orgset = Settings.InputType;
    ma.menu = Settings.InputType;

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    if (ma.selected) {
        Settings.InputType = ma.menu;
    }
    if (Settings.InputType != orgset) {
        saveSettingsField(&Settings, &(Settings.InputType), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="BlueTooth">

void init_bt() {
    if (Settings.AR_IS.BT && BT_PRESENT) {
        init_uart();
        BT_init();
    } else {
        BT_off();
    }
}

void SetAutoPowerOff() {
    InitSettingsMenuDefaults((&ma));
    ma.TotalMenuItems = 2;
    strcpy(ma.MenuTitle, "Auto Power-Off");
    strcpy(ma.MenuItem[SMTH_DISABLED], "Disabled");
    strcpy(ma.MenuItem[SMTH_ENABLED], "Enabled");
    ma.menu = Settings.AR_IS.AutoPowerOff;

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    if (ma.selected && Settings.AR_IS.AutoPowerOff != ma.menu) {
        Settings.AR_IS.AutoPowerOff = ma.menu;
        saveSettingsField(&Settings, &(Settings.AR_IS), 1);
    }
}

void BlueTooth() {
    InitSettingsMenuDefaults((&ma));
    ma.TotalMenuItems = 2;
    strcpy(ma.MenuTitle, "Set Bluetooth");
    strcpy(ma.MenuItem[SMTH_DISABLED], "Disabled");
    strcpy(ma.MenuItem[SMTH_ENABLED], "Enabled");
    ma.menu = Settings.AR_IS.BT;

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    if (ma.selected && Settings.AR_IS.BT != ma.menu) {
        Settings.AR_IS.BT = ma.menu;
        saveSettingsField(&Settings, &(Settings.AR_IS), 1);
        init_bt();
    }
}

void bt_set_par() {
    long par_idx = 0;
    long par_time = 0;
    char * endp[1];
    par_idx = strtol(bt_cmd_args_raw, endp, 10);

    if (par_idx > 0 && par_idx <= MAXPAR) {
        par_time = strtol(*endp + 1, endp, 10);
        // Par time between 1ms and 99000ms
        if (par_time > 0 && par_time < 99901) {
            Settings.ParTime[par_idx - 1] = (float) par_time / 1000;
            Settings.TotPar = par_idx;
            savePar(par_idx);
            saveSettingsField(&Settings, &(Settings.TotPar), 1);
            DAA_MSG_OK;
        } else {
            DAA_MSG_ERROR;
        }
    } else if (par_idx == 0) {
        clear_par();
        DAA_MSG_OK;
    } else {
        DAA_MSG_ERROR;
    }
}

void bt_set_mode() {
    int mode = 0;
    mode = atoi(bt_cmd_args_raw);
    if (mode >= 0 && mode < TOT_PAR_MODES) {
        lcd_clear_block(0, 0, LCD_WIDTH, UI_HEADER_END_LINE);
        Settings.ParMode = mode;
        saveSettingsField(&Settings, &(Settings.ParMode), 1);
        DAA_MSG_OK;
    } else {
        DAA_MSG_ERROR;
    }
}

void bt_set_delay() {
    int time = 0;
    time = atoi(bt_cmd_args_raw);
    if (time > -1 && time < 10000) {
        Settings.DelayMode = DELAY_MODE_Custom;
        Settings.DelayTime = time;
        saveSettingsField(&Settings, &(Settings.DelayTime), 4);
        saveSettingsField(&Settings, &(Settings.DelayMode), 1);
    } else {
        DAA_MSG_ERROR;
    }
}

void bt_get_pars() {
    uint8_t length = 0;
    char msg[16];
    if (Settings.TotPar > 0) {
        for (uint8_t i = 0; i < Settings.TotPar; i++) {
            length = sprintf(msg, "%d,%u\n", i + 1, (long)(Settings.ParTime[i] * 1000));
            sendString(msg, length);
        }
    } else {
        DAA_MSG_EMPTY;
    }
}

void handle_bt_commands() {
    uint8_t length = 0;
    char msg[16];
    switch (BT_COMMAND) {
        case BT_SendVersion:
            length = sprintf(msg, "%u\n", Settings.version);
            sendString(msg, length);
            break;
        case BT_StartTimer:
            STATE_HANDLE_COUNTDOWN;
            break;
        case BT_GetLastString:
            if (ShootString.TotShoots > 0) {
                send_all_shots();
            } else {
                DAA_MSG_EMPTY;
            }
            break;
        case BT_SetPar:
            bt_set_par();
            break;
        case BT_SetMode:
            bt_set_mode();
            break;
        case BT_ClearHistory:
            DAA_MSG_WAIT;
            clearHistory();
            lcd_clear();
            DAA_MSG_OK;
            break;
        case BT_DefaultSettings:
            getDefaultSettings();
            saveSettings();
            DAA_MSG_OK;
            break;
        case BT_Find:
            DAA_MSG_LISTEN;
            countdown_expired_signal();
            break;
        case BT_None:
            break;
        case BT_GetPars:
            bt_get_pars();
            break;
        case BT_SetDelay:
            bt_set_delay();
            DAA_MSG_OK;
            break;
        default:
            DAA_MSG_NOT_SUPPORTED;
            break;
    }
    BT_COMMAND = BT_None;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Microphone">
void fillMicrophoneMenu(){
    ma.TotalMenuItems = 3;
    strcpy(ma.MenuTitle, " Microphone ");
    sprintf(ma.MenuItem[0], " Sensitivity  - %d ", Settings.Sensitivity);
    sprintf(ma.MenuItem[1],  " Filter - %1.2fs ", (float) (Settings.Filter) / 1000);
    sprintf(ma.MenuItem[2],  " Slope - %d ", Settings.Slope);
}

void SetMicrophone(){
    InitSettingsMenuDefaults((&ma));
    ma.menu = 0;
    fillMicrophoneMenu();

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
        if(ma.selected){
            lcd_clear();
            switch (ma.menu){
                case 0:
                    SetSens();
                    break;
                case 1:
                    SetFilter();
                    break;
                case 2:
                    SetSlope();
                    break;
            }
            lcd_clear();
            ma.done = False;
            ma.selected = False;
            fillMicrophoneMenu();
        }
    } while (SettingsNotDone((&ma)));
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Display">
void SetDisplay(){
    InitSettingsMenuDefaults((&ma));
    ma.menu = 0;
    ma.TotalMenuItems = 2;
    sprintf(ma.MenuTitle, "Set Display ");
    sprintf(ma.MenuItem[0], " Backlight - %d ", Settings.BackLightLevel);
    sprintf(ma.MenuItem[1],  "  Orientation  ");

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
        if(ma.selected){
            lcd_clear();
            switch (ma.menu){
                case 0:
                    SetBacklight();
                    sprintf(ma.MenuItem[0], " Backlight - %d ", Settings.BackLightLevel);
                    ma.done = False;
                    ma.selected = False;
                    break;
                case 1:
                    SetOrientation();
                    ma.done = False;
                    ma.selected = False;
                    break;
            }
            lcd_clear();
        }
    } while (SettingsNotDone((&ma)));
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Settings Menu">
TBool userIsSure(const char * title){
    InitSettingsMenuDefaults((&ma));
    ma.menu = SMTH_ENABLED; // No
    ma.TotalMenuItems = 2;
    strcpy(ma.MenuTitle, title);
    strcpy(ma.MenuItem[SMTH_DISABLED], " Yes  ");
    strcpy(ma.MenuItem[SMTH_ENABLED],  "  No  ");

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    return (ma.menu == SMTH_DISABLED);
}

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

void DoSet(uint8_t menu) {
    lcd_clear();
    switch (menu) {
        case SETTINGS_INDEX_DELAY:
            SetDelay();
            break;
        case SETTINGS_INDEX_PAR:SetPar((&ma)); // By reference because it's used both in 2nd and 3rd level menu
            break;
        case SETTINGS_INDEX_BUZZER:
            SetBeep();
            break;
        case SETTINGS_INDEX_MIC:
            SetMicrophone();
            break;
        case SETTINDS_INDEX_AUTOSTART:
            SetAutoStart();
            break;
        case SETTINGS_INDEX_MODE:SetMode();
            break;
        case SETTINGS_INDEX_DISPLAY:
            SetDisplay();
            break;
        case SETTINDS_INDEX_CLOCK:SetClock();
            break;
        case SETTINDS_INDEX_COUNTDOWN:SetCountDown();
            break;
        case SETTINDS_INDEX_INPUT:
            SetInput();
            break;
        case SETTINDS_INDEX_BLUETOOTH:
            BlueTooth();
            break;
        case SETTINDS_INDEX_RESET:
            if(userIsSure(" Reset Settings ? ")){
                getDefaultSettings();
                saveSettings();
            }
            break;
        case SETTINDS_INDEX_CLEAR:
            if(userIsSure(" Clear History ? ")){
                clearHistory();
            }
            break;
        case SETTINDS_INDEX_AUTO_POWER: // For tests
            SetAutoPowerOff();
            break;
    }
    lcd_clear();
}

void SetSettingsMenu() {
    //{"Delay","Par","Beep","Auto","Mode","Clock","CountDown","Tilt","Bklight","Input","BT","Diag"};
    char tmpStr[16];
    SettingsMenu.TotalMenuItems = 15;
    sprintf(SettingsMenu.MenuTitle, "Settings ");

    print_delay(SettingsMenu.MenuItem[0], " Delay - ");
    if (Settings.TotPar > 0) {
        sprintf(SettingsMenu.MenuItem[1],
                " Par - %d 1st: %3.2f ",
                Settings.TotPar,
                Settings.ParTime[CurPar_idx]);
    } else {
        sprintf(SettingsMenu.MenuItem[1], " Par - Off ");
    }
    sprintf(SettingsMenu.MenuItem[2], " Buzzer - %d %dHz ",
            Settings.Volume, Settings.BuzzerFrequency);
    sprintf(SettingsMenu.MenuItem[3], " Microphone - %d %0.2fs ",
            Settings.Sensitivity, (float) Settings.Filter/1000);
    sprintf(SettingsMenu.MenuItem[4], " Mode - %s ",
            par_mode_header_names[Settings.ParMode]);
    sprintf(SettingsMenu.MenuItem[5], " Display ");
    sprintf(SettingsMenu.MenuItem[6], " Countdown ");
    sprintf(SettingsMenu.MenuItem[7], " Autostart - %s ",
            (Settings.AR_IS.Autostart)?"ON":"OFF");
    sprintf(SettingsMenu.MenuItem[8], " Clock ");
            switch (Settings.InputType) {
            case INPUT_TYPE_Microphone:
                sprintf(SettingsMenu.MenuItem[9], " Input - Microphone ");
                break;
            case INPUT_TYPE_A_and_B_single:
                sprintf(SettingsMenu.MenuItem[9], " Input - A+B single ");
                break;
            case INPUT_TYPE_A_or_B_multiple:
                sprintf(SettingsMenu.MenuItem[9], " Input - A/B multi ");
                break;
            default:
                sprintf(SettingsMenu.MenuItem[9], " Input ");
                break;
        }

    sprintf(SettingsMenu.MenuItem[10], " Bluetooth - %s ",
            (Settings.AR_IS.BT)?"ON":"OFF");
    sprintf(SettingsMenu.MenuItem[11], " Auto Power-off  - %s ",
            (Settings.AR_IS.AutoPowerOff)?"ON":"OFF");
    sprintf(SettingsMenu.MenuItem[12], " Clear History ");
    sprintf(SettingsMenu.MenuItem[13], " Reset Settings ");
    sprintf(SettingsMenu.MenuItem[14], " FW version: %02d ", Settings.version);
}

void DoSettings(void) {
    InitSettingsMenuDefaults((&SettingsMenu));
    SetSettingsMenu();
    lcd_clear();
    do {
        DisplaySettings(&SettingsMenu);
        SelectMenuItemCircular(&SettingsMenu);
        if (SettingsMenu.selected) {
            DoSet(SettingsMenu.menu);
            SettingsMenu.selected = False;
            lcd_clear();
            SetSettingsMenu();
        }
        // Never exit on OK/Cancel case here, only on screen change
        SettingsMenu.done = False;
    } while (SettingsNotDone((&SettingsMenu)));

    if (ui_state == SettingsScreen) {
        STATE_HANDLE_TIMER_IDLE();
    } else {
        lcd_clear();
    }
}
// </editor-fold>
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Review Screen">
#define REVIEW_SHOT_FORMAT      "%2d: %3.2f %c"
#define REVIEW_TOTAL_SHOT_FORMAT      "%2d Shots - %3.2fs"
#define REVIEW_SPLIT_FORMAT     "]%3.2f"
TBool reviewChanged = True;

void print_strings_line() {
    uint8_t col, line, i, page_start, page_size;
    TBool first_page;
    char message[20];
    line = LCD_HEIGHT - SmallFont->height;
    if (reviewChanged) {
        reviewChanged = False;
        lcd_fill_block(0, line, LCD_WIDTH, LCD_HEIGHT);
    }
    if (CurShootString < 10)
        page_size = 10;
    else
        page_size = 5;
    first_page = (CurShootString / page_size == 0);
    sprintf(message, "String<");
    col = 4;
    lcd_write_string(message, col, line, SmallFont, WHITE_OVER_BLACK);
    col += lcd_string_lenght(message, SmallFont) + 2;
    col += first_page ? 0 : 3;
    page_start = page_size * (CurShootString / page_size);
    for (i = 0; i < page_size; i++) {
        uint8_t polarity = (CurShootString % page_size == i);
        // Rounding to a page
        sprintf(message, "%d", page_start + i + 1);

        lcd_write_string(message, col, line, SmallFont, polarity);
        if (polarity)
            lcd_send_block_d(col - 2, line, col, line + SmallFont->height, ~polarity);
        col += lcd_string_lenght(message, SmallFont) + 3;
        if (polarity)
            lcd_send_block_d(col - 5, line, col - 3, line + SmallFont->height, !polarity);
    }
    sprintf(message, ">");
    lcd_write_string(message, col, line, SmallFont, WHITE_OVER_BLACK);
}

void ReviewDisplay() {
    uint8_t line = UI_HEADER_END_LINE,
            i,
            last_shot_index,
            halfline = 16;
    char message[20];
    // We're assuming here that Medium font has even number of bytes heigh
    if (!reviewChanged) return;
    uint8_t totShots = ReviewString.TotShoots;
    lcd_clear();
    // assuming ReviewString and ShootString differ only by the shot order in memory
    last_shot_index = ReviewTopShotDefault;
    // Stat line
    sprintf(ScreenTitle,
            REVIEW_TOTAL_SHOT_FORMAT,
            ReviewString.shots[last_shot_index].sn,
            (float) ReviewString.shots[last_shot_index].dt / 1000
            );
    print_header(true);
    //Shoot lines
    //1st ShootNumber 01, before it ShootNumber 00 time=0
    for (i = 0; i < SHOTS_ON_REVIEW_SCREEN; i++) {
        char * mode;
        uint8_t curr_index = get_rev_shot_index_in_arr(TopShotIndex + i, totShots);
        uint8_t next_index = get_rev_shot_index_in_arr(TopShotIndex + i + 1, totShots);

        if (Settings.InputType == INPUT_TYPE_Microphone) {
            mode = ' ';
        } else {
            mode = (ReviewString.shots[curr_index].is_b) ? 'B' : ((ReviewString.shots[curr_index].is_a) ? 'A' : ' ');
        }
        // Handle cases with 1 and 2 shots nicely
        if (i != 0 || totShots >= SHOTS_ON_REVIEW_SCREEN) {
            sprintf(message,
                    REVIEW_SHOT_FORMAT,
                    ReviewString.shots[curr_index].sn,
                    (float) ReviewString.shots[curr_index].dt / 1000,
                    mode
                    );
            if (lcd_string_lenght(message, MediumFont) > 134
                    ||((float) ReviewString.shots[curr_index].dt / 1000 > 99.7)
                    || ReviewString.shots[curr_index].sn > 99)
                lcd_write_string(message, 1, line, SmallFont, (i != 1)&0x01);
            else
                lcd_write_string(message, 1, line, MediumFont, (i != 1)&0x01);
        }
        line += halfline;
        if (i == totShots) break;
        // Don't print last diff at half line and not the latest
        if (i < SHOTS_ON_REVIEW_SCREEN - 1
            && ReviewString.shots[next_index].sn > ReviewString.shots[curr_index].sn) {
            
            sprintf(message,
                    REVIEW_SPLIT_FORMAT,
                    (float) (ReviewString.shots[next_index].dt - ReviewString.shots[curr_index].dt) / 1000);
            lcd_write_string(message, 135, line, MediumFont, BLACK_OVER_WHITE);
        }
        line += halfline;

    }
    //String line
    print_strings_line();
}

void review_scroll_shot_up() {
    if (ReviewString.TotShoots < SHOTS_ON_REVIEW_SCREEN) {
        Beep();
        return;
    }
    if (TopShotIndex > 0) {
        TopShotIndex--;
    } else {
        TopShotIndex = ReviewTopShotDefault;
    }
    reviewChanged = True;
}

void review_scroll_shot_down() {
    if (ReviewString.TotShoots < SHOTS_ON_REVIEW_SCREEN) {
        Beep();
        return;
    }
    if (TopShotIndex < ReviewTopShotDefault) {
        TopShotIndex++;
    } else {
        TopShotIndex = 0;
    }
    reviewChanged = True;
}

void review_previous_string() {
    if (CurShootString > 0) {
        CurShootString--;
        if (checkShotStringEmpty(CurShootString)) {
            Beep();
            CurShootString++;
        }
    } else {
        CurShootString = MAXSHOOTSTRINGS - 1;
        if (checkShotStringEmpty(CurShootString)) {
            Beep();
            CurShootString = 0;
        }
    }
    getShootString(CurShootString);
    reviewChanged = True;
    TopShotIndex = ReviewTopShotDefault;
}

void review_next_string() {
    if (CurShootString < MAXSHOOTSTRINGS - 1) {
        CurShootString++;
        if (checkShotStringEmpty(CurShootString)) {
            Beep();
            CurShootString--;
        }
    } else {
        CurShootString = 0;
    }
    getShootString(CurShootString);
    reviewChanged = True;
    TopShotIndex = ReviewTopShotDefault;
}

void DoReview() {
    getShootString(0);
    CurShootString = 0;
    TopShotIndex = ReviewTopShotDefault;
    if (ShootString.TotShoots == 0) {
        Beep();
        STATE_HANDLE_TIMER_IDLE();
        return;
    }
    reviewChanged = True;
    do {
        ReviewDisplay();
        define_input_action();
        BT_define_action();
        switch (comandToHandle) {
            case UpShort:
                review_scroll_shot_up();
                break;
            case ReviewShort:
            case DownShort:
                review_scroll_shot_down();
                break;
            case BackShort:
                review_next_string();
                break;
            case OkShort:
                review_previous_string();
                break;
            case StartLong:STATE_HANDLE_POWER_OFF();
                break;
            case StartShort:STATE_HANDLE_TIMER_IDLE();
                break;
            case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN();
                break;
            default:
                break;
        }
        comandToHandle = None;

    } while (ui_state == ReviewScreen);
    clear_screen_title;
    lcd_clear();
    getShootString(0);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Main Menu">

void DetectInit(void) {
    uint16_t Mean = 0;
    uint16_t Max = 0;
    uint16_t ADCvalue;

    switch (Settings.InputType) {
        case INPUT_TYPE_Microphone:
            ADC_DISABLE_INTERRUPT;
            for (uint8_t i = 0; i < 64; i++) {
                ADCvalue = ADC_Read(ENVELOPE);
                Mean += ADCvalue;
                if (Max < ADCvalue) Max = ADCvalue;
            }
            Mean = Mean >> 6;
//                DetectThreshold = Mean + threshold_offsets[Settings.Sensitivity - 1];
//            DetectThreshold = Mean + Settings.Sensitivity;
            DetectThreshold = Settings.Sensitivity;
            DetectSlopeThreshold = Settings.Slope;
            ADC_ENABLE_INTERRUPT_ENVELOPE;
            // Enable output driver for A/B I/O
            TRISDbits.TRISD0 = 0;
            TRISDbits.TRISD1 = 0;
            TRISDbits.TRISD2 = 0;
            break;
        default:
            InputFlags.A_RELEASED = True;
            InputFlags.B_RELEASED = True;
            // Disable output driver for A/B I/O
            TRISDbits.TRISD0 = 1;
            TRISDbits.TRISD1 = 1;
            TRISDbits.TRISD2 = 1;
            break;
    }
}

uint8_t print_title(TBool settings) {
    char message[30];
    uint8_t title_pos = 5;
    if(!settings){
        sprintf(message,
                "%02d:%02d ",
                get_hour(),
                get_minute());
        lcd_write_string(message, 1, 0, SmallFont, BLACK_OVER_WHITE);
        title_pos = 55;
    }
    sprintf(message, "%s", ScreenTitle);
    lcd_write_string(message, title_pos, 0, SmallFont, BLACK_OVER_WHITE);
    return SmallFont->height;
}

void print_batery_info() {
    uint8_t col = LCD_WIDTH - 35;
    uint8_t num_bars = number_of_battery_bars();
    lcd_draw_bitmap(col, 0, &battery_left_bitmap);
    col = col + battery_left_bitmap.width_in_bits;

    for (uint8_t i = 5; i > 0; i--) {
        if (i < num_bars + 1) {
            lcd_draw_bitmap(col, 0, &battery_middle_full_bitmap);
            col += battery_middle_full_bitmap.width_in_bits;
        } else {
            lcd_draw_bitmap(col, 0, &battery_middle_empty_bitmap);
            col += battery_middle_empty_bitmap.width_in_bits;
        }
    }
    lcd_draw_bitmap(col, 0, &battery_right_bitmap);
}

void print_bt_indication(){
    if (BT_PRESENT && BT_STATUS.connected) {
        lcd_draw_bitmap(LCD_WIDTH - 50, 0, &bt_bitmap_data);
    } else {
        lcd_clear_block(
                LCD_WIDTH - 50,
                0,
                LCD_WIDTH - 50 + bt_bitmap_data.width_in_bits,
                bt_bitmap_data.heigth_in_bytes * 8
                );
    }
}

uint8_t print_header(TBool settings) {
    print_title(settings);
    print_bt_indication();
    print_batery_info();
    lcd_draw_fullsize_hline(UI_HEADER_END_LINE - 1, LCD_MID_LINE_PAGE);
    return UI_HEADER_END_LINE;
}

void print_label_at_footer_grid(const char* msg, const uint8_t grid_x, const uint8_t grid_y) {
    lcd_write_string(msg, UI_FOOTER_GRID_X(grid_x), UI_FOOTER_GRID_Y(grid_y, UI_FOOTER_START_LINE), SmallFont, WHITE_OVER_BLACK);
}

void print_footer() {
    char message[20];
    if (!InputFlags.FOOTER_CHANGED)
        return;
    InputFlags.FOOTER_CHANGED = False;
    lcd_fill_block(0, UI_FOOTER_START_LINE, LCD_WIDTH, LCD_HEIGHT);
    sprintf(message, " 1st: %3.2f", (float) ShootString.shots[0].dt / 1000);
    print_label_at_footer_grid(message, 0, 0);
    sprintf(message, "Shots: %2d", ShootString.TotShoots);
    print_label_at_footer_grid(message, 1, 0);
    print_delay(message," Delay: ");
    print_label_at_footer_grid(message, 0, 1);

    switch(Settings.ParMode){
        case ParMode_Repetitive:
            if (ui_state == TimerIdle){
                sprintf(message,"Face%d:%3.1f",
                    1,
                    (float) Settings.RepetitiveFaceTime / 1000);
            } else {
                sprintf(message,"%s%d:%3.1f",
                    (repetitive_state==Face)?"Face":"Edge",
                    repetitive_counter + 1,
                    (float) repetitive_time / 1000);
            }
            break;
        default:
            if (Settings.TotPar > 0 && CurPar_idx < Settings.TotPar) {
                sprintf(message, "Par%2d:%3.2f", CurPar_idx + 1, Settings.ParTime[CurPar_idx]);
            } else {
                sprintf(message, "Par: Off");
            }
        break;
    }
//    sprintf(message, "%u", PORTD&0x7);
    print_label_at_footer_grid(message, 1, 1);
}

void StartListenShots(void) {
    ShootString_start_time = rtc_time.unix_time_ms;
    DetectInit();
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Power functions">

void DoPowerOff() {
    lcd_clear(); // Remove remaining picture on power on
    lcd_sleep();
    PWM6CONbits.EN = 0; // Disale PWM
    T2CONbits.ON = 0;
    CPUDOZEbits.IDLEN = 0;
    PIE0bits.TMR0IE = 0; // Disable 1ms timer interrupt
    BT_off();
    while (Keypressed); // Wait to button to release
    OSCCON3bits.CSWHOLD = 0; // Switch OSC when ready
    OSCCON1bits.NOSC = 0b100; // New oscillator is SOSC
    while (!OSCCON3bits.ORDY); // Wait new oscillator ready
    OSCENbits.HFOEN = 0; // Disable HFINTOSC
    ADC_DISABLE_INTERRUPT;
    InputFlags.INITIALIZED = False;
    // Configure interrupt for wakeup
    INT0IE = 1;
    // Enable RTC interrupt
    RTC_TIMER_IE = 1;
    PORTEbits.RE0 = 0;
    PORTEbits.RE2 = 0;
    PORTEbits.RE6 = 1;
    LATEbits.LATE2 = 0;
    LATEbits.LATE6 = 1;
    LATEbits.LATE0 = 0;
    Sleep();
    InputFlags.KEY_RELEASED = True;
    PIE0bits.TMR0IE = 1;
    OSCCON1bits.NOSC = 0b110; // New oscillator is HFINTOSC
    while (!OSCCON3bits.ORDY); // Wait new oscillator ready
    lcd_wakeup();
}

void DoPowerOn() {
    if (InputFlags.INITIALIZED) return;
    PIC_init();
    LATEbits.LATE0 = 1;
    initialize_backlight();
    spi_init();
    lcd_init();
    lcd_set_orientation();
    ADC_init();
    eeprom_init();

    // TODO: Review power on sequence
    RTC_TIMER_IE = 1; // Enable 2 s timer interrupt
    GIE = 1; // enable global interrupts
    INT0IE = 0; // Disable wakeup interrupt
    init_ms_timer0();
    initialize_rtc_timer();
    battery_mV = ADC_Read(BATTERY)*BAT_divider;
    ADC_ENABLE_INTERRUPT_BATTERY;
    getSettings();
    init_bt();
    update_rtc_time();
    timer_idle_last_action_time = rtc_time.sec;
    InputFlags.INITIALIZED = True;
    print_logo_splash();
}

void DoCharging() {
    char msg[10];
    if (charger_state_changed) {
        charger_display_state = charger_state;
        switch (charger_state) {
            case Charging:
                LATEbits.LATE0 = 1;
                lcd_clear();
                sprintf(msg, "Charging ");
                lcd_write_string(msg, UI_CHARGING_LBL_X, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
                break;
            case Complete:
                LATEbits.LATE0 = 1;
                lcd_clear();
                sprintf(msg, "Charged ");
                lcd_write_string(msg, UI_CHARGING_LBL_X, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
                break;
            case NotCharging:
                STATE_HANDLE_POWER_OFF();
                break;
            default:
                break;
        }
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Service functions">

void PlayParSound() {
    sendSignal("PAR", Settings.BuzzerParDuration, (long)(Settings.ParTime[CurPar_idx] * 1000));
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        TRISDbits.TRISD1 = 1;
        TRISDbits.TRISD2 = 1;
        LATDbits.LATD1 = 1;
        LATDbits.LATD2 = 1;
    }
    generate_sinus(Settings.Volume, Settings.BuzzerFrequency, Settings.BuzzerParDuration);
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        TRISDbits.TRISD1 = 0;
        TRISDbits.TRISD2 = 0;
        LATDbits.LATD1 = 0;
        LATDbits.LATD2 = 0;
    }
}

void PlayStartSound() {
    sendSignal("START", Settings.BuzzerStartDuration, 0.0);
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        TRISDbits.TRISD1 = 1;
        TRISDbits.TRISD2 = 1;
        LATDbits.LATD1 = 1;
        LATDbits.LATD2 = 1;
    }
    generate_sinus(Settings.Volume, Settings.BuzzerFrequency, Settings.BuzzerStartDuration);
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        TRISDbits.TRISD1 = 0;
        TRISDbits.TRISD2 = 0;
        LATDbits.LATD1 = 0;
        LATDbits.LATD2 = 0;
    }
}

void StartParTimer() {
    timerEventToHandle = None;
    if (CurPar_idx < Settings.TotPar) {
        ParNowCounting = true;
        InputFlags.FOOTER_CHANGED = True;
        parStartTime_ms = rtc_time.unix_time_ms;
    }
}

void StartCountdownTimer() {
    char msg[16];
    uint8_t length;
    ADC_ENABLE_INTERRUPT_BATTERY; // To get accurate battery readings if someone actively uses timer in Autostart mode
    switch(Settings.ParMode){
        case ParMode_Regular:
            CurPar_idx = 0;
            break;
        case ParMode_Repetitive:
            repetitive_counter = 0;
            repetitive_state = Face;
            repetitive_time = Settings.RepetitiveFaceTime;
            break;
    }
    InputFlags.FOOTER_CHANGED = True;
    switch (Settings.DelayMode) {
        case DELAY_MODE_Instant: Settings.DelayTime = 2; // To allow battery reading and not interfere with detection
            break;
        case DELAY_MODE_Fixed: Settings.DelayTime = 3000;
            break;
        case DELAY_MODE_Random:
            Settings.DelayTime = 2000 + random16(3000);// from 2s to 5s random delay
            break;
        case DELAY_MODE_Custom:
            eeprom_read_array(SettingAddress(Settings, Settings.DelayTime), (uint8_t *)&(Settings.DelayTime), 4);
            break;
    }
    update_rtc_time();
    countdown_start_time = rtc_time.unix_time_ms;
    ShootString_start_time = countdown_start_time;
    for (uint16_t i = 0; i < Size_of_ShootString; i++) {
        ShootString.data[i] = 0;
    }
    length = sprintf(msg, "STANDBY,%d,%d\n", Settings.DelayMode, Settings.DelayTime);
    sendString(msg, length);
}

void UpdateShot(time_t now, ShotInput_t input) {
    uint24_t dt, ddt;
    // Index var is for code size optimisation.
    uint8_t index , prev_index;
    index = get_shot_index_in_arr(ShootString.TotShoots);
    prev_index = get_shot_index_in_arr(top_shot_index()); // function used for optimization

    dt = (uint24_t) (now - ShootString_start_time);
    if (ShootString.TotShoots == 0) {
        ddt = 0;
    } else {
        ddt = ShootString.shots[prev_index].dt;
    }

    ddt = dt - ddt;
    //Don't count shoots less than Filter
    if (ddt > Settings.Filter) {
        ShootString.shots[index].dt = dt;
        ShootString.shots[index].is_flags = input;
        if (ShootString.TotShoots < MAX_REGISTERED_SHOTS) {
            ShootString.TotShoots++;
            ShootString.shots[index].sn = ShootString.TotShoots;
        }
        
        InputFlags.FOOTER_CHANGED = True;
    }
}

void UpdateShotNow(ShotInput_t x) {
    update_rtc_time();
    timer_idle_last_action_time = rtc_time.sec;
    UpdateShot(rtc_time.unix_time_ms, x);
}

void check_countdown_expired() {
    update_rtc_time();
    if (rtc_time.unix_time_ms - countdown_start_time > Settings.DelayTime) {
        comandToHandle = CountdownExpired;
    }
}

void increment_par() {
    if (CurPar_idx != Settings.TotPar - 1) {
        CurPar_idx++;
    } else {
        CurPar_idx = 0;
    }
    InputFlags.FOOTER_CHANGED = True;
}

void check_par_expired() {
    if (ParNowCounting) {
        update_rtc_time();
        switch (Settings.ParMode) {
            case ParMode_Repetitive:
                if(rtc_time.unix_time_ms - parStartTime_ms < repetitive_time) break;
                if(repetitive_counter < Settings.RepetitiveRepeat) {
                    if(repetitive_state == Face){
                        repetitive_state = Edge;
                        repetitive_time = Settings.RepetitiveEdgeTime;
                    } else {
                        repetitive_state = Face;
                        repetitive_time = Settings.RepetitiveFaceTime;
                        repetitive_counter++;
                    }
                    timerEventToHandle = ParEvent;
                } else {
                    timerEventToHandle = TimerTimeout;
                }
                InputFlags.FOOTER_CHANGED = True;
                break;
            default:
            {
                long par_ms = (long) (Settings.ParTime[CurPar_idx] * 1000);
                if (rtc_time.unix_time_ms - parStartTime_ms > par_ms) {
                    ParNowCounting = false;
                    timerEventToHandle = ParEvent;
                }
                break;
            }
        }
    }
}

void check_timer_max_time() {
    if (rtc_time.unix_time_ms - ShootString_start_time >= MAX_MEASUREMENT_TIME) {
        timerEventToHandle = TimerTimeout;
    }
}

void update_screen_model() {
    if (ui_state != TimerListening) return;
    switch (Settings.InputType) {
        case INPUT_TYPE_A_or_B_multiple:
            if (InputFlags.A_RELEASED && !AUX_A) {
                InputFlags.A_RELEASED = 0;
                UpdateShotNow(A);
            }
            if (InputFlags.B_RELEASED && !AUX_B) {
                InputFlags.B_RELEASED = 0;
                UpdateShotNow(B);
            }
            break;
        case INPUT_TYPE_A_and_B_single:
            if (InputFlags.A_RELEASED && !AUX_A) {
                InputFlags.A_RELEASED = 0;
                if (ShootString.TotShoots == 0 || ShootString.shots[0].is_b) {
                    UpdateShotNow(A);
                }
            }
            if (InputFlags.B_RELEASED && !AUX_B) {
                InputFlags.B_RELEASED = 0;
                if (ShootString.TotShoots == 0 || ShootString.shots[0].is_a) {
                    UpdateShotNow(B);
                }
            }
            if (ShootString.TotShoots == 2) timerEventToHandle = TimerTimeout;
            break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="ISR function">

void DetectMicShot() {
    int diff = ADC_LATEST_VALUE - ADC_PREV_VALUE;
    if (ui_state != TimerListening) return;
    if (diff <= DetectSlopeThreshold) return; // Detect raise only
    if (ADC_LATEST_VALUE < DetectThreshold) return; // only if greater than threshold
    UpdateShotNow(Mic);
}

static void interrupt isr(void) {
    // sinus value interrupt
    if (PIR5bits.TMR4IF){
        PIR5bits.TMR4IF = 0;
        sinus_value_expired();
    }
    // sinus duration interrupt
    if (PIR5bits.TMR8IF){
        PIR5bits.TMR8IF = 0;
        sinus_duration_expired();
    }
    if (PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        if (!Keypressed) {//Assignment will not work because of not native boolean
            InputFlags.KEY_RELEASED = True;
            LongPressCount = 0;
        }
        if (AUX_A) { // high is "open"
            InputFlags.A_RELEASED = True;
        }
        if (AUX_B) {
            InputFlags.B_RELEASED = True;
        }
        update_screen_model();
        if (ADPCH == ENVELOPE)
            ADCON0bits.ADGO = 1;
        InputFlags.ADC_DETECTED = 0;
    } 
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        if (ADPCH == ENVELOPE) {
            ADC_BUFFER_PUT(ADC_SAMPLE_REG_16_BIT);
            DetectMicShot();
        } else if (ADPCH == BATTERY) {
            ADCON0bits.ADGO = 0;
            adc_battery = ADC_SAMPLE_REG_16_BIT;
            battery_mV = min(adc_battery*BAT_divider, battery_mV);
            ADC_DISABLE_INTERRUPT;
        }
    } 
    if (RTC_TIMER_IF) {
        RTC_TIMER_IF = 0; // Clear Interrupt flag.
        update_rtc_time();
        InputFlags.FOOTER_CHANGED = 1;
        tic_2_sec();
        if (ui_state == PowerOff) {
            define_charger_state();
        } else if (ui_state != TimerListening && ui_state != TimerCountdown) {
            ADC_ENABLE_INTERRUPT_BATTERY;
        }
    } 
    if (INT0IF) {
        INT0IF = 0; // Wakeup happened, disable interrupt back
    }
    if (PIR3bits.TX1IF) {
        PIR3bits.TX1IF = 0;
        uart_tx_int_handler();
    }
    if (PIR3bits.RC1IF) {
        uart_rx_int_handler();
        PIR3bits.RC1IF = 0;
    }
}
// </editor-fold>

void main(void) {
    // <editor-fold defaultstate="collapsed" desc="Initialization">
    DoPowerOn();
    getSettings();
    if (Settings.version != FW_VERSION) {
        clearHistory();
        getDefaultSettings();
        saveSettings();
    }
    set_backlight(Settings.BackLightLevel);
    getShootString(0); // Show last shot properly after fresh boot
    // Initialization End
    // </editor-fold>
    // <editor-fold defaultstate="collapsed" desc="Main">
    lcd_clear();
    InputFlags.FOOTER_CHANGED = True;
    while (True) {
        //TODO: Integrate watchdog timer
        handle_ui();
    }
    // </editor-fold>
}
