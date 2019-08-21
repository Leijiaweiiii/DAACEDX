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
    PMD1 = 0;
    PMD2 = 0;
    PMD3 = 0;
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

    TRISE = 0b11111000; // E0 = POWER(+3), E1 = POWER(+5V), E2 = BUZZER EN
    ANSELE = 0b00000000;

    TRISF = 0b00000000; // F3 = DP_CS(OP), F4 = DP_A0(OP), F6 = DP_RST(OP)
    ANSELF = 0b00100000; // DAC
    DAC1CON0 = 0b10100000; // DAC enabled output on pin13 (RF5) with reference from VDD & VSS

    TRISG = 0x00; // CON8, Debug Header.
    ANSELG = 0x00;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Backlight PWM Function">
uint8_t current_dc;
void initialize_backlight() {
    uint8_t PRvalue, prescalar;
    TRISEbits.TRISE6 = 0; // Disable output.
    PWM6CON = 0; // Clear register.
    find_optimal_PWM_settings(1000, &PRvalue, &prescalar);
    RE6PPS = 0x0A; // PPS setting.
    // Initialize Timer2.
    PIR5bits.TMR2IF = 0; // Clear timer interrupt flag.
    T2CLKCON = 0b001; // Timer2 clock source = Fosc/4;
    T2CONbits.CKPS = find_set_bit_position(prescalar);
    PR2 = PRvalue;
    current_dc = 0xFF;
}


void set_backlight(uint8_t level) {
    uint8_t duty_cycle = (level == 0) ? 0 : level * 10 - 1;
    // if not changing brightness
    if(duty_cycle == current_dc) return;
    // when changing, turn everything off
    PWM6CON = 0;
    TRISEbits.TRISE6 = 1; // Enable output, to reset value
    LATEbits.LATE6 = 0; // Clear PWM pin
    current_dc = duty_cycle;
    // if need to turn off - remain shut down
    if (duty_cycle == 0 || duty_cycle > 99) return;
    // Otherwise set the new duty cycle
    TRISEbits.TRISE6 = 0; // Disable output driver to led PWM work
    uint16_t ON_value;
    uint8_t PR_value = PR2;
    ON_value = (duty_cycle * (PR_value + 1)) / 25;
    PWM6DCH = (ON_value >> 2);
    PWM6DCL = ((ON_value & 0x0003) << 6);
    PWM6CONbits.POL = 0; // Polarity.
    T2CONbits.ON = 1; // Start timer.
    PWM6CONbits.EN = 1; // Enable PWM.
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
    if (Settings.ParMode == ParMode_Spy) return;
    if (amplitude == 0) return;
    amplitude_index = amplitude - 1;
    sinus_dac_init();
    sinus_duration_timer_init(duration);
    sinus_value_timer_init(findex);
    beep_start = unix_time_ms;
    Stats.Signal++;
    saveStatsField(&(Stats.Signal), 4);
//    InputFlags.BEEP_GUARD = True;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Save and retrive DATA">

void clearHistory() {
    lcd_clear();
    lcd_write_string("Please wait", UI_CHARGING_LBL_X - 20, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
    eeprom_clear_block_bulk(ShootStringStartAddress, EEPROM_MAX_SIZE - ShootStringStartAddress);
    getShootString(0);
}

void saveSettings() {
    eeprom_write_array_bulk(SettingsStartAddress, &Settings, sizeof(Settings_t));
}

void restoreSettings() {
    eeprom_read_array(SettingsStartAddress, &Settings, sizeof(Settings_t));
}

void replaceParWithCustom(){
    Settings.TotPar = Settings.TotCustomPar;
    for (uint8_t i = 0; i < MAXPAR;i++){
        Settings.ParTime[i] = Settings.CustomPar[i];
    }
}
void getSettings(){
    restoreSettings();
    set_par_mode(Settings.ParMode);
}

void getDefaultSettings() {
    Settings.version = FW_VERSION;
    Settings.Sensitivity = DEFAULT_SENSITIVITY;
    Settings.Attenuator = ATTENUATOR_00_DBm;
    Settings.Filter = 80; // ms
    Settings.AR_IS.Autostart = On; // on
    Settings.AR_IS.AutoRotate = Off; // Off
    Settings.AR_IS.BT = Off; // Off by default
    Settings.AR_IS.AutoPowerOff = On; // ON by default
    Settings.AR_IS.Clock24h = On;     // 24h by default
    Settings.AR_IS.StartSound = On;  // ON by default
    Settings.InputType = INPUT_TYPE_Microphone;
    Settings.BuzzerFrequency = 2000; // Hz
    Settings.BuzzerParDuration = 300; // ms
    Settings.BuzzerStartDuration = 500; // ms
    Settings.Volume = 2; // Middle strength sound.
    Settings.CustomCDtime = 240; // 4 minutes in sec
    Settings.DelayMode = DELAY_MODE_Fixed;
    Settings.CUstomDelayTime = 2500; // ms before start signal
    Settings.BackLightLevel = 1; // Most dimmed visible
    Settings.ContrastValue = DEFAULT_CONTRAST_VALUE;
    Settings.TotPar = Off; // Par Off
    Settings.TotAutoPar = Off;
    Settings.TotCustomPar = Off;
    Settings.ParMode = ParMode_Regular;
    CurPar_idx = MAXPAR;
    while (CurPar_idx-- > 0) {
        Settings.ParTime[CurPar_idx] = 0.0;
        Settings.CustomPar[CurPar_idx] = 0.0;
        Settings.AutoPar[CurPar_idx].delay = 0.0;
        Settings.AutoPar[CurPar_idx].par = 0.0;
    }
    // TODO: Define proper defaults
    Settings.RepetitiveEdgeTime = 0;
    Settings.RepetitiveFaceTime = 0;
    Settings.RepetitiveRepeat = 1;
    repetitive_state = Face;
    repetitive_counter = 0;
    saveSettings();
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
    int offset = Settings.ParTime - (&Settings) + par_index;
    eeprom_write_array_bulk(SettingsStartAddress + offset, &Settings + offset, sizeof(float));
}

void storePar(){
    int offset = &(Settings.ParTime) - (&Settings);
    saveSettingsField(&Settings, &(Settings.TotPar), 1);
    eeprom_write_array_bulk(SettingsStartAddress + offset, &Settings + offset, sizeof(float) * MAXPAR); 
}

void restorePar() {
    int offset = Settings.ParTime - (&Settings);
    eeprom_read_array(SettingsStartAddress + offset, Settings.ParTime, sizeof(float) * MAXPAR);
    offset = (&(Settings.TotPar))-(&Settings);
    Settings.TotPar = eeprom_read_data(SettingsStartAddress + offset);
}

void storeCustom(){
    int offset = &(Settings.CustomPar) - (&Settings);
    saveSettingsField(&Settings,&(Settings.TotCustomPar),1);
    eeprom_write_array_bulk(SettingsStartAddress + offset, &Settings + offset, sizeof(float) * MAXPAR);   
}

void restoreCustom(){
    int offset = &(Settings.CustomPar) - (&Settings);
    eeprom_read_array(SettingsStartAddress + offset, &(Settings.CustomPar), sizeof(float) * MAXPAR);
    offset = (&(Settings.TotCustomPar))-(&Settings);
    Settings.TotCustomPar = eeprom_read_data(SettingsStartAddress + offset);    
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
    eeprom_write_data(addr, 0);// clear latest flag
    index++;
    if (index >= MAXSHOOTSTRINGS)
        index = 0;
    ShootString.latest = 1;
    addr = findStringAddress(index);
    eeprom_write_array_bulk(addr, &ShootString, Size_of_ShootString);
    eeprom_write_data(addr, 1); // Set latest flag
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
    ShootString_t * ss = (ui_state == TimerListening)?&ShootString:&ReviewString;
    for (uint8_t shot = 0; shot < ss -> TotShoots; shot++) {
        sendOneShot(&(ss -> shots[shot]));
        Delay(50);
    }
}

time_t last_sent_time = 0L;
void sendShotsIfRequired(){
    // Send shots only when in detection state
    if( ui_state != TimerListening ) return;
    update_rtc_time();
    if(unix_time_ms - last_sent_time < 50 ) return; // Don't send faster than once in 50ms
    uint8_t index_to_send = get_shot_index_in_arr(last_sent_index);
    uint8_t last_shot_index = get_shot_index_in_arr(ShootString.TotShoots);
    if(ShootString.TotShoots < MAX_REGISTERED_SHOTS && index_to_send != last_shot_index){
        sendOneShot(&(ShootString.shots[index_to_send]));
        last_sent_index++;
        last_sent_time = unix_time_ms;
        InputFlags.NEW_SHOT = False;
    } else if  (ShootString.TotShoots == MAX_REGISTERED_SHOTS && InputFlags.NEW_SHOT) {
        InputFlags.NEW_SHOT = False;
        sendOneShot(&(ShootString.shots[last_shot_index]));
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
    eeprom_read_array(addr, &ShootString, Size_of_ShootString);    
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
    return (t == 0);
}

void saveStats() {
    eeprom_write_array_bulk(StatsStartAddress, &Stats, sizeof(Stats_t));
}

void getStats() {
    eeprom_read_array(StatsStartAddress, &Stats, sizeof(Stats_t));
}

void saveStatsField(void * f, size_t l) {
    int offset = f - &Stats;
    eeprom_write_array_bulk(StatsStartAddress + offset, f, l);
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Helper functions">
void print_delay(char * str, const char * prefix, const char * postfix){
    switch (Settings.DelayMode) {
        case DELAY_MODE_Instant: sprintf(str, "%s0.00", prefix);
            break;
        case DELAY_MODE_Fixed: sprintf(str, "%s3.0%s", prefix, postfix);
            break;
        case DELAY_MODE_Random: sprintf(str, "%sRND", prefix);
            break;
        case DELAY_MODE_Custom: sprintf(str, "%s%1.1f%s", prefix, (float) (Settings.CUstomDelayTime) / 1000, postfix);
            break;
        case DELAY_MODE_Other: sprintf(str, "%s%1.1f%s", prefix, (float) (runtimeDelayTime) / 1000, postfix);
            break;
    }
}

uint8_t top_shot_index(){
    if(ShootString.TotShoots == 0) return 0;
    return ShootString.TotShoots - 1;
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
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Settings">
// <editor-fold defaultstate="collapsed" desc="Delay">

void SetCustomDelay() {
    NumberSelection_t n;
    strcpy(n.MenuTitle, "Custom Delay");
    InitSettingsNumberDefaults((&n));
    n.fmin = 0.1;
    n.fmax = 10.0;
    n.fstep = 0.1;
    n.fvalue = (float) Settings.CUstomDelayTime / 1000;
    n.fold_value = n.fvalue;
    n.format = " %2.1fs ";
    lcd_clear();
    do {
        DisplayDouble(&n);
        SelectDouble(&n);
    } while (SettingsNotDone((&n)));
    Settings.CUstomDelayTime = (time_t) (n.fvalue * 1000);
    if (n.fold_value != n.fvalue) {
        saveSettingsField(&Settings, &(Settings.CUstomDelayTime), 4);
    }
}

void SetDelay() {
    uint8_t oldValue = Settings.DelayMode;
    InitSettingsMenuDefaults((&ma));
    strcpy(ma.MenuTitle, "Delay");
    strcpy(ma.MenuItem[DELAY_MODE_Instant], "Instant");
    strcpy(ma.MenuItem[DELAY_MODE_Fixed], "Fixed|3.0 sec.");
    strcpy(ma.MenuItem[DELAY_MODE_Random], "Random");
    sprintf(ma.MenuItem[DELAY_MODE_Custom], "Custom|%3.1f sec.",((double)Settings.CUstomDelayTime)/1000);
    ma.TotalMenuItems = 4;
    ma.menu = Settings.DelayMode;

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
        if (ma.done && ma.selected && ma.menu == DELAY_MODE_Custom) {
            SetCustomDelay();
            lcd_clear();
            sprintf(ma.MenuItem[DELAY_MODE_Custom], "Custom|%3.1f sec.",((double)Settings.CUstomDelayTime)/1000);
            Settings.DelayMode = ma.menu;
            ma.changed = True;
            ma.done = False;
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

TBool EditPar(uint8_t par_index, float * pars) {
    NumberSelection_t b;
    b.fmin = 0.1;
    b.fmax = 99.9;
    b.fvalue = pars[par_index];
    b.fold_value = b.fvalue;
    sprintf(b.MenuTitle, "Par %d Settings ", par_index + 1);
    b.fstep = 0.05;
    b.format = "%3.2fs";
    b.done = False;
    lcd_clear();
    do {
        DisplayDouble(&b);
        SelectDouble(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected){
        pars[par_index] = b.fvalue;
    }
    return b.selected;
}

/**
 * 
 * @param m - Settings menu where lines should be set
 * @param i - First index of service items
 * @return 
 */
uint8_t setParMenuServiceItems(SettingsMenu_t * m, uint8_t i){
    if (i < MAXPAR)
        strncpy(m->MenuItem[i], "Add ", MAXItemLenght);
    else
        strncpy(m->MenuItem[i], "Max Par reached ", MAXItemLenght);
    strncpy(m->MenuItem[++i], "Delete Last ", MAXItemLenght);
    strncpy(m->MenuItem[++i], "Delete All ", MAXItemLenght);
    return ++i;
}

void FillParSettings(SettingsMenu_t * m, float * pars, uint8_t tot_par) {
    uint8_t i = 0;
    for (i = 0; i < tot_par; i++) {
        sprintf(m->MenuItem[i], "Par %d: %3.2fs  ", i + 1, pars[i]);
    }
    
    m->TotalMenuItems = setParMenuServiceItems(m, i);
}

uint8_t clear_par(float * pars, uint8_t tot_par) {
    while (0 < tot_par) {
        pars[--tot_par] = 0.0;
    }
    return tot_par;
}

uint8_t HandleParMenuSelection(SettingsMenu_t * m, float * pars, uint8_t tot_par) {
    if (m->menu < (m->TotalMenuItems - 3)) {
        EditPar(m->menu, pars);
    } else if (m->menu == (m->TotalMenuItems - 3)) {
        // Add new par
        if (tot_par < MAXPAR) {
            TBool res = False;
            pars[tot_par] = 1.0; // Default setting 1 second
            res = EditPar(tot_par++, pars);
            if (res) { // Roll back if not selected
                uint8_t oldPage = m->page;
                m->menu++;
                m->page = ItemToPage(m->menu);
                m->changed = True;
                m->page_changed = (oldPage != m->page);
            } else {
                pars[--tot_par] = 0.0;
            }
        } else {
            Beep();
        }
    } else if (m->menu == (m->TotalMenuItems - 2)) {
        // Delete last PAR
        if (tot_par > 0) {
            pars[tot_par--] = 0.0;
            uint8_t oldPage = m->page;
            m->menu--;
            m->page = ItemToPage(m->menu);
            m->changed = True;
            m->page_changed = (oldPage != m->page);
        }
    } else if (m->menu == (m->TotalMenuItems - 1)) {
        // Clear PAR
        tot_par = clear_par(pars,tot_par);
        m->menu = 0;
        m->page = 0;
        m->page_changed = True;
        m->changed = True;
    }
    lcd_clear();
    return tot_par;
}

uint8_t SetPar(SettingsMenu_t * m, float * pars, uint8_t tot_par) {
    InitSettingsMenuDefaults(m);
    strncpy(m->MenuTitle, "Par Settings ", MAXMenuTitleLength);
    do {
        FillParSettings(m, pars, tot_par);
        DisplaySettings(m);
        SelectBinaryMenuItem(m);
        if(m->selected){
            tot_par = HandleParMenuSelection(m, pars, tot_par);
            m->done = False;
            m->selected = False;
            m->changed = True;
        }
    } while (SettingsNotDone(m));
    return tot_par;
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

void SetContrast() {//PWM Backlight
    NumberSelection_t b;
    uint16_t tmpVal = Settings.ContrastValue;
    strcpy(b.MenuTitle, "Contrast");
    b.max = 0x0450;
    b.min = 0x0400;
    b.step = 1;
    b.value = tmpVal;
    b.old_value = b.value;
    b.format = "%u";
    b.done = False;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
        if(b.value - tmpVal){
            lcd_set_contrast(b.value);
            tmpVal = b.value;
        }
    } while (SettingsNotDone((&b)));

    if (b.selected && b.value != b.old_value) {
        Settings.ContrastValue = b.value;
        saveSettingsField(&Settings, &(Settings.ContrastValue), 2);
    } else {
        Settings.ContrastValue = b.old_value;
    }
    lcd_set_contrast(Settings.ContrastValue);
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
    b.max = 2400;
    b.value = Settings.BuzzerFrequency;
    b.old_value = b.value;
    tmp = b.value;
    strcpy(b.MenuTitle, "Set Frequency");
    b.step = 400;
    b.format = "%u";
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

void SetStartSound(){
    TBool orgset;
    InitSettingsMenuDefaults((&ma));
    ma.TotalMenuItems = 2;
    strcpy(ma.MenuTitle, "Startup sound");
    strcpy(ma.MenuItem[Off], " OFF ");
    strcpy(ma.MenuItem[On], " ON ");
    orgset = Settings.AR_IS.StartSound;
    ma.menu = Settings.AR_IS.StartSound;
    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    if (ma.selected) {
        Settings.AR_IS.StartSound = ma.menu;
        if (Settings.AR_IS.StartSound != orgset) {
            saveSettingsField(&Settings, &(Settings.AR_IS), 1);
        }
    }

}
void setBuzzerMenu(){
    sprintf(ma.MenuItem[0], " Frequency|%dHz ", Settings.BuzzerFrequency);
    sprintf(ma.MenuItem[1], " Volume|%d ", Settings.Volume);
    sprintf(ma.MenuItem[2], " Par Duration|%1.1fs ", (float) (Settings.BuzzerParDuration) / 1000);
    sprintf(ma.MenuItem[3], " Startup sound|%s ", Settings.AR_IS.StartSound?"ON":"OFF");
    strcpy(ma.MenuItem[4], " Test Beep ");
    ma.TotalMenuItems = 5;
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
                    SetStartSound();
                    break;
                case 4:
                    generate_sinus(Settings.Volume, Settings.BuzzerFrequency, Settings.BuzzerStartDuration);
                    break;
            }
            // Here we want it done only when back pressed
            // i.e. not selected and done
            ma.done = False;
            ma.selected = False;
            ma.changed = True;
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
    s.max = 800;
    s.min = 5;
    s.value = Settings.Sensitivity;
    s.old_value = Settings.Sensitivity;
    s.step = 5;
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

void SetAtt() {
    NumberSelection_t s;
    InitSettingsNumberDefaults((&s));
    strcpy(s.MenuTitle, "Set Attenuator");
    s.max = 3;
    s.min = 0;
    s.value = Settings.Attenuator;
    s.old_value = Settings.Attenuator;
    s.step = 1;
    s.format = "%d";
    do {
        DisplayInteger(&s);
        SelectInteger(&s);
    } while (SettingsNotDone((&s)));
    if (s.selected) {
        Settings.Attenuator = s.value;
        if (s.value != s.old_value) {
            saveSettingsField(&Settings, &(Settings.Attenuator), 1);
        }
    }
}
void SetMicSource() {
    InitSettingsMenuDefaults((&mx));
    strncpy(mx.MenuTitle, "Set Mic Source", MAXItemLenght);
    strncpy(mx.MenuItem[0],"Envelope", MAXItemLenght);
    strncpy(mx.MenuItem[1],"Microphone", MAXItemLenght);
    mx.TotalMenuItems = 2;
    mx.menu = Settings.AR_IS.MIC_SRC;
    do {
        DisplaySettings((&mx));
        SelectMenuItem((&mx));
    } while (SettingsNotDone((&mx)));
    if (mx.selected) {
        if (mx.menu != Settings.AR_IS.MIC_SRC) {
            Settings.AR_IS.MIC_SRC = mx.menu;
            saveSettingsField(&Settings, &(Settings.AR_IS), 1);
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
        case ParMode_Spy:
            Settings.DelayMode = DELAY_MODE_Instant;
            Settings.Volume = 0;
            break;
        case ParMode_Regular:
            restoreSettingsField(&Settings, &(Settings.DelayMode), 1);
            restorePar();
            CurPar_idx = 0;
            break;
        case ParMode_CUSTOM:
            replaceParWithCustom();
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
        case ParMode_Repetitive:
        case ParMode_AutoPar:
        default:
            // do nothing
            break;
    }
}

const char * par_mode_menu_names[TOT_PAR_MODES] = {
    "Normal Timer",
    "Spy Mode",
    "Repetitive Mode",
    "Auto Par",
    "Bianchi: Practical",
    "Bianchi: Barricade",
    "Bianchi: Falling Plate",
    "Bianchi: Moving Targ.",
    "NRA-PPC: A",
    "NRA-PPC: B",
    "NRA-PPC: C",
    "NRA-PPC: D",
    "Custom"
};

const char * par_mode_header_names[TOT_PAR_MODES] = {
    "Timer",
    "Spy Mode",
    "Repetitive",
    "Auto Par",
    "Practical",
    "Barricade",
    "Falling Plate",
    "Moving Targ",
    "NRA-PPC A",
    "NRA-PPC B",
    "NRA-PPC C",
    "NRA-PPC D",
    "Custom"
};
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Repetitive Mode">
void SetFaceTime() {
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b));
    strcpy(b.MenuTitle, "Set Face");
    b.fmin = 0.0;
    b.fmax = 99.9;
    b.fstep = 0.1;
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
    b.fmin = 0.0;
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
    sprintf(mx.MenuItem[FACE_IDX],"Face|%1.2fs", (float) Settings.RepetitiveFaceTime / 1000);
    sprintf(mx.MenuItem[EDGE_IDX],"Edge|%1.2fs", (float) Settings.RepetitiveEdgeTime / 1000);
    sprintf(mx.MenuItem[REPEAT_IDX],"Repeat|%d", Settings.RepetitiveRepeat);
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
            }
            // Here we want it done only when back pressed
            // i.e. not selected and done
            mx.done = False;
            mx.selected = False;
            mx.changed = True;
            lcd_clear();
        }
    } while (SettingsNotDone((&mx)));
    if(Settings.RepetitiveRepeat == 0){
        Settings.ParMode = ParMode_Regular;
    }
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Auto Par">

TBool EditDelay(uint8_t index, AutoPar_t * pars){
    NumberSelection_t m;
    InitSettingsNumberDefaults((&m));
    sprintf(m.MenuTitle, "Delay %u ", index + 1);
    m.fmin = 0.5;
    m.fmax = 99.9;
    m.fstep = 0.1;
    m.fvalue = pars[index].delay;
    m.fold_value = m.value;
    m.format = "%1.1fs";
    do {
        DisplayDouble((&m));
        SelectDouble((&m));
    } while (SettingsNotDone((&m)));
    if(m.selected) pars[index].delay = m.fvalue;
    return m.selected;
}

TBool EditParTime(uint8_t index, AutoPar_t * pars){
    NumberSelection_t m;
    InitSettingsNumberDefaults((&m));
    sprintf(m.MenuTitle, "Par %u ", index + 1);
    m.fmin = 0.0;
    m.fmax = 99.99;
    m.fstep = 0.05;
    m.fvalue = pars[index].par;
    m.fold_value = m.value;
    m.format = "%1.2fs";
    do {
        DisplayDouble((&m));
        SelectDouble((&m));
    } while (SettingsNotDone((&m)));
    if(m.selected) pars[index].par = m.fvalue;
    return m.selected;    
}

uint8_t EditAutoPar(uint8_t i, AutoPar_t * pars){
    if(EditDelay(i,pars)){
        lcd_clear();
        return EditParTime(i, pars);
    }
    return False;
}

void setAutoParMenu(){
    strcpy(mx.MenuTitle, "Auto Par ");
    mx.TotalMenuItems = Settings.TotAutoPar;
    for(uint8_t i = 0; i < mx.TotalMenuItems; i++){
        sprintf(mx.MenuItem[i],"%u:Delay %1.1fs|Par %1.2fs",
                i + 1,
                Settings.AutoPar[i].delay,
                Settings.AutoPar[i].par
                );
    }
    mx.TotalMenuItems = setParMenuServiceItems(&mx,mx.TotalMenuItems);
}

uint8_t HandleAutoParMenuSelection(SettingsMenu_t * m) {
    uint8_t tot_par = Settings.TotAutoPar;
    AutoPar_t * pars = Settings.AutoPar;
//    if(m->done) return;
    if (m->selected) {
        m->selected = False;
        m->changed = True;
        if (m->menu < (m->TotalMenuItems - 3)) {
            EditAutoPar(m->menu, pars);
        } else if (m->menu == (m->TotalMenuItems - 3)) {
            // Add new par
            if (tot_par < MAXPAR) {
                pars[tot_par].delay = 3.0;
                pars[tot_par].par = 1.0;
                if (EditAutoPar(tot_par, pars)) { // Roll back if not selected
                    uint8_t oldPage = m->page;
                    tot_par++;
                    m->menu++;
                    m->page = ItemToPage(m->menu);
                    m->changed = True;
                    m->page_changed = (oldPage != m->page);
                } else {
                    pars[tot_par].delay = 0.0;
                    pars[tot_par].par = 0.0;
                }
            } else {
                Beep();
            }
        } else if (m->menu == (m->TotalMenuItems - 2)) {
            // Delete last PAR
            if (tot_par-- > 0) {
                pars[tot_par].delay = 0.0;
                pars[tot_par].par = 0.0;
                uint8_t oldPage = m->page;
                m->menu--;
                m->page = ItemToPage(m->menu);
                m->changed = True;
                m->page_changed = (oldPage != m->page);
            }
        } else if (m->menu == (m->TotalMenuItems - 1)) {
            // Clear PAR
            while (0 < --tot_par) {
                pars[tot_par].delay = 0.0;
                pars[tot_par].par = 0.0;
            }
            m->menu = 0;
            m->page = 0;
            m->page_changed = True; 
            m->changed = True;
        }
        lcd_clear();
    }
    return tot_par;
}

void SetAutoPar(){
    InitSettingsMenuDefaults((&mx));
    do {
        setAutoParMenu();
        DisplaySettings((&mx));
        SelectMenuItem((&mx));
        if(mx.selected){
            lcd_clear();
            Settings.TotAutoPar = HandleAutoParMenuSelection(&mx);
            mx.selected = False;
            mx.done = False;
            lcd_clear();
        }
    } while (SettingsNotDone((&mx)));
    if(Settings.TotAutoPar == 0){
        // User didn't define any PARs or cleared them
        Settings.ParMode = ParMode_Regular;
        Settings.TotPar = 0;
        clear_par(Settings.ParTime,MAXPAR);
    }
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
        if(ma.selected){
            Settings.ParMode = ma.menu;
            saveSettingsField(&Settings, &(Settings.ParMode), 1);
            switch (ma.menu){
                case ParMode_Regular:
                case ParMode_Spy:
                    getSettings();
                    switch(Settings.ParMode){
                        case ParMode_Regular:
                        case ParMode_Spy:
                            break;
                        default:
                            clear_par(Settings.ParTime,MAXPAR);
                            Settings.TotPar = 0;
                            saveSettings();
                            break;
                    }
                    break;
                case ParMode_CUSTOM:
                    lcd_clear();
                    if(Settings.ParMode != ParMode_CUSTOM){
                        clear_par(Settings.CustomPar,MAXPAR);
                        Settings.TotCustomPar = 0;
                    }
                    Settings.TotCustomPar = SetPar(&mx, Settings.CustomPar, Settings.TotCustomPar);
                    saveSettings();
                    replaceParWithCustom();
                    break;
                case ParMode_Repetitive:
                    lcd_clear();
                    SetRepetitiveMode();
                    saveSettings();
                    break;
                case ParMode_AutoPar:
                    lcd_clear();
                    if(Settings.ParMode != ParMode_AutoPar){
                        uint8_t tot_par = MAXPAR;
                        while (0 < --tot_par) {
                          Settings.AutoPar[tot_par].delay = 0.0;
                          Settings.AutoPar[tot_par].par = 0.0;
                        }
                        Settings.TotAutoPar = 0;
                    }
                    SetAutoPar();
                    saveSettings();
                    break;
            }
            ma.changed = True;
        }
    } while (SettingsNotDone((&ma)));
    if (ma.selected) {
        STATE_HANDLE_TIMER_IDLE();
    }
    set_par_mode(Settings.ParMode);
    Stats.Modes[Settings.ParMode]++;
    CurPar_idx = 0;
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Clock">

void SetClock() {
    NumberSelection_t ts;
    uint8_t h = _hour, m = _minute;
    InitSettingsNumberDefaults((&ts));
    ts.min = 0;
    ts.max = 23;
    ts.value = h;
    ts.old_value = ts.value;
    ts.step = 1;
    strcpy(ts.MenuTitle, "Set Clock");
    ts.state = 0; // 0 - hour, 1 - Minute. DisplayTime knows to handle this
    set_screen_title(ts.MenuTitle);
    DisplayTime(ts.value, m, ts.state);
    do {
        if(ts.redraw){
            ts.redraw = False;
            DisplayTime(ts.value, m, ts.state);
        }
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
            if(ts.redraw){
                ts.redraw = False;
                DisplayTime(h, ts.value, ts.state);
            }
            SelectIntegerCircular(&ts);
        } while (SettingsNotDone((&ts)));
    }
    if (ts.selected && (h != _hour || ts.value != m)) {
        set_time(h, ts.value);
    }
}

void SetClockMode(){
    NumberSelection_t ts;
    InitSettingsNumberDefaults((&ts));
    strcpy(ts.MenuTitle, "Set Clock Format");
    ts.format = "%uh";
    ts.max = 24;
    ts.min = 12;
    ts.step = 12;
    ts.value = Settings.AR_IS.Clock24h? 24: 12;
    ts.old_value = ts.value;
    do {
        DisplayInteger((&ts));
        SelectInteger((&ts));
    } while (SettingsNotDone((&ts)));
    if(ts.value != ts.old_value){
        Settings.AR_IS.Clock24h = (ts.value==24);
        saveSettingsField(&Settings,&(Settings.AR_IS),1);
    }
}
void SetHour(){
    NumberSelection_t ts;
    InitSettingsNumberDefaults((&ts));
    strcpy(ts.MenuTitle,"Set Hour");
    ts.format = "%02u";
    ts.max = 23;
    ts.min = 0;
    ts.step = 1;
    ts.value = _hour;
    ts.old_value = ts.value;
    do {
        DisplayInteger((&ts));
        SelectIntegerCircular((&ts));
    } while (SettingsNotDone((&ts)));
    if(ts.selected) _hour = ts.value;
}

void SetMinute(){
    NumberSelection_t ts;
    InitSettingsNumberDefaults((&ts));
    strcpy(ts.MenuTitle,"Set Minute");
    ts.format = "%02u";
    ts.max = 59;
    ts.min = 0;
    ts.step = 1;
    ts.value = _minute;
    ts.old_value = ts.value;
    do {
        DisplayInteger((&ts));
        SelectIntegerCircular((&ts));
    } while (SettingsNotDone((&ts)));
    if(ts.selected) _minute = ts.value;
}

void SetClockMenuItems(){
    sprintf(ma.MenuItem[0], "Clock Format|%uh",Settings.AR_IS.Clock24h?24:12);
//    sprintf(ma.MenuItem[1], "Hour|%02u",get_hour(Settings.AR_IS.Clock24h));
//    sprintf(ma.MenuItem[2], "Minute|%02u",get_minute());
    sprintf(ma.MenuItem[1], "Clock|");
    rtc_print_time((ma.MenuItem[1] + 6),Settings.AR_IS.Clock24h);
    ma.TotalMenuItems = 2;
}
void SetClockMenu(){
    InitSettingsMenuDefaults((&ma));

    strcpy(ma.MenuTitle, "Set Clock");
    SetClockMenuItems();
    //Main Screen
    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
        if(ma.selected && ma.done){
            lcd_clear();
            switch (ma.menu){
                case 0:
                    SetClockMode();
                    break;
//                case 1:
//                    SetHour();
//                    break;
//                case 2:
//                    SetMinute();
//                    break;
                case 1:
                    SetClock();
                    break;
            }
            lcd_clear();
            ma.changed = True;
            ma.selected = False;
            ma.done = False;
            SetClockMenuItems();
        }
    } while (SettingsNotDone((&ma)));
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="CountDown">

uint8_t countdown_expired_signal() {
    for (uint8_t i = 0; i < 5; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            generate_sinus(
                    Settings.Volume,
                    Settings.BuzzerFrequency,
                    50
                    );
            Delay(100);
        }
        if (Keypressed)
            return False;
        Delay(400);
    }
    return True;
}

void CountDownMode(time_t countdown) {
    char msg[16];
    time_t reminder = countdown * 1000;
    time_t stop_time = unix_time_ms + reminder + 1;
    uint8_t minute, second;
    TBool done = False;
    lcd_clear();
    do {
        print_header(true);
        update_rtc_time();
        reminder = (stop_time - unix_time_ms) / 1000;
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
    }
    STATE_HANDLE_TIMER_IDLE();
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
    strcpy(ma.MenuItem[0], "3 minutes");
    strcpy(ma.MenuItem[1], "5 minutes");
    strcpy(ma.MenuItem[2], "Custom");

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
    InitSettingsMenuDefaults((&mx));
    mx.TotalMenuItems = 2;
    strcpy(mx.MenuTitle, "Orientation");
    strcpy(mx.MenuItem[ORIENTATION_NORMAL], "Upright");
    strcpy(mx.MenuItem[ORIENTATION_INVERTED], "Upside-down");
    mx.menu = Orientation;

    do {
        DisplaySettings((&mx));
        SelectMenuItem((&mx));
    } while (SettingsNotDone((&mx)));
    if (mx.selected && Orientation != mx.menu) {
        Orientation = mx.menu;
        saveSettingsField(&Settings, &(Settings.AR_IS), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Input">

void SetInput() {
    uint8_t orgset;
    InitSettingsMenuDefaults((&ma));
    strcpy(ma.MenuTitle, "Select Source");
    strcpy(ma.MenuItem[INPUT_TYPE_Microphone], "Microphone");
    strcpy(ma.MenuItem[INPUT_TYPE_A_or_B_multiple], "A or B (multiple)");
    strcpy(ma.MenuItem[INPUT_TYPE_A_and_B_single], "A and B (single)");
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
    Stats.InputModes[Settings.InputType]++;
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="BlueTooth">

void init_bt() {
    init_uart();
    BT_init();
    if (!Settings.AR_IS.BT) {
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
        lcd_clear();
        lcd_write_string("Please wait", UI_CHARGING_LBL_X - 20, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
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
            float par_time_f = (float) par_time / 1000;
            Settings.ParTime[par_idx - 1] = par_time_f;
            Settings.TotPar = par_idx;
            savePar(par_idx);
            saveSettingsField(&Settings, &(Settings.TotPar), 1);
            Stats.Menu[SETTINGS_INDEX_PAR]++;
            saveStats();
            DAA_MSG_OK;
        } else {
            DAA_MSG_ERROR;
        }
    } else if (par_idx == 0) {
        clear_par(Settings.ParTime,Settings.TotPar);
        DAA_MSG_OK;
    } else {
        DAA_MSG_ERROR;
    }
}

void bt_set_custom() {
    long par_idx = 0;
    long par_time = 0;
    char * endp[1];
    par_idx = strtol(bt_cmd_args_raw, endp, 10);

    if (par_idx > 0 && par_idx <= MAXPAR) {
        par_time = strtol(*endp + 1, endp, 10);
        // Par time between 1ms and 99000ms
        if (par_time > 0 && par_time < 99901) {
            float par_time_f = (float) par_time / 1000;
            Settings.CustomPar[par_idx - 1] = par_time_f;
            Settings.TotCustomPar = par_idx;
            storeCustom();
            Stats.Menu[SETTINGS_INDEX_PAR]++;
            saveStats();
            DAA_MSG_OK;
        } else {
            DAA_MSG_ERROR;
        }
    } else if (par_idx == 0) {
        clear_par(Settings.ParTime,Settings.TotPar);
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
        Stats.Modes[mode]++;
        saveStats();
        DAA_MSG_OK;
        
    } else {
        DAA_MSG_ERROR;
    }
}

void bt_set_delay() {
    int time = 0;
    time = atoi(bt_cmd_args_raw);
    if (time > -1 && time < 10001) {
        Settings.DelayMode = DELAY_MODE_Custom;
        Settings.CUstomDelayTime = time;
        saveSettingsField(&Settings, &(Settings.CUstomDelayTime), 4);
        saveSettingsField(&Settings, &(Settings.DelayMode), 1);
        DAA_MSG_OK;
    } else {
        DAA_MSG_ERROR;
    }
}

void bt_get_custom(){
    uint8_t length = 0;
    char msg[16];
    if (Settings.TotCustomPar > 0) {
        for (uint8_t i = 0; i < Settings.TotCustomPar; i++) {
            length = sprintf(msg, "%d,%u\n", i + 1, (long)(Settings.CustomPar[i] * 1000));
            sendString(msg, length);
            Delay(50);
        }
    } else {
        DAA_MSG_EMPTY;
    }
}

void bt_get_pars() {
    uint8_t length = 0;
    char msg[16];
    if (Settings.TotPar > 0) {
        for (uint8_t i = 0; i < Settings.TotPar; i++) {
            length = sprintf(msg, "%d,%u\n", i + 1, (long)(Settings.ParTime[i] * 1000));
            sendString(msg, length);
            Delay(50);
        }
    } else {
        DAA_MSG_EMPTY;
    }
}

void handle_bt_commands() {
    uint8_t length = 0;
    char msg[20];
    sendShotsIfRequired();
    BT_COMMAND_T btc = BT_define_action();
    switch (btc) {
        case BT_SendVersion:
            length = sprintf(msg, "%u\n", Settings.version);
            sendString(msg, length);
            break;
        case BT_StartTimer:
            STATE_HANDLE_COUNTDOWN();
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
        case BT_SetCustom:
                bt_set_custom();
            break;
        case BT_GetCustomSequence:
            bt_get_custom();
        case BT_SetMode:
            bt_set_mode();
            set_par_mode(Settings.ParMode);
            break;
        case BT_ClearHistory:
            DAA_MSG_WAIT;
            clearHistory();
            lcd_clear();
            DAA_MSG_OK;
            break;
        case BT_DefaultSettings:
            getDefaultSettings();
            DAA_MSG_OK;
            break;
        case BT_Find:
            DAA_MSG_LISTEN;
            if(countdown_expired_signal()){
                DAA_MSG_OK;
            } else {
                DAA_MSG_ERROR;
            }
            break;
        case BT_GetStats:
            length = sprintf(msg, "ID,%12s\n",mac_addr);
            sendString(msg, length);
            length = sprintf(msg,"PowerOn,%u\n",Stats.PowerOn);
            Delay(50);
            sendString(msg, length);
            length = sprintf(msg,"Signal,%u\n",Stats.Signal);
            Delay(50);
            sendString(msg, length);
            length = sprintf(msg,"Review,%u\n",Stats.Review);
            Delay(50);
            sendString(msg, length);
            length = sprintf(msg,"Settings,%u\n",Stats.Settings);
            Delay(50);
            sendString(msg, length);
            length = sprintf(msg,"Charging,%u\n",Stats.Charging);
            Delay(50);
            sendString(msg, length);
            length = sprintf(msg,"Charged,%u\n",Stats.Charged);
            Delay(50);
            sendString(msg, length);
            length = sprintf(msg,"LowPower,%u\n",Stats.LowPower);
            Delay(50);
            sendString(msg, length);
            for(uint8_t i=0;i<3;i++){
                length = sprintf(msg,"Input-%u,%u\n",i,Stats.InputModes[i]);
                Delay(50);
                sendString(msg, length);
            }
            for(uint8_t i=0;i<TOT_PAR_MODES;i++){
                length = sprintf(msg,"Mode-%u,%u\n",i,Stats.Modes[i]);
                Delay(50);
                sendString(msg, length);
            }
            for(uint8_t i=0;i<MAXMenuItems;i++){
                length = sprintf(msg,"Menu-%u,%u\n",i,Stats.Menu[i]);
                Delay(50);
                sendString(msg, length);
            }
            break;
        case BT_GetPars:
            bt_get_pars();
            break;
        case BT_SetDelay:
            bt_set_delay();
            break;
        case BT_GetBatteryMV:
            length = sprintf(msg,"%u", battery_average());
            sendString(msg, length);
            break;
        case BT_None:
            break;
        default:
            DAA_MSG_NOT_SUPPORTED;
            break;
    }
    // Don't let the timer sleep if it's actively used remotely
    if(btc != BT_None)
        timer_idle_last_action_time = unix_time_ms_sec;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Microphone">
void fillMicrophoneMenu(){
    ma.TotalMenuItems = 4;
    strcpy(ma.MenuTitle, " Microphone ");
    sprintf(ma.MenuItem[0], "Sensitivity|%d", Settings.Sensitivity);
    sprintf(ma.MenuItem[1],  "Filter|%1.2fs", (float) (Settings.Filter) / 1000);
    sprintf(ma.MenuItem[2],  "Attenuator|%d", Settings.Attenuator);
    sprintf(ma.MenuItem[3],  "Mic Source|%s", Settings.AR_IS.MIC_SRC?"MIC":"ENV");
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
                    SetAtt();
                    break;
                case 3:
                    SetMicSource();
                    break;
            }
            fillMicrophoneMenu();
            lcd_clear();
            ma.done = False;
            ma.selected = False;
            ma.changed = True;
        }
    } while (SettingsNotDone((&ma)));
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Display">
void SetDisplayMenu(){
    ma.TotalMenuItems = 2;
    sprintf(ma.MenuTitle, "Set Display ");
    sprintf(ma.MenuItem[0], "Backlight|%u", Settings.BackLightLevel);
    sprintf(ma.MenuItem[1],  "Orientation|%s", Orientation?"Down":"Up");
//    sprintf(ma.MenuItem[2],  "Contrast|%u", Settings.ContrastValue);
}

void SetDisplay(){
    InitSettingsMenuDefaults((&ma));
    ma.menu = 0;
    SetDisplayMenu();

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
        if(ma.selected){
            lcd_clear();
            switch (ma.menu){
                case 0:
                    SetBacklight();
                    ma.done = False;
                    ma.selected = False;
                    break;
                case 1:
                    SetOrientation();
                    ma.done = False;
                    ma.selected = False;
                    break;
//                case 2:
//                    SetContrast();
//                    ma.done = False;
//                    ma.selected = False;
//                    break;
            }
            lcd_clear();
            ma.changed = True;
            SetDisplayMenu();
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
    strcpy(ma.MenuItem[SMTH_DISABLED], " Yes");
    strcpy(ma.MenuItem[SMTH_ENABLED],  " No");

    do {
        DisplaySettings((&ma));
        SelectMenuItem((&ma));
    } while (SettingsNotDone((&ma)));
    return (ma.menu == SMTH_DISABLED);
}

void DoSet(uint8_t menu) {
    Stats.Menu[menu]++;
    lcd_clear();
    switch (menu) {
        case SETTINGS_INDEX_DELAY:
            SetDelay();
            break;
        case SETTINGS_INDEX_PAR:
            switch(Settings.ParMode){
                case ParMode_CUSTOM:
                    Settings.TotCustomPar = SetPar((&ma), Settings.CustomPar, Settings.TotCustomPar);
                    saveSettings();
                    replaceParWithCustom();// I know it's kind'a hacky but don't have time for different par mechanizms
                    break;
                case ParMode_AutoPar:
                    SetAutoPar();
                    saveSettings();
                    break;
                case ParMode_Repetitive:
                    SetRepetitiveMode();
                    saveSettings();
                    break;
                default:
                    Settings.TotPar = SetPar((&ma), Settings.ParTime, Settings.TotPar); // By reference because it's used both in 2nd and 3rd level menu
                    saveSettings();
                break;
            }
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
        case SETTINDS_INDEX_CLOCK:SetClockMenu();
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
#define ParMenuPattern "Par|%d 1st: %3.2f" 
void SetSettingsMenu() {
    sprintf(SettingsMenu.MenuTitle, "Settings ");

    print_delay(SettingsMenu.MenuItem[SETTINGS_INDEX_DELAY], "Delay|"," Sec.");

    switch(Settings.ParMode){
        case ParMode_AutoPar:
            if (Settings.TotAutoPar > 0) {
                sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_PAR],
                ParMenuPattern,
                Settings.TotAutoPar,
                Settings.AutoPar[0].par);
            } else {
                sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_PAR], "Par|Off");
            }
            break;
        case ParMode_CUSTOM:
            if (Settings.TotCustomPar > 0) {
                sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_PAR],
                ParMenuPattern,
                Settings.TotCustomPar,
                Settings.CustomPar[0]);
            } else {
                sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_PAR], "Par|Off");
            }
            break;
        case ParMode_Repetitive:
            sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_PAR],
                "Par| %0.1fs %0.1fs %d",
                (float)Settings.RepetitiveFaceTime / 1000,
                (float)Settings.RepetitiveEdgeTime / 1000,
                Settings.RepetitiveRepeat);
            break;
        default:
            if (Settings.TotPar > 0) {
                sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_PAR],
                ParMenuPattern,
                Settings.TotPar,
                Settings.ParTime[0]);
            } else {
                sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_PAR], "Par|Off");
            }
            break;
    }

    sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_BUZZER], "Buzzer|%d %dHz",
            Settings.Volume, Settings.BuzzerFrequency);
    sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_MIC], "Microphone|%d %0.2f",
            Settings.Sensitivity, (float) Settings.Filter/1000);
    sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_MODE], "Mode|%s",
            par_mode_header_names[Settings.ParMode]);
    sprintf(SettingsMenu.MenuItem[SETTINGS_INDEX_DISPLAY], "Display|%s %u",
            Orientation ? "Down" : "Up",
            Settings.BackLightLevel);
    sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_COUNTDOWN], "Countdown");
    sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_AUTOSTART], "Auto Start|%s",
            (Settings.AR_IS.Autostart)?"ON":"OFF");
    strcpy(SettingsMenu.MenuItem[SETTINDS_INDEX_CLOCK], "Clock|");
    rtc_print_time((SettingsMenu.MenuItem[SETTINDS_INDEX_CLOCK] + 6), Settings.AR_IS.Clock24h);
    switch (Settings.InputType) {
        case INPUT_TYPE_Microphone:
            sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_INPUT], "Input|Microphone");
            break;
        case INPUT_TYPE_A_and_B_single:
            sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_INPUT], "Input|A+B single");
            break;
        case INPUT_TYPE_A_or_B_multiple:
            sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_INPUT], "Input|A or B multi");
            break;
        default:
            sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_INPUT], "Input");
            break;
    }

    sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_BLUETOOTH], "Bluetooth|%s",
            (Settings.AR_IS.BT)?"ON":"OFF");
    sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_AUTO_POWER], "Auto Power-off|%s",
            (Settings.AR_IS.AutoPowerOff)?"ON":"OFF");
    sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_CLEAR], "Clear History");
    sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_RESET], "Reset Settings");
    sprintf(SettingsMenu.MenuItem[SETTINDS_INDEX_VERSION], "Device ID|%6s-%02u", device_id, Settings.version);
    SettingsMenu.TotalMenuItems = 15;
}

void DoSettings(void) {
    InitSettingsMenuDefaults((&SettingsMenu));
    getSettings(); // Edit the copy from the EEPROM because of manipulations with PAR time in custom mode
    Stats.Settings++;
    SetSettingsMenu();
    lcd_clear();
    do {
        DisplaySettings(&SettingsMenu);
        SelectMenuItemCircular(&SettingsMenu);
        if (SettingsMenu.selected) {
            DoSet(SettingsMenu.menu);
            lcd_clear();
            SetSettingsMenu();
            SettingsMenu.selected = False;
            SettingsMenu.changed = True;
        }
        // Never exit on OK/Cancel case here, only on screen change
        SettingsMenu.done = False;
    } while (SettingsNotDone((&SettingsMenu)));

    if (ui_state == SettingsScreen) {
        STATE_HANDLE_TIMER_IDLE();
    } else {
        lcd_clear();
    }
    saveStats();
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
    FONT_INFO * font;
    // We're assuming here that Medium font has even number of bytes heigh
    if (!reviewChanged) return;
    uint8_t totShots = ReviewString.TotShoots;
    lcd_clear();
    // assuming ReviewString and ShootString differ only by the shot order in memory
    last_shot_index = ReviewTopShotDefault;
    // Stat line
    sprintf(ScreenTitle,
            REVIEW_TOTAL_SHOT_FORMAT,
            ReviewString.TotShoots,
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
                    ||(ReviewString.shots[curr_index].dt > 99700)
                    || ReviewString.shots[curr_index].sn > 99)
                font = SmallFont;
            else
                font = MediumFont;
            uint8_t x_pos = lcd_write_string(message, 0, line, font, (i != 1));
            lcd_send_block_d(x_pos, line, 134, line + font->height, (i == 1));
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
        getShootString(CurShootString);
        if (ReviewString.TotShoots == 0) {
            Beep();
            CurShootString++;
            getShootString(CurShootString);
        }
    } else {
        CurShootString = MAXSHOOTSTRINGS - 1;
        getShootString(CurShootString);
        if (ReviewString.TotShoots == 0) {
            Beep();
            CurShootString = 0;
            getShootString(CurShootString);
        }
    }
    getShootString(CurShootString);
    reviewChanged = True;
    TopShotIndex = ReviewTopShotDefault;
}

void review_next_string() {
    if (CurShootString < MAXSHOOTSTRINGS - 1) {
        CurShootString++;
        getShootString(CurShootString);
        if (ReviewString.TotShoots == 0) {
            Beep();
            CurShootString--;
            getShootString(CurShootString);
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
    if (ReviewString.TotShoots == 0) {
        Beep();
        STATE_HANDLE_TIMER_IDLE();
        return;
    }
    Stats.Review++;
    saveStatsField(&(Stats.Review), 2);
    reviewChanged = True;
    do {
        ReviewDisplay();
        define_input_action();
        handle_bt_commands();
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
            case ChargerConnected:
                STATE_HANDLE_CHARGER();
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
            if(Settings.AR_IS.MIC_SRC) shot_detection_source = MICROPHONE;
            else shot_detection_source = ENVELOPE;
            for (uint8_t i = 0; i < 64; i++) {
                ADCvalue = ADC_Read(shot_detection_source);
                Mean += ADCvalue;
                if (Max < ADCvalue) Max = ADCvalue;
            }
            Mean = Mean >> 6;
//            DetectThreshold = Mean + threshold_offsets[Settings.Sensitivity - 1];
            DetectThreshold = Mean - Settings.Sensitivity;
//            DetectThreshold = Settings.Sensitivity;
            SetAttenuator(Settings.Attenuator);
            ADC_ENABLE_INTERRUPT_SHOT_DETECTION;
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
        rtc_print_time(message, Settings.AR_IS.Clock24h);
        title_pos = lcd_write_string(message, 1, 0, SmallFont, BLACK_OVER_WHITE);
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
    if (Settings.AR_IS.BT) {
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

    switch(Settings.ParMode){
        case ParMode_Repetitive:
            print_delay(message," Delay: ","");
            print_label_at_footer_grid(message, 0, 1);
            if (ui_state == TimerIdle){
                sprintf(message,"Face%d:%3.1f",
                    1,
                    (float) Settings.RepetitiveFaceTime / 1000);
            } else if(repetitive_counter < Settings.RepetitiveRepeat){
                sprintf(message,"%s%d:%3.1f",
                    (repetitive_state==Face)?"Face":"Edge",
                    repetitive_counter + 1,
                    (float) next_par_ms / 1000);
            } else {
                sprintf(message,"%s%d:%3.1f",
                    "Edge",
                    Settings.RepetitiveRepeat,
                    (float) Settings.RepetitiveEdgeTime / 1000);
            }
            break;
        case ParMode_AutoPar:
            if (Settings.TotAutoPar == 0) {
                sprintf(message,"Delay 0: %1.1f", Settings.AutoPar[0].delay);
                print_label_at_footer_grid(message, 0, 1);
                sprintf(message, "Par: Off");
            } else if(CurPar_idx == Settings.TotAutoPar){
                sprintf(message,"Delay %d: %1.1f", CurPar_idx, Settings.AutoPar[CurPar_idx - 1].delay);
                print_label_at_footer_grid(message, 0, 1);
                sprintf(message, "Par%2d:%3.2f", CurPar_idx, Settings.AutoPar[CurPar_idx - 1].par);
            } else {
                sprintf(message,"Delay %d: %1.1f", CurPar_idx + 1, Settings.AutoPar[CurPar_idx].delay);
                print_label_at_footer_grid(message, 0, 1);
                sprintf(message, "Par%2d:%3.2f", CurPar_idx + 1, Settings.AutoPar[CurPar_idx].par);
            }
            break;
        default:
            print_delay(message," Delay: ","");
            print_label_at_footer_grid(message, 0, 1);
            if (Settings.TotPar == 0) {
                sprintf(message, "Par: Off");
            } else if(CurPar_idx == Settings.TotPar){
                sprintf(message, "Par%2d:%3.2f", CurPar_idx, Settings.ParTime[CurPar_idx - 1]);
            } else {
                sprintf(message, "Par%2d:%3.2f", CurPar_idx + 1, Settings.ParTime[CurPar_idx]);
            }
        break;
    }
//    sprintf(message, "%u", PORTD&0x7);
//    sprintf(message, "%u", ADC_LATEST_VALUE);
    print_label_at_footer_grid(message, 1, 1);
}

void StartListenShots(void) {
    ShootString_start_time = unix_time_ms;
    last_sent_index = 0;
    DetectInit();
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Power functions">
void PowerOffSound(){
    if(Settings.AR_IS.StartSound == Off) return;
    generate_sinus(1,1260,80);
    Delay(100);
    generate_sinus(1,960,80);
    Delay(100);
    generate_sinus(1,880,300);
    Delay(300);
}

void PowerOnSound(){
    if(Settings.AR_IS.StartSound == Off) return;
    generate_sinus(1,880,80);
    Delay(100);
    generate_sinus(1,960,80);
    Delay(100);
    generate_sinus(1,1260,300);
}

void DoPowerOff() {
    lcd_clear(); // Remove remaining picture on power on
    lcd_sleep();
    LATG = 0;     // Attenuator
    TRISG = 0xFF; // Disable PortG output driver
    PWM6CON = 0; // Disale PWM
    T2CONbits.ON = 0;
    CPUDOZEbits.IDLEN = 0;
    PIE0bits.TMR0IE = 0; // Disable 1ms timer interrupt
    BT_off();
    while (Keypressed); // Wait to button to release
    OSCCON3bits.CSWHOLD = 1; // Switch OSC when ready
    OSCENbits.HFOEN = 0; // Disable HFINTOSC
    OSCCON1bits.NOSC = 0b100; // New oscillator is SOSC
    ADC_DISABLE_INTERRUPT;
    InputFlags.INITIALIZED = False;
    // Configure interrupt for wakeup
    INT0IE = 1;
    // Enable RTC interrupt
    RTC_TIMER_IE = 1;
    LATEbits.LATE1 = 0;     // +5V
    LATEbits.LATE2 = 0;     // BUZZER driver
    LATEbits.LATE6 = 0;     // Backlight
    LATEbits.LATE0 = 0;     // +3
    VREGCON = 2;
    OSCCON3bits.CSWHOLD = 0; // Switch OSC
    Sleep();
    InputFlags.KEY_RELEASED = True;
    PIE0bits.TMR0IE = 1;
    OSCCON1bits.NOSC = 0b110; // New oscillator is HFINTOSC
    OSCCON3bits.CSWHOLD = 0; // Switch OSC when ready
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
    eeprom_init();
    getSettings();
    lcd_set_contrast(Settings.ContrastValue);
    getStats();
    set_backlight(Settings.BackLightLevel);
    ADC_init();
    InitAttenuator();
    // TODO: Review power on sequence
    RTC_TIMER_IE = 1; // Enable 2 s timer interrupt
    GIE = 1; // enable global interrupts
    INT0IE = 0; // Disable wakeup interrupt
    init_ms_timer0();
    initialize_rtc_timer();
    for(uint8_t i = 0;i<BAT_BUFFER_SIZE;i++){
        battery_mV = ADC_Read(BATTERY)*BAT_divider;
        BAT_BUFFER_PUT(battery_mV);
        Delay(10);
    }
    if(Settings.InputType==INPUT_TYPE_Microphone){
        TRISDbits.TRISD1 = 0;
        TRISDbits.TRISD2 = 0;
        LATDbits.LATD1 = 0;
        LATDbits.LATD2 = 0;
    } else {
        TRISDbits.TRISD1 = 1;
        TRISDbits.TRISD2 = 1;
    }
    init_bt();
    Stats.PowerOn++;
    saveStatsField(&(Stats.PowerOn), 4);
    PowerOnSound();
    
    update_rtc_time();
    timer_idle_last_action_time = unix_time_ms_sec;
    InputFlags.INITIALIZED = True;
}

void DoCharging() {
    char msg[10];
    if (charger_state_changed) {
        charger_display_state = charger_state;
        switch (charger_state) {
            case Charging:
                lcd_clear();
                sprintf(msg, "Charging ");
                lcd_write_string(msg, UI_CHARGING_LBL_X, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
                Stats.Charging++;
                saveStatsField(&(Stats.Charging), 2);
                break;
            case Complete:
                lcd_clear();
                sprintf(msg, "Charged ");
                lcd_write_string(msg, UI_CHARGING_LBL_X, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
                Stats.Charged++;
                saveStatsField(&(Stats.Charged), 2);
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

void StartPlayParSound() {
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        LATDbits.LATD1 = 1;
        LATDbits.LATD2 = 1;
    }
    generate_sinus(Settings.Volume, Settings.BuzzerFrequency, Settings.BuzzerParDuration);
}

void StartPlayStartSound() {
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        LATDbits.LATD1 = 1;
        LATDbits.LATD2 = 1;
    }
    generate_sinus(Settings.Volume, Settings.BuzzerFrequency, Settings.BuzzerStartDuration);
    // TODO: Test here timings
    sendSignal("START", Settings.BuzzerStartDuration, 0L);
}

void StartCountdownTimer() {
    char msg[16];
    uint8_t length;

    InputFlags.FOOTER_CHANGED = True;
    restoreSettingsField(&Settings,&(Settings.DelayMode),1);
    switch(Settings.ParMode) {
        case ParMode_Repetitive:
            repetitive_counter = 0;
            repetitive_state = Face;
            next_par_ms = Settings.RepetitiveFaceTime;
            break;
        case ParMode_AutoPar:
            {
                float d = Settings.AutoPar[CurPar_idx].delay;
                runtimeDelayTime = (long) (d * 1000);
                runtimeDelayTime -= AUTO_PAR_OVER_DETECT_MS; // To allow shot detection slightly after the par signal
                d = Settings.AutoPar[CurPar_idx].par;
                next_par_ms = (long)(d * 1000);
                Settings.DelayMode = DELAY_MODE_Other;
            }
            break;
        case ParMode_Regular:
        case ParMode_Spy:
            CurPar_idx = 0;
            // intentional fail through
        default:
            next_par_ms = (long)Settings.ParTime[CurPar_idx] * 1000;
            break;
    }
    switch (Settings.DelayMode) {
        case DELAY_MODE_Instant: runtimeDelayTime = 50; // To allow battery reading and not interfere with detection
            break;
        case DELAY_MODE_Fixed: runtimeDelayTime = 3000;
            break;
        case DELAY_MODE_Random:
            runtimeDelayTime = 1000 + random16(3000);// from 2s to 5s random delay
            break;
        case DELAY_MODE_Custom:
            eeprom_read_array(SettingAddress(Settings, Settings.CUstomDelayTime), (uint8_t *)&(Settings.CUstomDelayTime), 4);
            runtimeDelayTime = Settings.CUstomDelayTime;
            break;
    }
    update_rtc_time();
    countdown_start_time = unix_time_ms;
    ShootString_start_time = countdown_start_time;

    for (uint16_t i = 0; i < Size_of_ShootString; i++) {
        ((uint8_t *)(&ShootString))[i] = 0;
    }
    length = sprintf(msg, "STANDBY,%d,%u\n", Settings.DelayMode, runtimeDelayTime);
    sendString(msg, length);
}

void UpdateShot(time_t now, ShotInput_t input) {
    uint24_t dt, ddt;
    // Index var is for code size optimisation.
    uint8_t index , prev_index;
    index = get_shot_index_in_arr(ShootString.TotShoots);
    if(ShootString.TotShoots == MAX_REGISTERED_SHOTS)
        index--;
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
        }
        ShootString.shots[index].sn = ShootString.TotShoots;
        InputFlags.FOOTER_CHANGED = True;
        InputFlags.NEW_SHOT = True;
    }
}

void UpdateShotNow(ShotInput_t x) {
    update_rtc_time();
    timer_idle_last_action_time = unix_time_ms_sec;
    UpdateShot(unix_time_ms, x);
}

void check_countdown_expired() {
    update_rtc_time();
    if (unix_time_ms - countdown_start_time > runtimeDelayTime) {
        comandToHandle = CountdownExpired;
    }
}

void increment_par() {
    if (CurPar_idx < Settings.TotPar - 1) {
        CurPar_idx++;
    } else {
        CurPar_idx = 0;
    }
    InputFlags.FOOTER_CHANGED = True;
}

void decrement_par(){
    if (CurPar_idx > 0) {
        CurPar_idx--;
    } else {
        CurPar_idx = Settings.TotPar - 1;
    }
    InputFlags.FOOTER_CHANGED = True;
}

void check_par_expired() {
    if (ParFlags.ParNowCounting) {
        update_rtc_time();
        if(unix_time_ms - parStartTime_ms < next_par_ms) return;
        ParFlags.ParNowCounting = False; // Should be re-enabled in event handler
        switch (Settings.ParMode) {
            case ParMode_Repetitive:
                timerEventToHandle = RepetitiveParEvent;
                break;
            case ParMode_AutoPar:
                timerEventToHandle = AutoParEvent;
                break;
            case ParMode_Regular:
                timerEventToHandle = ParEvent;
                break;
            default:
                timerEventToHandle = BianchiParEvent;
                break;
        }
    } else if (ParFlags.AutoParOverDetect){
        update_rtc_time();
        if(unix_time_ms - parStartTime_ms < next_par_ms) return;
        ParFlags.AutoParOverDetect = False;
        timerEventToHandle = AutoParCompletedEvent;
    }
}

void check_timer_max_time() {
    if (unix_time_ms - ShootString_start_time >= MAX_MEASUREMENT_TIME) {
        timerEventToHandle = TimerTimeout;
    }
}

void detect_aux_shots() {
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
    if (ui_state != TimerListening) return;
    if (ADC_LATEST_VALUE > DetectThreshold) return; // Detecting negative spikes
//    if (InputFlags.BEEP_GUARD){
//        InputFlags.BEEP_GUARD = False;
//        return;
//    }
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
//        InputFlags.BEEP_GUARD = True;
        // If we turned off the sound, turn off external sound too
        if(LATEbits.LATE2 == 0){
            if (Settings.InputType == INPUT_TYPE_Microphone) {
                LATDbits.LATD1 = 0;
                LATDbits.LATD2 = 0;
            }
        }
    }
    if (PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        if (!Keypressed) {//Assignment will not work because of not native boolean
            InputFlags.KEY_RELEASED = True;
            LongPressCount = 0;
        }
        if (ui_state == TimerListening){
            if (AUX_A) { // high is "open"
                InputFlags.A_RELEASED = True;
            }
            if (AUX_B) {
                InputFlags.B_RELEASED = True;
            }
            detect_aux_shots();
        }
        check_par_expired();
        // TODO: Work with ADC as threshold interrupt
        if (ADPCH == shot_detection_source)
            ADCON0bits.ADGO = 1;
    } 
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        if (ADPCH == shot_detection_source) {
            ADC_BUFFER_PUT(ADC_SAMPLE_REG_16_BIT);
            DetectMicShot();
        } else if (ADPCH == BATTERY) {
            ADCON0bits.ADGO = 0;
            adc_battery = ADC_SAMPLE_REG_16_BIT;
            battery_mV = adc_battery*BAT_divider;
            BAT_BUFFER_PUT(battery_mV);
            ADC_DISABLE_INTERRUPT;
        }
    } 
    if (RTC_TIMER_IF) {
        RTC_TIMER_IF = 0; // Clear Interrupt flag.
        InputFlags.FOOTER_CHANGED = 1;
        uint8_t const_minute = _minute;
        tic_2_sec();
        if(_minute != const_minute){
            if (ui_state == PowerOff) {
                define_charger_state();
            } else if (ui_state != TimerListening && ui_state != TimerCountdown) {
                ADC_ENABLE_INTERRUPT_BATTERY;
            }
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
void battery_test(){
    int i = 0;
        char msg[64];
        do {
            i++;
            ADC_DISABLE_INTERRUPT;
            generate_sinus(3,1800,500);
            for (int j = 0; j < 1000;j+=10){
                ADC_ENABLE_INTERRUPT_BATTERY;
                Delay(10);
            }
            sprintf(msg, "%u %04d/%04dmV", i, battery_mV, battery_average());
            lcd_clear();
            lcd_write_string(msg,2,40,SmallFont,BLACK_OVER_WHITE);
        } while (battery_average() > battery_voltage_thresholds[5] || Key == 0);
}
void main(void) {
    // <editor-fold defaultstate="collapsed" desc="Initialization">
    DoPowerOn();
    if (Settings.version != FW_VERSION) {
        clearHistory();
        getDefaultSettings();
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
