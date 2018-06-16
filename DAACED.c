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
// </editor-fold>

void strmycpy(char * to, const char * from) {
    char * cp;

    cp = to;
    while (*cp = *from) {
        cp++;
        from++;
    }
}

// <editor-fold defaultstate="collapsed" desc="PIC_init">

void PIC_init(void) {
    // 0 = OUTPUT, 1 = INPUT
    // 0 = DIGITAL, 1 = ANALOG

    OSCFRQ = 0b00001000; // 64 MHz Fosc.

    TRISA = 0b11111111; // ADC inputs 0..3
    ANSELA = 0b00001111;
    OSCENbits.ADOEN = 1; // Enable ADC oscillator;

    TRISB = 0b11111111; //
    ANSELB = 0b00000000;

    TRISC = 0b11100101; // C6 = TX, C7 RX
    // C3 = DP_SCL(OP), C4 = DP_SDA(OP)

    TRISD = 0b00110111; // EEPROM SPI SDI=5 SCK=6 SDO=7
    ANSELD = 0b00000000;

    TRISE = 0b10111010; // E0 = POWER(OP), E6 = BL_EN(OP)
    ANSELE = 0b00000000;

    TRISF = 0b00000000; // F3 = DP_CS(OP), F4 = DP_A0(OP), F6 = DP_RST(OP)
    ANSELF = 0b00100000; // DAC
    DAC1CON0 = 0b10100000; // DAC enabled output on pin13 (RF5) with reference from VDD & VSS

    TRISG = 0x00; // CON8, Debug Header.
    ANSELG = 0x00;
    CPUDOZEbits.IDLEN = 0;
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

void set_backlight(uint8_t duty_cycle) {
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

uint8_t is_power_of2(uint8_t n) {
    return n && (!(n & (n - 1)));
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

// <editor-fold defaultstate="collapsed" desc="Sinus Generator">
uint8_t sinus_table[32] = {0x10, // 11.25
    0x13, // 22.50
    0x16, // 33.75
    0x18, // 45.00
    0x1B, // 56.25
    0x1D, // 68.50
    0x1E, // 79.75
    0x1F, // 90.00
    0x20, // 101.25
    0x1F, // 112.50
    0x1E, // 123.75
    0x1D, // 135.00
    0x1B, // 146.25
    0x18, // 157.50
    0x16, // 168.75
    0x13, // 180.00
    0x10, // 191.25
    0x0C, // 202.50
    0x09, // 213.75
    0x07, // 225.00
    0x04, // 236.25
    0x02, // 247.50
    0x01, // 258.75
    0x00, // 270.00
    0x00, // 281.25
    0x00, // 292.50
    0x01, // 303.75
    0x02, // 315.00
    0x04, // 326.25
    0x07, // 337.50
    0x09, // 348.75
    0x0C}; // 360.00

void sinus_dac_init() {
    TRISFbits.TRISF5 = 0;
    ANSELFbits.ANSELF5 = 0;
    DAC1CON0 = 0x00;
    DAC1CON0bits.DAC1EN = 1; // DAC enabled. = 0b10100000;       // DAC enabled output on pin13 (RF5) with reference from VDD & VSS
    DAC1CON0bits.OE1 = 1;
    DAC1CON0bits.PSS = 0; // Vdd
    DAC1CON0bits.NSS = 0; // Vss
    LATEbits.LATE2 = 1; // Driver Enable
}

void generate_sinus(uint8_t amplitude, uint16_t frequency, int16_t duration) {
    float sinus_time_period_us = (1000000UL / (frequency * 2));
    uint32_t sinus_sample_update_period_us = (uint32_t) (sinus_time_period_us / 57.0); ///32.0)+0.5f);
    int32_t no_of_cycles = ((int32_t) duration * 580) / sinus_time_period_us; //1000

    sinus_dac_init();

    //TODO: Stop sound when button pressed
    while (no_of_cycles--) {
        for (uint8_t count = 0; count < 32; count++) {
            dac_value = (sinus_table[count] * amplitude) >> 2;
            if (dac_value > 31) dac_value = 31;
            if (dac_value < 0) dac_value = 0;
            DAC1CON1 = dac_value;
            for (uint16_t delay = 0; delay < sinus_sample_update_period_us; delay++)
                __delay_us(1);
        }
    }
    stop_sinus();
}

void stop_sinus() {
    //    T0CON0bits.T0EN = 0; // Timer OFF
    DAC1CON0bits.EN = 0; // DAC Off
    LATEbits.LATE2 = 0; // Driver OFF
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Save and retrive DATA">

void clearHistory() {
    lcd_write_string("Please wait", UI_CHARGING_LBL_X - 20, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
    eeprom_clear_block(ShootStringStartAddress, EEPROM_MAX_SIZE - ShootStringStartAddress);
}

void saveSettings() {
    eeprom_write_array(SettingsStartAddress, Settings.data, SettingsDataSize);
}

void getSettings() {
    eeprom_read_array(SettingsStartAddress, Settings.data, SettingsDataSize);
}

void getDefaultSettings() {
    Settings.version = FW_VERSION;
    Settings.Sensitivity = DEFAULT_SENSITIVITY;
    Settings.Filter = 70; // ms
    Settings.AR_IS.Autostart = 1; // on
    Settings.AR_IS.AutoRotate = 0; // Off
    Settings.AR_IS.BT = 1; // On, TBD
    Settings.InputType = INPUT_TYPE_Microphone;
    Settings.BuzzerFrequency = 1500; // Hz
    Settings.BuzzerParDuration = 300; // ms
    Settings.BuzzerStartDuration = 500; // ms
    Settings.BuzzerLevel = 6; // Middle strength sound.
    Settings.CustomCDtime = 240; // 4 minutes in sec
    Settings.DelayMode = DELAY_MODE_Fixed;
    Settings.DelayTime = 3000; // ms before start signal
    Settings.BackLightLevel = 10; // Most dimmed visible
    Settings.TotPar = 0; // Par Off
    Settings.ParMode = ParMode_Regular;
    CurPar_idx = 0;
    for (uint8_t i = 0; i < MAXPAR; i++) {
        Settings.ParTime[i] = 0;
    }
}

void saveSettingsField(Settings_t * s, void * f, size_t l) {
    int offset = f - s;
    eeprom_write_array(SettingsStartAddress + offset, s->data + offset, l);
}

void savePar(uint8_t par_index) {
    int offset = &(Settings.ParTime) - (&Settings) + par_index;
    eeprom_write_array(SettingsStartAddress + offset, Settings.data + offset, 3);
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
    eeprom_write_array(addr, ShootString.data, Size_of_ShootString);
    eeprom_write_data(addr, 1);
    index = findCurStringIndex();

}

TBool getShootString(uint8_t offset) {
    uint16_t addr;
    int8_t index;
    index = findCurStringIndex();
    if (index >= offset)
        index -= offset;
    else
        index = MAXSHOOTSTRINGS - offset + index;
    addr = findStringAddress(index);
    eeprom_read_array(addr, ShootString.data, Size_of_ShootString);
    return True;
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

// <editor-fold defaultstate="collapsed" desc="Settings">
// <editor-fold defaultstate="collapsed" desc="Delay Settings">

void SetCustomDelay() {
    NumberSelection_t n;
    strmycpy(n.MenuTitle, "Custom Delay");
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

void SetDelay(SettingsMenu_t * m) {
    uint8_t oldValue = Settings.DelayMode;
    InitSettingsMenuDefaults(m);
    strmycpy(m->MenuTitle, "Delay");
    strmycpy(m->MenuItem[DELAY_MODE_Instant], " Instant ");
    strmycpy(m->MenuItem[DELAY_MODE_Fixed], " 3.0 sec. ");
    strmycpy(m->MenuItem[DELAY_MODE_Random], " Random");
    strmycpy(m->MenuItem[DELAY_MODE_Custom], " Custom ");
    m->TotalMenuItems = 4;
    m->menu = Settings.DelayMode;

    do {
        DisplaySettings(m);
        SelectMenuItem(m);

    } while (SettingsNotDone(m));
    if (m->selected) {
        Settings.DelayMode = m->menu;
        if (Settings.DelayMode == DELAY_MODE_Custom) {
            SetCustomDelay();
            lcd_clear();
        }
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
    b.fmax = 100.0;
    b.fvalue = (float) (Settings.ParTime[par_index]) / 1000;
    b.fold_value = b.fvalue;
    sprintf(b.MenuTitle, "Par %d Settings ", par_index);
    b.fstep = 0.1;
    b.format = " %3.1fs ";
    b.done = False;
    lcd_clear();
    do {
        DisplayDouble(&b);
        SelectDouble(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.ParTime[par_index] = (long) (b.fvalue * 1000);
        if (b.fold_value != b.fvalue)
            savePar(par_index);
    }
    return b.selected;
}

void FillParSettings(SettingsMenu_t * m) {
    uint8_t i = 0;
    for (i = 0; i < Settings.TotPar; i++) {
        sprintf(m->MenuItem[i], "Par %d: %3.1fs  ", i + 1, (float) (Settings.ParTime[i]) / 1000);
    }
    if (i < MAXPAR)
        strmycpy(m->MenuItem[i], "Add ");
    else
        strmycpy(m->MenuItem[i], "Max Par reached ");
    strmycpy(m->MenuItem[++i], "Delete Last ");
    strmycpy(m->MenuItem[++i], "Delete All ");
    m->TotalMenuItems = i + 1;
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

                Settings.ParTime[Settings.TotPar] = 1000; // Default setting 1 second
                res = EditPar(Settings.TotPar);
                if (res) { // Roll back if not selected
                    Settings.TotPar++;
                } else {
                    Settings.ParTime[Settings.TotPar] = 0;
                }
            }
        } else if (m->menu == (m->TotalMenuItems - 2)) {
            // Delete last PAR
            if (Settings.TotPar > 0) {
                Settings.TotPar--;
                Settings.ParTime[Settings.TotPar] = 0;
            }
        } else if (m->menu == (m->TotalMenuItems - 1)) {
            // Clear PAR
            for (uint8_t i = 0; i < Settings.TotPar; i++) {
                Settings.ParTime[i] = 0;
            }
            Settings.TotPar = 0;
            m->menu = 1;
            m->page = 0;
        }
        lcd_clear();
    }
}

void SetPar(SettingsMenu_t * m) {
    uint8_t oldTotPar = Settings.TotPar;
    InitSettingsMenuDefaults(m);
    strmycpy(m->MenuTitle, "Par Settings ");
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
    uint8_t tmp;
    strmycpy(b.MenuTitle, "Backlight ");
    b.max = 9;
    b.min = 0;
    b.step = 1;
    b.value = (Settings.BackLightLevel + 1) / 10;
    b.old_value = b.value;
    tmp = b.value;
    b.format = "%u";
    b.done = False;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
        if (b.value != tmp) {
            set_backlight(b.value * 10 - 1);
            tmp = b.value;
        }
    } while (SettingsNotDone((&b)));

    if (b.selected && b.value != b.old_value) {
        Settings.BackLightLevel = b.value * 10 - 1;
        saveSettingsField(&Settings, &(Settings.BackLightLevel), 1);
    }
    set_backlight(Settings.BackLightLevel);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Buzzer Settings">

void SetBeepFreq() {
    NumberSelection_t b;
    b.fmin = 0.8;
    b.fmax = 3.0;
    b.fvalue = (float) (Settings.BuzzerFrequency) / 1000;
    b.fold_value = b.fvalue;
    strmycpy(b.MenuTitle, "Tone");
    b.fstep = 0.1;
    b.format = "%1.1fK";
    b.done = False;
    do {
        DisplayDouble(&b);
        SelectDouble(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.BuzzerFrequency = (uint16_t) (b.fvalue * 1000);
        if (b.fvalue != b.fold_value) {
            saveSettingsField(&Settings, &(Settings.BuzzerFrequency), 2);
        }
    }
}

void SetBeepLevel() {
    NumberSelection_t b;
    InitSettingsNumberDefaults((&b));
    if (Settings.BuzzerLevel > 20) Settings.BuzzerLevel = 20;
    strmycpy(b.MenuTitle, "Volume");
    b.min = 0;
    b.max = 20;
    b.step = 1;
    b.format = " %2d ";
    b.value = Settings.BuzzerLevel;
    b.old_value = b.value;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
    } while (SettingsNotDone((&b)));
    if (b.selected) {
        Settings.BuzzerLevel = b.value;
        if (b.value != b.old_value) {
            saveSettingsField(&Settings, &(Settings.BuzzerLevel), 1);
        }
    }
}

void SetBeepTime(TBool Par) {
    NumberSelection_t d;
    InitSettingsNumberDefaults((&d));
    if (Par) {
        d.fvalue = (float) (Settings.BuzzerParDuration) / 1000;
        strmycpy(d.MenuTitle, "Par Duration ");
    } else {
        d.fvalue = (float) (Settings.BuzzerStartDuration) / 1000;
        strmycpy(d.MenuTitle, "Start Duration ");
    }
    d.fmin = 0.050;
    d.fmax = 1.0;
    d.fstep = 0.050;
    d.fold_value = d.fvalue;
    d.done = False;
    d.format = " %1.2fs ";
    do {
        DisplayDouble(&d);
        SelectDouble(&d);
    } while (SettingsNotDone((&d)));

    if (d.selected) {
        if (d.fvalue != d.fold_value) {
            if (Par) {
                Settings.BuzzerParDuration = (int) (d.fvalue * 1000);
                saveSettingsField(&Settings, &(Settings.BuzzerParDuration), 2);
            } else {
                Settings.BuzzerStartDuration = (int) (d.fvalue * 1000);
                saveSettingsField(&Settings, &(Settings.BuzzerStartDuration), 2);
            }
        }
    }
}

void SetBeep(SettingsMenu_t * m) {
    InitSettingsMenuDefaults(m);

    strmycpy(m->MenuTitle, "Beep ");
    strmycpy(m->MenuItem[0], " Frequency ");
    strmycpy(m->MenuItem[1], " Volume ");
    strmycpy(m->MenuItem[2], " Par Duration ");
    strmycpy(m->MenuItem[3], " Start Duration ");
    strmycpy(m->MenuItem[4], " Test Beep ");
    m->TotalMenuItems = 5;

    do {
        DisplaySettings(m);
        SelectMenuItem(m);
        if (m->selected) {
            lcd_clear();
            switch (m->menu) {
                case 0:
                    SetBeepFreq();
                    break;
                case 1:
                    SetBeepLevel();
                    break;
                case 2:
                    SetBeepTime(True);
                    break;
                case 3:
                    SetBeepTime(False);
                    break;
                case 4:
                    generate_sinus(Settings.BuzzerLevel, Settings.BuzzerFrequency, Settings.BuzzerParDuration);
                    break;
            }
            // Here we want it done only when back pressed
            // i.e. not selected and done
            m->done = False;
            m->selected = False;
            lcd_clear();
        }
    } while (SettingsNotDone(m));
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Sensitivity">

void SetSens() {//Sensitivity
    NumberSelection_t s;
    InitSettingsNumberDefaults((&s));
    //    if (Settings.Sensitivity > DETECT_THRESHOLD_LEVELS) Settings.Sensitivity = DETECT_THRESHOLD_LEVELS;
    strmycpy(s.MenuTitle, "Sensitivity");
    //    s.max = DETECT_THRESHOLD_LEVELS;
    //    s.min = 1;
    s.max = 200;
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
            saveSettingsField(&Settings, &(Settings.Sensitivity), 3);
        }
    }
}

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Filter">

void SetFilter() {
    if (Settings.Filter > 100) Settings.Filter = 100;
    NumberSelection_t f;
    strmycpy(f.MenuTitle, "Filter");
    f.fmin = 0.01;
    f.fmax = 0.2;
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

void set_autostert_label(char * l) {
    if (AutoStart)
        strmycpy(l, " Auto Start ON ");
    else
        strmycpy(l, " Auto Start OFF ");
}

void SetAutoStart(SettingsMenu_t * m) {
    TBool orgset;
    InitSettingsMenuDefaults(m);
    m->TotalMenuItems = 1;
    strmycpy(m->MenuTitle, "Autostart");
    set_autostert_label(m->MenuItem[0]);
    orgset = AutoStart;
    do {
        DisplaySettings(m);
        SelectBinaryMenuItem(m);
        if (m->selected) {
            m->selected = False;
            m->done = False;
            if (AutoStart)
                AutoStart = False;
            else
                AutoStart = True;
            set_autostert_label(m->MenuItem[0]);
        }
    } while (SettingsNotDone(m));
    if (AutoStart != orgset) {
        saveSettingsField(&Settings, &(Settings.AR_IS), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="TimerMode">
// <editor-fold defaultstate="collapsed" desc="">

void fill_par_bianci() {
    Settings.TotPar = 12;
    Settings.ParTime[0] = 3000;
    Settings.ParTime[1] = 4000;
    Settings.ParTime[2] = 8000;
    Settings.ParTime[3] = 4000;
    Settings.ParTime[4] = 5000;
    Settings.ParTime[5] = 6000;
    Settings.ParTime[6] = 5000;
    Settings.ParTime[7] = 6000;
    Settings.ParTime[8] = 7000;
    Settings.ParTime[9] = 7000;
    Settings.ParTime[10] = 10000;
    Settings.ParTime[11] = 15000;
}

void fill_par_barricade() {
    Settings.TotPar = 8;
    Settings.ParTime[0] = 5000;
    Settings.ParTime[1] = 5000;
    Settings.ParTime[2] = 6000;
    Settings.ParTime[3] = 6000;
    Settings.ParTime[4] = 7000;
    Settings.ParTime[5] = 7000;
    Settings.ParTime[6] = 8000;
    Settings.ParTime[7] = 8000;
}

void fill_par_falling_plate() {
    Settings.TotPar = 8;
    Settings.ParTime[0] = 6000;
    Settings.ParTime[1] = 6000;
    Settings.ParTime[2] = 7000;
    Settings.ParTime[3] = 7000;
    Settings.ParTime[4] = 8000;
    Settings.ParTime[5] = 8000;
    Settings.ParTime[6] = 9000;
    Settings.ParTime[7] = 9000;
}

void fill_par_nra_ppc_a() {
    Settings.TotPar = 4;
    Settings.ParTime[0] = 20000;
    Settings.ParTime[1] = 90000;
    Settings.ParTime[2] = 165000;
    Settings.ParTime[3] = 12000;
}

void fill_par_nra_ppc_b() {
    Settings.TotPar = 5;
    Settings.ParTime[0] = 20000;
    Settings.ParTime[1] = 12000;
    Settings.ParTime[2] = 90000;
    Settings.ParTime[3] = 12000;
    Settings.ParTime[4] = 120000;
}

void fill_par_nra_ppc_c() {
    Settings.TotPar = 4;
    Settings.ParTime[0] = 20000;
    Settings.ParTime[1] = 90000;
    Settings.ParTime[2] = 12000;
    Settings.ParTime[3] = 165000;
}

void fill_par_nra_ppc_d() {
    Settings.TotPar = 4;
    Settings.ParTime[0] = 8000;
    Settings.ParTime[1] = 20000;
    Settings.ParTime[2] = 20000;
    Settings.ParTime[3] = 90000;
}
// </editor-fold>

void SetMode(SettingsMenu_t * m) {
    uint8_t oldPar = Settings.ParMode;
    InitSettingsMenuDefaults(m);
    m->TotalMenuItems = 8;
    strmycpy(m->MenuTitle, "Timer Mode");
    strmycpy(m->MenuItem[ParMode_Regular], " Regular ");
    strmycpy(m->MenuItem[ParMode_Practical], " Practical ");
    strmycpy(m->MenuItem[ParMode_Barricade], " Barricade ");
    strmycpy(m->MenuItem[ParMode_FallingPlate], " Falling Plate ");
    strmycpy(m->MenuItem[ParMode_NRA_PPC_A], " NRA-PPC A ");
    strmycpy(m->MenuItem[ParMode_NRA_PPC_B], " NRA-PPC B ");
    strmycpy(m->MenuItem[ParMode_NRA_PPC_C], " NRA-PPC C ");
    strmycpy(m->MenuItem[ParMode_NRA_PPC_D], " NRA-PPC D ");

    m->menu = Settings.ParMode;
    //Main Screen
    do {
        DisplaySettings(m);
        SelectMenuItem(m);
    } while (SettingsNotDone(m));
    if (m->selected) {
        Settings.ParMode = m->menu;
        switch (m->menu) {
            case ParMode_Regular:
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
    if (oldPar != Settings.ParMode) {
        saveSettingsField(&Settings, &(Settings.ParMode), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Clock">

void SetClock() {
    NumberSelection_t ts;
    time_t minute;
    InitSettingsNumberDefaults((&ts));
    minute = rtc_time.sec / 30;
    ts.min = 0;
    ts.max = 0x7FFFFF;
    ts.value = minute;
    ts.old_value = ts.value;
    ts.step = 60;
    strmycpy(ts.MenuTitle, "Clock");
    ts.state = 0; // 0 - hour, 1 - Minute. DisplayTime knows to handle this
    do {
        DisplayTime(&ts);
        SelectInteger(&ts);
        if (ts.selected && ts.state == 0) {
            ts.state = 1;
            ts.selected = False;
            ts.done = False;
            ts.step = 1;
        }
    } while (SettingsNotDone((&ts)));
    if (ts.selected && ts.old_value != ts.value) {
        set_rtc_time(ts.value * 30);
        comandToHandle = TimeChanged;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="CountDown">

void CountDownMode(time_t countdown) {
    char msg[16];
    time_t reminder;
    time_t stop_time = rtc_time.sec + countdown;
    uint8_t minute, second;
    TBool done = False;
    lcd_clear();
    do {
        print_header();
        reminder = stop_time - rtc_time.sec;
        minute = reminder / 60;
        second = reminder % 60;
        sprintf(msg,
                "%2d%s%02d",
                minute,
                (rtc_time.msec > 500 || rtc_time.msec < 250) ? "." : ":",
                second
                );
        display_big_font_label(msg);
        define_input_action();
        switch (comandToHandle) {
            case StartLong:STATE_HANDLE_POWER_OFF;
                break;
            case StartShort:STATE_HANDLE_TIMER_IDLE;
                break;
            case ChargerEvent:STATE_HANDLE_CHARGING;
                break;
            case BackShort:
            case BackLong:
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
        for (uint8_t i = 0; i < 5; i++) {
            for (uint8_t j = 0; j < 4; j++) {
                generate_sinus(
                        Settings.BuzzerLevel,
                        1000,
                        50
                        );
                delay_rtc_ms(50);
            }
            if (Keypressed)
                break;
            delay_rtc_ms(400);
        }
    }
}

TBool SetCustomCountDown() {
    NumberSelection_t ts;
    uint8_t minute, second;
    char msg[16];
    InitSettingsNumberDefaults((&ts));
    strmycpy(ts.MenuTitle, "Custom Count");

    // in seconds
    ts.value = Settings.CustomCDtime;
    ts.old_value = ts.value;
    ts.max = 999;
    ts.min = 30;
    ts.step = 1;
    lcd_clear();
    do {
        print_header();
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

void SetCountDown(SettingsMenu_t * m) {
    InitSettingsMenuDefaults(m);
    m->TotalMenuItems = 4;
    strmycpy(m->MenuTitle, "Countdown");
    strmycpy(m->MenuItem[0], " Off ");
    strmycpy(m->MenuItem[1], " 3 minutes ");
    strmycpy(m->MenuItem[2], " 5 minutes ");
    strmycpy(m->MenuItem[3], " Custom ");

    //Main Screen
    do {
        DisplaySettings(m);
        SelectMenuItem(m);
    } while (SettingsNotDone(m));
    switch (m->menu) {
        case 1: CountDownMode(180);
            break;
        case 2: CountDownMode(300);
            break;
        case 3:
            if (SetCustomCountDown())
                CountDownMode(Settings.CustomCDtime);
            break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Tilt">

void SetTilt(SettingsMenu_t * m) {
    uint8_t orgset;
    InitSettingsMenuDefaults(m);
    m->TotalMenuItems = 1;
    strmycpy(m->MenuTitle, "Tilt");
    if (Settings.AR_IS.AutoRotate) strmycpy(m->MenuItem[0], " Auto Rotate ON ");
    else strmycpy(m->MenuItem[0], " Auto Rotate OFF ");

    // SettingsDisplay(&SetTiltMenu);
    orgset = Settings.AR_IS.AR_IS;

    do {
        DisplaySettings(m);
        SelectBinaryMenuItem(m);
        if (m->selected) {
            m->selected = False;
            Settings.AR_IS.AutoRotate = !Settings.AR_IS.AutoRotate;
            if (Settings.AR_IS.AutoRotate)
                strmycpy(m->MenuItem[0], " Auto Rotate ON ");
            else
                strmycpy(m->MenuItem[0], " Auto Rotate OFF");
        }
    } while (SettingsNotDone(m));
    if (Settings.AR_IS.AR_IS != orgset) {
        saveSettingsField(&Settings, &(Settings.AR_IS), 1);
        InputFlags.orientation = ORIENTATION_NORMAL;
        InputFlags.TiltChanged = True;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Input">

void UpdateIS(SettingsMenu_t * sm) {
    strmycpy(sm->MenuItem[INPUT_TYPE_Microphone], " Microphone ");
    strmycpy(sm->MenuItem[INPUT_TYPE_A_or_B_multiple], " A or B (multiple) ");
    strmycpy(sm->MenuItem[INPUT_TYPE_A_and_B_single], " A and B (single) ");
    sm->TotalMenuItems = 3;
}

void SetInput(SettingsMenu_t * m) {
    uint8_t orgset;
    InitSettingsMenuDefaults(m);
    strmycpy(m->MenuTitle, "Input Source");
    UpdateIS(m);
    DisplaySettings(m);
    orgset = Settings.InputType;
    m->menu = Settings.InputType;

    do {
        DisplaySettings(m);
        SelectMenuItem(m);
    } while (SettingsNotDone(m));
    if (m->selected) {
        Settings.InputType = m->menu;
    }
    if (Settings.InputType != orgset) {
        saveSettingsField(&Settings, &(Settings.InputType), 1);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="BlueTooth">

void BlueTooth() {
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_write_string(" TBD ", 3, 40, MediumFont, BLACK_OVER_WHITE);
    __delay_ms(500);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Settings Menu">

void DoSet(uint8_t menu) {
    SettingsMenu_t * m = &ma;
    lcd_clear();
    switch (menu) {
        case 0:SetDelay(m);
            break;
        case 1:SetPar(m);
            break;
        case 2:SetBeep(m);
            break;
        case 3:SetAutoStart(m);
            break;
        case 4:SetMode(m);
            break;
        case 5:SetClock();
            break;
        case 6:SetCountDown(m);
            break;
        case 7:SetTilt(m);
            break;
        case 8:SetBacklight();
            break;
        case 9:SetSens();
            break;
        case 10:SetFilter();
            break;
        case 11:SetInput(m);
            break;
        case 12:BlueTooth();
            break;
        case 13:
            getDefaultSettings();
            saveSettings();
            break;
        case 14:
            //            DoDiagnostics();
            clearHistory();
            break;
    }
    lcd_clear();
}

void SetSettingsMenu(SettingsMenu_t * SettingsMenu) {
    //{"Delay","Par","Beep","Auto","Mode","Clock","CountDown","Tilt","Bklight","Input","BT","Diag"};

    SettingsMenu->TotalMenuItems = 16;

    strmycpy(SettingsMenu->MenuTitle, " Settings ");
    strmycpy(SettingsMenu->MenuItem[0], " Delay ");
    strmycpy(SettingsMenu->MenuItem[1], " Par ");
    strmycpy(SettingsMenu->MenuItem[2], " Buzzer ");
    strmycpy(SettingsMenu->MenuItem[3], " Auto Start ");
    strmycpy(SettingsMenu->MenuItem[4], " Timer Mode ");
    strmycpy(SettingsMenu->MenuItem[5], " Clock ");
    strmycpy(SettingsMenu->MenuItem[6], " Countdown ");
    strmycpy(SettingsMenu->MenuItem[7], " Tilt ");
    strmycpy(SettingsMenu->MenuItem[8], " Backlight ");
    strmycpy(SettingsMenu->MenuItem[9], " Sensitivity ");
    strmycpy(SettingsMenu->MenuItem[10], " Filter ");
    strmycpy(SettingsMenu->MenuItem[11], " Input ");
    strmycpy(SettingsMenu->MenuItem[12], " Bluetooth ");
    strmycpy(SettingsMenu->MenuItem[13], " Reset Settings ");
    strmycpy(SettingsMenu->MenuItem[14], " Clear History ");
    sprintf(SettingsMenu->MenuItem[15], " FW version: %0X ", Settings.version);
}

void DoSettings(void) {
    set_screen_title("Settings");

    InitSettingsMenuDefaults((&SettingsMenu));
    SetSettingsMenu(&SettingsMenu);
    lcd_clear();
    do {
        handle_timer_idle_shutdown();
        handle_rotation();
        DisplaySettings(&SettingsMenu);
        SelectMenuItemCircular(&SettingsMenu);
        if (SettingsMenu.selected) {
            DoSet(SettingsMenu.menu);
            SettingsMenu.selected = False;
            lcd_clear();
        }
        // Never exit on OK/Cancel case here, only on screen change
        SettingsMenu.done = False;
    } while (SettingsNotDone((&SettingsMenu)));

    if (ui_state == SettingsScreen) {
        STATE_HANDLE_TIMER_IDLE;
    } else {
        lcd_clear();
    }
}
// </editor-fold>
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Review Screen">
#define REVIEW_SHOT_FORMAT      "%2d: %3.2f %c"
#define REVIEW_TOTAL_SHOT_FORMAT      " %2d / %3.2fs"
#define REVIEW_SPLIT_FORMAT     "]%3.2f"
TBool reviewChanged = True;

void ReviewDisplay() {
    uint8_t line = UI_HEADER_END_LINE;
    uint8_t col, i, page_start, page_size;
    TBool first_page;
    char message[20];
    // We're assuming here that Medium font has even number of bytes heigh
    uint8_t halfline = 16;
    if (reviewChanged) {
        lcd_clear();
    }

    // Stat line
    sprintf(ScreenTitle,
            REVIEW_TOTAL_SHOT_FORMAT,
            ShootString.TotShoots,
            (float) ShootString.shots[ShootString.TotShoots - 1].dt / 1000
            );
    print_header();
    //Shoot lines
    //1st ShootNumber 01, before it ShootNumber 00 time=0
    for (i = 0; i < SHOTS_ON_REVIEW_SCREEN; i++) {
        char * mode;
        uint8_t curr_index = (CurShoot + i) % ShootString.TotShoots;
        uint8_t subtrahend_index = (CurShoot + i + 1) % ShootString.TotShoots;
        if (Settings.InputType == INPUT_TYPE_Microphone) {
            mode = ' ';
        } else {
            mode = (ShootString.shots[curr_index].is_b) ? 'B' : ((ShootString.shots[curr_index].is_a) ? 'A' : ' ');
        }
        sprintf(message,
                REVIEW_SHOT_FORMAT,
                curr_index + 1,
                (float) ShootString.shots[curr_index].dt / 1000,
                mode
                );
        lcd_write_string(message, 5, line, MediumFont, (i != 1)&0x01);
        line += halfline;

        // Don't print last diff at half line and not the latest
        if (i < SHOTS_ON_REVIEW_SCREEN - 1 &&
                curr_index != ShootString.TotShoots - 1) {
            sprintf(message,
                    REVIEW_SPLIT_FORMAT,
                    (float) (ShootString.shots[subtrahend_index].dt - ShootString.shots[curr_index].dt) / 1000);
            lcd_write_string(message, 135, line, MediumFont, BLACK_OVER_WHITE);
        }
        line += halfline;
    }
    if (reviewChanged) {
        reviewChanged = False;
        lcd_fill_block(0, line, LCD_WIDTH, LCD_HEIGHT - 8);
    }
    //String line
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

void review_scroll_shot_up() {
    if (CurShoot > 0) {
        CurShoot--;
    } else {
        CurShoot = ShootString.TotShoots;
    }
    reviewChanged = True;
}

void review_scroll_shot_down() {
    if (CurShoot < ShootString.TotShoots) {
        CurShoot++;
    } else {
        CurShoot = 1;
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
    CurShoot = ShootString.TotShoots - 1;
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
    CurShoot = ShootString.TotShoots - 1;
}

void DoReview() {
    getShootString(0);
    CurShootString = 0;
    CurShoot = ShootString.TotShoots - 1;
    reviewChanged = True;
    do {
        ReviewDisplay();
        define_input_action();
        handle_timer_idle_shutdown();
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
            case StartLong:STATE_HANDLE_POWER_OFF;
                break;
            case StartShort:STATE_HANDLE_TIMER_IDLE;
                break;
            case ReviewLong:STATE_HANDLE_SETTINGS_SCREEN;
                break;
            default:
                break;
        }
        comandToHandle = None;

    } while (ui_state == ReviewScreen);
    getShootString(0);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Diagnostics">

void print_label_at_diagnostics_grid(const char* msg, const uint8_t grid_x, const uint8_t grid_y) {
    lcd_write_string(msg, UI_FOOTER_GRID_X(grid_x), UI_FOOTER_GRID_Y(grid_y, UI_DIAG_GRID_START_LINE), SmallFont, BLACK_OVER_WHITE);
}

void print_stats() {
    char message[32];
    for (uint8_t y = UI_DIAG_GRID_START_LINE; y < LCD_HEIGHT; y += UI_FOOTER_GRID_HEIGH) {
        lcd_draw_fullsize_hgridline(y, LCD_MID_LINE_PAGE);
    }

    lcd_draw_vgrid_lines(UI_HEADER_END_LINE);
    // TODO: Implement some diagnostics
    for (uint8_t x = 0; x < 4; x++)
        for (uint8_t y = 0; y < 5; y++) {
            uint8_t p = UI_FOOTER_GRID_Y(y, UI_DIAG_GRID_START_LINE);
            sprintf(message, " %d %d,%d", PAGE(p), x, y);
            print_label_at_diagnostics_grid(message, x, y);
        }
}

void DoDiagnostics() {
    SettingsMenu_t * s = &mx;
    s->selected = False;
    s->done = False;
    strmycpy(ScreenTitle, " Diagnostics");
    do {
        print_header();
        print_stats();
        SelectMenuItem(s);
    } while (SettingsNotDone(s));

}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Main Menu">

void DetectInit(void) {
    uint16_t Mean = 0;
    uint16_t Peak = 0;
    uint16_t ADCvalue;

    for (uint8_t i = 0; i < 64; i++) {
        ADCvalue = ADC_Read(ENVELOPE);
        Mean += ADCvalue;
        if (Peak < ADCvalue) Peak = ADCvalue;
    }
    Mean = Mean >> 6;
    //    DetectThreshold = Mean + threshold_offsets[Settings.Sensitivity - 1];
    DetectThreshold = Mean + Settings.Sensitivity;
    InputFlags.A_RELEASED = True;
    InputFlags.B_RELEASED = True;
}

uint8_t print_time() {
    char message[30];
    sprintf(message,
            "%02d%s%02d ",
            get_hour(),
            (rtc_time.msec < 250 || rtc_time.msec > 1750) ? "." : ":",
            get_minute());
    lcd_write_string(message, 1, 0, SmallFont, BLACK_OVER_WHITE);
    sprintf(message, "%s", ScreenTitle);
    lcd_write_string(message, 55, 0, SmallFont, BLACK_OVER_WHITE);
    return SmallFont->height;
}
uint8_t old_bat_length = 0;

void print_batery_text_info() {
    char message[32];
    sprintf(message,
            "%d",
            battery_level
            //            (PORTD & 0b11000)
            );
    uint8_t width = lcd_string_lenght(message, SmallFont);
    if (old_bat_length > width)
        lcd_clear_block(LCD_WIDTH - 10 - old_bat_length, 0, LCD_WIDTH - 8, SmallFont->height + 1);
    old_bat_length = width;
    lcd_write_string(message, LCD_WIDTH - 8 - width, 0, SmallFont, BLACK_OVER_WHITE);
}

TBool battery_level_ok(uint8_t l) {
    uint16_t level = (battery_level - 3000) / 220;
    return (level > l);
}

void print_batery_info() {
    uint8_t col = LCD_WIDTH - 35;

    lcd_draw_bitmap(col, 0, &battery_left_bitmap);
    col = col + battery_left_bitmap.width_in_bits;
    for (uint8_t i = 5; i > 0; i--) {
        if (battery_level_ok(i - 1)) {
            lcd_draw_bitmap(col, 0, &battery_middle_full_bitmap);
            col += battery_middle_full_bitmap.width_in_bits;
        } else {
            lcd_draw_bitmap(col, 0, &battery_middle_empty_bitmap);
            col += battery_middle_empty_bitmap.width_in_bits;
        }
    }
    lcd_draw_bitmap(col, 0, &battery_right_bitmap);
}

uint8_t print_header() {
    print_time();
    if (BT_INSERT == 0) {
        lcd_draw_bitmap(LCD_WIDTH - 53, 0, &bt_bitmap_data);
    } else {
        lcd_clear_block(
                LCD_WIDTH - 53,
                0,
                LCD_WIDTH - 53 + bt_bitmap_data.width_in_bits,
                bt_bitmap_data.heigth_in_bytes * 8
                );
    }
    //    print_batery_text_info();
    print_batery_info();
    lcd_draw_fullsize_hline(UI_HEADER_END_LINE - 1, LCD_MID_LINE_PAGE);
    return UI_HEADER_END_LINE;
}

void print_label_at_footer_grid(const char* msg, const uint8_t grid_x, const uint8_t grid_y) {
    lcd_write_string(msg, UI_FOOTER_GRID_X(grid_x), UI_FOOTER_GRID_Y(grid_y, UI_FOOTER_START_LINE), SmallFont, WHITE_OVER_BLACK);
}

void print_footer() {
    char message[20];
    lcd_fill_block(0, UI_FOOTER_START_LINE, LCD_WIDTH, LCD_HEIGHT);
    sprintf(message, " 1st: %3.2f", (float) ShootString.shots[0].dt / 1000);
    print_label_at_footer_grid(message, 0, 0);
    sprintf(message, " Shots: %2d", ShootString.TotShoots);
    print_label_at_footer_grid(message, 1, 0);
    switch (Settings.DelayMode) {
        case DELAY_MODE_Instant: sprintf(message, " Delay: 0.0s");
            break;
        case DELAY_MODE_Fixed: sprintf(message, " Delay: 3.0s");
            break;
        case DELAY_MODE_Random: sprintf(message, " Delay: RND");
            break;
        case DELAY_MODE_Custom: sprintf(message, " Delay: %1.1fs", (float) (Settings.DelayTime) / 1000);
            break;
    }
    //    sprintf(message, "%c:%u", get_time_source(), rtc_time.unix_time_ms);
    print_label_at_footer_grid(message, 0, 1);

    if (Settings.TotPar > 0) {
        sprintf(message, "P%02d:%3.1f", CurPar_idx + 1, (float) Settings.ParTime[CurPar_idx] / 1000);
    } else {
        sprintf(message, " P: Off");
    }
    //    sprintf(message, "%u", rtc_time.msec);
    print_label_at_footer_grid(message, 1, 1);
}

void StartListenShots(void) {
    ShootString_start_time = rtc_time.unix_time_ms;
    DetectInit();
}
// </editor-fold>

void DoPowerOff() {
    set_backlight(0);
    lcd_clear();
    LATEbits.LATE0 = 0;
    // TODO: Implement SLEEP mode when powering off
    OSCFRQ = 0b00000000; // 1MHz clock for power saving

    //    Sleep();
}

void DoPowerOn() {
    PIC_init();
    initialize_backlight();
    set_backlight(5);
    spi_init();
    lcd_init();
    ADC_init();
    eeprom_init();
    LATEbits.LATE0 = 1;
    OSCFRQ = 0b00001000; // Switch back to 64MHz
    // TODO: Review power on sequence
    set_backlight(Settings.BackLightLevel);
}

void DoCharging() {
    char msg[10];
    if (charger_state_changed) {
        LATEbits.LATE0 = 1;
        switch (charger_state) {
            case Charging:
                lcd_clear();
                sprintf(msg, "Charging ");
                lcd_write_string(msg, UI_CHARGING_LBL_X, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
                break;
            case Complete:
                lcd_clear();
                sprintf(msg, "Charged  ");
                lcd_write_string(msg, UI_CHARGING_LBL_X, UI_CHARGING_LBL_Y, MediumFont, BLACK_OVER_WHITE);
                break;
            case NotCharging:
                STATE_HANDLE_POWER_OFF;
                break;
            default:
                break;
        }
        charger_state_changed = false;
    }
}

void clear_timer_area() {
    lcd_clear_block(0, UI_HEADER_END_LINE, 0, BigFont->height + MediumFont->height);
}

void print_shot_origin(const shot_t * s) {
    uint8_t y_pos = UI_HEADER_END_LINE + LCD_PAGE_HEIGTH;
    if (s->is_a) {
        lcd_write_string(
                "A",
                3,
                y_pos,
                MediumFont,
                BLACK_OVER_WHITE
                );
    }
    y_pos += MediumFont->height;
    if (s->is_b) {
        lcd_write_string(
                "B",
                3,
                y_pos, MediumFont,
                BLACK_OVER_WHITE
                );
    }
}

void update_shot_time_on_screen() {
    uint24_t t = 0;
    if (ShootString.TotShoots > 0) {
        t = ShootString.shots[ShootString.TotShoots - 1].dt;
        print_shot_origin(&(ShootString.shots[ShootString.TotShoots - 1]));
    }
    print_big_time_label(t);
}

void PlayParSound() {
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        AUX_A = 1;
        AUX_B = 1;
    }
    generate_sinus(Settings.BuzzerLevel, Settings.BuzzerFrequency, Settings.BuzzerParDuration);
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        AUX_A = 0;
        AUX_B = 0;
    }
}

void PlayStartSound() {
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        AUX_A = 1;
        AUX_B = 1;
    }
    generate_sinus(Settings.BuzzerLevel, Settings.BuzzerFrequency, Settings.BuzzerStartDuration);
    if (Settings.InputType == INPUT_TYPE_Microphone) {
        AUX_A = 0;
        AUX_B = 0;
    }
}

void StartParTimer() {
    timerEventToHandle = None;
    if (CurPar_idx < Settings.TotPar) {
        ParNowCounting = true;
        parStartTime_ms = rtc_time.unix_time_ms;
    }
}

void StartCountdownTimer() {
    switch (Settings.DelayMode) {
        case DELAY_MODE_Instant: Settings.DelayTime = 0;
            break;
        case DELAY_MODE_Fixed: Settings.DelayTime = 3000;
            break;
        case DELAY_MODE_Random:
            if (Settings.DelayTime < 500)
                Settings.DelayTime = 1134;
            Settings.DelayTime = (5 * Settings.DelayTime) % 3199;
            break;
        case DELAY_MODE_Custom:
            eeprom_read_array(SettingAddress(Settings, Settings.DelayTime), (uint8_t *)&(Settings.DelayTime), 4);
            break;
    }
    countdown_start_time = rtc_time.unix_time_ms;
    ShootString_start_time = countdown_start_time;
    for (uint16_t i = 0; i < Size_of_ShootString; i++) {
        ShootString.data[i] = 0;
    }
}

void UpdateShot(time_t now, ShotInput_t input) {
    uint24_t dt, ddt;
    dt = (uint24_t) (now - ShootString_start_time);
    if (ShootString.TotShoots == 0) {
        ddt = 0;
    } else {
        ddt = ShootString.shots[ShootString.TotShoots - 1].dt;
    }
    ddt = dt - ddt;
    //Don't count shoots less than Filter
    if (ddt > Settings.Filter) {
        ShootString.shots[ShootString.TotShoots].dt = dt;
        ShootString.shots[ShootString.TotShoots].is_flags = input;
        ShootString.TotShoots++;
        if (ShootString.TotShoots >= MAXSHOOT)
            timerEventToHandle = TimerTimeout;
    }
}

void update_screen_model() {
    time_t now = rtc_time.unix_time_ms;
    switch (ui_state) {
        case TimerListening:
            if (now - ShootString_start_time >= MAX_MEASUREMENT_TIME) {
                timerEventToHandle = TimerTimeout;
            }
            switch (Settings.InputType) {
                case INPUT_TYPE_Microphone:
                    ADC_ENABLE_INTERRUPT_ENVELOPE;
                    break;
                case INPUT_TYPE_A_or_B_multiple:
                    if (InputFlags.A_RELEASED && AUX_A) {
                        UpdateShootNow(A);
                    } else if (InputFlags.B_RELEASED && AUX_B) {
                        UpdateShootNow(B);
                    }
                    break;
                case INPUT_TYPE_A_and_B_single:
                    if (InputFlags.A_RELEASED && AUX_A) {
                        UpdateShootNow(A);
                    } else if (InputFlags.A_RELEASED && AUX_B) {
                        UpdateShootNow(B);
                    }
                    // TODO: Fix This should capture only one of A and one of B
                    timerEventToHandle = TimerTimeout;
                    break;
            }
            if (ParNowCounting) {
                // Software "interrupt" emulation
                if (now - parStartTime_ms > Settings.ParTime[CurPar_idx]) {
                    ParNowCounting = false;
                    timerEventToHandle = ParEvent;
                    CurPar_idx++;
                }
            }
            break;
        case TimerCountdown:
            if (now - countdown_start_time >= Settings.DelayTime) {
                comandToHandle = CountdownExpired;
            }
            break;
        default:

            //do nothing, we're in ISR
            break;
    }
}

void handle_rotation() {
    if (Autorotate){
        if(InputFlags.TiltChanged) {
        InputFlags.TiltChanged = False;
        lcd_clear();
        lcd_set_orientation();
    }
    }
}
// <editor-fold defaultstate="collapsed" desc="ISR function">

static void interrupt isr(void) {

    if (RTC_TIMER_IF) {
        RTC_TIMER_IF = 0; // Clear Interrupt flag.
        PIR6bits.TMR1GIF = 0;
        frames_count = 0;
        define_charger_state();
        switch (ui_state) {
            case TimerListening:
            case TimerCountdown:
                break;
            default:
                ADC_ENABLE_INTERRUPT_BATTERY;
                break;
        }
    }
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        while (GO_nDONE);

        switch (ADPCH) {
            case ENVELOPE:
                ADC_BUFFER_PUT(ADC_SAMPLE_REG_16_BIT);
                if (ui_state == TimerListening && ADC_LATEST_VALUE > DetectThreshold) {
                    UpdateShootNow(Mic);
                }
                break;
            case BATTERY:
            {
                adc_battery = ADC_SAMPLE_REG_16_BIT;
                uint16_t battery_mV = adc_battery*BAT_divider;
                battery_level = battery_mV;
                //                battery_level = (battery_mV / 8) - 320; // "/10" ((battery_mV-3200)*100)/(3900-3200)
                //                if (battery_level > 99) battery_level = 99;
                // Trigger measurement of orientation
                ADC_ENABLE_INTERRUPT_ACCELEROMETR;
            }
                break;
            case ACCELEROMETER:
            {
                uint8_t old_orientation = InputFlags.orientation;
                uint16_t a = ADC_SAMPLE_REG_16_BIT;
                InputFlags.orientation = (a > ORIENTATION_INVERSE_THRESHOLD);
                InputFlags.TiltChanged = old_orientation!=InputFlags.orientation;
            }
                break;
        }
    }
    if (PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        update_rtc_time;
        if (rtc_time.msec < 100)
            update_rtc_time;
        if (!Keypressed) {//Assignment will not work because of not native boolean
            InputFlags.KEY_RELEASED = True;
        }
        update_screen_model();
    }
}
// </editor-fold>

void main(void) {
    // <editor-fold defaultstate="collapsed" desc="Initialization">
    DoPowerOn();
    init_ms_timer0();
    initialize_rtc_timer();
    ei();
    getSettings();
    getShootString(0);

    if (Settings.version != FW_VERSION) {
        clearHistory();
        getDefaultSettings();
        saveSettings();
    }
    set_backlight(Settings.BackLightLevel);
    getShootString(0);


    // Initialization End
    // </editor-fold>
    // <editor-fold defaultstate="collapsed" desc="Main">
    //    eeprom_clear_block(0x0, EEPROM_MAX_SIZE);
    lcd_clear();
    while (True) {
        //TODO: Integrate watchdog timer
        handle_rotation();
        handle_ui();
        frames_count++;
    }
    // </editor-fold>
}
