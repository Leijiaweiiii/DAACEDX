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

// <editor-fold defaultstate="collapsed" desc="EEPROM Functions">
// Helper functions.

void eeprom_spi_init() {
    EEPROM_CS_INIT();
    EEPROM_HOLD_INIT();
    EEPROM_WP_INIT();

    EEPROM_CS_DESELECT();
    EEPROM_HOLD_DIS();
    EEPROM_WP_DIS();

    RD7PPS = 0x1C; // SPI2 dataout (RD7)
    SSP2DATPPS = 0x1D; // SPI2 datain.
    RD6PPS = 0x1B; // SPI2 clock.

    SSP2STAT &= 0x3F;
    SSP2CON1 = 0x00; // power on state.
    SSP2CON1bits.SSPM = 0b0010;
    SSP2STATbits.SMP = 0;

    SSP2CON1bits.CKP = 1;
    SSP2STATbits.CKE = 0;
    SSP2CON1bits.SSPEN = 1;
    SSP2CON1bits.WCOL = 0;
}

uint8_t eeprom_spi_write(uint8_t data) {
    uint8_t temp_var = SSP2BUF; // Clear buffer.
    UNUSED(temp_var);
    PIR3bits.SSP2IF = 0; // Clear interrupt flag bit
    SSP2CON1bits.WCOL = 0; // Clear write collision bit if any collision occurs
    SSP2BUF = data;
    while (SSP2STATbits.BF == 0);
    PIR3bits.SSP2IF = 0; // clear interrupt flag bit
    return SSP2BUF;
}

void eeprom_init() {
    eeprom_spi_init();
}

void eeprom_write_data(uint16_t address, uint8_t data) {
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_WRSR);
    eeprom_spi_write(0x02); // Enable Write Latch.
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
    eeprom_write_data(address, data & 0xFF);
    eeprom_write_data(address + 1, (data >> 8) & 0xFF);
}

void eeprom_write_tdata(uint16_t address, uint24_t data) {
    eeprom_write_data(address, data & 0xFF);
    eeprom_write_data(address + 1, (data >> 8) & 0xFF);
    eeprom_write_data(address + 2, (data >> 16) & 0xFF);
}

uint8_t eeprom_read_data(uint16_t address) {
    uint8_t read_data;
    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_data = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_data);
}

uint16_t eeprom_read_array(uint16_t address, uint8_t *data, uint16_t no_of_bytes) {
    uint16_t index;

    if (address > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    for (index = 0; index < no_of_bytes; index++) {
        if (address + index > EEPROM_MAX_SIZE) return (index);
        data[index] = eeprom_spi_write(0x00);
    }
    EEPROM_CS_DESELECT();
    return (index);
}

uint16_t eeprom_read_wdata(uint16_t address) {
    uint8_t read_least, read_most;
    if (address + 1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_least = eeprom_spi_write(0x00);
    read_most = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_most << 8)+read_least;
}

uint24_t eeprom_read_tdata(uint16_t address) {
    uint8_t read_least, read_mid, read_most;
    if (address + 1 > EEPROM_MAX_SIZE) return 0;
    eeprom_busy_wait();

    EEPROM_CS_SELECT();
    eeprom_spi_write(CMD_READ);
    eeprom_spi_write(address >> 8);
    eeprom_spi_write(address & 0xFF);
    read_least = eeprom_spi_write(0x00);
    read_mid = eeprom_spi_write(0x00);
    read_most = eeprom_spi_write(0x00);
    EEPROM_CS_DESELECT();
    return (read_most << 16)+(read_mid << 8) + read_least;
}

void eeprom_busy_wait() {
    while (eeprom_read_status_reg() & 0x01);
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

// <editor-fold defaultstate="collapsed" desc="Small Routines">

void TestBattery(void) {
    if (ui_state == TimerListening) return; // Don't measure anything when testing shots
    uint16_t battery = ADC_Read(BATTERY);
    uint16_t battery_mV = battery*BAT_divider;
    battery_level = (battery_mV / 8) - 320; // "/10" ((battery_mV-3200)*100)/(3900-3200)
    if (battery_level > 99) battery_level = 99;
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Settings Display">

uint8_t SettingsTitle(SettingsMenu_t* sm) {
    set_screen_title(sm->MenuTitle);
    print_header();
    return UI_HEADER_END_LINE;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Save and retrive DATA">

void saveSettings() {
    eeprom_write_wdata(Sensitivity_Address, Sensitivity);
    eeprom_write_wdata(Filter_Address, Filter);
    eeprom_write_wdata(AutoStart_Address, AutoStart);
    eeprom_write_wdata(AR_IS_Address, AR_IS.AR_IS);
    eeprom_write_wdata(BuzzerFrequency_Address, BuzzerFrequency);
    eeprom_write_wdata(BuzzerParDuration_Address, BuzzerParDuration);
    eeprom_write_wdata(BuzzerStartDuration_Address, BuzzerStartDuration);
    eeprom_write_wdata(BuzzerLevel_Address, BuzzerLevel);
    eeprom_write_wdata(CustomCDtime_Address, CustomCDtime);
    //    eeprom_write_wdata(BT_Address, BT);
    eeprom_write_wdata(Delay_Address, DelayMode);
    eeprom_write_wdata(DelayTime_Address, DelayTime);
    eeprom_write_wdata(BackLightLevel_Address, BackLightLevel);
}

void getSettings() {
    Sensitivity = eeprom_read_wdata(Sensitivity_Address);
    Filter = eeprom_read_wdata(Filter_Address);
    //    AutoStart = eeprom_read_wdata(AutoStart_Address);
    AR_IS.AR_IS = eeprom_read_wdata(AR_IS_Address);
    BuzzerFrequency = eeprom_read_wdata(BuzzerFrequency_Address);
    BuzzerParDuration = eeprom_read_wdata(BuzzerParDuration_Address);
    BuzzerStartDuration = eeprom_read_wdata(BuzzerStartDuration_Address);
    BuzzerLevel = eeprom_read_wdata(BuzzerLevel_Address);
    CustomCDtime = eeprom_read_wdata(CustomCDtime_Address);
    //    BT = eeprom_read_wdata(BT_Address);
    DelayMode = eeprom_read_wdata(Delay_Address);
    DelayTime = eeprom_read_wdata(DelayTime_Address);
    BackLightLevel = eeprom_read_wdata(BackLightLevel_Address);
}

void getDefaultSettings() {
    Sensitivity = 5;
    Filter = 70;
    AR_IS.Autostart = 1;
    AR_IS.Mic = 1;
    AR_IS.AutoRotate = 0;
    AR_IS.BT = 1;
    //    AR_IS.AR_IS=0xFF;
    BuzzerFrequency = 1500;
    BuzzerParDuration = 200;
    BuzzerStartDuration = 300;
    BuzzerLevel = 1;
    CustomCDtime = 2000;
    DelayMode = Fixed;
    DelayTime = 3000;
    BackLightLevel = 2;
}

void savePar(uint8_t i) {
    eeprom_write_tdata(ParAddress + (i * 3) + 1, ParTime[i]);
}

void saveTotPar() {
    eeprom_write_data(ParAddress, TotPar);
}

void getPar() {
    TotPar = eeprom_read_data(ParAddress);
    for (uint8_t i = 0; i < TotPar; i++)
        ParTime[i] = eeprom_read_tdata(ParAddress + (i * 3) + 1);
}

uint16_t findCurrStringAddress() {
    uint16_t add = ShootStringStartAddress;
    CurrStringStartAddress = 0;
    uint8_t data;
    do {
        data = eeprom_read_data(add);
        if (data == 1)
            CurrStringStartAddress = add;
        else add += sizeof (ShootString);
    } while ((data != 1) && (add < (ShootStringStartAddress + 30 * sizeof (ShootString))));
    return CurrStringStartAddress;
}

void saveShootString(void) {
    uint16_t Address;
    // current string is overwritten over the older string

    findCurrStringAddress(); // the address of stored shoot string 0
    if (ShootStringStartAddress == CurrStringStartAddress)
        Address = ShootStringStartAddress + (29 * Size_of_ShootString);
    else Address = (CurrStringStartAddress - Size_of_ShootString);
    eeprom_write_data(CurrStringStartAddress, 0); //No longer current
    eeprom_write_data(Address, 1); //Mark current string
    Address++;
    eeprom_write_data(Address, ShootString.TotShoots);
    Address++;
    for (uint8_t i = 0; i < ShootString.TotShoots; i++) {
        eeprom_write_tdata(Address, ShootString.ShootTime[i]);
        Address += 3;
    }
}

TBool getShootString(uint8_t ShootStrNum) {
    uint16_t Address;

    findCurrStringAddress(); // the address of shoot string 0
    uint16_t StrBeforeCurr = ((CurrStringStartAddress - ShootStringStartAddress) / Size_of_ShootString);
    if ((30 - StrBeforeCurr) > ShootStrNum) Address = CurrStringStartAddress + (ShootStrNum * Size_of_ShootString);
    else Address = ShootStringStartAddress + (((ShootStrNum + StrBeforeCurr) - 30) * Size_of_ShootString);
    uint8_t mark = eeprom_read_data(Address);
    Address++;
    ShootString.TotShoots = eeprom_read_data(Address);
    Address++;
    for (uint8_t i = 0; i < ShootString.TotShoots; i++) {
        ShootString.ShootTime[i] = eeprom_read_tdata(Address);
        Address += 3;
    }
    return (((ShootStrNum == 0) && (mark == 1)) || ((ShootStrNum > 0) && ((mark == 0))));
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Settings">

// <editor-fold defaultstate="collapsed" desc="Delay Settings">

void SetCustomDelay() {
    NumberSelection_t n;
    // TODO: Check if static strmycpy may be replaced with assignment
    strmycpy(n.MenuTitle, "Custom Delay");
    n.fmin = 0.1;
    n.fmax = 10.0;
    n.fstep = 0.1;
    n.fvalue = (double) DelayTime / 1000;
    n.fold_value = n.fvalue;
    n.format = "%2.1f";
    do {
        DisplayDouble(&n);
        SelectDouble(&n);
    } while (!n.done);
    // TODO: Apply or function to save flag everywhere like here
    SaveToEEPROM |= (n.fold_value != n.fvalue);
}

void SetDelay(SettingsMenu_t * m) {
    InitSettingsMenuDefaults(m);
    m->TotMenuItems = 4;
    strmycpy(m->MenuTitle, "Delay");
    strmycpy(m->MenuItem[0], " Instant ");
    strmycpy(m->MenuItem[1], " Fixed 3sec. ");
    strmycpy(m->MenuItem[2], " Random");
    strmycpy(m->MenuItem[3], " Custom ");

    switch (DelayMode) {
        case Instant: m->menu = 1;
            break;
        case Fixed: m->menu = 2;
            break;
        case Random: m->menu = 3;
            break;
        case Custom: m->menu = 4;
            break;
        default: m->menu = 2;
            break;
    }

    do {
        DisplaySettings(m);
        SelectMenuItem(m);
    } while (!m->done);

    switch (m->menu) {
        case 1: DelayMode = Instant;
            SaveToEEPROM = True;
            break;
        case 2: DelayMode = Fixed;
            DelayTime = 30;
            SaveToEEPROM = True;
            break;
        case 3: DelayMode = Random;
            SaveToEEPROM = True;
            break;
        case 4: DelayMode = Custom;
            SetCustomDelay();
            break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Par">

TBool DeletePar(uint8_t Par_i) {
    char msg[20];
    sprintf(msg, "Delete Par%d ?", Par_i + 1);
    while (Keypressed);
    if (PopMsg(msg, 0) == KeyIn) {
        for (uint8_t i = Par_i; i < TotPar; i++) {
            ParTime[i] = ParTime[i + 1];
            savePar(i);
        }
        TotPar--;
        saveTotPar();
        sprintf(msg, "Par%d Deleted", Par_i + 1);
        PopMsg(msg, 200);
        return True;
    } else
        return False;
}

void edit_par(SettingsMenu_t * s) {
    uint8_t i, j, k;
    TBool Done = False;
    char msg[15];
    i = 10;
    j = 0;
    k = 0;
    while (!Done) {
        if (Keypressed) {
            switch (Key) {
                case KeyUp:
                    if ((ParTime[s->menu - 1] + i) <= ((ParTime[s->menu])-(BuzzerParDuration))) {
                        ParTime[s->menu - 1] += i;
                    } else {
                        DeletePar(s->menu - 1); //DeletePar will save if needed no need change
                        Done = True;
                    }
                    break;
                case KeyDw:
                    if (TotPar > 1) {
                        if ((ParTime[s->menu - 1] - i) >= ((ParTime[s->menu - 2]) + BuzzerParDuration)) {
                            ParTime[s->menu - 1] -= i;
                        } else {
                            DeletePar(s->menu - 1);
                            Done = True;
                        }
                    } else if (TotPar == 1) {
                        if (ParTime[0] >= ((BuzzerStartDuration) + i)) {
                            ParTime[0] -= i;
                        } else {
                            DeletePar(0);
                            Done = True;
                        }
                    }
                    break;
                default: Done = True;
            }

            sprintf(msg, "%5.1f", (float) ParTime[s->menu - 1] / 1000); //unit is 1mS
            lcd_write_string(msg, 30, UI_HEADER_END_LINE, BigFont, BLACK_OVER_WHITE);
            sprintf(msg, " Par %2d: %5.1f", s->menu, (float) ParTime[s->menu - 1] / 1000); //unit is 1mS
            strmycpy(s->MenuItem[s->menu - 1], msg);

        }
        while ((Keypressed) && (k < 250)) {
            __delay_ms(1);
            k++;
        }
        k = 0;
        if (Keypressed) j++;
        if (j > 1) {
            if (i * 2 < 255) i = i * 2;
            else i = 255;
            j = 0;
        } //if after 500mS still pressed go faster
        if (!Keypressed) {
            i = 10;
            j = 0;
        } //if key not pressed go slow (again))
    }
}

void SetPar(SettingsMenu_t * m) {
    char msg[15];

    TBool changed;
    uint8_t redraw, i;

    //Main Screen
    InitSettingsMenuDefaults(m);
    redraw = 2;
    changed = False;
    while (m->menu > 0) {
        if (redraw > 0) {
            i = 0;
            TotPar = 0;
            do {
                //TODO: Check if this is not a cause of any problem
                if ((ParTime[i] > 0) && (ParTime[i] < 100000)) {
                    sprintf(msg, " Par %d: %5.02f", i + 1, (float) ParTime[i] / 1000); //unit is 1mS
                    strmycpy(m->MenuItem[i], msg);
                    TotPar++;
                }
                i++;
            } while ((ParTime[i] > 0) && (ParTime[i] < 100000) && (i <= MAXPAR));

            sprintf(m->MenuTitle, "Total Par=%d", TotPar);
            sprintf(m->MenuItem[TotPar], " Par %d: Off", TotPar + 1);
            m->TotMenuItems = TotPar + 1;
        }
        DisplaySettings(m);

        SelectMenuItem(m);
        if (m->menu > 0) {
            if (m->menu == m->TotMenuItems) {
                if (TotPar > 0) {
                    ParTime[m->menu - 1] = ParTime[m->menu - 2] + BuzzerParDuration;
                    TotPar++;
                } else {
                    ParTime[0] = BuzzerStartDuration;
                    TotPar = 1;
                    changed = True;
                    redraw = 1;
                }
            }
            edit_par(m);
        }
    }
    if (changed) {
        saveTotPar();
        for (i = 0; i < TotPar; i++)
            savePar(i);
        PopMsg("Par Saved", 200);
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Backlight">

void SetBacklight() {//PWM Backlite
    NumberSelection_t b;
    strmycpy(b.MenuTitle, "Backlight");
    b.max = 99;
    b.min = 1;
    b.step = 1;
    b.value = BackLightLevel;
    b.old_value = b.value;
    b.format = "%u";
    b.done = False;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
    } while (!b.done);
    BackLightLevel = b.value;
    SaveToEEPROM != (b.value != b.old_value);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Buzzer Settings">

void SetBeepFreq() {
    NumberSelection_t b;
    b.min = 800;
    b.max = 3000;
    b.value = BuzzerFrequency;
    b.old_value = b.value;
    strmycpy(b.MenuTitle, "Tone");
    b.step = 100;
    b.format = "%u";
    b.done = False;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
    } while (!b.done);
    BuzzerFrequency = b.value;
    SaveToEEPROM |= (b.value != b.old_value);
}

void SetBeepLevel() {
    NumberSelection_t b;
    if (BuzzerLevel > 100) BuzzerLevel = 100;
    strmycpy(b.MenuTitle, "Loudness");
    b.min = 0;
    b.max = 100;
    b.step = 1;
    b.format = "%u";
    b.value = BuzzerLevel;
    b.old_value = b.value;
    b.done = False;
    do {
        DisplayInteger(&b);
        SelectInteger(&b);
    } while (!b.done);
    BuzzerLevel = b.value;
    SaveToEEPROM |= (b.value != b.old_value);
}

void SetBeepTime(TBool Par) {
    NumberSelection_t d;
    if (Par) {
        d.value = BuzzerParDuration;
        strmycpy(d.MenuTitle, "Par Duration ms");
    } else {
        d.value = BuzzerStartDuration;
        strmycpy(d.MenuTitle, "Start Duration ms");
    }
    d.min = 50;
    d.max = 1000;
    d.step = 50;
    d.old_value = d.value;
    d.done = False;
    do {

    } while (!d.done);

    if (Par) BuzzerParDuration = d.value;
    else BuzzerStartDuration = d.value;
    SaveToEEPROM |= (d.value != d.old_value);
}

void SetBeep(SettingsMenu_t * m) {
    InitSettingsMenuDefaults(m);
    m->TotMenuItems = 5;
    strmycpy(m->MenuTitle, "Beep");
    strmycpy(m->MenuItem[0], " Frequency ");
    strmycpy(m->MenuItem[1], " Loudness ");
    strmycpy(m->MenuItem[2], " Par Duration ");
    strmycpy(m->MenuItem[3], " Start Duration ");
    strmycpy(m->MenuItem[4], " Test Beep ");


    do {
        DisplaySettings(m);
        SelectMenuItem(m);
        switch (m->menu) {
            case 1:
                SetBeepFreq();
                break;
            case 2:
                SetBeepLevel();
                break;
            case 3:
                SetBeepTime(True);
                break;
            case 4:
                SetBeepTime(False);
                break;
            case 5:
                generate_sinus(BuzzerLevel, BuzzerFrequency, BuzzerParDuration);
                break;
        }
    } while (m->menu != 0);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Sensitivity">

void SetSens() {//Sensitivity
    NumberSelection_t s;
    if (Sensitivity > 10) Sensitivity = 10;
    strmycpy(s.MenuTitle, "Sensitivity");
    s.max = 10;
    s.min = 1;
    s.value = Sensitivity;
    s.old_value = Sensitivity;
    s.step = 1;
    s.format = "%u";
    do {
        DisplayInteger(&s);
        SelectInteger(&s);
    } while (!s.done);

    SaveToEEPROM = (s.value != s.old_value);
}

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Filter">

void SetFilter() {
    if (Filter > 100) Filter = 100;
    NumberSelection_t f;
    strmycpy(f.MenuTitle, "Filter");
    f.min = 10;
    f.max = 100;
    f.step = 10;
    f.value = Filter;
    f.old_value = f.value;
    f.done = False;
    f.format = "%3d";
    do {
        DisplayInteger(&f);
        SelectInteger(&f);
    } while (!f.done);
    Filter = f.value;
    SaveToEEPROM |= (f.value != f.old_value);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="AutoStart">

void SetAutoStart(SettingsMenu_t * m) {
    TBool orgset;
    InitSettingsMenuDefaults(m);
    m->menu = (AutoStart) ? 2 : 1;
    m->TotMenuItems = 2;
    strmycpy(m->MenuTitle, "Autostart");
    strmycpy(m->MenuItem[0], " Auto Start OFF ");
    strmycpy(m->MenuItem[1], " Auto Start ON ");
    SettingsTitle(m);


    orgset = AutoStart;
    do {
        DisplaySettings(m);
        SelectMenuItem(m);
    } while (!m->done);
    AutoStart = (m->menu == 2) ? True : False;
    SaveToEEPROM |= (AutoStart != orgset);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="TimerMode">

void SetMode(SettingsMenu_t * m) {
    TBool orgset;
    InitSettingsMenuDefaults(m);
    m->TotMenuItems = 8;
    strmycpy(m->MenuTitle, "Mode");
    strmycpy(m->MenuItem[0], " Timer Mode ");
    strmycpy(m->MenuItem[1], " Biancchi ");
    strmycpy(m->MenuItem[2], " Barricade ");
    strmycpy(m->MenuItem[3], " Falling Plate ");
    strmycpy(m->MenuItem[4], " NRA-PPC A ");
    strmycpy(m->MenuItem[5], " NRA-PPC B ");
    strmycpy(m->MenuItem[6], " NRA-PPC C ");
    strmycpy(m->MenuItem[7], " NRA-PPC D ");

    SettingsTitle(m);

    //Main Screen
    do {
        DisplaySettings(m);
        SelectMenuItem(m);
    } while (!m->done);
    switch (m->menu) { //TODO implement
        case 1://TBC
            break;
        case 2:
            break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Clock">

void SetClock() {
    uint8_t hour = get_hour();
    uint8_t minute = get_minute();
    strmycpy(ts.MenuTitle, "Clock");
    if (hour > 23) hour = 23;
    if (minute > 59) minute = 59;
    ts.hour = hour;
    ts.minute = minute;
    do {
        DisplayTime(&ts);
        SelectTime(&ts);
    } while (!ts.done);

    set_time(ts.hour, ts.minute, 0);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="CountDown">

void CountDownMode(uint8_t cdt, SettingsMenu_t * m) {
    uint8_t top, prev_cdtime, tcount;
    uint16_t cdtime;
    TBool Run;
    char msg[20];

    prev_cdtime = 0;
    strmycpy(m->MenuTitle, "Countdown");
    top = SettingsTitle(m);
    while (!Exit) {
        cdtime = cdt * 10;
        Run = False;
        tcount = 0;
        while (cdtime > 0) {
            if (cdtime != prev_cdtime) {
                sprintf(msg, "%02d:%02d", cdtime / 60, cdtime % 60);
                lcd_write_string(msg, 10, top + 30, BigFont, BLACK_OVER_WHITE);
                prev_cdtime = cdtime;
            }
            switch (Key) {
                case KeySt: Run = True;
                    break;
                case KeyRw: Run = False;
                    break;
                case KeyBk: Run = False;
                    cdtime = 0;
                    break; //Exit no sound
                case KeyIn: Run = True;
                    cdtime = 0;
                    break; //Exit with sound
            }
            if (Run && (tcount > 98)) {
                tcount = 0;
                cdtime--;
            }
            __delay_ms(10);
            if (Run) tcount++;
        }
        if (Run) {
            for (char i = 0; i < 5; i++) {
                for (char j = 0; j < 3; j++) {
                    generate_sinus(1, BuzzerFrequency, 50);
                    for (char t = 0; t < 100; t++) {
                        if (Keypressed) return;
                        __delay_ms(1);
                    }
                }
                for (char t = 0; t < 200; t++) {
                    if (Keypressed) return;
                    __delay_ms(2);
                }
            }
        }
    }
}

void SetCustomCountDown() {
    strmycpy(ts.MenuTitle, "Custom Countdown");
    // TODO: Review time format here
    // Hour means minute here
    ts.hour = CustomCDtime / 60000;
    // Minute means seconds
    ts.minute = CustomCDtime / 1000;
    ts.old_hour = ts.hour;
    ts.old_minute = ts.minute;
    ts.done = False;
    do {
        DisplayTime(&ts);
        SelectTime(&ts);
    } while (!ts.done);
    CustomCDtime = ts.hour * 60000 + ts.minute * 1000;
    SaveToEEPROM = (ts.hour != ts.old_hour || ts.minute != ts.old_minute);
}

void SetCountDown(SettingsMenu_t * m) {
    InitSettingsMenuDefaults(m);
    m->TotMenuItems = 4;
    strmycpy(m->MenuTitle, "Countdown");
    strmycpy(m->MenuItem[0], " Off ");
    strmycpy(m->MenuItem[1], " 3 minutes ");
    strmycpy(m->MenuItem[2], " 5 minutes ");
    strmycpy(m->MenuItem[3], " Custom ");

    //Main Screen
    do {
        DisplaySettings(m);
        SelectMenuItem(m);
    } while (!m->done);
    switch (m->menu) {
        case 2: CountDownMode(18, m);
            break;
        case 3: CountDownMode(30, m);
            break;
        case 4: SetCustomCountDown();
            CountDownMode(CustomCDtime, m);
            break;
    }
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Tilt">

void SetTilt(SettingsMenu_t * m) {
    uint8_t orgset;
    InitSettingsMenuDefaults(m);
    m->TotMenuItems = 1;
    strmycpy(m->MenuTitle, "Tilt");
    if (AR_IS.AutoRotate) strmycpy(m->MenuItem[0], " Auto Rotate ON ");
    else strmycpy(m->MenuItem[0], " Auto Rotate OFF ");
    SettingsTitle(m);
    // SettingsDisplay(&SetTiltMenu);
    orgset = AR_IS.AR_IS;

    while (!m->done) {
        DisplaySettings(m);
        if (Keypressed) {
            switch (Key) {
                case KeyIn:
                    AR_IS.AutoRotate != AR_IS.AutoRotate;
                    if (AR_IS.AutoRotate) strmycpy(m->MenuItem[0],
                            " Auto Rotate ON ");
                    else strmycpy(m->MenuItem[0], " Auto Rotate OFF ");

                    break;
                default:
                    //Done = True;
                    break;
            }
            while (Keypressed); // wait here till key is released
        }
    }
    SaveToEEPROM = (AR_IS.AR_IS != orgset);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Input">

void UpdateIS(SettingsMenu_t * sm) {
    if (AR_IS.Mic) strmycpy(sm->MenuItem[0], " Microphone: ON  ");
    else strmycpy(sm->MenuItem[0], " Microphone: OFF ");
    if (AR_IS.A) strmycpy(sm->MenuItem[1], " A: ON  ");
    else strmycpy(sm->MenuItem[1], " A: OFF ");
    if (AR_IS.B) strmycpy(sm->MenuItem[2], " B: ON  ");
    else strmycpy(sm->MenuItem[2], " B: OFF ");
}

void SetInput(SettingsMenu_t * m) {
    uint8_t orgset;
    InitSettingsMenuDefaults(m);
    m->TotMenuItems = 3;
    strmycpy(m->MenuTitle, "Input");
    UpdateIS(m);
    DisplaySettings(m);
    orgset = AR_IS.AR_IS;

    do {
        m->selected = 0;
        DisplaySettings(m);
        SelectBinaryMenuItem(m);
        switch (m->selected) {
            case 1:
                AR_IS.Mic ^= AR_IS.Mic;
                break;
            case 2:
                AR_IS.A ^= AR_IS.A;
                break;
            case 3:
                AR_IS.B ^= AR_IS.B;
                break;
        }
        UpdateIS(m);
    } while (!m->done);

    SaveToEEPROM = (AR_IS.AR_IS != orgset);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="BlueTooth">

void BlueTooth() {
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_write_string(" TBD9 ", 3, 40, MediumFont, BLACK_OVER_WHITE);
    __delay_ms(500);
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Settings Menu">

void DoSet(SettingsMenu_t * s) {
    SettingsMenu_t * m = &ma;
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    switch (s->menu) {
        case 1:SetDelay(m);
            break;
        case 2:SetPar(m);
            break;
        case 3:SetBeep(m);
            break;
        case 4:SetAutoStart(m);
            break;
        case 5:SetMode(m);
            break;
        case 6:SetClock();
            break;
        case 7:SetCountDown(m);
            break;
        case 8:SetTilt(m);
            break;
        case 9:SetBacklight();
            break;
        case 10:SetSens();
            break;
        case 11:SetFilter();
            break;
        case 12:SetInput(m);
            break;
        case 13:BlueTooth();
            break;
            //        case 14:
            //            saveSettings();
            //            PopMsg("Saved", 1000);
            //            break;
            //        case 15:Diagnostics();
            //            break;
    }
}

void SetSettingsMenu(SettingsMenu_t * SettingsMenu) {
    //{"Delay","Par","Beep","Auto","Mode","Clock","CountDown","Tilt","Bklight","Input","BT","Diag"};

    SettingsMenu->TotMenuItems = 13;

    strmycpy(SettingsMenu->MenuTitle, "Settings");
    strmycpy(SettingsMenu->MenuItem[0], " Delay");
    strmycpy(SettingsMenu->MenuItem[1], " Par");
    strmycpy(SettingsMenu->MenuItem[2], " Buzzer");
    strmycpy(SettingsMenu->MenuItem[3], " Auto Start");
    strmycpy(SettingsMenu->MenuItem[4], " Timer Mode");
    strmycpy(SettingsMenu->MenuItem[5], " Clock");
    strmycpy(SettingsMenu->MenuItem[6], " Countdown");
    strmycpy(SettingsMenu->MenuItem[7], " Tilt");
    strmycpy(SettingsMenu->MenuItem[8], " Backlight");
    strmycpy(SettingsMenu->MenuItem[9], " Sensitivity");
    strmycpy(SettingsMenu->MenuItem[10], " Filter");
    strmycpy(SettingsMenu->MenuItem[11], " Input");
    strmycpy(SettingsMenu->MenuItem[12], " Bluetooth");
    strmycpy(SettingsMenu->MenuItem[13], " SaveSettings");
    strmycpy(SettingsMenu->MenuItem[14], " Diagnostics");
}

void DoSettings(void) {
    set_screen_title("Settings");

    SettingsMenu.menu = 0;
    SettingsMenu.page = 0; //Force refresh
    SettingsMenu.selected = 0;
    SetSettingsMenu(&SettingsMenu);
    SettingsTitle(&SettingsMenu);
    SaveToEEPROM = False;

    lcd_clear_data_ram();
    do {
        handle_rotation();
        print_header();
        DisplaySettings(&SettingsMenu);
        SelectMenuItem(&SettingsMenu);
        if (SettingsMenu.selected > 0) DoSet(&SettingsMenu);
    } while (ui_state == SettingsScreen);
    if (SaveToEEPROM) {
        saveSettings();
        PopMsg("Saved", 200);
        return;
    }
}
// </editor-fold>
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="ReviewMenu">
#define REVIEW_SHOT_FORMAT      "#%2d: %3.2f "
#define REVIEW_SPLIT_FORMAT     "}%3.2f "

void ReviewDisplay(uint8_t battery, uint8_t CurShoot, uint8_t CurShootStringDisp, TBool scroll_shots) {
    uint8_t line = UI_HEADER_END_LINE;
    char message[20];
    // We're assuming here that Medium font has even number of bytes heigh
    uint8_t halfline = (MediumFont->height / 2);
    print_header();

    //String line

    line += halfline;
    sprintf(message, "Str#%02d/%02d - %3.2f ",
            CurShootStringDisp,
            ShootString.TotShoots,
            (float) ShootString.ShootTime[ShootString.TotShoots] / 1000);
    lcd_write_string(message, 12, line, MediumFont, BLACK_OVER_WHITE);
    line += MediumFont->height;
    for (uint8_t col = 5; col < LCD_WIDTH; col += lcd_string_lenght("_", MediumFont)) {
        lcd_write_char('_', col, line, MediumFont, BLACK_OVER_WHITE);
    }
    for (uint8_t i = UI_HEADER_END_LINE; i < line; i+=PAGE_HEIGTH) {
        if (scroll_shots) {
            lcd_write_string(" ", 0, i, MediumFont, BLACK_OVER_WHITE);
        } else {
            lcd_write_string("|", 1, i, MediumFont, BLACK_OVER_WHITE);
        }
    }

    line += MediumFont->height;
    //Shoot lines
    //1st ShootNumber 01, before it ShootNumber 00 time=0
    for (int8_t i = 0; i < SHOTS_ON_REVIEW_SCREEN; i++) {
        if (ShootString.ShootTime[CurShoot + i] > 0) {
            if (scroll_shots) {
                lcd_write_string("|", 1, line, MediumFont, BLACK_OVER_WHITE);
            } else {
                lcd_write_string(" ", 0, line, MediumFont, BLACK_OVER_WHITE);
            }
            sprintf(message,
                    REVIEW_SHOT_FORMAT,
                    CurShoot + i,
                    (float) ShootString.ShootTime[CurShoot + i] / 1000
                    );
            lcd_write_string(message, 5, line, MediumFont, (i != 1)& 0x01);
            line += halfline;

            // Don't print last diff at half line
            if (i < SHOTS_ON_REVIEW_SCREEN - 1) {
                sprintf(message,
                        REVIEW_SPLIT_FORMAT,
                        (float) (ShootString.ShootTime[CurShoot + i + 1] - ShootString.ShootTime[CurShoot + i]) / 1000);
                lcd_write_string(message, 100, line, MediumFont, BLACK_OVER_WHITE);
            }
            line += halfline;
        }
    }
}

void review_scroll_shot_up() {
    if (CurShoot > 1) {
        CurShoot--;
    } else Beep();
}

void review_scroll_shot_down() {
    if (CurShoot < ShootString.TotShoots - SHOTS_ON_REVIEW_SCREEN+1) {
        CurShoot++;
    } else Beep();
}

void review_previous_string() {
    if (CurShootString > 0) {
        CurShootString--;
        getShootString(CurShootString);
    } else Beep();
}

void review_next_string() {
    if (CurShootString < 30) {
        CurShootString++;
        getShootString(CurShootString);
    } else Beep();
}

void DoReview() {
    TBool scroll_shots = True;
    CurShootString = 0;
    CurShoot = 1;
    set_screen_title("Review");

    //    getShootString(CurShootString);
    lcd_clear_block(0, 0, LCD_WIDTH, LCD_HEIGHT);
    do {
        ReviewDisplay(battery_level, CurShoot, CurShootString + 1, scroll_shots);
        define_input_action();
        switch (comandToHandle) {
            case UpShort:
                if (scroll_shots)
                    review_scroll_shot_up();
                else
                    review_next_string();
                break;
            case ReviewShort:
            case DownShort:
                if (scroll_shots)
                    review_scroll_shot_down();
                else
                    review_previous_string();
                break;
            case BackShort:
            case OkShort:
                scroll_shots = !scroll_shots;
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
}
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Main Menu">

#define DETECT_THRESHOLD_LEVELS 10
uint8_t threshold_offsets[DETECT_THRESHOLD_LEVELS] = {/*220, 190, 160, */148, 124, 104, 87, 73, 61, 51, 43, 36, 33};

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
    DetectThreshold = Mean + threshold_offsets[Sensitivity - 1];
}

TBool Detect(void) {
    switch (DetectMode) {
        case Mic: return ADC_Read(ENVELOPE) > DetectThreshold;
        case AuxA: return AUX_A;
        case AuxB: return AUX_B;
        default: return False;
    }
}

uint8_t print_time() {
    char message[20];
    sprintf(message,
            "%02d%s%02d %s",
            get_hour(),
            (rtc_time.sec % 4) ? ":" : ".",
            //            ":",
            get_minute(),
            ScreenTitle);
    lcd_write_string(message, 0, 0, SmallFont, BLACK_OVER_WHITE);
    return SmallFont->height;
}

void print_batery_text_info() {
    char message[20];
    sprintf(message,
            "%d%%", (battery_level > 100) ? 100 : battery_level);
    uint8_t width = lcd_string_lenght(message, SmallFont);
    lcd_write_string(message, LCD_WIDTH - 8 - width, 0, SmallFont, BLACK_OVER_WHITE);
}

uint8_t print_header() {
    print_time();
    print_batery_text_info();
    return UI_HEADER_END_LINE;
}

void print_stats() {
    char message[32];
    uint8_t pos = 8;
    sprintf(message, "FPS:%d", frames_count);
    lcd_clear_block(pos, (LCD_HEIGHT - SmallFont->height), LCD_WIDTH, LCD_HEIGHT);
    lcd_write_string(message, pos, LCD_HEIGHT - SmallFont->height, SmallFont, BLACK_OVER_WHITE);
    sprintf(message, "RTC: %u.%03u", rtc_time_sec, get_ms_corrected());
    lcd_write_string(message, 55, LCD_HEIGHT - SmallFont->height, SmallFont, BLACK_OVER_WHITE);
    //
    //    sprintf(message, "TMR3=%d", TMR3);
    //    lcd_write_string(message, pos, (LCD_HEIGHT - 2 * SmallFont->height), SmallFont, BLACK_OVER_WHITE);
    //    sprintf(message, "TMR5=%d", TMR5);
    //    lcd_write_string(message, pos, (LCD_HEIGHT - 3 * SmallFont->height), SmallFont, BLACK_OVER_WHITE);
    //    sprintf(message, "t=%u.%u",rtc_time_sec,rtc_time_msec);
    //    lcd_clear_block(pos, (LCD_HEIGHT - 4 * SmallFont->height), LCD_WIDTH, (LCD_HEIGHT - 3 * SmallFont->height - 9));
    //    lcd_write_string(message, pos, (LCD_HEIGHT - 4 * SmallFont->height - 8), SmallFont, BLACK_OVER_WHITE);
    //    sprintf(message, "c=%.3f",(float)corrected_time_msec()/1000);
    //    lcd_clear_block(pos, (LCD_HEIGHT - 5 * SmallFont->height), LCD_WIDTH, (LCD_HEIGHT - 3 * SmallFont->height - 9));
    //    lcd_write_string(message, pos, (LCD_HEIGHT - 5 * SmallFont->height - 8), SmallFont, BLACK_OVER_WHITE);
    //
    ////    sprintf(message, "SYNC EVC=%d", ms_int);
    ////    lcd_write_string(message, pos, (LCD_HEIGHT - 5 * SmallFont->height - 8), SmallFont, BLACK_OVER_WHITE);
}

// TODO: Implement
//void print_footer_grid() {
//    for (uint8_t i = UI_FOOTER_START_LINE; i <= LCD_HEIGHT; i += UI_FOOTER_GRID_HEIGH) {
//        lcd_draw_hline(0, LCD_WIDTH, i, BLACK_OVER_WHITE);
//    }
//    for (uint8_t i = 0; i <= LCD_WIDTH; i += UI_FOOTER_GRID_WIDTH) {
//        lcd_draw_line(i, UI_FOOTER_START_LINE, i, LCD_HEIGHT, BLACK_OVER_WHITE);
//    }
//}

void print_label_at_footer_grid(const char* msg, const uint8_t grid_x, const uint8_t grid_y) {
    //    lcd_clear_block_d(
    //            UI_FOOTER_GRID_X(grid_x),
    //            UI_FOOTER_GRID_Y(grid_y),
    //            UI_FOOTER_GRID_X(grid_x)+UI_FOOTER_GRID_WIDTH,
    //            UI_FOOTER_GRID_Y(grid_y)+UI_FOOTER_GRID_HEIGH
    //            );
    lcd_write_string(msg, UI_FOOTER_GRID_X(grid_x), UI_FOOTER_GRID_Y(grid_y), SmallFont, BLACK_OVER_WHITE);
}

uint8_t print_footer() {
    //    print_stats();
    uint8_t line = UI_FOOTER_START_LINE + 2;
    char message[20];
    //    print_footer_grid();
    switch (DelayMode) {
        case Instant:sprintf(message, " Instant");
            break;
        case Fixed: sprintf(message, " Fixed");
            break;
        case Random: sprintf(message, " Random");
            break;
        case Custom: sprintf(message, " Custom");
            break;
    }
    print_label_at_footer_grid(message, 0, 0);
    if (AR_IS.Mic) sprintf(message, "  Mic: %d", Sensitivity);
    else sprintf(message, " Mic: Off");
    print_label_at_footer_grid(message, 1, 0);

    //    if (ParTime[CurPar_idx] > 0) {
    //        sprintf(message, "Par%d: %3.1f", CurPar_idx + 1, (float) ParTime[CurPar_idx] / 1000);
    sprintf(message, "ADC: %d   ", ADC_LATEST_VALUE);
    //    } else {
    //        sprintf(message, " Par: Off");
    //    }
    print_label_at_footer_grid(message, 0, 1);
    if (AR_IS.Aux) print_label_at_footer_grid(" Aux: ON", 1, 1);
    else print_label_at_footer_grid(" Aux: Off", 1, 1);

    //    sprintf(message, " Buz:%d", BuzzerLevel);
    sprintf(message, "FPS:%02d ", frames_count);
    print_label_at_footer_grid(message, 2, 0);

    sprintf(message, " %s %s %s",
            (AR_IS.A) ? "A" : " ",
            (AR_IS.B) ? "B" : " ",
            (AR_IS.BT) ? "BT" : " "
            );
    print_label_at_footer_grid(message, 2, 1);

    return line - UI_FOOTER_START_LINE;
}

void DoMain(void) {
    measurement_start_time_msec = rtc_time.unix_time_ms;
    ShootString.ShootStringMark = 1;
    ShootString.TotShoots = 0;
    for (int i = 0; i < MAXSHOOT; i++)
        ShootString.ShootTime[i] = 0;
    DetectInit();
    //    DoOldMain();
}
// </editor-fold>

void DoPowerOff() {
    PowerOFF;
    set_backlight(0);
    lcd_clear_data_ram();
}

void DoPowerOn() {
    PowerON;
    set_backlight(BackLightLevel);
}

void clear_timer_area() {
    lcd_clear_block(0, UI_HEADER_END_LINE, 0, BigFont->height + MediumFont->height);
}

void update_shot_time_on_screen() {
    //    clear_timer_area();
    uint8_t c = ShootString.TotShoots; // Declare for code ease. TODO: Clean after
    time_t t = 0, dt = 0;
    switch (c) {
        case 0:
            t = 0;
            break;
        case 1:
            t = ShootString.ShootTime[c - 1];
            dt = t;
            break;
        default:
            t = ShootString.ShootTime[c - 1];
            dt = t - ShootString.ShootTime[c - 2];
            break;
    }
    print_line_with_shots_and_split(c, dt);
    print_big_time_label(t);
}

void PlayParSound() {
    generate_sinus(BuzzerLevel, BuzzerFrequency, BuzzerParDuration);
}

void PlayStartSound() {
    generate_sinus(BuzzerLevel, BuzzerFrequency, BuzzerStartDuration);
}

void StartParTimer() {
    if (CurPar_idx < MAXPAR) {
        CurPar_idx++;
        ParNowCounting = true;
        parStartTime_ms = rtc_time_msec;
    }
}

void StartCountdownTimer() {
    switch (DelayMode) {
        case Instant: DelayTime = 0;
            break;
        case Fixed: DelayTime = 3000;
            break;
        case Random: DelayTime = (5 * DelayTime) % 4999;
            break;
        case Custom: DelayTime = eeprom_read_wdata(DelayTime_Address); //TODO: read model once at the power on
            //Read again in case was changed from other mode
            break;
    }
    countdown_start_time = rtc_time.unix_time_ms;
}

void UpdateShot(time_t now) {
    time_t dt, ddt;
    dt = now - measurement_start_time_msec;
    if (ShootString.TotShoots == 0) {
        ddt = 0;
    } else {
        ddt = ShootString.ShootTime[ShootString.TotShoots - 1];
    }
    ddt = dt - ddt;
    //Don't count shoots less than Filter
    if (ddt > Filter) {
        ShootString.ShootTime[ShootString.TotShoots] = dt;
        ShootString.TotShoots++;
        if (ShootString.TotShoots >= MAXSHOOT)
            timerEventToHandle = TimerTimeout;
    }
}

void UpdateShootNow() {
    UpdateShot(rtc_time.unix_time_ms);
}

void update_screen_model() {
    time_t now = rtc_time.unix_time_ms;
    switch (ui_state) {
        case TimerListening:
            ADC_ENABLE_INTERRUPT_ENVELOPE;
            if (ParNowCounting) {
                // Software "interrupt" emulation
                if (now - parStartTime_ms >= ParTime[CurPar_idx]) {
                    ParNowCounting = false;
                    timerEventToHandle = ParEvent;
                }
            }
            break;
        case TimerCountdown:
            if (now - countdown_start_time >= DelayTime) {
                comandToHandle = CountdownExpired;
            }
            break;
        default:
            if(rtc_time.unix_time_ms%2){
                ADC_ENABLE_INTERRUPT_BATTERY;
            } else {
                ADC_ENABLE_INTERRUPT_ACCELEROMETR;
            }
            //do nothing, we're stimm in ISR
            break;
    }
}

void handle_rotation() {
    if (Autorotate && ui_state != TimerListening) {
        lcd_set_orientation();
    }
}
// <editor-fold defaultstate="collapsed" desc="ISR function">

static void interrupt isr(void) {

    if (RTC_TIMER_IF) {
        RTC_TIMER_IF = 0; // Clear Interrupt flag.
        PIR6bits.TMR1GIF = 0;
        frames_count = 0;
    }
    if (PIR1bits.ADIF) {
        PIR1bits.ADIF = 0;
        while (GO_nDONE);

        switch (ADPCH) {
            case ENVELOPE:
                ADC_BUFFER_PUT((ADRESH << 8) | ADRESL);
                if (ui_state == TimerListening && ADC_LATEST_VALUE > DetectThreshold) {
                    UpdateShootNow();
                }
                break;
            case BATTERY:
            {
                uint16_t battery = ((ADRESH << 8) | ADRESL);
                uint16_t battery_mV = battery*BAT_divider;
                battery_level = (battery_mV / 8) - 320; // "/10" ((battery_mV-3200)*100)/(3900-3200)
                if (battery_level > 99) battery_level = 99;
            }
                break;
            case ACCELEROMETER:
                orientation = ((ADRESH << 8) | ADRESL) > ORIENTATION_INVERSE_THRESHOLD;
                break;
        }

    }
    if (PIR0bits.TMR0IF) {
        PIR0bits.TMR0IF = 0;
        update_rtc_time;
        if (!Keypressed) {//Assignment will not work because of not native boolean
            KeyReleased = true;
        }
        
        update_screen_model();

    }
}
// </editor-fold>
//
//void DoThresholdGrapg(uint8_t column) {
//    //    for (uint8_t i = 0; i < DETECT_THRESHOLD_LEVELS; i++) {
//    //        //        if (ADC_LATEST_VALUE > Mean + threshold_offsets[i])
//    //        //        if (ADC_MIDDLE_VALUE - cma_n>threshold_offsets[i])
//    //        if (MeanValue() > threshold_offsets[i])
//    //            lcd_send_page_mark(column, PAGE(LCD_HEIGHT) - 1 - i, BLACK_OVER_WHITE);
//    //        else
//    //            lcd_send_page_mark(column, PAGE(LCD_HEIGHT) - 1 - i, WHITE_OVER_BLACK);
//    //    }
//    lcd_draw_scope_column(column, ADC_LATEST_VALUE >> 2);
//}
//
//void DoAdcGraph() {
//    size_t column = 0;
//    time_t t, t_1 = 0;
//    int written = 0;
//    char lbl[16];
//    uint8_t lap = 0;
//    lbl[24] = 0;
//    t = rtc_time_msec;
//    DetectInit();
//
//    while (True) {
//        column = (column + 1) % 140;
//        lcd_draw_bit_mark_column(5); // bit scale
//        DoThresholdGrapg(10 + column);
//
//        lcd_send_page(10 + column, PAGE(25), 0x0F, lap % 2);
//        if (column == 0) {
//            lap++;
//            if (Keypressed) {
//                ui_state = TimerIdle;
//                return;
//            }
//        }
//        t_1 = t;
//        t = rtc_time_msec;
//        written = sprintf(lbl, "dt:%02u ", t - t_1);
//        sprintf(lbl + written, "%x ", ADC_LATEST_VALUE);
//        lcd_write_string(lbl, 5, 8, MediumFont, BLACK_OVER_WHITE);
//        delay_rtc_ms(15);
//    }
//}

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
    //    getSettings();
    getDefaultSettings();
    getPar();
    set_backlight(BackLightLevel);
    init_ms_timer0();
    initialize_rtc_timer();
    ei();
    // Initialization End
    // </editor-fold>
    // <editor-fold defaultstate="collapsed" desc="Main">

    lcd_clear_data_ram();
    while (True) {
        handle_rotation();
        handle_ui();
        //        lcd_demo();
        frames_count++;
    }
    //        DoAdcGraph();

    // </editor-fold>
}
