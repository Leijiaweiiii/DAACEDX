/* 
 * File:   eeprom.h
 * Author: navado
 *
 * Created on 29 May 2018, 20:59
 */

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "DAACEDcommon.h"


// <editor-fold defaultstate="collapsed" desc="EEPROM Specificaion">
/* EEPROM specification */
#define EEPROM_PAGE_SIZE    (64)
// We're using 128kbit version
#define EEPROM_MAX_SIZE     (0x4000)

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
void eeprom_write_array(uint16_t address, uint8_t * data, uint16_t size);
void eeprom_write_wdata(uint16_t address, uint16_t data);
void eeprom_write_tdata(uint16_t address, uint24_t data);
uint16_t eeprom_read_wdata(uint16_t address);
uint24_t eeprom_read_tdata(uint16_t address);
//</editor-fold>


#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

