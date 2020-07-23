
#ifndef __FG_H_
#define __FG_H_

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

#define BATTERY_DESIGN_CAPACITY 3800    /* mA-h */

typedef struct {
    uint8_t rsoc;   // %
    uint8_t rsoh;   // %
    uint16_t rcap;  // mA-h
    uint16_t fcap;  // mA-h
} power_readings_t;

power_readings_t power_readings;

void fg_read();
void fg_init();
void fg_reset();
void fg_irq();

#endif	/* __FG_H */

