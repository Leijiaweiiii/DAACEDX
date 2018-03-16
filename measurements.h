// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MEASUREMENTS_H
#define	MEASUREMENTS_H

#include <xc.h>

#include "DAACED.h" // include processor files - each processor file is guarded.  

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#define SHOOT_IF            (PIR5bits.TMR1IF)
#define ACCELEROMETR_IF     (PIR5bits.TMR1IF)
void mark_to_flip_screen(){};
TBool orientation_changed(){return false;};
void save_shoot_time(){};
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* MEASUREMENTS_H */

