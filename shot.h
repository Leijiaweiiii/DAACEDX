/* 
 * File:   shot.h
 * Author: navado
 *
 * Created on 21 July 2018, 13:15
 */

#ifndef SHOT_H
#define	SHOT_H
#include "stdint.h"
#ifdef	__cplusplus
extern "C" {
#endif
#define MAXSHOOTSTRINGS              (30)
#define MAX_REGISTERED_SHOTS        (250)
#define MAX_SAVED_SHOTS             (100)
#define Size_of_ShootString         (402)
#define SIZE_OF_SHOT_T              (4)
typedef union {
    uint8_t data[SIZE_OF_SHOT_T];

    struct {
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
    };
} shot_t;

typedef union {
    uint8_t data[Size_of_ShootString];

    struct {
        uint8_t latest;
        uint8_t TotShoots; //Total shoots in current string

        shot_t shots[MAX_REGISTERED_SHOTS]; //in 1mS unit
    };
} ShootString_t;



#ifdef	__cplusplus
}
#endif

#endif	/* SHOT_H */

