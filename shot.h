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
#define MAXSHOOTSTRINGS             (30)
#define MAXSHOOT                    (250)
#define LAST_SHOTS_TO_SAVE          (99)
#define Size_of_ShootString         (402)
#define Size_of_ShootString_header  (2 + sizeof(shot_t))
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
                unsigned ov : 1;// The overflow shot
                unsigned unused : 4;
            };
        };
        uint24_t dt;
    };
} shot_t;

typedef union {
    uint8_t data[MAXSHOOT*sizeof(shot_t)+2];

    struct {
        uint8_t latest;
        uint8_t TotShoots; //Total shoots in current string

        shot_t shots[MAXSHOOT]; //in 1mS unit
    };
} ShootString_t;



#ifdef	__cplusplus
}
#endif

#endif	/* SHOT_H */

