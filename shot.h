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
#define MAX_SAVED_SHOTS             (20)

uint8_t get_shot_index_in_arr( uint8_t x);

typedef struct {
        union {
            uint8_t is_flags;

            struct {
                unsigned is_mic : 1;
                unsigned is_a : 1;
                unsigned is_b : 1;
                unsigned unused : 5;
            };
        };
        uint8_t sn;
        uint24_t dt;
} shot_t;

typedef struct {
        uint8_t latest;
        uint8_t TotShoots; //Total shoots in current string
        shot_t shots[MAX_SAVED_SHOTS]; //in 1mS unit
} ShootString_t;

#define SIZE_OF_SHOT_T              (sizeof(shot_t))
#define Size_of_ShootString         (sizeof(ShootString_t))
#ifdef	__cplusplus
}
#endif

#endif	/* SHOT_H */

