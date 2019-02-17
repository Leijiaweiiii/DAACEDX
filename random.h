/* 
 * File:   random.h
 * Author: navado
 *
 * Created on 16 February 2019, 21:02
 */

#ifndef RANDOM_H
#define	RANDOM_H
#include <stdint.h>
uint32_t random32(uint32_t limit);
#ifdef	__cplusplus
extern "C" {
#endif

uint16_t random_seed = 2345;
//uint32_t random32(uint32_t limit);
uint16_t random16(uint16_t limit);
#define random_set_seed(seed)   { random_seed = seed; }
#define random_get_seed()       (random_seed)

#ifdef	__cplusplus
}
#endif

#endif	/* RANDOM_H */

