#include "random.h"

/* The state word must be initialized to non-zero.
 * Source: https://en.wikipedia.org/wiki/Xorshift */
uint16_t xorshift16()
{
	uint16_t x = random_seed;
	x ^= x << 7;
	x ^= x >> 11;
	x ^= x << 5;
	random_seed = x;
	return x;
}

uint16_t random16(uint16_t limit){
    uint16_t next = xorshift16();
    while(next > limit){
        next -= limit;
    }
    return next;
}