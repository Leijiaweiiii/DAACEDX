#include "random.h"

/* The state word must be initialized to non-zero.
 * Source: https://en.wikipedia.org/wiki/Xorshift */
uint32_t xorshift32(uint32_t *state)
{
	/* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
	uint32_t x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return x;
}

uint16_t xorshift16(uint16_t *state)
{
	/* Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs" */
	uint16_t x = *state;
	x ^= x << 7;
	x ^= x >> 11;
	x ^= x << 5;
	*state = x;
	return x;
}

uint32_t random32(uint32_t limit){
    uint32_t next = xorshift32(&random_seed);
    while(next > limit){
        next -= limit;
    }
    return next;
}

uint16_t random16(uint16_t limit){
    uint16_t next = xorshift16(&random_seed);
    while(next > limit){
        next -= limit;
    }
    return next;
}