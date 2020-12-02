#include <stdint.h>
#include "tbool.h"

int8_t fg_get_rsoc(void);
int8_t fg_get_rsoh(void);
int16_t fg_get_rcap(void);
int16_t fg_get_fcap(void);
int16_t fg_get_fcap_nom(void);
int16_t fg_get_vcel(TBool average);
int16_t fg_get_curr(TBool average);
int16_t fg_get_power(TBool average);
void fg_init(void);
