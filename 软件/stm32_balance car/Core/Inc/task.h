
#include "main.h"
#define PERIODIC(T) \
static uint32_t nxt; \
if(HAL_GetTick()<nxt) return; \
nxt+=(T);

#define PERIODIC_STAERT(NAME,T) \
static uint32_t NAME##_nxt=0; \
if(HAL_GetTick()>=NAME##_nxt) {\
NAME##_nxt +=(T);


#define PERIODIC_END }
