#ifndef H_TIMING_H
#define H_TIMING_H

#include <ch32v003fun.h>
#include <stdint.h>

// Number of ticks elapsed per millisecond (48,000 when using 48MHz Clock)
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)
// Number of ticks elapsed per microsecond (48 when using 48MHz Clock)
#define SYSTICK_ONE_MICROSECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000000)

// Simple macro functions to give a arduino-like functions to call
// millis() reads the incremented systick variable
// micros() reads the raw SysTick Count, and divides it by the number of
// ticks per microsecond ( WARN: Wraps every 90 seconds!)
#define millis()                (systick_millis)
#define micros()                (SysTick->CNT / SYSTICK_ONE_MICROSECOND)


#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t systick_millis;


void systick_init();

#ifdef __cplusplus
} // extern C
#endif

#endif // H_TIMING_H
