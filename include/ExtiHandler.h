#ifndef H_EXTIHANDLER_H
#define H_EXTIHANDLER_H

#include "SwiMuxDelegateFunc.h"
#include <stdint.h>
#include <ch32fun.h>

#define MAKE_BITMASK(n) ((1ULL < > (n)) - 1)


class ExtiHandler {
  public:
    static constexpr int HANDLED_EXTIs = 5; // Handles from EXTI0 to EXTI4

    inline static bool setHandler(uint8_t extiChannel, SwiMuxDelegateFunc_t<void(void)> handler)
    {
        EXTI->EVENR &= ~(1UL << extiChannel);
        EXTI->FTENR &= ~(1UL << extiChannel);
        EXTI->RTENR &= ~(1UL << extiChannel);
        if (extiChannel >= HANDLED_EXTIs) {
            return false;
        }
        handlers[extiChannel] = handler;
        EXTI->EVENR |= (1UL << extiChannel);
        EXTI->FTENR |= (1UL << extiChannel);
        EXTI->RTENR |= (1UL << extiChannel);

        return true;
    }

    inline static bool removeHandler(SwiMuxDelegateFunc_t<void(void)> handler)
    {
        if (!handler)
            return false;
        for (int idx = 0; idx < HANDLED_EXTIs; idx++) {
            if (handlers[idx] == handler) { // same function and target object ?
                removeHandler(idx);
            }
        }
        return true;
    }

    inline static bool removeHandler(uint8_t extiChannel)
    {
        if (extiChannel >= HANDLED_EXTIs) {
            return false;
        }
        EXTI->EVENR &= ~(1UL << extiChannel);
        EXTI->FTENR &= ~(1UL << extiChannel);
        EXTI->RTENR &= ~(1UL << extiChannel);
        handlers[extiChannel].clear();
        return true;
    }

    inline static void clear(uint8_t extiChannel)
    {
        // Disable events and edge-detection ISRs
        EXTI->EVENR &= ~((1ULL << (HANDLED_EXTIs)) - 1);
        EXTI->FTENR &= ~((1ULL << (HANDLED_EXTIs)) - 1);
        EXTI->RTENR &= ~((1ULL << (HANDLED_EXTIs)) - 1);
        for (int idx = 0; idx < HANDLED_EXTIs; idx++)
            handlers[idx].clear();
    }


  private:
    friend void EXTI0_IRQHandler(void);
    friend void EXTI1_IRQHandler(void);
    friend void EXTI2_IRQHandler(void);
    friend void EXTI3_IRQHandler(void);
    friend void EXTI4_IRQHandler(void);
    static SwiMuxDelegateFunc_t<void(void)> handlers[HANDLED_EXTIs];

    static void handleEXTI(uint8_t index);
};

#endif // H_EXTIHANDLER_H
