#include "ExtiHandler.h"

/*extern "C" {
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
}*/

void EXTI0_IRQHandler(void)
{
    ExtiHandler::handleEXTI(0);
}

void EXTI1_IRQHandler(void)
{
    ExtiHandler::handleEXTI(1);
}

void EXTI2_IRQHandler(void)
{
    ExtiHandler::handleEXTI(2);
}

void EXTI3_IRQHandler(void)
{
    ExtiHandler::handleEXTI(3);
}

void EXTI4_IRQHandler(void)
{
    ExtiHandler::handleEXTI(4);
}


SwiMuxDelegateFunc_t<void(void)> ExtiHandler::handlers[ExtiHandler::HANDLED_EXTIs];


void ExtiHandler::handleEXTI(uint8_t index)
{
    if (handlers[index])
        handlers[index]();
}