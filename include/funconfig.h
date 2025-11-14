#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#define FUNCONF_USE_PLL           1 // Use built-in 2x PLL
#define FUNCONF_USE_HSI           1 // Use HSI Internal Oscillator
#define FUNCONF_SYSTEM_CORE_CLOCK 48000000
#define FUNCONF_ENABLE_HPE        0

#define FUNCONF_DEBUG_HARDFAULT   0
#define FUNCONF_USE_DEBUGPRINTF   0
//#define FUNCONF_NULL_PRINTF
//#define FUNCONF_USE_UARTPRINTF 0
//#define FUNCONF_UART_PRINTF_BAUD 57600
#define UART_PINOUT_DEFAULT




#define SYS_USE_VECTORS  1
#define CH32V003         1

#define SWIMUX_USES_SLIP (1)
#define NUMBER_OF_BUSES  (6)
#define RX_BUFF_SIZE     (140)
#define TX_BUFF_SIZE     (208)


#define OW_DIOPORT_BUS0 GPIOD
#define OW_DIOPORT_BUS1 GPIOD
#define OW_DIOPORT_BUS2 GPIOD
#define OW_DIOPORT_BUS3 GPIOC
#define OW_DIOPORT_BUS4 GPIOC
#define OW_DIOPORT_BUS5 GPIOC

#define OW_DIOPIN_BUS0 (3)
#define OW_DIOPIN_BUS1 (2)
#define OW_DIOPIN_BUS2 (0)
#define OW_DIOPIN_BUS3 (7)
#define OW_DIOPIN_BUS4 (6)
#define OW_DIOPIN_BUS5 (5)


#define OW_PUPPORT_BUS0 nullptr
#define OW_PUPPORT_BUS1 nullptr
#define OW_PUPPORT_BUS2 nullptr
#define OW_PUPPORT_BUS3 nullptr
#define OW_PUPPORT_BUS4 nullptr
#define OW_PUPPORT_BUS5 nullptr

#define OW_PUPPIN_BUS0 (-1)
#define OW_PUPPIN_BUS1 (-1)
#define OW_PUPPIN_BUS2 (-1)
#define OW_PUPPIN_BUS3 (-1)
#define OW_PUPPIN_BUS4 (-1)
#define OW_PUPPIN_BUS5 (-1)

//#define OW_DIO_INPUT_PULLSDOWN

#define AUTOSLEEP_ENABLED
//#define ADD_CONSOLE_DEBUGGING
#ifdef ADD_CONSOLE_DEBUGGING
#define DO_ERASURE_TEST
#define FULL_ERASE_AFTER_TEST
#endif

#ifdef ADD_CONSOLE_DEBUGGING
#define dbgp(fmt)       printf(fmt)
#define dbgpf(fmt, ...) printf(fmt, __VA_ARGS__)
#else
#define dbgpf(fmt, ...)
#define dbgp(fmt)
#endif

#endif