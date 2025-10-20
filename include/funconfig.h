#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#define FUNCONF_USE_PLL 1               // Use built-in 2x PLL 
#define FUNCONF_USE_HSI 1               // Use HSI Internal Oscillator
#define FUNCONF_SYSTEM_CORE_CLOCK 48000000
#define FUNCONF_ENABLE_HPE 0

#define FUNCONF_DEBUG_HARDFAULT 0
#define FUNCONF_USE_DEBUGPRINTF 0
//#define FUNCONF_NULL_PRINTF
#define FUNCONF_USE_UARTPRINTF 0
//#define FUNCONF_UART_PRINTF_BAUD 57600
#define UART_PINOUT_DEFAULT


#define SYS_USE_VECTORS 1
#define CH32V003        1

#define NUMBER_OF_BUSES (6)
#define RX_BUFF_SIZE (140)
#define TX_BUFF_SIZE (208)
#define ADD_CONSOLE_DEBUGGING


#endif