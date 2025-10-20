#include "ch32fun.h"

#if defined(ADD_CONSOLE_DEBUGGING)
#include "lib_uart.h"
#include "stdio.h"
#include "debug_macros.h"
#include "SwiMuxComms.hpp"
#include "AT21CS01.h"

extern AT21CS01 devs[NUMBER_OF_BUSES];
extern uint8_t usart_rx_mem[RX_BUFF_SIZE];
extern uint8_t usart_tx_mem[TX_BUFF_SIZE];
extern SwiMuxComms_t decoder;
extern void ch32mcuReset();
extern const AT21CS01::SwiBusConfig_t busConfig[NUMBER_OF_BUSES];

static void printByteHex(uint8_t byte)
{

    uart_putC('0');
    uart_putC('x');
    uart_putC((byte & 0xF0) < 0xA0 ? ((byte >> 4) + '0') : (((byte >> 4) - 0xA) + 'A'));
    uart_putC((byte & 0xF) < 0xA ? ((byte & 0xF) + '0') : (((byte & 0xF) - 0xA) + 'A'));
}

static void printbuff(uint8_t* buff, size_t len)
{
    if (buff == nullptr)
        return;
    while (len--) {
        printByteHex(*buff++);
        uart_putC(' ');
    }
}

static void printStr(void* src, size_t maxlen, char adorner = 0)
{
    if (src == nullptr)
        return;
    char* str = (char*)src;
    if (adorner)
        uart_putC(adorner);
    while (maxlen) {
        if (*str) {
            uart_putC(*str++);
        } else
            break;
    }
    if (adorner)
        uart_putC(adorner);
}



void testEndPrompt()
{
    uart_flush_RX();

    printf("Redo tests (Y/N) ? :>");

    int typed;
    do {
        typed = uart_getC();
        if (typed < 0)
            continue;
        else {
            switch (typed) {
                case 'y':
                    [[fallthrough]];
                case 'Y':
                    uart_putC('y');
                    uart_putC('\n');
                    Delay_Ms(100);
                    ch32mcuReset();
                    break; // put by convention only: any code after the previous statement won't be executed since the device has already reset.
                case 'n':
                    [[fallthrough]];
                case 'N':
                    uart_putC('n');
                    uart_putC('\n');
                    return;
                default:
                    break;
            }
        }
    } while (1);
}

const uint8_t testArray[8] = { 0xCA, 0xFE, 1, 2, 3, 4, 5, 6 };
#define DEVTESTSTRING "Tamaneko-chan"
const char* devStr = DEVTESTSTRING;



uint8_t writeToMemPool[140];
size_t writeToMemCount;

static void testSWI()
{


    printf("Now testing the AT21CS01 class:\r\n");



    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        printf("\r\n\n Device bus #%d\r\n", idx);
        if (devs[idx].init(busConfig[idx])) {
            printf("Failed to initialize device on bus #%d\r\n", idx);
            continue;
        }
        printf("  · initialized\n");
        if (devs[idx].reset()) {
            printf("  !! reset failed\r\n");
            continue;
        }
        printf("  · reset done.\r\n");


        uint8_t buff[16];
        strncpy((char*)buff, "Not erased!!!!", 16);
        if (devs[idx].readBytes(0, buff, 11)) {
            printf("  !! first read FAILED !!\r\n");
            continue;
        }
        printf("  · fist read[0..10] from init: ");
        printbuff(buff, 11);
        printStr(buff, 11, '"');
        uart_putC('\n');
        strncpy((char*)buff, "Not erased!!!!", 16);

        int eraseFailure = 256; // some impossible value, to confirm the test.
        if (devs[idx].eraseAll(&eraseFailure)) {
            printf("  !! eraseAll FAILED at index #%d!!\r\n", eraseFailure);
            continue;
        }
        printf("  · eraseAll succeeded (index: %d)\r\n", eraseFailure);

        if (devs[idx].readBytes(0, buff, 11)) {
            printf("  !! second read FAILED !!\r\n");
            continue;
        }
        printf("  · second read[0..10] from clear: ");
        printbuff(buff, 11);
        printStr(buff, 11, '"');
        uart_putC('\n');


        if (devs[idx].writeBytes(0, (uint8_t*)devStr, sizeof(DEVTESTSTRING))) {
            printf("  !! writePage FAILED !!\r\n");
            continue;
        }
        printf("  · writePage succeeded\r\n");


        memset(buff, 0, sizeof(buff));
        printf("  · cleared buffer contents: ");
        printbuff(buff, 11);
        printStr(buff, 11, '"');
        uart_putC('\n');


        if (devs[idx].readBytes(0, buff, 16)) {
            printf("  !! third readBytes FAILED !!\r\n");
            continue;
        }
        printf("  · third read[0..10] from wrote: %s\r\n", (char*)buff);


        uint8_t uuid[8] = { 0 };
        if (devs[idx].readUID(uuid)) {
            printf("  !! readUID FAILED !!");
            continue;
        }

        printf("  · UID is ");
        printbuff(uuid, 8);
        printf("\r\nBus #%d passed all tests.\r\n", idx);
        Delay_Ms(100);
    }

    printf("\nAll test successfull on the AT21CS01 class.\r\n");
}


size_t makeBuffer(uint8_t* buffer, size_t len, bool overwrite)
{
    uint8_t* ptr = buffer;
    if (overwrite) {
        for (size_t idx = 0; idx < len; idx++)
            *ptr++ = idx ^ 0xA7;
    } else {
        for (size_t idx = 0; idx < len; idx++)
            if (*ptr++ != (idx ^ 0xA7))
                return idx;
    }
    return 0;
}



void doTests()
{
    testSWI();
    testEndPrompt();
}


#endif //defined(ADD_CONSOLE_DEBUGGING)
