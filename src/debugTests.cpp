#include "ch32fun.h"

#if defined(ADD_CONSOLE_DEBUGGING)
#include "lib_uart.h"
#include "stdio.h"
#include "debug_macros.h"
#include "SwiMuxComms.hpp"
#include "DS28E07.h"


extern uint8_t usart_rx_mem[RX_BUFF_SIZE];
extern uint8_t usart_tx_mem[TX_BUFF_SIZE];
extern SwiMuxComms_t decoder;
extern void ch32mcuReset();

#define CARET_BLINK_TICK_PERIOD (Ticks_from_Ms(500))

static bool blinkCaret(bool reset = false, bool resetState = true)
{
    static bool writeCaret  = true;
    uint32_t lastChangeTick = 0;
    if (reset) {
        writeCaret = resetState;
    }

    if (!reset && ((SysTick->CNT - lastChangeTick) >= CARET_BLINK_TICK_PERIOD)) {
        return false;
    }
    lastChangeTick = SysTick->CNT;
    if (writeCaret) {
        uart_putC('_');
        return true;
    } else {
        uart_putC('\b');
        uart_putC(' ');
        return false;
    }
    writeCaret = !writeCaret;
}


int readSerialInt(const char* optionalChars = nullptr)
{
    bool wroteCaret = blinkCaret(true);
    int value       = 0;
    do {
        int charVal = uart_getC();
        if (charVal >= '0' && charVal <= '9') {
            value *= 10;
            value += charVal - '0';
            wroteCaret = blinkCaret(true, wroteCaret);
            uart_putC((char)charVal);

        } else if (charVal == '\b') {
            value /= 10;
            wroteCaret = blinkCaret(true, wroteCaret);
            printf("\b \b");
        } else if (charVal == 13) {
            return value;

        } else if (value == 0 && optionalChars && strchr(optionalChars, charVal) != nullptr) {
            return charVal;
        } else {
            wroteCaret = blinkCaret();
        }
    } while (1);
}

extern "C" {
int _write(int fd, const char* buf, int size)
{
    (void)fd; // Unused parameter

    uart_err_t err = uart_write(buf, size);

    if (err == UART_OK) {
        return size; // Return the number of characters written
    } else {
        return -1; // Indicate an error
    }
}
} // extern "C"

static void printByteHex(uint8_t byte, bool withprefix = false)
{
    static const char HEXCHARS[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
    if (withprefix) {
        uart_putC('0');
        uart_putC('x');
    }
    uart_putC(HEXCHARS[byte >> 4]);
    uart_putC(HEXCHARS[byte & 0xF]);
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
    uint32_t lastMillis = SysTick->CNT;
    bool flipFlop       = false;
    uart_flush_RX();

    printf("Redo tests (Y/N) ? :>");

    int typed;
    do {
        typed = uart_getC();

        flipFlop = !flipFlop;
        if (typed < 0) {
            if ((SysTick->CNT - lastMillis) > Ticks_from_Ms(250)) {
                lastMillis = SysTick->CNT;
                if (flipFlop) {
                    uart_putC('_');
                    uart_putC('\b');
                } else {
                    uart_putC(' ');
                    uart_putC('\b');
                }
            }
            continue;
        } else {
            switch (typed) {
                case 'y':
                    [[fallthrough]];
                case 'Y':
                    uart_putC('y');
                    uart_putC('\n');
                    Delay_Ms(100);
                    handle_reset();
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

// clang-format off
const char* ONEWIRE_ERRORS_NAMES[] = {
    "NO_ERROR",
    "DIO_PORT_NULL",
    "DIO_PORT_INVALID",
    "DIO_PIN_INVALID",
    "PULLUP_PORT_INVALID",
    "PULLUP_PIN_INVALID",
    "NULL_INPUT_BUFFER",
    "NULL_OUTPUT_BUFFER",
    "NO_BUS_POWER",
    "BUS_HELD_LOW",
    "NO_DEVICE_PRESENT",
    "READ_ROM_FAILED",
    "ALIGNED_WRITE_HEAD_PREREAD",
    "ALIGNED_WRITE_TAIL_PREREAD",
    "MEMADDRESS_OUT_OF_BOUNDS", // Specified address is out of memory bounds.
    "OUT_OF_BOUNDS", // Specified length exceeds out of memory bounds.
    "WRITE_MEM_FAILED",
    "MULTIDROP_ID_UNREADABLE",
    "WRITE_SCRATCHPAD_PRESELECT",
    "WRITE_SCRATCHPAD_CRC16",
    "READ_SCRATCHPAD_PRESELECT",
    "READ_SCRATCHPAD_CRC16",
    "SCRATCHPAD_PF", // Power loss or scratchpad not full
    "WRITTEN_SCRATCHPAD_MISMATCH",
    "COPY_SCRATCHPAD_PRESELECT",
    "COPY_SCRATCHPAD" // Memory protected or scratchpad error (less likely)
};
// clang-format on

static const char* getErrorName(const OneWireError_e errorCode)
{
    if (errorCode > sizeof(ONEWIRE_ERRORS_NAMES) / sizeof(const char*))
        return "<unknown error>";
    return ONEWIRE_ERRORS_NAMES[errorCode];
}

uint8_t writeToMemPool[140];
size_t writeToMemCount;

static constexpr size_t MS_BETWEEN_DEVICES = 250;
static constexpr size_t MS_BETWEEN_TESTS   = 15;

static void testSWI(DS28E07 devices[], const OneWire::OneWireConfig_t cfgs[])
{


    printf("Now testing the DS28E07 class:\r\n");
    uint8_t buff[16];


    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        Delay_Ms(MS_BETWEEN_DEVICES); // Leave some time between tests.
        printf("\r\n\nDevice bus #%d\r\n", idx);
        if (devices[idx].begin(cfgs[idx]) != OneWireError_e::NO_ERROR) {
            printf("Failed to initialize device on bus #%d: %s\r\n", idx, getErrorName(devices[idx].getLastError()));
            continue;
        }
        printf("  • initialized\r\n");
        Delay_Ms(MS_BETWEEN_TESTS);


        if (!devices[idx].reset()) {
            printf("  • DS28E07.reset() FAILED: %s\r\n", getErrorName(devices[idx].getLastError()));
            continue;
        }
        printf("  • DS28E07.reset() succeeded.\r\n");
        Delay_Ms(MS_BETWEEN_TESTS);
        for (int redo = 0; redo < 1; redo++) {

            uint8_t uuid[8] = { 0 };
            if (!devices[idx].getUid(uuid)) {
                printf("  • DS28E07.getUid() #%d FAILED: %s", redo, getErrorName(devices[idx].getLastError()));
                continue;
            }
            printf("  • DS28E07.getUid() #%d gives: ", redo);
            printbuff(uuid, 8);
            printf("\r\n");
            Delay_Ms(MS_BETWEEN_TESTS);
        }


        strncpy((char*)buff, "Not erased!!!!", 16);
        if (devices[idx].read(0, buff, sizeof(buff)) != sizeof(buff)) {
            printf("  • first DS28E07.read FAILED: %s", getErrorName(devices[idx].getLastError()));
            continue;
        }
        printf("  • fist read[0..10] from init: ");
        printbuff(buff, sizeof(buff));
        printStr(buff, sizeof(buff), '"');
        printf("\r\n");
        Delay_Ms(MS_BETWEEN_TESTS);

#ifdef DO_ERASURE_TEST
        if (!devices[idx].reset()) {
            printf("  • reset before DS28E07.eraseAll() FAILED: %s", getErrorName(devices[idx].getLastError()));
            continue;
        }
        printf("  • reset before DS28E07.eraseAll() successful.\r\n");
        Delay_Ms(MS_BETWEEN_TESTS);
        {
            uint16_t erasedBytes = devices[idx].eraseAll(DS28E07::ErasurePassCode);
            if (erasedBytes != DS28E07::MemorySize_Bytes) {
                printf("  • DS28E07.eraseAll() FAILED: only %d/%d bytes erased.\r\n", erasedBytes, DS28E07::MemorySize_Bytes);
                continue;
            }
        }
        printf("  • DS28E07.eraseAll succeeded.\r\n");
        Delay_Ms(MS_BETWEEN_TESTS);

        strncpy((char*)buff, "Not erased!!!!", 16);
        if (devices[idx].read(0, buff, 11) != 11) {
            printf("  • second DS28E07.read() FAILED: %s\r\n", getErrorName(devices[idx].getLastError()));
            continue;
        }
        printf("  • second read[0..10] from clear: ");
        printbuff(buff, 11);
        printStr(buff, 11, '"');
        printf("\r\n");
        Delay_Ms(MS_BETWEEN_TESTS);
#endif

        printf("  • writing \"%s\" to memory,\r\n  contents:", devStr);
        printbuff((uint8_t*)devStr, sizeof(DEVTESTSTRING));
        uint16_t len = devices[idx].write(0, (uint8_t*)devStr, sizeof(DEVTESTSTRING));
        if (len != sizeof(DEVTESTSTRING)) {
            printf("\r\n  • DS28E07.write() FAILED, len=%d, %s\r\n", len, getErrorName(devices[idx].getLastError()));
            continue;
        }
        printf("\r\n  • DS28E07.write() succeeded\r\n");
        Delay_Ms(MS_BETWEEN_TESTS);

        memset(buff, 0, sizeof(buff));
        printf("  • buffer clear ");
        {
            bool clearedAll = true; // default
            for (int idx = 0; idx < sizeof(buff); idx++) {
                if (buff[idx] != 0) {
                    clearedAll = false;
                    break;
                }
            }
            if (!clearedAll) {
                printf("FAILED !!\r\n");
                continue;
            }
            printf("for next read operation\r\n");
        }


        if (devices[idx].read(0, buff, sizeof(buff)) != sizeof(buff)) {
            printf("  • third DS28E07.read FAILED: %s\r\n", getErrorName(devices[idx].getLastError()));
            continue;
        }
        printf("  • third read[0..10] from previous string wrote:\r\n  ");
        printbuff(buff, sizeof(buff));
        printStr(buff, sizeof(buff), '"');
        printf("\r\n");

#ifdef FULL_ERASE_AFTER_TEST
        if (devices[idx].eraseAll(DS28E07::ErasurePassCode) != DS28E07::MemorySize_Bytes) {
            printf("  • Final full erasure FAILED: %s\r\n", getErrorName(devices[idx].getLastError()));
            continue;
        }
#endif

        printf("\r\nBus #%d passed all tests.\r\n", idx);
    }

    printf("\nAll test successfull on the DS28E07 class.\r\n");
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



static void readTest(DS28E07 devices[])
{
    printf("\r\nWhich bus to read ? Enter a value from [0:%d]", NUMBER_OF_BUSES);
    printf(" or 'A' for all present devices:\r\n>");
    int selection = readSerialInt("aA");
    if (selection == 'A' || selection == 'a') { // We need the comparision to 'a' (lowercase) so we don't fall into the next `if` statement.
        selection = 'A';
    } else if (selection < 0 || selection >= NUMBER_OF_BUSES) {
        printf("\r\nWrong entry.");
        return;
    }
    uint8_t buffer[8];
    if (selection == 'A') {
        printf("\r\nReading all devices:");
        for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
            printf("\r\nDevice on bus #%d: ", idx);
            if (!devices[idx].reset()) { // no device present ?
                printf(" none\r\n");
                continue; // skip reading it.
            }

            for (uint16_t address = 0; address < 128; address += 8) {
                memset(buffer, 0, 8);
                if (devices[idx].read(address, buffer, 8) != 8) {
                    printf("Failed to read device #%d @ 0x%04x", idx, address);
                }
                printf("\r\n  ");
                printbuff(buffer, 8);
                Delay_Ms(10);
            }
        }
    } else {
        printf("\r\nDevice on bus #%d: ", selection);
        if (!devices[selection].reset()) { // no device present ?
            printf(" none\r\n");
            return; // skip reading it.
        }

        for (uint16_t address = 0; address < 128; address += 8) {
            memset(buffer, 0, 8);
            if (devices[selection].read(address, buffer, 8) != 8) {
                printf("Failed to read device #%d @ 0x%04x", selection, address);
            }
            printf("\r\n  ");
            printbuff(buffer, 8);
            Delay_Ms(10);
        }
    }
    printf("\r\n");
}


static void overwriteTest(DS28E07 devices[]) {}

void doTests(DS28E07 devices[], const OneWire::OneWireConfig_t configs[])
{
    //testSwiLines(devices);
    bool quit = false;
    printf("Initialized device(s): ");
    {
        int successes = 0;
        for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
            if (devices[idx].begin(configs[idx]) == OneWireError_e::NO_ERROR) {
                printf("%d, ", idx);
                successes++;
                Delay_Ms(10);
            }
        }
        switch (successes) {
            case 0:
                printf(" none detected.\r\n");
                break;
            case NUMBER_OF_BUSES:
                printf(" i.e all %d buses.\r\n", NUMBER_OF_BUSES);
                break;
            default:
                printf(" all other buses silent.");
                break;
        }
    }


    printf("\r\nSwiMux internal tests:\r\n");
    printf(" 0) Read memory\r\n");
    printf(" 1) Overwrite memory\r\n");
    printf(" f) Full tests sequence.\r\n");
    printf(" q) Quit this menu\r\n");
    printf("Enter your choice:");
    do {
        int charVal = uart_getC();
        switch (charVal) {
            case '0':
                uart_print("0\r\n");
                readTest(devices);
                quit = true;
                break;
            case '1':
                uart_print("1\r\n");
                overwriteTest(devices);
                quit = true;
                break;
            case 'f':
            case 'F':
                uart_print("f\r\n");
                testSWI(devices, configs);
                quit = true;
                break;
            case 'q':
            case 'Q':
                quit = true;
                break;
            default:
                break;
        }
    } while (!quit);



    testEndPrompt();
}


#endif //defined(ADD_CONSOLE_DEBUGGING)
