#include <ch32fun.h>
#include <stdio.h>
#include "debug_macros.h"
#include "lib_uart.h"
#include <AT21CS01.h>
#include <stdarg.h>
#include <string.h>
#include "SwiMuxComms.hpp"
#ifdef ADD_CONSOLE_DEBUGGING
#include "debugTests.h"
#endif

#define DEFAULT_AUTOSLEEP_DELAY_MS (2000)



AT21CS01 devs[NUMBER_OF_BUSES];
uint8_t usart_rx_mem[RX_BUFF_SIZE];
uint8_t usart_tx_mem[TX_BUFF_SIZE];

SwiMuxComms_t decoder;



const AT21CS01::SwiBusConfig_t busConfig[NUMBER_OF_BUSES] = {
    // clang-format off
        {GPIOD, 3, nullptr, -1 }, //GPIOA, 1},
        {GPIOD, 2, nullptr, -1 }, //GPIOA, 2},
        {GPIOD, 0, nullptr, -1 }, //GPIOC, 1},
        {GPIOC, 7, nullptr, -1 }, //GPIOC, 2},
        {GPIOC, 6, nullptr, -1 }, //GPIOC, 3}, 
        {GPIOC, 5, nullptr, -1 } //GPIOC, 4}
    }; // clang-format on

#ifdef ADD_CONSOLE_DEBUGGING

#define WAITFORUARTDEBUG_PROMPT_STRING "Waiting for debug, press [ENTER] to proceed, any other key to skip."

void doTests();
bool waitForUartDebug()
{
    uint32_t lastUpdTick = 0, idx = 0;
    int entry;

    const char* promptStr = WAITFORUARTDEBUG_PROMPT_STRING;
    const uint32_t PERIOD = sizeof(WAITFORUARTDEBUG_PROMPT_STRING) - 1;

    // Purge input buffer
    uart_flush_RX();

    do {
        entry = uart_getC();
        if ((SysTick->CNT - lastUpdTick) > Ticks_from_Ms(33)) {
            lastUpdTick = SysTick->CNT;
            if (idx < PERIOD) {
                uart_putC(promptStr[idx]);
            } else if (idx < PERIOD * 2) {
                __NOP(); // do nothing, keep the string printed for a full period
            } else if (idx < PERIOD * 3) {
                printf("\b \b"); // backspace before the char, print a space over it, then backspace before the space itself.
            } else {
                idx = 0;
                continue;
            }
            idx++;
        }

    } while (entry < 0);

    while (idx--) {
        printf("\b \b");
        Delay_Ms(10);
    }
    // Purge input buffer again
    printf("\r%c", '\n');
    uart_flush_RX();

    return entry == 13;
}
#endif

static uint8_t* payload;
static size_t plength;
static uint32_t lastReception_ms;



void setup()
{

    SystemInit();
    uart_config_t cfg = { .baudrate = uart_baudrate_t::UART_BAUD_57600,
        .wordlength                 = uart_wordlength_t::UART_WORDLENGTH_8,
        .parity                     = uart_parity_t::UART_PARITY_NONE,
        .stopbits                   = uart_stopbits_t::UART_STOPBITS_ONE,
        .flowctrl                   = uart_flowctrl_t::UART_FLOWCTRL_NONE };
    uart_init(usart_rx_mem, RX_BUFF_SIZE, &cfg);
    // Enable all GPIO ports.
    RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

#ifdef ADD_CONSOLE_DEBUGGING
    if (waitForUartDebug()) {
        doTests();
    }
#endif

    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++)
        devs[idx].init(busConfig[idx]);

    decoder.reset();
}

static void processRead(), processWrite(), processGetUID(), processRollCall(), processSleep(bool autoSleep = false), processPresenceReport();
static void sendAck(SwiMuxOpcodes_e opcode);
static void sendNack(SwiMuxError_e error);
static void sendAckArgs(SwiMuxOpcodes_e opcode, uint8_t arg);
static void sendUid(SwiMuxRespUID_t& uidResp);

void ch32mcuReset()
{
    PFIC->CFGR = (0xBEEF << 16) | (1U << 7); // Do a system reset;
}


int main(void)
{
#ifdef ADD_CONSOLE_DEBUGGING
    static uint32_t lastTick = 0;
    uint32_t ticksOnStart;
#endif

    setup();

#ifdef ADD_CONSOLE_DEBUGGING
    printf("Testing USART wake up: device will suspend in 5 seconds");
    ticksOnStart = SysTick->CNT;
#endif
    // The following statement made the SwiMux send a presence report on reset.
    //processPresenceReport(); // Disabled until further notice.
#ifdef AUTOSLEEP_ENABLED
    lastReception_ms = SysTick->CNT;
#endif
    while (1) {
#ifdef ADD_CONSOLE_DEBUGGING
        if ((SysTick->CNT - lastTick) > Ticks_from_Ms(1000)) {
            lastTick = SysTick->CNT;
            uart_write(".", 1);
        }
        if ((SysTick->CNT - ticksOnStart) > Ticks_from_Ms(5000)) {
            ticksOnStart = SysTick->CNT;
            uart_write("ZzZzzz\r\n", 8);
            Delay_Ms(20); // Let the uart send its last char.
            processSleep();
        }
#endif

        //#define LOREM_IPSUM                                                                                                                                  \
//    "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim "  \
//    "veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in "          \
//    "voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia "        \
//    "deserunt mollit anim id est laborum."
        //        const char* LoremIpsum = LOREM_IPSUM;
        //
        //        do {
        //            for (int idx = 0; idx < sizeof(LOREM_IPSUM); idx++) {
        //                uart_writeByte(static_cast<uint8_t>(LoremIpsum[idx]));
        //            }
        //            uart_writeByte('\r');
        //            uart_writeByte('\n');
        //            uart_writeByte('\n');
        //            Delay_Ms(500);
        //        } while (1);


        while (uart_available() > 0) {
            lastReception_ms  = SysTick->CNT;
            SwiMuxError_e err = decoder.decode(uart_getC(), payload, plength);
            if (err != SMERR_None) {
                sendNack(err);
                continue;
            }
            if (payload == nullptr || plength == 0) {
                continue; // still no packet.
            }
            if ((payload[0] ^ payload[1]) != 0xFF) // Wrong opcode and inverted opcode
                continue;
            // Now to intepret the command
            __NOP();
            switch (payload[0]) {
                case SMCMD_ReadBytes:
                    processRead();
                    break;
                case SMCMD_WriteBytes:
                    processWrite();
                    break;
                case SMCMD_GetUID:
                    processGetUID();
                    break;
                case SMCMD_RollCall:
                    processRollCall();
                    break;
                case SMCMD_Sleep:
                    processSleep();
                    break;
                case SMCMD_GetPresence:
                    processPresenceReport(); // just acknowledge the command. "I'm here !"
                    break;
                default:
                    sendNack(SMERR_UnkownCommand);
                    break;
            }
            Delay_Us(5);
        }
#ifdef AUTOSLEEP_ENABLED
        if ((SysTick->CNT - lastReception_ms) > Ticks_from_Ms(DEFAULT_AUTOSLEEP_DELAY_MS)) {
            lastReception_ms = SysTick->CNT; // per good practice, even though `processSleep` will reset the mcu once awaken.
            processSleep(true);
        }
#endif
    }
}

static void processRead()
{
    if (plength < 5) { // not enough bytes  (opcode + ~opcode + bus index + address + length)
        sendNack(SwiMuxError_e::SMERR_ReadBytesParams);
        return;
    }
    SwiMuxCmdRead_t* pCmd = (SwiMuxCmdRead_t*)payload;
    if (pCmd->busIndex > 5) {
        sendNack(SwiMuxError_e::SMERR_BusIndexOutOfRange);
        return;
    }
    if (pCmd->offset >= 128) {
        sendNack(SMERR_MemOffsetOutOfRange);
        return;
    }
    if ((pCmd->offset + pCmd->length) > devs[pCmd->busIndex].memSize()) {
        sendNack(SMERR_ReadLengthOutOfRange);
        return;
    }


    // Copy valid the command as a response header
    memcpy(usart_tx_mem, pCmd, sizeof(SwiMuxCmdRead_t));

    if (SwiError_e::SUCCESS == devs[pCmd->busIndex].readBytes(pCmd->offset, &usart_tx_mem[sizeof(SwiMuxCmdRead_t)], pCmd->length)) {
        // Device read successful.
        if (decoder.encode(usart_tx_mem, pCmd->length, uart_writeByte) == false) {
            sendNack(SMERR_ReadEncodingRefused); // failed to read
        } // otherwise the `encode` success is producing the ACK itself on the serial line.
    } else {
        // Device read failed !
        sendNack(SMERR_ReadMemoryFailed);
    }
}


void processRollCall()
{
    static uint8_t buff[8];

    if (plength < 2) {
        sendNack(SMERR_Framing);
        return;
    }

    SwiMuxRollCallResult_t res;
    res.Opcode    = SMCMD_RollCall;
    res.NegOpCode = ~res.Opcode;
    for (uint8_t idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        if (devs[idx].readUID(buff)) {
            // Valuate the uid we got
            memcpy((void*)&res.orderedUids[idx], buff, 8);
        } else {
            res.orderedUids[idx] = 0ULL;
        }
    }
    // Now send `res`
    if (!decoder.encode((const uint8_t*)&res, sizeof(SwiMuxRollCallResult_t), uart_writeByte)) {
        sendNack(SMERR_ReadEncodingRefused); // Could not encode `res` .
    }
}


static void processWrite()
{
    SwiMuxCmdWrite_t* cmd = (SwiMuxCmdWrite_t*)payload;
    if (cmd->busIndex > 5) {
        sendNack(SwiMuxError_e::SMERR_BusIndexOutOfRange);
        return;
    }
    if (cmd->offset >= 128) {
        sendNack(SMERR_MemOffsetOutOfRange);
        return;
    }
    if ((cmd->offset + cmd->length) > devs[cmd->busIndex].memSize()) {
        sendNack(SMERR_WriteLengthOutOfRange);
        return;
    }

    uint8_t written = 0;
    // Write data from the first byte after cmd->len (that's our received payload).
    if (SwiError_e::SUCCESS != devs[cmd->busIndex].writeBytes(cmd->offset, &(&cmd->length)[1], cmd->length, &written)) {
        sendNack(SMERR_WriteFailed);
    } else { // Acknowledge with the effective count of written bytes.
        sendAckArgs(SMCMD_WriteBytes, written);
    }
}

static void processGetUID()
{
    if (plength < 3) {
        sendNack(SMERR_Framing);
        return;
    }
    SwiMuxGetUID_t* cmd = (SwiMuxGetUID_t*)payload;
    if (cmd->busIndex > 5) {
        sendNack(SwiMuxError_e::SMERR_BusIndexOutOfRange);
        return;
    }
    SwiMuxRespUID_t uidResp;
    int resp = devs[cmd->busIndex].readUID(uidResp.uid);
    if (resp == SwiError_e::SUCCESS) {
        // Send the Guid response packet
        sendUid(uidResp);
    } else {
        sendNack(SMERR_GuidUnreadable);
    }
}



static void processSleep(bool autoSleep)
{
    sendAck(SMCMD_Sleep);
#ifdef AUTOSLEEP_ENABLED
    if (autoSleep) {
        sendAck(SMCMD_Autosleep);
    } else {
        sendAck(SMCMD_Sleep);
    }
    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        devs[idx].disableBus(); // Cut power if possible
        devs[idx].disableChangeDetection(); // enableChangeDetection(); //Enable pin change detection on all buses.
    }
    Delay_Us(1000); // wait 1ms (1000µs)

    SysTick->CTLR = 0; // Disable system counter.

    // An event on the buses or the uart should wake us up.
    __WFE();
    ch32mcuReset();
#endif
}

static void processPresenceReport()
{
    uint32_t result = 0;
    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        result |= devs[idx].isPowered() ? (1 << idx) : 0;
    }
    SwiMuxCmdPresence_t cmd = { .Opcode = SMCMD_ConnectEvent,
        .NegOpcode                      = (uint8_t)(0xFF & ~SMCMD_ConnectEvent),
        .presenceLSB                    = (uint8_t)(result & 0xff),
        .presenceMSB                    = (uint8_t)((result >> 8) & 0xFF),
        .busesCount                     = NUMBER_OF_BUSES };
    decoder.encode((uint8_t*)&cmd, sizeof(cmd), uart_writeByte);
}



/**  @brief Sends a error packet. */
static void sendNack(SwiMuxError_e error)
{
    uint8_t msg[3] = { SMCMD_Nack, (uint8_t)(0xFF & ~SMCMD_Nack), error };
    decoder.encode(msg, 3, uart_writeByte);
}

/**  @brief Sends an acknowledgement packet. */
static void sendAck(SwiMuxOpcodes_e opcode)
{
    uint8_t msg[3] = { SMCMD_Ack, (uint8_t)(0xFF & ~SMCMD_Ack), opcode };
    decoder.encode(msg, 3, uart_writeByte);
}

/**  @brief Sends an acknowledgement packet. */
static void sendAckArgs(SwiMuxOpcodes_e opcode, uint8_t arg)
{
    uint8_t msg[4] = { SMCMD_Ack, (uint8_t)(0xFF & ~SMCMD_Ack), opcode, arg };
    decoder.encode(msg, 4, uart_writeByte);
}

static void sendUid(SwiMuxRespUID_t& uidResp)
{
    uidResp.Opcode    = SMCMD_HaveUID;
    uidResp.NegOpcode = ~SMCMD_HaveUID;
    decoder.encode((uint8_t*)&uidResp, sizeof(SwiMuxRespUID_t), uart_writeByte);
}

