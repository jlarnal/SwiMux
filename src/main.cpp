#include <ch32fun.h>
#include <stdio.h>
#include "debug_macros.h"
#include "lib_uart.h"
#include "DS28E07.h"
#include <stdarg.h>
#include <string.h>
#include "SwiMuxComms.hpp"

#ifdef ADD_CONSOLE_DEBUGGING
#include "debugTests.h"
#endif

#define DEFAULT_AUTOSLEEP_DELAY_MS (2000)



DS28E07 devs[NUMBER_OF_BUSES];
uint8_t usart_rx_mem[RX_BUFF_SIZE];
uint8_t usart_tx_mem[TX_BUFF_SIZE];



SwiMuxComms_t decoder;



const OneWire::OneWireConfig_t busesConfigs[NUMBER_OF_BUSES] = {
    // clang-format off
        {OW_DIOPORT_BUS0, OW_DIOPIN_BUS0, OW_PUPPORT_BUS0, OW_PUPPIN_BUS0 },
        {OW_DIOPORT_BUS1, OW_DIOPIN_BUS1, OW_PUPPORT_BUS1, OW_PUPPIN_BUS1 },
        {OW_DIOPORT_BUS2, OW_DIOPIN_BUS2, OW_PUPPORT_BUS2, OW_PUPPIN_BUS2 },
        {OW_DIOPORT_BUS3, OW_DIOPIN_BUS3, OW_PUPPORT_BUS3, OW_PUPPIN_BUS3 },
        {OW_DIOPORT_BUS4, OW_DIOPIN_BUS4, OW_PUPPORT_BUS4, OW_PUPPIN_BUS4 },
        {OW_DIOPORT_BUS5, OW_DIOPIN_BUS5, OW_PUPPORT_BUS5, OW_PUPPIN_BUS5 } 
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

static uint8_t* g_payload;
static size_t plength;
static uint32_t lastReception_ticks;



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
#ifndef ADD_CONSOLE_DEBUGGING
    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++)
        devs[idx].begin(busesConfigs[idx]);
#endif
    decoder.reset();
#ifdef ADD_CONSOLE_DEBUGGING

    //funPinMode(PC5, (GPIO_Speed_50MHz | GPIO_CNF_OUT_OD));
    //funPinMode(PC6, (GPIO_Speed_50MHz | GPIO_CNF_OUT_OD));
    //funPinMode(PC7, (GPIO_Speed_50MHz | GPIO_CNF_OUT_OD));
    //funPinMode(PD0, (GPIO_Speed_50MHz | GPIO_CNF_OUT_OD));
    //funPinMode(PD2, (GPIO_Speed_50MHz | GPIO_CNF_OUT_OD));
    //funPinMode(PD3, (GPIO_Speed_50MHz | GPIO_CNF_OUT_OD));
    //
    //funDigitalWrite(PC5, 0);
    //funDigitalWrite(PC6, 0);
    //funDigitalWrite(PC7, 0);
    //funDigitalWrite(PD0, 0);
    //funDigitalWrite(PD2, 0);
    //funDigitalWrite(PD3, 0);



    if (waitForUartDebug()) {
        doTests(devs, busesConfigs);
    } else {
        printf("\r\nTests skipped.\r\n");
    }
#endif
}

static void processWakeup(), processRead(), processWrite(), processGetUID(), processRollCall(), processSleep(bool autoSleep = false),
  processPresenceReport();
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
    lastReception_ticks = SysTick->CNT;
#endif
    while (1) {

        while (uart_available() > 0) {
            SwiMuxError_e err = decoder.decode(uart_getC(), g_payload, plength);
            if (err >= SMERR_ERRORS) {
                sendNack(err);
                continue;
            }
            if (g_payload == nullptr || plength == 0) {
                continue; // still no packet.
            }
            if ((g_payload[0] ^ g_payload[1]) != 0xFF) // Wrong opcode and inverted opcode
                continue;
            // Now to intepret the command
            __NOP();
            switch (g_payload[0]) {
                case SMCMD_Wakeup:
                    processWakeup();
                    break;
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
            lastReception_ticks = SysTick->CNT;
        }
#ifdef AUTOSLEEP_ENABLED
        if ((SysTick->CNT - lastReception_ticks) > Ticks_from_Ms(DEFAULT_AUTOSLEEP_DELAY_MS)) {
            lastReception_ticks = SysTick->CNT; // per good practice, even though `processSleep` will reset the mcu once awaken.
            processSleep(true);
        }
#endif
    }
}

static void processWakeup()
{
    sendAck(SMCMD_Wakeup);
}

static void processRead()
{
    if (plength < 5) { // not enough bytes  (opcode + ~opcode + bus index + address + length)
        sendNack(SwiMuxError_e::SMERR_ReadBytesParams);
        return;
    }
    SwiMuxCmdRead_t* pCmd = (SwiMuxCmdRead_t*)g_payload;
    if (pCmd->busIndex > 5) {
        sendNack(SwiMuxError_e::SMERR_BusIndexOutOfRange);
        return;
    }
    if (pCmd->offset >= 128) {
        sendNack(SMERR_MemOffsetOutOfRange);
        return;
    }
    if ((pCmd->offset + pCmd->length) > DS28E07::MemorySize_Bytes) {
        sendNack(SMERR_ReadLengthOutOfRange);
        return;
    }


    // Copy valid the command as a response header
    memcpy(usart_tx_mem, pCmd, sizeof(SwiMuxCmdRead_t));

    if (devs[pCmd->busIndex].read(pCmd->offset, &usart_tx_mem[sizeof(SwiMuxCmdRead_t)], pCmd->length) == pCmd->length) {
        // Device read successful.
        if (decoder.encode(usart_tx_mem, pCmd->length + sizeof(SwiMuxCmdRead_t), uart_writeByte) == false) {
            sendNack(SMERR_ResponseEncodingFailed); // failed to read
        } // otherwise the `encode` success is producing the ACK itself on the serial line.
    } else {
        // Device read failed !
        sendNack(SMERR_ReadMemoryFailed);
    }
}



void processRollCall()
{

    if (plength < 2) {
        sendNack(SMERR_Framing);
        return;
    }

    SwiMuxRollCallResult_t answer;
    memset(&answer, 0, sizeof(SwiMuxRollCallResult_t));
    answer.Opcode    = SMCMD_RollCall;
    answer.NegOpCode = ~answer.Opcode;
    for (uint8_t idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        if (!devs[idx].getUid(answer.orderedUids[idx].bytes_LE)) {
            answer.orderedUids[idx].value = 0ULL;
        }
    }
    __NOP();
    // Now send `answer`
    if (!decoder.encode((const uint8_t*)&answer, sizeof(SwiMuxRollCallResult_t), uart_writeByte)) {
        sendNack(SMERR_ResponseEncodingFailed); // Could not encode `answer` .
    }
}


static void processWrite()
{
    SwiMuxCmdWrite_t* cmd = (SwiMuxCmdWrite_t*)g_payload;
    if (cmd->busIndex > 5) {
        sendNack(SwiMuxError_e::SMERR_BusIndexOutOfRange);
        return;
    }
    if (cmd->offset >= 128) {
        sendNack(SMERR_MemOffsetOutOfRange);
        return;
    }
    if ((cmd->offset + cmd->length) > DS28E07::MemorySize_Bytes) {
        sendNack(SMERR_WriteLengthOutOfRange);
        return;
    }

    uint16_t written = 0;
    // Write data from the first byte after cmd->len (that's our received payload).
    written = devs[cmd->busIndex].write((const uint16_t)cmd->offset, &(&cmd->length)[1], cmd->length);
    if (written != cmd->length) {
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
    SwiMuxGetUID_t* cmd = (SwiMuxGetUID_t*)g_payload;
    if (cmd->busIndex >= NUMBER_OF_BUSES) {
        sendNack(SwiMuxError_e::SMERR_BusIndexOutOfRange);
        return;
    }
    SwiMuxRespUID_t uidResp;
    uidResp.uid.value = 0ULL;
    int retries       = 3;
    while (retries--) {
        if (devs[cmd->busIndex].getUid(uidResp.uid.bytes_LE)) {
            if (uidResp.uid.value != UINT64_MAX)
                break; // good value
        }
    }
    if (retries > 0 && uidResp.uid.value != 0 && uidResp.uid.value != UINT64_MAX) {
        // Send the Guid response packet
        sendUid(uidResp);
    } else {
        sendNack(SMERR_GuidUnreadable);
    }
}



static void processSleep(bool autoSleep)
{

#ifndef AUTOSLEEP_ENABLED
    if (!autoSleep) {
        sendNack(SwiMuxError_e::SMERR_CommandDisabled);
        
    }
#else
    if (!autoSleep) {
        sendAck(SMCMD_Sleep);
    }
    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        devs[idx].disableBus(); // Cut power (if originating from the SwiMux).
        //devs[idx].enableChangeDetection(); //Enable pin change detection on all buses.
    }
    Delay_Us(1000); // wait 1ms (1000µs)

    SysTick->CTLR = 0; // Disable system counter.

    // An event on the buses or the uart should wake us up.
    __WFE(); // sleep.

#if defined(FUNCONF_SYSTICK_USE_HCLK) && FUNCONF_SYSTICK_USE_HCLK
    // Reenable Systick counter
    SysTick->CTLR = 5;
#else
    // Reenable Systick counter
    SysTick->CTLR = 1;
#endif
    // Reenable the buses.
    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        devs[idx].enableBus();
        //devs[idx].disableChangeDetection();
    }
#endif
}



static void processPresenceReport()
{
    uint32_t result = 0;
    for (int idx = 0; idx < NUMBER_OF_BUSES; idx++) {
        result |= devs[idx].get_self_address() ? (1 << idx) : 0;
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
