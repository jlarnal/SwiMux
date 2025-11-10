#ifndef H_INTERPRETER_H
#define H_INTERPRETER_H

#ifdef CH32V00X
#include <ch32fun.h>
#endif

#include <stdint.h>
#include <stddef.h>
#include "StaticBuffer.hpp"

enum SwiMuxOpcodes_e : uint8_t
{
    SMCMD_ZERO       = 0,
    SMCMD_ReadBytes  = 1,
    SMCMD_WriteBytes = 2,
    SMCMD_GetUID     = 3,
    SMCMD_RollCall   = 4,
    SMCMD_ConnectEvent,
    SMCMD_GetPresence = SMCMD_ConnectEvent,
    SMCMD_HaveUID,
    SMCMD_Sleep,
    SMCMD_Autosleep,
    SMCMD_Ack,
    SMCMD_Nack,
};

enum SwiMuxError_e : uint8_t
{
    SMERR_Ok            = 0,
    SMERR_InProgress    = SMERR_Ok,
    SMERR_Done          = 1,
    SMERR_UnkownCommand = 0xC0,
    SMERR_ERRORS = SMERR_UnkownCommand,
    SMERR_Framing,
    SMERR_WrongEscape,
    SMERR_ReadBytesParams,
    SMERR_BusIndexOutOfRange,
    SMERR_MemOffsetOutOfRange,
    SMERR_ReadLengthOutOfRange,
    SMERR_ReadMemoryFailed,
    SMERR_ResponseEncodingFailed,
    SMERR_WriteLengthOutOfRange,
    SMERR_WriteFailed,
    SMERR_GuidUnreadable,
    SMERR_BadCrc,
    SMERR_BADFUNCALL, // critial software error
};

union __attribute__((packed)) UidType_t {        
    uint8_t bytes_LE[8];
    uint64_t value;
};

struct __attribute__((packed)) SwiMuxCmdRead_t {
    const SwiMuxOpcodes_e Opcode;
    const uint8_t NegOpcode;
    const uint8_t busIndex;
    const uint8_t offset;
    const uint8_t length;
};


struct __attribute__((packed)) SwiMuxCmdWrite_t {
    SwiMuxOpcodes_e Opcode;
    uint8_t NegOpcode;
    uint8_t busIndex;
    uint8_t offset;
    uint8_t length;
};

struct __attribute__((packed)) SwiMuxCmdPresence_t {
    SwiMuxOpcodes_e Opcode;
    uint8_t NegOpcode;
    uint8_t presenceLSB;
    uint8_t presenceMSB;
    uint8_t busesCount;
};

struct __attribute__((packed)) SwiMuxGetUID_t {
    const SwiMuxOpcodes_e Opcode;
    const uint8_t NegOpcode;
    const uint8_t busIndex;
    SwiMuxGetUID_t() : Opcode(SwiMuxOpcodes_e::SMCMD_GetUID), NegOpcode(0xFF & (~(uint8_t)SwiMuxOpcodes_e::SMCMD_GetUID)), busIndex(0) {}
    SwiMuxGetUID_t(uint8_t busIndex)
        : Opcode(SwiMuxOpcodes_e::SMCMD_GetUID), NegOpcode(0xFF & (~(uint8_t)SwiMuxOpcodes_e::SMCMD_GetUID)), busIndex(busIndex)
    {}
};


struct __attribute__((packed)) SwiMuxRespUID_t {
    SwiMuxOpcodes_e Opcode;
    uint8_t NegOpcode;
    UidType_t uid;
    SwiMuxRespUID_t() : Opcode(SMCMD_ZERO), NegOpcode(0) {}
};

struct __attribute__((packed)) SwiMuxRollCallResult_t {
    SwiMuxOpcodes_e Opcode;
    uint8_t NegOpCode;
    UidType_t orderedUids[NUMBER_OF_BUSES]; // Six uint64 values, some may be all zeros (absent)
};

struct RollCallArray_t {
    uint64_t bus[NUMBER_OF_BUSES];
};

const uint8_t SwiMuxRequest_GetPresence[2] = { SMCMD_GetPresence, (uint8_t)(0xFF & ~SMCMD_GetPresence) };
const uint8_t SwiMuxRequest_Sleep[2]       = { SMCMD_Sleep, (uint8_t)(0xFF & ~SMCMD_Sleep) };

template <typename> class SwiMuxDelegateFunc_t;

template <typename Ret, typename... Args> class SwiMuxDelegateFunc_t<Ret(Args...)> {
    using FnPtr = Ret (*)(void*, Args...);

    void* obj;
    FnPtr fn;

  public:
    SwiMuxDelegateFunc_t() : obj(nullptr), fn(nullptr) {}

    // lambdas / functors
    template <typename T> SwiMuxDelegateFunc_t(const T& lambda)
    {
        // We must cast away the const to store it in a void*, but it's safe
        // because the lambda's call operator is const by default.
        obj = const_cast<void*>(static_cast<const void*>(&lambda));
        fn  = [](void* o, Args... args) -> Ret {
            // Cast back to a const T* to correctly invoke the call operator
            return (*static_cast<const T*>(o))(args...);
        };
    }
    // raw function pointer
    SwiMuxDelegateFunc_t(Ret (*func)(Args...))
    {
        obj = reinterpret_cast<void*>(func);
        fn  = [](void* o, Args... args) -> Ret {
            auto f = reinterpret_cast<Ret (*)(Args...)>(o);
            return f(args...);
        };
    }


    // allow = nullptr
    SwiMuxDelegateFunc_t(nullptr_t) : obj(nullptr), fn(nullptr) {}

    Ret operator()(Args... args) const { return fn(obj, args...); }

    explicit operator bool() const { return fn != nullptr; }
};



class SwiMuxComms_t {
  public:
    SwiMuxComms_t()
        : _calculated_crc(0),
#ifdef SWIMUX_USES_COBS
          _code(0),
#endif
          _block(0),
          _framedOrEscaped(false),
          _resetOnNextDecode(true)
    {}

    bool encode(const uint8_t* input, size_t len, SwiMuxDelegateFunc_t<void(uint8_t)> resultWriter);

    SwiMuxError_e decode(const uint8_t byte, uint8_t*& payload, size_t& payloadLength);

    bool waitForAckTo(const SwiMuxOpcodes_e opCode, SwiMuxDelegateFunc_t<unsigned long()> getTime_ms, SwiMuxDelegateFunc_t<int(void)> readFunc,
      SwiMuxDelegateFunc_t<void(unsigned long)> delay_ms_func = {});

    void resync(SwiMuxDelegateFunc_t<void(uint8_t)> writer, SwiMuxDelegateFunc_t<void(unsigned long)> delay_ms_func = {}, bool do_reset = true);

    void reset();

    void sendUid(SwiMuxRespUID_t& uidResp, SwiMuxDelegateFunc_t<void(uint8_t)> writer);
    void sendNack(SwiMuxError_e error, SwiMuxDelegateFunc_t<void(uint8_t)> writer);
    void sendAck(SwiMuxOpcodes_e opcode, SwiMuxDelegateFunc_t<void(uint8_t)> writer);
    void sendAckArgs(SwiMuxOpcodes_e opcode, uint8_t arg, SwiMuxDelegateFunc_t<void(uint8_t)> writer);

  private:
    static constexpr uint8_t ESC     = 0xDB;
    static constexpr uint8_t ESC_ESC = 0xDD;
    static constexpr uint8_t END     = 0xC0;
    static constexpr uint8_t ESC_END = 0xDC;

    static constexpr size_t SWIMUX_BUFF_SIZE = 140;
    StaticBuffer_t<SWIMUX_BUFF_SIZE> _buffer;
    uint32_t _calculated_crc, _crc_consumed;
#ifdef SWIMUX_USES_COBS
    uint8_t _code;
#endif
    uint8_t _block;
    bool _framedOrEscaped, _resetOnNextDecode;
};


#endif //!H_INTERPRETER_H
