#include "DS28E07.h"
#include "ch32fun.h"



__attribute__((__always_inline__, nonnull(1, 2))) static inline void flipBytes64(uint8_t* dest, const uint8_t* const src)
{
    dest[7] = src[0];
    dest[6] = src[1];
    dest[5] = src[2];
    dest[4] = src[3];
    dest[3] = src[4];
    dest[2] = src[5];
    dest[1] = src[6];
    dest[0] = src[7];
}


OneWireError_e DS28E07::begin(GPIO_TypeDef* dioPort, int8_t dioPin, GPIO_TypeDef* pullupPort, int8_t pullupPin)
{
    // First, we initialize the scalar members (if any)...

    _lastError = OneWireError_e::NO_ERROR;

    // ...and then OneWire object, which gets us our return value.
    OneWireError_e res = _ow.begin(dioPort, dioPin, pullupPort, pullupPin);
    if (res == OneWireError_e::NO_ERROR) {
        if (get_self_address() == false) {
            dbgpf("DS28E07.get_self_address() FAILED with code #0x%X\r\n", _lastError);
            return _lastError;
        }
    } else {
        _lastError = res;
        dbgpf("Error #0x%X on OneWire.begin().\r\n", res);
    }
    return _lastError;
}


bool DS28E07::get_self_address()
{
    _isSelected = false;
    {
        OneWireError_e res = _ow.reset_bus();
        if (res != OneWireError_e::NO_ERROR) {
            _lastError = res;
            return false;
        }
    }
    if (!read_rom(_devAddress)) {
        _hasAddress = false;
        memset(_devAddress, 0, 8);
    } else {
        _hasAddress = true;
    }
    return _hasAddress;
}

bool DS28E07::getUid(uint8_t* result)
{
    if (result == nullptr) {
        _lastError = OneWireError_e::NULL_INPUT_BUFFER;
        return false;
    }

    return read_rom(result);
}



bool DS28E07::read_rom(uint8_t* uid)
{
    if (uid == nullptr) {
        _lastError = OneWireError_e::NULL_INPUT_BUFFER;
        return false;
    }
    {
        OneWireError_e res = _ow.reset_bus();
        if (res != OneWireError_e::NO_ERROR) {
            _isSelected = false;
            _lastError  = res;
            return false;
        }
    }

    // Send the command
    _ow.write(OW_ROM_COMMANDS_e::OW_READ_ROM);
    _isSelected = false;
    if (_ow.read(uid, 8) != 8) {

        _lastError = READ_ROM_FAILED;
        return false;
    }
    _lastError = NO_ERROR;


    return true;
}

void DS28E07::match_rom(const uint64_t uid)
{
    uint8_t* device;
#if BYTE_ORDER == LITTLE_ENDIAN
    device = (uint8_t*)&uid;
#else
    uint8_t dest[8], *src = (uint8_t*)&uid;
    flipBytes64(dest, src);
#endif
    _ow.select(device);
}

uint64_t DS28E07::search_rom()
{

    uint64_t result = 0;
    if (_ow.search((uint8_t*)&result)) {
        return result;
    }
    _isSelected = false;
    return 0ULL;
}

bool DS28E07::assertSelected(bool multidrop, bool overdrive)
{
    OneWireError_e res;
    res = _ow.reset_bus();
    // Always reset before select/resume
    if (res != OneWireError_e::NO_ERROR) {
        _lastError = res;
        return false;
    }

    //if (!_isSelected) {
    // Find out out addresss if unknown on multi-drop.
    if (multidrop && !_hasAddress) {
        if (!get_self_address()) {
            _isSelected = false;
            _lastError  = OneWireError_e::MULTIDROP_ID_UNREADABLE;
            return false;
        }
    }

    res = _ow.reset_bus();
    if (res != OneWireError_e::NO_ERROR) {
        _lastError = res;
        return false;
    }
    // Select the target device
    if (multidrop) {
        _ow.select(_devAddress, overdrive);
    } else {
        _ow.skip(overdrive);
    }
    _isSelected = true;
    //} else {
    //    _ow.write(OW_RESUME);
    //}
    return true;
}


uint16_t DS28E07::write(const uint16_t address, const uint8_t* data, const uint16_t length, bool multidrop, bool overdrive, uint8_t retries)
{
    if (data == nullptr) {
        _lastError = OneWireError_e::NULL_INPUT_BUFFER;
        return 0;
    }
    if (length == 0)
        return 0;
    if (retries == 0)
        retries = DEFAULT_RETRIES_COUNT;
    if (address >= MemorySize_Bytes) {
        _lastError = OneWireError_e::MEMADDRESS_OUT_OF_BOUNDS;
        return 0;
    }
    if ((address + length) > MemorySize_Bytes) { // too much for the DS28E07.
        _lastError = OneWireError_e::OUT_OF_BOUNDS;
        return 0;
    }

    uint16_t remains    = length;
    uint16_t wrtAddress = address;
    uint16_t offset     = address & 7; // offset of the 1st byte on 8-bytes boundaries.
    // Ensure the device is selected
    if (!assertSelected(multidrop, overdrive))
        return 0;

    ScratchPadWCP_Arg_t scpArgs;
    // Start writing. In case of unaligned address,
    // we'll have to perform a READ-MODIFY-WRITE, overwriting the 8-bytes chunk last bytes.
    if (offset) { // unaligned starting write address ?
        // We'll read the aligned block of 8 bytes the first bytes to write belongs to.
        uint8_t tmp[8];
        uint16_t bytesInChunk = 8 - offset;
        if (read(address - offset, tmp, 8, multidrop, overdrive) != 8) {
            _lastError = OneWireError_e::ALIGNED_WRITE_HEAD_PREREAD;
            return 0;
        }



        // We overwrite the bytes we initialy wanted to write...
        memcpy(&tmp[offset], data, 8 - offset);
        // And then we overwrite the 8-bytes clump itself
        scpArgs = ScratchPadWCP_Arg_t((uint16_t)(address - offset), tmp, retries, multidrop, overdrive);
        if (!scratch_wrt_chk_cpy(scpArgs)) {
            // _lastError has been set by `scratch_wrt_chk_cpy` itself.
            return 0;
        }
        data += bytesInChunk; // advance our source pointer
        wrtAddress += bytesInChunk;
        remains -= bytesInChunk;
    }


    // Then, as long as there's more than 7 bytes to write
    while (remains > 7) {
        scpArgs.address = wrtAddress;
        scpArgs.buffer  = data;
        if (!scratch_wrt_chk_cpy(scpArgs)) {
            return length - remains;
        }
        data += 8; // advance our source pointer
        remains -= 8;
    }

    // Are there still bytes to write (but not more than 7).
    // If so, it's a READ-MODIFY-WRITE again,
    // but we overwite the starting bytes of the chunk this time.
    if (remains) {
        uint8_t tmp[8];
        if (read(wrtAddress, tmp, 8, multidrop, overdrive) != 8) {
            _lastError  = OneWireError_e::ALIGNED_WRITE_TAIL_PREREAD;
            _isSelected = false;
        }
        memcpy(tmp, data, remains);
        scpArgs.address = (uint16_t)(address - offset);
        scpArgs.buffer  = tmp;
        if (!scratch_wrt_chk_cpy(scpArgs)) {
            // _lastError has been set by `scratch_wrt_chk_cpy` itself.
            return length - remains;
        }
        remains = 0; // just for tidyness, if we expand this method further.
    }
    // We're done, the whole buffer has been written, aligned or not.

    return length;
}



uint16_t DS28E07::read(const uint16_t address, uint8_t* buff, const uint16_t length, bool multidrop, bool overdrive)
{
    if (length == 0) // Read nothing ?
        return 0;
    if (buff == nullptr) {
        _lastError = OneWireError_e::NULL_OUTPUT_BUFFER;
        return 0;
    }
    if (address >= MemorySize_Bytes) {
        _lastError = OneWireError_e::MEMADDRESS_OUT_OF_BOUNDS;
        return 0;
    }
    if ((address + length) > MemorySize_Bytes) { // too much for the DS28E07.
        _lastError = OneWireError_e::OUT_OF_BOUNDS;
        return 0;
    }

    if (!assertSelected(multidrop, overdrive))
        return 0;
    // Otherwise, if the device was already selected, the `overdrive` & `multidrop` arguments are dismissed.

    // Now we read the memory
    _ow.write(OW_READ_MEMORY);
    _ow.write(address & 0xFF); // address' LSB
    _ow.write(address >> 8); // then MSB
    return _ow.read(buff, length);
}

bool DS28E07::scratch_wrt_chk_cpy(ScratchPadWCP_Arg_t& args)
{
    // First, let's declare some internal types to ease the code of this function.
    union WriteScratchpadSeq_t {
        struct __packed {
            union {
                struct __packed {
                    uint8_t cmd;
                    uint8_t T1;
                    uint8_t T2;
                    uint8_t scratchpad[8];
                };
                uint8_t bytes[sizeof(cmd) + sizeof(T1) + sizeof(T2) + sizeof(scratchpad)];
            } payload;
            union {
                struct __packed {
                    uint8_t LSB;
                    uint8_t MSB;
                };
                uint8_t bytes[2];
            } crc16;
        };
        uint8_t bytes[sizeof(payload) + sizeof(crc16)];
    };
    union ReadScratchpadSeq_t {
        struct __packed {
            uint8_t cmd;
            union {
                struct __packed {
                    uint8_t T1;
                    uint8_t T2;
                    uint8_t ES;
                    uint8_t scratchpad[8];
                    union {
                        struct __packed {
                            uint8_t LSB;
                            uint8_t MSB;
                        };
                        uint8_t bytes[2];
                    } crc16;
                };
                uint8_t bytes[sizeof(T1) + sizeof(T2) + sizeof(ES) + sizeof(scratchpad) + sizeof(crc16)];
            } payload;
        };
        uint8_t bytes[sizeof(cmd) + sizeof(payload)];
    };

    union TargetAddressAndEndStatus_t {
        struct {
            uint8_t T1;
            uint8_t T2;
            uint8_t ES;
        };
        uint8_t bytes[sizeof(T1) + sizeof(T2) + sizeof(ES)];
    };
    enum ESbits_e
    {
        ES_E0    = 0x01,
        ES_E1    = 0x02,
        ES_E2    = 0x04,
        ES_Emask = 0x07,
        ES_PF    = 0x20,
        ES_AA    = 0x80,

    };

    // Number of bytes processed for a READ_SCRATCHPAD crc16
    constexpr size_t CRC16_SCOPE_LENGTH
      = sizeof(ReadScratchpadSeq_t::cmd) + sizeof(ReadScratchpadSeq_t::payload) - sizeof(ReadScratchpadSeq_t::payload.crc16);



    uint8_t tmp[sizeof(ReadScratchpadSeq_t)]; // Can hold either the WriteScratchPad and the ReadScratchPad sequences.
    WriteScratchpadSeq_t* wsp = (WriteScratchpadSeq_t*)(void*)tmp;
    ReadScratchpadSeq_t* rsp  = (ReadScratchpadSeq_t*)tmp;
#ifdef ADD_CONSOLE_DEBUGGING
#warning "scratch_wrt_chk_cpy() retries are disabled for debug purposes"
    args.retries = 1;
#endif
    while (--args.retries >= 0) {
        if (!assertSelected(args.multidrop, args.overdrive)) {
            _lastError = OneWireError_e::WRITE_SCRATCHPAD_PRESELECT;
            return false;
        }
        wsp->payload.cmd = OW_WRITE_SCRATCHPAD;
        wsp->payload.T1  = args.address & 0xF8; // aligned on 8 bytes, as per datasheet says, so we dismiss bits [2:0].
        wsp->payload.T2  = (args.address >> 8); // always 0 on the DS28E07, but we do it anyways.
        memcpy(wsp->payload.scratchpad, args.buffer, 8); // copy our given contents.

        _ow.writeBytes(wsp->payload.bytes, sizeof(wsp->payload)); // command, TA1, TA2 and scratchpad (1+1+1+8)
        _ow.read(wsp->crc16.bytes, 2); // read the crc16 the DS28E07 responds with
        // Check the received crc16
        if (!_ow.checkCrc16(wsp->bytes, sizeof(wsp->payload), wsp->crc16.bytes, CRC16_INITIAL_VALUE)) {
            _lastError = OneWireError_e::WRITE_SCRATCHPAD_CRC16;
            continue; // loop back
        }

        // Reack back the scratchpad (and the end address status)

        if (!assertSelected(args.multidrop, args.overdrive)) {
            _lastError = OneWireError_e::READ_SCRATCHPAD_PRESELECT;
            return false;
        }
        rsp->cmd = OW_READ_SCRATCHPAD;
        _ow.write(rsp->cmd);
        _ow.read(rsp->payload.bytes, sizeof(ReadScratchpadSeq_t::payload));
        if (!_ow.checkCrc16(rsp->bytes, CRC16_SCOPE_LENGTH, rsp->payload.crc16.bytes, CRC16_INITIAL_VALUE)) {
            _lastError = OneWireError_e::READ_SCRATCHPAD_CRC16;
            continue;
        }

        if (rsp->payload.ES & ES_PF) {
            _lastError = OneWireError_e::SCRATCHPAD_PF;
            continue;
        }

        if (memcmp(rsp->payload.scratchpad, args.buffer, 8) != 0) {
            _lastError = OneWireError_e::WRITTEN_SCRATCHPAD_MISMATCH;
            continue; // buffer and scratchpad matches !
        }

        // Copy scratchpad to eeprom
        if (!assertSelected(args.multidrop, args.overdrive)) {
            _lastError = OneWireError_e::COPY_SCRATCHPAD_PRESELECT;
            return false;
        }
        rsp->cmd = OW_COPY_SCRATCHPAD;
        _ow.writeBytes(rsp->bytes, 4);
        // Now we offer 13ms for the device write its memory
        Delay_Ms(13); // that's slow as hell for a less than 10k endurance EEPROM.
        if (_ow.read() != 0xAA) {
            _lastError = OneWireError_e::COPY_SCRATCHPAD;
            continue;
        }
    }

    return args.retries < 0;
}


uint16_t DS28E07::eraseAll(uint32_t passcode)
{
    const uint8_t blankValues[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    if (passcode != DS28E07::ErasurePassCode) {
        return false;
    }


    uint16_t erasedBytes = 0; // by default
    for (uint16_t address = 0; address < MemorySize_Bytes; address += 8) {
        ScratchPadWCP_Arg_t args(address, blankValues, 4);
        if (scratch_wrt_chk_cpy(args))
            erasedBytes += 8;
    }
    return erasedBytes;
}