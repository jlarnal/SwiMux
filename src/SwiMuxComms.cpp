#include "SwiMuxComms.hpp"



// CRC32 (reflected poly 0xEDB88320), bitwise, small-footprint
static inline uint32_t crc32_init(void)
{
    return 0xFFFFFFFFu;
}

static inline uint32_t crc32_update(uint32_t crc, uint8_t data)
{
    crc ^= (uint32_t)data;
    for (int i = 0; i < 8; ++i) {
        if (crc & 1u)
            crc = (crc >> 1) ^ 0xEDB88320u;
        else
            crc >>= 1;
    }
    return crc;
}

static inline uint32_t crc32_finalize(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}



/**  @brief Sends a error packet. */
void SwiMuxComms_t::sendNack(SwiMuxError_e error, SwiMuxDelegateFunc_t<void(uint8_t)> writer)
{
    uint8_t msg[3] = { SMCMD_Nack, (uint8_t)(0xFF & ~SMCMD_Nack), error };
    encode(msg, 3, writer);
}

void SwiMuxComms_t::sendAck(SwiMuxOpcodes_e opcode, SwiMuxDelegateFunc_t<void(uint8_t)> writer)
{
    uint8_t msg[3] = { SMCMD_Ack, (uint8_t)(0xFF & ~SMCMD_Ack), opcode };
    encode(msg, 3, writer);
}

void SwiMuxComms_t::sendAckArgs(SwiMuxOpcodes_e opcode, uint8_t arg, SwiMuxDelegateFunc_t<void(uint8_t)> writer)
{
    uint8_t msg[4] = { SMCMD_Ack, (uint8_t)(0xFF & ~SMCMD_Ack), opcode, arg };
    encode(msg, 4, writer);
}

void SwiMuxComms_t::sendUid(SwiMuxRespUID_t& uidResp, SwiMuxDelegateFunc_t<void(uint8_t)> writer)
{
    uidResp.Opcode    = SMCMD_HaveUID;
    uidResp.NegOpcode = ~SMCMD_HaveUID;
    encode((uint8_t*)&uidResp, sizeof(SwiMuxRespUID_t), writer);
}


bool SwiMuxComms_t::waitForAckTo(const SwiMuxOpcodes_e opCode, SwiMuxDelegateFunc_t<unsigned long()> getTime_ms,
  SwiMuxDelegateFunc_t<int(void)> readFunc, SwiMuxDelegateFunc_t<void(unsigned long)> delay_ms_func)
{
    uint32_t timeout_ms = 0;
    switch (opCode) {
        case SwiMuxOpcodes_e::SMCMD_Wakeup:
            timeout_ms = 2;
            break;
        case SwiMuxOpcodes_e::SMCMD_GetPresence:
            [[fallthrough]];
        case SwiMuxOpcodes_e::SMCMD_Sleep:
            timeout_ms = 10;
            break;
        default:
            timeout_ms = 100;
            break;
    }

    uint32_t startTime = getTime_ms();
    uint8_t* payload   = nullptr;
    size_t pLen        = 0;
    int val;

    do {
        val = readFunc();
        if (val > -1) {
            SwiMuxError_e res = decode((uint8_t)val, payload, pLen);
            if (res == SMERR_Done) {
                if (pLen != 0 && payload != nullptr) { // frame ?
                    // We had our response frame, let's check its contents.
                    if (pLen >= 3 && payload[0] == (uint8_t)SwiMuxOpcodes_e::SMCMD_Ack && payload[1] == ((uint8_t)0xFF ^ SMCMD_Ack)
                      && payload[2] == (uint8_t)opCode) {
                        return true; // opCode was acknowledged ! Yay
                    } else {
                        return false; // error in the frame.
                    }
                }
            }
            // Have some time to receive other bytes.
            if (delay_ms_func)
                delay_ms_func(1);
        }
    } while ((getTime_ms() - startTime) <= timeout_ms);
    return false;
}

void SwiMuxComms_t::reset()
{
    _buffer.clear();
    _framedOrEscaped = false;
    _crc_consumed    = 0;
    _calculated_crc  = crc32_init(); // initialize incremental CRC state
#ifdef SWIMUX_USES_COBS
    _code = SMERR_Ok;
#endif
    _resetOnNextDecode = false;
}

#ifdef SWIMUX_USES_COBS
/**
 * @brief COBS-encodes input||CRC32 and write result using resultWriter callback.
 *
 * @param input pointer to payload bytes
 * @param len payload length (must be <= SWIMUX_BUFF_SIZE)
 * @param resultWriter reference to a function that writes out each encoded byte (signed char)
 * @return true on success, false on invalid input (len too large)
 */
bool SwiMuxComms_t::encode(const uint8_t* input, size_t len, SwiMuxDelegateFunc_t<void(uint8_t)> resultWriter)
{
    // enforce your stated maximum frame size (payload <= SWIMUX_BUFF_SIZE). Reject otherwise.
    if (len > SWIMUX_BUFF_SIZE)
        return false;

    // compute CRC32 over payload
    uint32_t crc = crc32_init();
    for (size_t i = 0; i < len; ++i)
        crc = crc32_update(crc, input[i]);
    uint32_t final_crc = crc32_finalize(crc);

    // CRC bytes (big-endian)
    uint8_t crc_bytes[4];
    crc_bytes[0] = (uint8_t)((final_crc >> 24) & 0xFFu);
    crc_bytes[1] = (uint8_t)((final_crc >> 16) & 0xFFu);
    crc_bytes[2] = (uint8_t)((final_crc >> 8) & 0xFFu);
    crc_bytes[3] = (uint8_t)(final_crc & 0xFFu);

    // total logical length of stream to encode = payload + 4 CRC bytes
    const size_t total_len = len + 4;

    // helper to read the k-th byte of the logical stream:
    // - if k < len -> input[k]
    // - else -> crc_bytes[k - len]
    auto getByteAt = [&](size_t k) -> uint8_t { return (k < len) ? input[k] : crc_bytes[k - len]; };

    // COBS encoding main loop:
    // at pos 'i' we have to find block_len = number of consecutive non-zero bytes
    // starting at i, but not more than 254 bytes (per COBS) to allow _code==0xFF.
    size_t i = 0;
    while (i < total_len) {
        // find block_len
        size_t j = i;
        // scan up to SWIMUX_BUFF_SIZE bytes or until a zero encountered
        const size_t maxBlockData = SWIMUX_BUFF_SIZE + 4; // maximum number of data bytes in a _block
        while (j < total_len && (j - i) < maxBlockData && getByteAt(j) != 0) {
            ++j;
        }
        size_t block_len = j - i; // number of non-zero bytes to emit
        uint8_t _code    = (uint8_t)(block_len + 1u); // _code byte to emit

        // write _code byte
        resultWriter(static_cast<char>(_code));

        // write _block data bytes (if any) directly from input/crc_bytes
        for (size_t k = i; k < i + block_len; ++k) {
            resultWriter(static_cast<char>(getByteAt(k)));
        }

        // advance i:
        // - if we stopped because of an encountered zero (j < total_len && getByteAt(j) == 0),
        //   skip that zero (i = j + 1)
        // - else (we stopped due to hitting maxBlockData or reaching end), set i = j
        if (j < total_len && getByteAt(j) == 0) {
            i = j + 1; // skip the zero; it's represented by _code<0xFF
        } else {
            i = j; // either end reached or _block was exactly maxBlockData (_code==0xFF)
        }
    }

    // final frame delimiter
    resultWriter(static_cast<char>(0x00));

    return true;
}



bool SwiMuxComms_t::decode(const uint8_t byte, uint8_t*& payload, size_t& payloadLength)
{
    payload       = nullptr;
    payloadLength = 0;
    if (_resetOnNextDecode) {
        reset();
    }

    // waiting for first _code byte
    if (!_framedOrEscaped) {
        if (byte == 0)
            return true; // still waiting for start
        _framedOrEscaped = true;
        _code            = byte;
        _block           = _code - 1;
        _buffer.clear();
        return true;
    }

    // inside a _block expecting '_block' raw bytes
    if (_block > 0) {
        if (!_buffer.append(byte)) {
            reset();
            return false;
        }
        --_block;
        return true;
    }

    // _block == 0: byte is either the next _code, or 0 => end of frame
    if (byte == 0) {
        // end-of-frame: buffer contains decoded bytes = payload || CRC(4)
        size_t total = _buffer.count();
        if (total < 4) {
            reset();
            return false;
        } // not enough data for CRC

        size_t dataLen = total - 4;

        // compute CRC32 over the payload bytes
        uint32_t crc = crc32_init();
        for (size_t i = 0; i < dataLen; ++i)
            crc = crc32_update(crc, _buffer[i]);
        uint32_t computed = crc32_finalize(crc);

        // reconstruct appended CRC (big-endian)
        uint32_t appended = ((uint32_t)_buffer[dataLen] << 24) | ((uint32_t)_buffer[dataLen + 1] << 16) | ((uint32_t)_buffer[dataLen + 2] << 8)
          | ((uint32_t)_buffer[dataLen + 3]);

        if (computed == appended) {
            payload       = (uint8_t*)_buffer;
            payloadLength = dataLen;
            reset();
            return true;
        } else {
            reset();
            return false;
        }
    } else {
        // new _code byte
        _code  = byte;
        _block = _code - 1;
        if (_code < 0xFF) {
            // implicit zero between blocks
            if (!_buffer.append(0)) {
                reset();
                return false;
            }
        }
        return true;
    }
}
#elif defined(SWIMUX_USES_SLIP)


/**
 * @brief SLIP-encodes input||CRC32 and write result using resultWriter callback.
 *
 * @param input pointer to payload bytes
 * @param len payload length (must be <= SWIMUX_BUFF_SIZE)
 * @param resultWriter reference to a function that writes out each encoded byte (signed char)
 * @return true on success, false on invalid input (len too large)
 */
bool SwiMuxComms_t::encode(const uint8_t* input, size_t len, SwiMuxDelegateFunc_t<void(uint8_t)> resultWriter)
{
    // Validate inputs
    if (len > SWIMUX_BUFF_SIZE)
        return false;
    if (len > 0 && input == nullptr)
        return false;

    // Compute CRC over payload
    uint32_t crc = crc32_init();
    for (size_t i = 0; i < len; ++i) {
        crc = crc32_update(crc, input[i]);

        switch (input[i]) {
            case SwiMuxComms_t::ESC:
                resultWriter(SwiMuxComms_t::ESC);
                resultWriter(SwiMuxComms_t::ESC_ESC);
                break;
            case SwiMuxComms_t::END:
                resultWriter(SwiMuxComms_t::ESC);
                resultWriter(SwiMuxComms_t::ESC_END);
                break; // <-- missing in original
            default:
                resultWriter(input[i]);
                break;
        }
    }
    crc = crc32_finalize(crc);

    // Emit CRC bytes in little-endian order, escaping as needed
    for (int b = 0; b < 4; ++b) {
        uint8_t cb = static_cast<uint8_t>((crc >> (8 * b)) & 0xFF); // explicit little-endian
        switch (cb) {
            case SwiMuxComms_t::ESC:
                resultWriter(SwiMuxComms_t::ESC);
                resultWriter(SwiMuxComms_t::ESC_ESC);
                break;
            case SwiMuxComms_t::END:
                resultWriter(SwiMuxComms_t::ESC);
                resultWriter(SwiMuxComms_t::ESC_END);
                break;
            default:
                resultWriter(cb); // <-- use cb, not *input
                break;
        }
    }

    // Write final frame terminator
    resultWriter(SwiMuxComms_t::END);
    return true;
}

/**
 * @brief Decodes a single byte from a SwiMuxComms_t-encoded stream.
 * This method processes bytes one at a time. While decoding, it verifies the CRC32 checksum and, once decode, sets @p payload and @p payloadLength parameters to valid values. Upon error (framing, buffer overflow, bad CRC), the internal buffer is cleared, and the method returns false.
 * @note After a successful decode (when this function returns a non-null @p payload and non-zero @p payloadLength), the caller must use the contents of @p payload BEFORE calling this method again.
 * @param byte The incoming byte from the serial stream.
 * @param payload A reference to a pointer that will be set to the start of the decoded data.
 * @param payloadLength A reference to a size_t that will be set to the length of the decoded data (excluding the CRC32).
 * @return A value of SwiMuxError_e, preferably @c SwiMuxError_e::SMERR_Ok if everything went smoothly or SMERR_Done if a payload has been decoded * 
 */
SwiMuxError_e SwiMuxComms_t::decode(const uint8_t byte, uint8_t*& payload, size_t& payloadLength)
{
    // Safe defaults
    payload       = nullptr;
    payloadLength = 0;
    if (_resetOnNextDecode) {
        reset();
    }

    // If we were waiting for an escaped byte, handle it first
    if (_framedOrEscaped) {
        _framedOrEscaped = false;

        uint8_t translated;
        if (byte == SwiMuxComms_t::ESC_END)
            translated = SwiMuxComms_t::END;
        else if (byte == SwiMuxComms_t::ESC_ESC)
            translated = SwiMuxComms_t::ESC;
        else {
            // Protocol violation after ESC
            SwiMuxError_e ret  = SMERR_Framing;
            _resetOnNextDecode = true;
            return ret;
        }

        if (!_buffer.append(translated)) {
            SwiMuxError_e ret  = SMERR_Framing;
            _resetOnNextDecode = true;
            return ret;
        }

        // Update incremental CRC for any newly "safe" payload bytes
        const size_t bufCount      = _buffer.count();
        const size_t known_payload = (bufCount > 4) ? (bufCount - 4) : 0;
        while (_crc_consumed < known_payload)
            _calculated_crc = crc32_update(_calculated_crc, _buffer[_crc_consumed++]);


        return SMERR_InProgress;
    }

    // Normal byte handling
    switch (byte) {
        case SwiMuxComms_t::END:
            {
                const size_t count = _buffer.count();

                if (count < 4) {
                    // If buffer had bytes but less than CRC => framing error, otherwise just delimiter
                    SwiMuxError_e ret  = (count > 0) ? SMERR_Framing : SMERR_InProgress;
                    _resetOnNextDecode = true;

                    return ret;
                }

                const size_t payload_len = count - 4;

                // Ensure all payload bytes have been fed into CRC
                while (_crc_consumed < payload_len)
                    _calculated_crc = crc32_update(_calculated_crc, _buffer[_crc_consumed++]);

                // Extract received CRC (little-endian)
                uint32_t received_crc = (uint32_t)_buffer[payload_len + 0] | ((uint32_t)_buffer[payload_len + 1] << 8)
                  | ((uint32_t)_buffer[payload_len + 2] << 16) | ((uint32_t)_buffer[payload_len + 3] << 24);

                const uint32_t calculated_crc = crc32_finalize(_calculated_crc);

                if (calculated_crc == received_crc) {
                    // Success: point to internal buffer memory and return success
                    payload       = (uint8_t*)_buffer; // keep same convention you used earlier
                    payloadLength = payload_len;


                    _resetOnNextDecode = true;
                    return SMERR_Done;
                } else {
                    SwiMuxError_e ret  = SMERR_BadCrc;
                    _resetOnNextDecode = true;
                    return ret;
                }
            }

        case SwiMuxComms_t::ESC:
            _framedOrEscaped = true;
            return SMERR_InProgress;

        default:
            if (!_buffer.append(byte)) {
                SwiMuxError_e ret  = SMERR_Framing;
                _resetOnNextDecode = true;
                return ret;
            }

            // Update incremental CRC for any newly "safe" payload bytes
            {
                const size_t bufCount      = _buffer.count();
                const size_t known_payload = (bufCount > 4) ? (bufCount - 4) : 0;
                while (_crc_consumed < known_payload)
                    _calculated_crc = crc32_update(_calculated_crc, _buffer[_crc_consumed++]);
            }

            return SMERR_InProgress;
    }
}


#else
#error "No encoding scheme selected for SWIMUX"
#endif


void SwiMuxComms_t::resync(SwiMuxDelegateFunc_t<void(uint8_t)> writer, SwiMuxDelegateFunc_t<void(unsigned long)> delay_ms_func, bool do_reset)
{
    writer(END);
    writer(END);
    writer(END);
    if (delay_ms_func)
        delay_ms_func(2); // almost 4x the time under 57600 bauds.

    if (do_reset)
        reset();
}