#ifndef H_DS28E0_H
#define H_DS28E0_H
#include "OneWire.h"



class DS28E07 {
  public:
    DS28E07() : _lastError(OneWireError_e::NO_ERROR) {}

    OneWireError_e begin(GPIO_TypeDef* dioPort, int8_t dioPin, GPIO_TypeDef* pullupPort = nullptr, int8_t pullupPin = -1);
    inline OneWireError_e begin(const OneWire::OneWireConfig_t& config)
    {
        return begin(config.dioPort, config.dioPin, config.pullupPort, config.pullupPin);
    }


    inline void disableBus() { _ow.disableBus(); }
    inline void disableChangeDetection() { _ow.disableChangeDetection(); }
    inline void enableBus() { _ow.enableBus(); }
    inline void enableChangeDetection() { _ow.enableChangeDetection(); }

    /** @brief Returns the laser-etched ROM ID of the sole device on the bus. 
     * @param[out] uid An buffer of at least 8 bytes to receive the result.
     * @returns <false> if the read failed, <true> otherwise.
     * @note For single-device buses only. Do NOT use this method on multi-drop buses, the result is guaranteed to be erroneous due to collisions.
    */
    bool getUid(uint8_t* uid);

    /** @brief Makes the instance take the address of the 1st device found, and return <true> if successful. */
    bool get_self_address();



    /** @brief Resets the bus the device is connected to. */
    inline bool reset()
    {
        _lastError = _ow.reset_bus();
        return _lastError == OneWireError_e::NO_ERROR;
    }

    /**
     * @brief Reads a series of contiguous bytes from the device, starting at a given source offset.
     * @note If the device is already selected, the `overdrive` argument is dismissed. To enforce the overdrive, first reset the bus
     * @param address Address within the device from which to start reading.
     * @param buff Destination buffer
     * @param length Number of bytes to read
     * @return The effective number of bytes read, 0 (zero) in case of error (use `getLastError()` to know why).
     */
    uint16_t read(const uint16_t address, uint8_t* buff, const uint16_t length, bool multidrop = false, bool overdrive = false);

    /**
     * @brief Writes a buffer of bytes to the device, starting at a given destination offset .
     * @param offset Address in the device from which to start the writing.
     * @param data Data to write
     * @param length Number of bytes to write
     * @param[optional] retries Number of retries to perform if the 1-wire communication fails. Defaults to `DS28E07::DEFAULT_RETRIES_COUNT`.
     * @return The effective number of bytes written, 0 (zero) in case of error (use `getLastError()` to know why).
     */
    uint16_t write(const uint16_t address, const uint8_t* data, const uint16_t length, bool multidrop = false, bool overdrive = false,
      uint8_t retries = DEFAULT_RETRIES_COUNT);

    /**
     * @brief Erase all contents of the DS28E07 device. 
     * @param passcode A safeguard value. Pass it `DS28E07::ErasurePassCode" to perform the operation.
     * @return The number of effectively erased bytes.
     */
    uint16_t eraseAll(uint32_t passcode);

    /** @brief Gets the last error encountered by the instance. Expect `OneWireError_e::NO_ERROR` for success.  */
    inline OneWireError_e getLastError() { return _lastError; }

    static constexpr size_t MemorySize_Bytes  = 128;
    static constexpr uint32_t ErasurePassCode = 0xBADBEEF;

  protected:
    static constexpr uint8_t DEFAULT_RETRIES_COUNT = 16;
    static constexpr uint16_t CRC16_INITIAL_VALUE  = 0;

    OneWire _ow;
    uint8_t _devAddress[8]    = { 0, 0, 0, 0, 0, 0, 0, 0 };
    OneWireError_e _lastError = OneWireError_e::NO_ERROR;
    bool _hasAddress = false, _isSelected = false;



    /**
     * @brief Reads the currently selected device's ID (lasered ROM)
     * @param[out] uid The result of the 64-bits read operation.
     */
    bool read_rom(uint8_t* uid);


    /**
     * @brief Selects a device on a multi-drop bus by its UID and keeps it selected until bus reset.
     * @param uid 
     * @return 
     */
    void match_rom(const uint64_t uid);



    /** 
     * @brief Returns the next detected device ID on the bus.
     */
    uint64_t search_rom();

    /**
     * @brief Resets the rom search.
     */
    void reset_search_rom() { _ow.resetSearch(); }

    /**
     * @brief Skips the device address (ID) selection (for single-device buses only) and enters MEMORY addressing directly.
     */
    void skip_rom() { _ow.skip(); }

    struct ScratchPadWCP_Arg_t {
        const uint8_t* buffer;
        uint16_t address;
        int16_t retries;
        bool multidrop, overdrive;

        ScratchPadWCP_Arg_t(const uint16_t addrs, const uint8_t* buff, uint16_t retriesCount = DS28E07::DEFAULT_RETRIES_COUNT, bool multiDrop = false,
          bool overDrive = false)
            : buffer(buff), address(addrs), retries(retriesCount), multidrop(multiDrop), overdrive(overDrive)
        {}
        ScratchPadWCP_Arg_t() : buffer(nullptr), address(0), retries(0), multidrop(false), overdrive(false) {}
        ScratchPadWCP_Arg_t(ScratchPadWCP_Arg_t&& other)
            : buffer(other.buffer), address(other.address), retries(other.retries), multidrop(other.multidrop), overdrive(other.overdrive)
        {}
        ScratchPadWCP_Arg_t& operator=(ScratchPadWCP_Arg_t&& other)
        {
            buffer    = other.buffer;
            address   = other.address;
            retries   = other.retries;
            multidrop = other.multidrop;
            overdrive = other.overdrive;
            return *this;
        }
    };

    /**
     * @brief Writes 8 bytes to memory at the specified address.
     * @note This method should work wether the device is in overdrive speed or not.
     * @param address Address at which the write are to take place. MUST be a multiple of 8.
     * @param buffer An 8-bytes array.
     * @param retries Number of retries to take place before giving up.
     */
    bool scratch_wrt_chk_cpy(ScratchPadWCP_Arg_t& args);

    bool assertSelected(bool multidrop, bool overdrive);
};

#endif // H_DS28E0_H
