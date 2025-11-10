#include "OneWire.h"


static bool isGpioPort(GPIO_TypeDef* port)
{

    // Check for the validity of the given GPIO port
    return 0
#ifdef GPIOA
      || port == GPIOA
#endif
#ifdef GPIOB
      || port == GPIOB
#endif
#ifdef GPIOC
      || port == GPIOC
#endif
#ifdef GPIOD
      || port == GPIOD
#endif
#ifdef GPIOE
      || port == GPIOE
#endif
      ;
}

static void gpioRccClockEnable(GPIO_TypeDef* port)
{
    RCC->APB2PCENR |= RCC_IOPAEN << (((uint32_t)(void*)port - GPIOA_BASE) >> 10);
}

OneWireError_e OneWire::begin(GPIO_TypeDef* dio_port, const uint8_t dio_pin, GPIO_TypeDef* pullup_port, int8_t pullup_pin)
{

    // Check DIO port and pin
    if (dio_port == NULL)
        return OneWireError_e::DIO_PORT_NULL;
    if (!isGpioPort(dio_port))
        return OneWireError_e::DIO_PORT_INVALID;
    if (dio_pin < 0 || dio_pin > 15)
        return OneWireError_e::DIO_PIN_INVALID;

    // Check Pull-up port and pin
    if (pullup_port != nullptr) {
        if (!isGpioPort(pullup_port))
            return OneWireError_e::PULLUP_PORT_INVALID;
        if (pullup_pin < 0 || pullup_pin > 15)
            return OneWireError_e::PULLUP_PIN_INVALID;
    }


    // --- Configure DIO Pin ---
    //_dio_port   = dio_port;
    _dioPinMask = 1UL << dio_pin;
    _dioBSR     = &(dio_port->BSHR); // Bis set register (we won't use its 'reset' MSW)
    _dioBCR     = &(dio_port->BCR); // Bit clear regiser
    _dioINDR    = &(dio_port->INDR); // Port "data in" register
    if (dio_pin < 8) {
        // Which config port to address, and how, depending on the pin's index (each register handle pin [0..7] and [8..15] respectivly).
        _dioCFGR    = &(dio_port->CFGLR);
        _dioCfgMask = ~(0xFUL << (dio_pin * 4));
        _dioCfgOD   = (GPIO_CFGLR_OUT_10Mhz_OD << (dio_pin * 4));
        _dioCfgPUD  = (GPIO_CFGLR_IN_PUPD << (dio_pin * 4));
        _dioCfgFlt  = (GPIO_CFGLR_IN_FLOAT << (dio_pin * 4));
    } else {
        _dioCFGR    = &(dio_port->CFGHR);
        _dioCfgMask = ~(0xFUL << ((dio_pin - 8) * 4));
        _dioCfgOD   = (GPIO_CFGLR_OUT_10Mhz_OD << ((dio_pin - 8) * 4));
        _dioCfgPUD  = (GPIO_CFGLR_IN_PUPD << ((dio_pin - 8) * 4));
        _dioCfgFlt  = (GPIO_CFGLR_IN_FLOAT << ((dio_pin - 8) * 4));
    }
    gpioRccClockEnable(dio_port); // Enable DIO port clock
    ll_mode_input(); // Set DIO pin to high-impedance input for idle state

    if (pullup_port != nullptr) {
        // --- Configure Pull-up Power Pin, same tricks as DIO ---
        //_pullup_port = pullup_port;
        _pupBSR      = &(pullup_port->BSHR); // To set bits
        _pupBCR      = &(pullup_port->BCR); // To clear bits
        _pupPinIndex = pullup_pin;
        _pupCfgMask  = ~(0xFUL << (pullup_pin * 4));
        _pupPortCFGR = (pullup_pin < 8) ? &(pullup_port->CFGLR) : &(pullup_port->CFGHR);


        gpioRccClockEnable(pullup_port); // Enable pull-up port clock
        // Configure the pull-up pin as a push-pull output
        *_pupPortCFGR = (pullup_port->CFGLR & _pupCfgMask) | (GPIO_CFGLR_OUT_10Mhz_PP << (pullup_pin * 4));
        power(); // Default to bus being powered off
    } else {
        _pupBSR      = nullptr;
        _pupBCR      = nullptr;
        _pupPinIndex = 0;
        _pupCfgMask  = 0;
        _pupPortCFGR = nullptr;
    }


    ll_mode_input();
    resetSearch();

    power(true);
    Delay_Us(125);
    if (ll_read() == 0) {
        power(false);
        return OneWireError_e::NO_BUS_POWER;
    }
    power(false);
    return OneWireError_e::NO_ERROR;
}

void OneWire::power(bool enable)
{
    if (_pupBSR != nullptr) {
        if (enable) {
            *_pupBSR = 1UL << _pupPinIndex;
        } else {
            *_pupBCR = 1UL << _pupPinIndex;
        }
    }
}

// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
OneWireError_e OneWire::reset_bus(uint8_t retries)
{
    bool pulled_low = false;

    ll_mode_input();
    // wait until the wire is high... just in case
    do {
        if (--retries == 0)
            return OneWireError_e::BUS_HELD_LOW;
        Delay_Us(2);
    } while (!ll_read());

    ll_write_0();
    ll_mode_output(); // drive output low

    Delay_Us(OW_Timings_e::OWT_RSTL); // emit the reset pulse
    _overdriven = false; // whichever mode we were in, the bus has now been reset into standard speed.
    
    ll_mode_input(); // and make it high-z again.
    uint32_t timeoutEntryTime = SysTick->CNT;
    do {
        pulled_low = ll_read() == 0;
        if (pulled_low)
            break;
    } while ((SysTick->CNT - timeoutEntryTime) < Ticks_from_Us(OW_Timings_e::OWT_MSP)); // Wait for the presence pulse and then...
    if (!pulled_low)
        return OneWireError_e::NO_DEVICE_PRESENT;

    // Pulled_low is now asserted <true>.
    timeoutEntryTime = SysTick->CNT; // timeout the wait
    while (ll_read() == 0) { //
        if ((SysTick->CNT - timeoutEntryTime) > Ticks_from_Us(OW_Timings_e::OWT_PDL_TIMEOUT)) {
            return OneWireError_e::BUS_HELD_LOW; // device presence pulse too long, timeout triggered !!
        }
    }
    // We let OWT_RSTL elapse, counting from the time we detected the presence pulse.
    do {
        __NOP();
    } while ((SysTick->CNT - timeoutEntryTime) < Ticks_from_Us(OW_Timings_e::OWT_RSTL));



    return OneWireError_e::NO_ERROR;
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void OneWire::writeBit(uint8_t v)
{

    if (_overdriven) {
        if (v & 1) {
            ll_write_0();
            ll_mode_output(); // drive output low
            Delay_Us(OWT_W1L_OVR);
            ll_write_1(); // drive output high

            Delay_Us(OWT_W1H_OVR);
        } else {

            ll_write_0();
            ll_mode_output(); // drive output low
            Delay_Us(OWT_W0L_OVR);
            ll_write_1(); // drive output high
            Delay_Us(OWT_W0H_OVR);
        }
    } else {
        if (v & 1) {

            ll_write_0();
            ll_mode_output(); // drive output low
            Delay_Us(OWT_W1L);
            ll_write_1(); // drive output high

            Delay_Us(OWT_W1H);
        } else {

            ll_write_0();
            ll_mode_output(); // drive output low
            Delay_Us(OWT_W0L);
            ll_write_1(); // drive output high

            Delay_Us(OWT_W0H);
        }
    }
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
uint8_t OneWire::readBit(void)
{
    uint8_t r;
    if (_overdriven) {
        ll_mode_output();
        ll_write_0();
        Delay_Us(OWT_READ_RL_OVR); // the master marks the time slot
        ll_mode_input(); // let pin float, pull up will raise
        Delay_Us(OWT_READ_MSR_OVR); // the slave finally responds (or not)
        r = ll_read(); // and the master samples the bit
        Delay_Us(OWT_READ_PADDING_OVR); // the time slot finishes
    } else {
        ll_mode_output();
        ll_write_0();
        Delay_Us(OWT_READ_RL); // the master marks the time slot
        ll_mode_input(); // let pin float, pull up will raise
        Delay_Us(OWT_READ_MSR); // the slave finally responds (or not)
        r = ll_read(); // and the master samples the bit
        Delay_Us(OWT_READ_PADDING); // the time slot finishes
    }    
    return r;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
void OneWire::write(uint8_t v)
{
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
        writeBit((bitMask & v) ? 1 : 0);
    }
}

void OneWire::writeBytes(const uint8_t* buf, uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
        write(buf[i]);
}


//
// Read a byte
//
uint8_t OneWire::read()
{
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
        if (readBit())
            r |= bitMask;
    }
    return r;
}

uint16_t OneWire::read(uint8_t* buf, uint16_t count)
{
    for (int i = 0; i < count; i++)
        buf[i] = read();
    return count;
}



//
// Do a ROM select
//
void OneWire::select(const uint8_t rom[8], bool overdrive)
{
    uint8_t i;
    write(overdrive ? OW_MATCH_ROM_OVERDRIVE : OW_MATCH_ROM); // Choose ROM
    for (i = 0; i < 8; i++)
        write(rom[i]);
    _overdriven = overdrive;
}



//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void OneWire::resetSearch()
{
    // reset the search state
    _lastDiscrepancy       = 0;
    _lastDeviceFlag        = false;
    _lastFamilyDiscrepancy = 0;
    for (int i = 7;; i--) {
        _romId[i] = 0;
        if (i == 0)
            break;
    }
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
//
void OneWire::targetSearch(uint8_t family_code)
{
    // set the search state to find SearchFamily type devices
    _romId[0] = family_code;
    for (uint8_t i = 1; i < 8; i++)
        _romId[i] = 0;
    _lastDiscrepancy       = 64;
    _lastFamilyDiscrepancy = 0;
    _lastDeviceFlag        = false;
}

//
// Perform a search. If this function returns <true> then it has
// enumerated the next device and you may retrieve the ROM from the
// @p newAddr parameter.
// If there are no devices, no further devices, or something horrible happens in the middle of the
// enumeration then <false> is returned.
// If a new device is found then
// its address is copied to newAddr.  Use OneWire::Reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in _romId buffer
//        FALSE : device not found, end of search
//
bool OneWire::search(uint8_t* newAddr, bool alarms_only)
{
    uint8_t id_bit_number;
    uint8_t last_zero, rom_byte_number;
    bool search_result;
    uint8_t id_bit, cmp_id_bit;

    unsigned char rom_byte_mask, search_direction;

    // initialize for search
    id_bit_number   = 1;
    last_zero       = 0;
    rom_byte_number = 0;
    rom_byte_mask   = 1;
    search_result   = false;

    // if the last call was not the last one
    if (!_lastDeviceFlag) {
        // 1-Wire reset
        if (reset_bus() != NO_ERROR) {
            // reset the search
            _lastDiscrepancy       = 0;
            _lastDeviceFlag        = false;
            _lastFamilyDiscrepancy = 0;
            return false;
        }

        // issue the search command
        if (alarms_only == false) {
            write(OW_SEARCH_ROM); // NORMAL SEARCH
        } else {
            write(OW_SEARCH_COND); // CONDITIONAL SEARCH
        }

        // loop to do the search
        do {
            // read a bit and its complement
            id_bit     = readBit();
            cmp_id_bit = readBit();

            // check for no devices on 1-wire
            if ((id_bit == 1) && (cmp_id_bit == 1)) {
                break;
            } else {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit) {
                    search_direction = id_bit; // bit write value for search
                } else {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < _lastDiscrepancy) {
                        search_direction = ((_romId[rom_byte_number] & rom_byte_mask) > 0);
                    } else {
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == _lastDiscrepancy);
                    }
                    // if 0 was picked then record its position in LastZero
                    if (search_direction == 0) {
                        last_zero = id_bit_number;

                        // check for Last discrepancy in family
                        if (last_zero < 9)
                            _lastFamilyDiscrepancy = last_zero;
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction == 1)
                    _romId[rom_byte_number] |= rom_byte_mask;
                else
                    _romId[rom_byte_number] &= ~rom_byte_mask;

                // serial number search direction write bit
                writeBit(search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0) {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65)) {
            // search successful so set _lastDiscrepancy,LastDeviceFlag,search_result
            _lastDiscrepancy = last_zero;

            // check for last device
            if (_lastDiscrepancy == 0) {
                _lastDeviceFlag = true;
            }
            search_result = true;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !_romId[0]) {
        _lastDiscrepancy       = 0;
        _lastDeviceFlag        = false;
        _lastFamilyDiscrepancy = 0;
        search_result          = false;
    } else {
        for (int i = 0; i < 8; i++)
            newAddr[i] = _romId[i];
    }
    return search_result;
}



// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but a little smaller, than the lookup table.
//
uint8_t OneWire::crc8(const uint8_t* addr, uint8_t len)
{
    uint8_t crc = 0;

    while (len--) {
        uint8_t inbyte = *addr++;
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

bool OneWire::checkCrc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc)
{
    crc = ~crc16(input, len, crc);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

uint16_t OneWire::crc16(const uint8_t* input, uint16_t len, uint16_t crc)
{

    static const uint8_t oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0; i < len; i++) {
        // Even though we're just copying a byte from the input,
        // we'll be doing 16-bit computation with it.
        uint16_t cdata = input[i];
        cdata          = (cdata ^ crc) & 0xff;
        crc >>= 8;

        if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
            crc ^= 0xC001;

        cdata <<= 6;
        crc ^= cdata;
        cdata <<= 1;
        crc ^= cdata;
    }

    return crc;
}
