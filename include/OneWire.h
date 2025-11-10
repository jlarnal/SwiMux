/*

Single-File One Wire Communication Functions for CH32V003

Relies on the CH32V003fun library from: 
  https://github.com/cnlohr/ch32v003fun

This is very heavily derived from the Arduino  library, 
  at https://github.com/PaulStoffregen/


Original copyright notices follow:
--------------------------------------------

Copyright (c) 2007, Jim Studt  (original old version - many contributors since)

The latest version of this library may be found at:
  http://www.pjrc.com/teensy/td_libs_.html

 has been maintained by Paul Stoffregen (paul@pjrc.com) since
January 2010.

DO NOT EMAIL for technical support, especially not for ESP chips!
All project support questions must be posted on public forums
relevant to the board or chips used.  If using Arduino, post on
Arduino's forum.  If using ESP, post on the ESP community forums.
There is ABSOLUTELY NO TECH SUPPORT BY PRIVATE EMAIL!

Github's issue tracker for  should be used only to report
specific bugs.  DO NOT request project support via Github.  All
project and tech support questions must be posted on forums, not
github issues.  If you experience a problem and you are not
absolutely sure it's an issue with the library, ask on a forum
first.  Only use github to report issues after experts have
confirmed the issue is with  rather than your project.

Back in 2010,  was in need of many bug fixes, but had
been abandoned the original author (Jim Studt).  None of the known
contributors were interested in maintaining .  Paul typically
works on  every 6 to 12 months.  Patches usually wait that
long.  If anyone is interested in more actively maintaining ,
please contact Paul (this is pretty much the only reason to use
private email about ).

 is now very mature code.  No changes other than adding
definitions for newer hardware support are anticipated.

  ESP32 mods authored by stickbreaker:
  @stickbreaker 30APR2018 add IRAM_ATTR to read_bit() writeBit() to solve ICache miss timing failure. 
      thanks @everslick re:  https://github.com/espressif/arduino-esp32/issues/1335
  Altered by garyd9 for clean merge with Paul Stoffregen's source

Version 2.3:
  Unknown chip fallback mode, Roger Clark
  Teensy-LC compatibility, Paul Stoffregen
  search bug fix, Love Nystrom

Version 2.2:
  Teensy 3.0 compatibility, Paul Stoffregen, paul@pjrc.com
  Arduino Due compatibility, http://arduino.cc/forum/index.php?topic=141030
  Fix DS18B20 example negative temperature
  Fix DS18B20 example's low res modes, Ken Butcher
  Improve reset timing, Mark Tillotson
  Add const qualifiers, Bertrik Sikken
  Add initial value input to crc16, Bertrik Sikken
  Add target_search() function, Scott Roberts

Version 2.1:
  Arduino 1.0 compatibility, Paul Stoffregen
  Improve temperature example, Paul Stoffregen
  DS250x_PROM example, Guillermo Lovato
  PIC32 (chipKit) compatibility, Jason Dangel, dangel.jason AT gmail.com
  Improvements from Glenn Trewitt:
  - crc16() now works
  - check_crc16() does all of calculation/checking work.
  - Added read_bytes() and write_bytes(), to reduce tedious loops.
  - Added ds2408 example.
  Delete very old, out-of-date readme file (info is here)

Version 2.0: Modifications by Paul Stoffregen, January 2010:
http://www.pjrc.com/teensy/td_libs_.html
  search fix from Robin James
    http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27
  Use direct optimized I/O in all cases
  Disable interrupts during timing critical sections
    (this solves many random communication errors)
  Disable interrupts during read-modify-write I/O
  Reduce RAM consumption by eliminating unnecessary
    variables and trimming many to 8 bits
  Optimize both crc8 - table version moved to flash

Modified to work with larger numbers of devices - avoids loop.
Tested in Arduino 11 alpha with 12 sensors.
26 Sept 2008 -- Robin James
http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1238032295/27#27

Updated to work with arduino-0008 and to include skip() as of
2007/07/06. --RJL20

Modified to calculate the 8-bit CRC directly, avoiding the need for
the 256-byte lookup table to be loaded in RAM.  Tested in arduino-0010
-- Tom Pollard, Jan 23, 2008

Jim Studt's original library was modified by Josh Larios.

Tom Pollard, pollard@alum.mit.edu, contributed around May 20, 2008

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Much of the code was inspired by Derek Yerger's code, though I don't
think much of that remains.  In any event that was..
    (copyleft) 2006 by Derek Yerger - Free to distribute freely.

The CRC code was excerpted and inspired by the Dallas Semiconductor
sample code bearing this copyright.
//---------------------------------------------------------------------------
// Copyright (C) 2000 Dallas Semiconductor Corporation, All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL DALLAS SEMICONDUCTOR BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Dallas Semiconductor
// shall not be used except as stated in the Dallas Semiconductor
// Branding Policy.
//--------------------------------------------------------------------------
*/

#ifndef H_ONEWIRE_H
#define H_ONEWIRE_H

#include "ch32v003fun.h"
#include "ch32v003_GPIO_branchless.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>


enum OneWireError_e : uint
{
    NO_ERROR = 0,
    DIO_PORT_NULL,
    DIO_PORT_INVALID,
    DIO_PIN_INVALID,
    PULLUP_PORT_INVALID,
    PULLUP_PIN_INVALID,
    NULL_INPUT_BUFFER,
    NULL_OUTPUT_BUFFER,
    NO_BUS_POWER,
    BUS_HELD_LOW,
    NO_DEVICE_PRESENT,
    READ_ROM_FAILED,
    ALIGNED_WRITE_HEAD_PREREAD,
    ALIGNED_WRITE_TAIL_PREREAD,
    MEMADDRESS_OUT_OF_BOUNDS, // Specified address is out of memory bounds.
    OUT_OF_BOUNDS, // Specified length exceeds out of memory bounds.
    WRITE_MEM_FAILED,
    MULTIDROP_ID_UNREADABLE,
    WRITE_SCRATCHPAD_PRESELECT,
    WRITE_SCRATCHPAD_CRC16,
    READ_SCRATCHPAD_PRESELECT,
    READ_SCRATCHPAD_CRC16,
    SCRATCHPAD_PF, // Power loss or scratchpad not full
    WRITTEN_SCRATCHPAD_MISMATCH,
    COPY_SCRATCHPAD_PRESELECT,
    COPY_SCRATCHPAD, // Memory protected or scratchpad error (less likely)
};



enum OW_ROM_COMMANDS_e : uint8_t
{
    OW_READ_ROM            = 0x33,
    OW_MATCH_ROM           = 0x55,
    OW_SEARCH_COND         = 0xEC, // Search for devices in alarm mode (such as DS1921G `Thermochron ` iButtons )
    OW_SEARCH_ROM          = 0xF0,
    OW_SKIP_ROM            = 0xCC,
    OW_RESUME              = 0xA5,
    OW_SKIP_ROM_OVERDRIVE  = 0x3C,
    OW_MATCH_ROM_OVERDRIVE = 0x69,
};

enum OW_MEM_COMMANDS_e : uint8_t
{
    OW_WRITE_SCRATCHPAD = 0x0F,
    OW_READ_SCRATCHPAD  = 0xAA,
    OW_COPY_SCRATCHPAD  = 0x55,
    OW_READ_MEMORY      = 0xF0,
};



class OneWire {

  public:
    static constexpr uint8_t DEFAULT_RETRIES_COUNT = 16;

    /** @brief Holds the configuration for OneWire class's instances.
     * @param dioPort Reference to the GPIO port of the DIO pin.
     * @param dioPin Index of the DIO pin within @p dioPort .
     * @param[optional] powerPort Pointer to the port of the power pull-up pin, or null if none (DEFAULT).
     * @param[optional] powerPin Index of the power pull-up pin within @p powerPort , or -1 of none (DEFAULT).
     */
    struct OneWireConfig_t {
        GPIO_TypeDef* dioPort;
        uint8_t dioPin;
        GPIO_TypeDef* pullupPort;
        int8_t pullupPin;
    };

    OneWireError_e begin(GPIO_TypeDef* dio_port, const uint8_t dio_pin, GPIO_TypeDef* pullup_port, int8_t pullup_pin);
    inline OneWireError_e begin(const OneWireConfig_t& config) { return begin(config.dioPort, config.dioPin, config.pullupPort, config.pullupPin); }

    /** @brief Perform a 1-Wire reset cycle. Returns 1 if a device responds
    * with a presence pulse.  Returns 0 if there is no device or the
    * bus is shorted or otherwise held low for more than 250uS. */
    OneWireError_e reset_bus(uint8_t retries = DEFAULT_RETRIES_COUNT);

    /** @brief Issue a 1-Wire rom select command, you do the reset first. */
    void select(const uint8_t rom[8], bool overdrive = false);


    /** @brief Issues a 1-Wire rom skip command, to address all on bus. */
    void skip(bool overdrive = false)
    {
        write(overdrive ? OW_SKIP_ROM_OVERDRIVE : OW_SKIP_ROM);
        _overdriven = overdrive;
    }

    /** @brief Write a byte. 
     * @param value Value of the byte to write.
    * another read or write. Default is <true> */
    void write(uint8_t value);

    void writeBytes(const uint8_t* buf, uint16_t count);


    /** @brief Reads a byte. */
    uint8_t read(void);

    /** @brief Reads several bytes in a row from the device's current address.
     * @param buf Destination buffer
     * @param count Number of bytes to fetch 
     * @returns The effective number of bytes read, 0 (zero) in case of error.
     */
    uint16_t read(uint8_t* buf, uint16_t count);


    /** @brief Stop forcing power onto the bus. You only need to do this if
    * you used the 'power' flag to write() or used a write_bit() call
    * and aren't about to do another read or write. You would rather 
    * not leave this powered if you don't have to, just in case 
    * someone shorts your bus. */
    void power(bool enable = false);


    /** @brief Clears the search state so that if will start from the beginning again. */
    void resetSearch();

    /** @brief Sets up the search to find the device type 'family_code' on the next call
    * to search(*newAddr) if it is present. */
    void targetSearch(uint8_t family_code);

    /** @brief Looks for the next device. Returns 1 if a new address has been
    * returned.
    * @note A zero might mean that the bus is shorted, there are
    * no devices, or you have already retrieved all of them.  It
    * might be a good idea to check the CRC to make sure you didn't
    * get garbage.  The order is deterministic. You will always get
    * the same devices in the same order. 
    * @param[out] newAddr Buffer intended to store the newly-discovered ID.
    * @param[optional] alarms_only Search only for devices presenting a raised alarm flag.
    */
    bool search(uint8_t* newAddr, bool alarms_only = false);

    /** @brief Compute a Dallas Semiconductor 8 bit CRC
    *  @note These are used in the ROM and scratchpad registers. 
    */
    uint8_t crc8(const uint8_t* addr, uint8_t len);


    /** @brief Compute the 1-Wire CRC16 and compare it against the received CRC.
    * @example Reading a DS2408
    *    // Put everything in a buffer so we can compute the CRC easily.
    *    uint8_t buf[13];
    *    buf[0] = 0xF0;    // Read PIO Registers
    *    buf[1] = 0x88;    // LSB address
    *    buf[2] = 0x00;    // MSB address
    *    WriteBytes(net, buf, 3);    // Write 3 cmd bytes
    *    ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
    *    if (!CheckCRC16(buf, 11, &buf[11])) {
    *        // Handle error.
    *    }
    *
    * @param input - Array of bytes to checksum.
    * @param len - How many bytes to use.
    * @param inverted_crc - The two CRC16 bytes in the received data.
    *                       This should just point into the received data,
    *                       *not* at a 16-bit integer.
    * @param crc - The crc starting value (optional)
    * @return True, iff the CRC matches.
    */
    bool checkCrc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc);

    /** @brief Compute a Dallas Semiconductor 16 bit CRC.  This is required to check
    * the integrity of data received from many 1-Wire devices.  Note that the
    * CRC computed here is *not* what you'll get from the 1-Wire network,
    * for two reasons:
    *   1) The CRC is transmitted bitwise inverted.
    *   2) Depending on the endian-ness of your processor, the binary
    *      representation of the two-byte return value may have a different
    *      byte order than the two bytes you get from 1-Wire.
    * @param input - Array of bytes to checksum.
    * @param len - How many bytes to use.
    * @param crc - The crc starting value (optional)
    * @return The CRC16, as defined by Dallas Semiconductor.
    */
    uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc);

  private:
    // One-Wire timings, in microseconds (1e-6s)
    enum OW_Timings_e : uint32_t
    {

        OWT_MSP         = 70,
        OWT_RSTL        = 490, // minimum required is 480.
        OWT_PDL_TIMEOUT = 2060, // Note 16 of the datasheet says 2ms. Let's add 60µS more.
        OWT_SLOT        = 65,
        OWT_REC         = 5, // recovery time
        OWT_REC_OVR     = 3, // recovery time, overdrivee speed

        OWT_W1L          = 10,
        OWT_W1H          = OWT_SLOT - OWT_W1L,
        OWT_W0L          = 62,
        OWT_W0H          = OWT_SLOT - OWT_W0L,
        OWT_READ_RL      = 3, // Master-forced low time for a read slot
        OWT_READ_MSR     = 10, // Delay before master sampling read
        OWT_READ_PADDING = OWT_SLOT - (OWT_READ_RL + OWT_READ_MSR + OWT_REC / 2), // Time to wait before ending the bit read time slot
        // Overriden timings
        OWT_SLOT_OVR     = 9, // Bit time slot duration in overdrive speed.
        OWT_W1L_OVR      = 2, // Override mode '1' low duration
        OWT_W1H_OVR      = OWT_SLOT_OVR - OWT_W1L_OVR, // Override mode '1' high duration
        OWT_W0L_OVR      = 7, // Override mode '0' low duration
        OWT_W0H_OVR      = OWT_SLOT_OVR - OWT_W0L_OVR, // Ovveride mode '0' high
        OWT_READ_RL_OVR  = 1, // Master-forced low time for a read slot, overdrive speed
        OWT_READ_MSR_OVR = 1, // Delay before master sampling read, overdrive speed
        OWT_READ_PADDING_OVR
        = OWT_SLOT_OVR - (OWT_READ_RL_OVR + OWT_READ_MSR + OWT_REC_OVR / 2), // Time to wait before ending the bit read time slot in overdrive speed.

    };

    // global search state
    uint8_t _romId[8], _lastDiscrepancy, _lastFamilyDiscrepancy;
    volatile uint32_t _dioPinMask = 0, *_dioBSR = nullptr, *_dioBCR = nullptr, *_dioINDR = nullptr, *_dioCFGR = nullptr, _dioCfgOD = 0,
                      _dioCfgPUD = 0, _dioCfgFlt = 0, _dioCfgMask = 0;
    volatile uint32_t *_pupBSR = nullptr, *_pupBCR = nullptr, _pupPinIndex = 0, _pupCfgMask = 0, *_pupPortCFGR = nullptr;
    bool _lastDeviceFlag, _overdriven = false;



    // Write a bit. The bus is always left powered at the end, see
    // note in write() about that.
    void writeBit(uint8_t v);

    // reads a bit.
    uint8_t readBit(void);



    inline __attribute__((always_inline)) uint8_t ll_read() { return (*_dioINDR & _dioPinMask) == _dioPinMask; }

    inline __attribute__((always_inline)) void ll_mode_input()
    {
        *_dioCFGR = (*_dioCFGR & _dioCfgMask) | _dioCfgPUD;
        *_dioBCR  = _dioPinMask;
    }

    inline __attribute__((always_inline)) void ll_mode_output() { *_dioCFGR = (*_dioCFGR & _dioCfgMask) | _dioCfgOD; }

    inline __attribute__((always_inline)) void ll_write_0() { *_dioBCR = _dioPinMask; }

    inline __attribute__((always_inline)) void ll_write_1() { *_dioBSR = _dioPinMask; }
};

#endif // H_ONEWIRE_H
