/****************************************************************************
** Copyright (C) 2020 MikroElektronika d.o.o.
** Contact: https://www.mikroe.com/contact
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to deal
** in the Software without restriction, including without limitation the rights
** to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
** copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
** IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
** DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
** OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
**  USE OR OTHER DEALINGS IN THE SOFTWARE.
****************************************************************************/



#ifndef H_AT21CS01_CLASS_H
#define H_AT21CS01_CLASS_H

#include <ch32fun.h>

enum SwiError_e : int32_t
{
    SUCCESS                        = 0,
    FAILED                         = -1,
    SIO_PORT_NULL                  = -2, // The pointer for `sio_read_port` was NULL.
    SIO_PORT_INVALID               = -3, // The pointer for `sio_read_port` is not valid on this CH32 device.
    SIO_PIN_INVALID                = -4, // The pin number for `sio_read_pin` isn't valid.
    PULLUP_PORT_INVALID            = -5, // The given port for writing to SIO pullup is not valid on this CH32 device.
    PULLUP_PIN_INVALID             = -6, // The given pin number for powering the pull-up is isn't valid.
    FAILED_RESET                   = -7, // Self-explanatory
    NO_DEVICE_PRESENT              = -8, // Bus scan didn't detect any device acknowledging its presence.
    REFUSED_EEPROM_ADDRESS_SETUP   = -9,
    FAILED_ACK_EEPROM_ADDRESS      = -10,
    REFUSED_EEPROM_READ_CMD        = -11,
    REFUSED_SECURITY_ADDRESS_SETUP = -12,
    FAILED_ACK_SECURITY_ADDRESS    = -13,
    REFUSED_SECURITY_READ_CMD      = -14,
    OFFSET_OUT_OF_BOUNDS           = -15,
    NULL_PARAMETER                 = -16,
    SINGLE_WRITE_FAILED            = -17,

};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// error codes for physical hardware dependent module

#define SWI_FUNCTION_RETCODE_SUCCESS ((uint8_t)0x00) //communication with device succeeded.
#define SWI_FUNCTION_RETCODE_TIMEOUT ((uint8_t)0xF1) //communication timed out.
#define SWI_FUNCTION_RETCODE_RX_FAIL ((uint8_t)0xF9) //communication failed after at least one byte was received.
#define SWI_ADDR_NAK_FAIL            ((uint8_t)0xF2) //NAK during address.
#define SWI_DATA_NAK_FAIL            ((uint8_t)0xF3) //communication failed after at least one byte was received.


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//define Reset and Discovery Timing

#define tDSCHG                       200000 //min spec = 150us
#define tRESET                       500000 //min spec = 480us (STD Speed)
#define tRRT                         10000 //min spec = 8us
#define tDRR                         1000 //min spec = 1us; max spec = 2us
#define tMSDR                        2000 //min spec = 2us; max spec = 6us
#define tHTSS                        200000 //min spec = 150us

#define tDACK_DLY                    Delay_ns(8000)
#define tRRT_DLY                     Delay_ns(tRRT)
#define tDRR_DLY                     Delay_ns(tDRR)
#define tMSDR_DLY                    Delay_ns(tMSDR)
#define tDSCHG_DLY                   Delay_ns(tDSCHG)
#define tDRESET_DLY                  Delay_ns(tRESET)
#define tHTSS_DLY                    Delay_ns(tHTSS)


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//define High-Speed Mode Communication Timing

#define tLOW0_MIN                    6000
#define tLOW0_MAX                    16000
#define tLOW1_MIN                    1000
#define tLOW1_MAX                    2000
#define tBIT_MIN                     8000 //tLOW0 + tPUP + tRCV
#define tBIT_MAX                     25000
#define tRCV_MIN                     2000

#define tLWO_TYPICAL                 (tLOW0_MIN + ((tLOW0_MAX - tLOW0_MIN) / 2)) //creates typical data timing (AVG of Min and Max)
#define tLW1_TYPICAL                 (tLOW1_MIN + ((tLOW1_MAX - tLOW1_MIN) / 2)) //creates typical data timing (AVG of Min and Max)
#define tBIT_TYPICAL                 (tBIT_MIN + ((tBIT_MAX - tBIT_MIN) / 2)) //creates typical data timing (AVG of Min and Max)

#define tLOW0_HDLY                   Delay_ns(11000) //min spec = 6us; max spec = 16us
#define tRD_HDLY                     Delay_ns(1200) //min spec = 1us; max spec = 2us
#define tLOW1_HDLY                   Delay_ns(1500) //min spec = 1us; max spec = 2us
#define tRCV0_HDLY                   Delay_ns(11000)
#define tRCV1_HDLY                   Delay_ns(14000)

#define tRD_DLY                      Delay_ns(1200) //min spec = 1us; max spec = 2us
#define tSWIN_DLY                    Delay_ns(1500) //delay to put master strobe within sample window

#define tLOW0_DLY                    Delay_ns(tLWO_TYPICAL)
#define tLOW1_DLY                    Delay_ns(tLW1_TYPICAL)

#define tBIT_DLY                     Delay_ns(tBIT_TYPICAL)
#define tRCV0_DLY                    Delay_ns(tBIT_TYPICAL - tLWO_TYPICAL)
#define tRCV1_DLY                    Delay_ns(tBIT_TYPICAL - tLW1_TYPICAL)




void Delay_ns(uint32_t n);

class AT21CS01 {

  public:
    struct SwiBusConfig_t {
        GPIO_TypeDef* sioPort;
        int8_t sioPin;
        GPIO_TypeDef* pupPort;
        int8_t pupPin;
    };

    SwiError_e init(GPIO_TypeDef* sio_port, int8_t sio_pin, GPIO_TypeDef* pullup_port = nullptr, int8_t pullup_pin = -1);
    inline SwiError_e init(const SwiBusConfig_t& config) { return init(config.sioPort, config.sioPin, config.pupPort, config.pupPin); }

    /** @brief Resets the devices on the bus. */
    SwiError_e reset();
    /** @brief  Scans the bus for a device to drop its ACK. */
    SwiError_e scan_devices();



    /**
     * @brief SWI EEPROM memory write function.
     * @details This function writes data to the specified memory address.
     * @param[in] start_addr : Byte address to write at [0-127].
     * @param[in] data_in : Value to write.     
    */
    SwiError_e writeByteAt(uint8_t start_addr, const uint8_t data_in);

    /**
     * @brief SWI EEPROM memory write page function.
     * @details This function writes data to the specified memory address page.
     * @param[in] start_addr : Byte address to write at [0-127].
     * @param[in] data_in : Data to be written on the page.
     * @param[in] len : Number bytes to write (up to 8 bytes).
     * @note None.
     */
    SwiError_e writePage(uint8_t start_addr, const uint8_t* data_in, uint8_t len);

    /**
     * @brief Writes a full buffer of bytes, without page size limitation
     * @param offset Address to start writing at.
     * @param buff Data to be written 
     * @param len Length of @p buff to be written.
     * @param[out] writtenCount Effective number of written bytes (optional)
     * @return 
     */
    SwiError_e writeBytes(uint8_t offset, const uint8_t* buff, uint8_t len, uint8_t* writtenCount = nullptr);

    /**
    * @brief Reads data from the specified memory address.  
    * @param[in] start_addr : Address byte [0-127].
    * @param[out] data_out : Read data.
    * @param[in] len : Number of data bytes to read.    
    */
    SwiError_e readBytes(uint8_t start_addr, uint8_t* data_out, uint8_t len);

    /**
     * @brief Clears the while memory chip to 0 (zero) values.
     * @param[out] failurePageIndex : Index of the page at which the failure (if any) happened.
     */
    SwiError_e eraseAll(int* failurePageIndex = nullptr);

    /**
     * @brief Gets the 64-bits UID of the device.
     * @param[out] uidOut Pointer to the value to read to.
     */
    SwiError_e readUID(uint8_t* uidOut);

    inline size_t memSize() { return 128; }


    void enableBus(uint32_t charge_delay_us = 500), disableBus(), enableChangeDetection(), disableChangeDetection();

    bool isPowered()
    {
        pin_weakPullDown();
        Delay_Us(150);
        bool result = (*_sioINDR & _sioPinMask) != 0;
        pin_release();
        return result;
    }

  private:
    //GPIO_TypeDef *_sio_port = nullptr, *_pullup_port = nullptr;
    volatile uint32_t *_sioCFGR = nullptr, *_pupPortCFGR = nullptr; // Pointers to the config registers (aka mode) of the pins, either CFGLR or CFGHR.
    volatile uint32_t* _sioINDR = nullptr; // SIO port input data register.
    volatile uint32_t *_sioBSR = nullptr, *_sioBCR = nullptr, *_pupBSR = nullptr,
                      *_pupBCR = nullptr; // Bit set and bit clear registers for both pins ports.
    uint32_t _sioCfgMask = UINT32_MAX, _pupCfgMask = UINT32_MAX; // Mask value to used clear the sio pin config.


    uint32_t _sioCfgOD   = 0; // Config value for an Open Drain SIO pin.
    uint32_t _sioCfgPUD  = 0; // Config value for an input with pull up/down.
    uint32_t _sioCfgFlt  = 0; // Config value for a Floatring SIO pin.
    uint16_t _sioPinMask = 0;
    uint8_t _pupPinIndex = 0, _slaveAddress = 0;
    /** @brief SWI EEPROM device ID setting, specific device ID of the MikroE "SWI EEPROM Click driver". */
    const uint32_t DEVICE_ID = 0xD200;

    /** @brief Settings for registers of SWI EEPROM Click driver. */
    enum SWIEEPROM_OPCD : uint8_t
    {
        OPCODE_EEPROM     = 0xA0,
        OPCODE_SECURITY   = 0xB0,
        OPCODE_LOCK       = 0x20,
        OPCODE_ROM        = 0x70,
        OPCODE_FREEZE_ROM = 0x10,
        OPCODE_ID         = 0xC0,
        OPCODE_STDN_SPEED = 0xD0,
        OPCODE_HIGH_SPEED = 0xE0,
    };



    inline void pin_init()
    {
        *_sioCFGR = (*_sioCFGR & _sioCfgMask) | _sioCfgFlt;
        enableBus();
    }

    inline void pin_low()
    {
        *_sioBCR  = _sioPinMask; //  Force output latch low.
        *_sioCFGR = (*_sioCFGR & _sioCfgMask) | _sioCfgOD; // make it a open drain output.
    }

    inline void pin_weakPullDown()
    {
        disableBus();
        *_sioCFGR = (*_sioCFGR & _sioCfgMask) | _sioCfgPUD;
    }

    inline void pin_release()
    {
        *_sioCFGR = (*_sioCFGR & _sioCfgMask) | _sioCfgFlt; // make it a floating input.
        enableBus();
    }

    inline uint8_t pin_get()
    {
        *_sioCFGR = (*_sioCFGR & _sioCfgMask) | _sioCfgFlt; // make it a floating input.
        return (*_sioINDR & _sioPinMask) != 0 ? 1 : 0;
    }



    //inline void start_stop(), restart(), logic_write_0(), logic_write_1(), receive_byte(uint8_t* byte_to_receive, uint8_t ack_nack);
    //inline uint8_t logic_read(), send_byte(uint8_t byte_to_send);
    //
    /**************************************************************************************************************************** */
    /*******************************************                               ************************************************** */
    /*******************************************           SWI_PHY             ************************************************** */
    /*******************************************                               ************************************************** */
    /**************************************************************************************************************************** */



    //define error codes
    static constexpr int NO_DEVICE_DETECTED                  = 0xFF;
    static constexpr int SWI_SUCCESS                         = (0);
    static constexpr int AT21CS01_SWI_SUCCESS                = (0);
    static constexpr int AT21CS01_SWI_GENERAL_ERROR          = (-1);
    static constexpr int AT21CS01_SWI_WRITE_NACK             = (-2);
    static constexpr int AT21CS01_SWI_READ_NACK              = (-3);
    static constexpr int AT21CS01_SWI_REG_LOCKED             = (-4);
    static constexpr int AT21CS01_SWI_INVALID_EEPROM_ADDRESS = (-5);
    static constexpr int AT21CS01_SWI_INVALID_SIZE           = (-6);
    static constexpr int AT21CS01_SWI_OUT_OF_BOUNDS          = (-7);

    //define op codes
    static constexpr uint8_t EEPROM_ADDRESS     = 0x0A;
    static constexpr uint8_t SECREGACCESS       = 0x0B;
    static constexpr uint8_t LOCKSECREG         = 0x02;
    static constexpr uint8_t ROMZONEREGACCESS   = 0x07;
    static constexpr uint8_t FREEZEROMZONESTATE = 0x01;
    static constexpr uint8_t MFGIDREAD          = 0x0C;
    static constexpr uint8_t STDSPEEDMODE       = 0x0D;
    static constexpr uint8_t HIGHSPEEDMODE      = 0x0E;

    //define word address for the various ROM zones
    static constexpr uint8_t OMZONEREG_MEMZONE0 = ((uint8_t)0x01);
    static constexpr uint8_t OMZONEREG_MEMZONE1 = ((uint8_t)0x02);
    static constexpr uint8_t OMZONEREG_MEMZONE2 = ((uint8_t)0x04);
    static constexpr uint8_t OMZONEREG_MEMZONE3 = ((uint8_t)0x08);

    uint8_t cmdWriteBuffer[64]; //defines array for write buffer
    uint16_t tlow0; //defines variable for SI/O low time for a logic '0' depending on communication speed


    //defines the structure for the packet

    typedef struct {
        uint8_t dev_addr; //SWI chip address to communicate with.
        uint8_t opcode; //op code
        uint8_t mem_addr; //SWI address/commands to issue to the other chip (node).
        uint8_t mem_addr_length; //as
        uint8_t* buffer; //where to find the data.
        uint16_t wlen; //how many bytes do we want to write.
        uint16_t rlen; //how many bytes do we want to read.
        uint8_t chk_ack_only_flag; //this flag tells the low level drive to check for ack only
    } __attribute__((packed)) swi_package_t; //ensures that swi_package_t aligns to one-byte boundaries

    //function protocol

    void swi_enable(void), swi_set_device_id(uint8_t id), swi_set_signal_pin(uint8_t end), sendAck(void), sendNack(void), swi_start_stop_cond(void);

    uint8_t swi_send_bytes(uint8_t count, uint8_t* buffer), swi_send_byte(uint8_t value), swi_receive_bytes(uint8_t count, uint8_t* buffer),
      swi_device_discovery(void), swi_read(const swi_package_t* pkg), swi_write(const swi_package_t* pkg),
      swi_write_stdspeed_cmd(const swi_package_t* packet);

    /**************************************************************************************************************************** */
    /*******************************************                               ************************************************** */
    /*******************************************           SWI_COMM            ************************************************** */
    /*******************************************                               ************************************************** */
    /**************************************************************************************************************************** */
    //uint8_t set_vcc_level(uint16_t vcc);
    uint8_t read_mfg_id(uint8_t dev_addr, uint8_t* buf);
    uint8_t read_security_register(uint8_t dev_addr, uint8_t parm1, uint8_t parm2, uint8_t* buf);
    uint8_t read_serial_number(uint8_t dev_addr, uint8_t* buf);
    uint8_t chk_high_speed_mode(uint8_t dev_addr, uint8_t* buf);
    uint8_t chk_std_speed_mode(uint8_t dev_addr, uint8_t* buf);
    uint8_t set_std_speed_mode(uint8_t dev_addr, uint8_t* buf);
    uint8_t set_high_speed_mode(uint8_t dev_addr, uint8_t* buf);
    uint8_t freeze_rom_zone_register(uint8_t dev_addr, uint8_t* buf);
    uint8_t reading_rom_zone_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf);
    uint8_t writing_rom_zone_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf);
    uint8_t freeze_rom_zone_state(uint8_t dev_addr, uint8_t* buf);
    uint8_t lock_security_register(uint8_t dev_addr, uint8_t* buf);
    uint8_t write_security_register(uint8_t dev_addr, uint8_t arg1, uint8_t arg2, uint8_t* buf);
    uint8_t check_lock_command(uint8_t dev_addr, uint8_t* buf);
    uint8_t read_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t rlen, uint8_t* buf);
    uint8_t write_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t wlen, const uint8_t* buf);
    uint8_t scan_swi_device_addr(void);
};

#endif // !H_AT21CS01_CLASS_H
