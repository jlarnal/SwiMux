#include "AT21CS01.h"
#include <ch32fun.h>
#include <stdio.h>

#define GP        0x107 /* x^8 + x^2 + x + 1 */
#define DI        0x07
#define PAGE_SIZE 0x08

#ifndef CYCLES_PER_LOOP
#define CYCLES_PER_LOOP 3U
#endif

#include <stdint.h>

/* Assumed fixed core clock (48 MHz). No runtime * or / used on variables. */
#define CORE_MHZ        48U /* 48 MHz */
#define CORE_MULT       CORE_MHZ /* multiply factor in numerator (48) */
#define CORE_DIV        1000U /* denominator to convert ns->cycles: 48/1000 */

/* Estimated cycles burned per loop iteration (nop + addi + bnez). */
#define CYCLES_PER_LOOP 3U

/* Multiply two 32-bit unsigned using shift-and-add -> 64-bit result.
 * No '*' instruction emitted (pure shifts/adds/branches).
 */
static uint64_t mul_u32_u32_shiftadd(uint32_t a, uint32_t b)
{
    uint64_t res = 0;
    uint64_t aa  = a;
    while (b) {
        if (b & 1U)
            res += aa;
        aa <<= 1;
        b >>= 1;
    }
    return res;
}

/* 64-bit / 32-bit binary long division using shifts/subtracts only.
 * Returns floor(num / den). No '/' instruction emitted.
 */
static uint64_t div_u64_by_u32_long(uint64_t num, uint32_t den)
{
    if (den == 0)
        return 0; /* caller must not divide by zero */
    uint64_t q = 0;
    uint64_t r = 0;
    for (int i = 63; i >= 0; --i) {
        r = (r << 1) | ((num >> i) & 1ULL);
        if (r >= den) {
            r -= den;
            q |= (1ULL << i);
        }
    }
    return q;
}

/* Public API: delay for approximately 'nanoseconds' (no MUL/DIV opcodes).
 * Minimum granularity is one CPU cycle (~20.833 ns at 48 MHz).
 */
inline void Delay_ns(uint32_t nanoseconds)
{
    if (nanoseconds == 0)
        return;

    /* product = nanoseconds * CORE_MULT (here 48) computed without '*' */
    uint64_t prod = mul_u32_u32_shiftadd(nanoseconds, CORE_MULT);

    /* cycles = floor(prod / CORE_DIV)  -> cycles ≈ nanoseconds * 48 / 1000 */
    uint64_t cycles = div_u64_by_u32_long(prod, CORE_DIV);

    /* If requested ns is > 0 but computed cycles == 0, use at least 1 cycle. */
    if (cycles == 0)
        cycles = 1;

    /* iterations = ceil(cycles / CYCLES_PER_LOOP) using long division */
    uint64_t numerator  = cycles + (CYCLES_PER_LOOP - 1);
    uint64_t iterations = div_u64_by_u32_long(numerator, CYCLES_PER_LOOP);

    /* run in 32-bit chunks for inline asm */
    while (iterations) {
        uint32_t cnt = (iterations > 0x7FFFFFFFU) ? 0x7FFFFFFFU : (uint32_t)iterations;
        iterations -= cnt;

        /* Tight loop: about 3 cycles / iteration on typical core.
         *   nop           -> 1 cycle
         *   addi cnt,-1   -> 1 cycle
         *   bnez cnt,1b   -> 1 cycle (branch cost may vary)
         */
        asm volatile("1: nop\n"
                     "   addi %0, %0, -1\n"
                     "   bnez %0, 1b\n"
          : "+r"(cnt)
          :
          : "memory");
    }
}

static int indrToPortIndex(volatile uint32_t* indrPort)
{
    // returns ((<port> - <offset>) - <ports base>) / <port stride>
    // <offset> is the offset of INDR withing GPIO_Typedef (as in hardware)
    // <port base> is the base addres of the first port
    // <port stride> is the offset between two contiguous port, i.e 1024 bytes, i.e 0x400, i.e a 10 bits shift.
    return (((size_t)indrPort - offsetof(GPIO_TypeDef, GPIO_TypeDef::INDR)) - GPIOA_BASE) >> 10;
}

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

SwiError_e AT21CS01::init(GPIO_TypeDef* sio_port, int8_t sio_pin, GPIO_TypeDef* pullup_port, int8_t pullup_pin)
{
    // Check SIO port and pin
    if (sio_port == NULL)
        return SwiError_e::SIO_PORT_NULL;
    if (!isGpioPort(sio_port))
        return SwiError_e::SIO_PORT_INVALID;
    if (sio_pin < 0 || sio_pin > 15)
        return SwiError_e::SIO_PIN_INVALID;

    // Check Pull-up port and pin
    if (pullup_port != nullptr) {
        if (!isGpioPort(pullup_port))
            return SwiError_e::PULLUP_PORT_INVALID;
        if (pullup_pin < 0 || pullup_pin > 15)
            return SwiError_e::PULLUP_PIN_INVALID;
    }


    // --- Configure SIO Pin ---
    //_sio_port   = sio_port;
    _sioPinMask = 1 << sio_pin;
    _sioBSR     = &(sio_port->BSHR); // Bis set register (we won't use its 'reset' MSW)
    _sioBCR     = &(sio_port->BCR); // Bit clear regiser
    _sioINDR    = &(sio_port->INDR); // Port "data in" register
    if (sio_pin
      < 8) { // Which config port to address, and how, depending on the pin's index (each register handle pin [0..7] and [8..15] respectivly).
        _sioCFGR    = &(sio_port->CFGLR);
        _sioCfgMask = ~(0xF << (sio_pin * 4));
        _sioCfgOD   = (GPIO_CFGLR_OUT_10Mhz_OD << (sio_pin * 4));
        _sioCfgPUD  = (GPIO_CFGLR_IN_PUPD << (sio_pin * 4));
        _sioCfgFlt  = (GPIO_CFGLR_IN_FLOAT << (sio_pin * 4));
    } else {
        _sioCFGR    = &(sio_port->CFGHR);
        _sioCfgMask = ~(0xF << ((sio_pin - 8) * 4));
        _sioCfgOD   = (GPIO_CFGLR_OUT_10Mhz_OD << ((sio_pin - 8) * 4));
        _sioCfgPUD  = (GPIO_CFGLR_IN_PUPD << ((sio_pin - 8) * 4));
        _sioCfgFlt  = (GPIO_CFGLR_IN_FLOAT << ((sio_pin - 8) * 4));
    }
    gpioRccClockEnable(sio_port); // Enable SIO port clock
    pin_release(); // Set SIO pin to high-impedance input for idle state

    if (pullup_port != nullptr) {
        // --- Configure Pull-up Power Pin, same tricks as SIO ---
        //_pullup_port = pullup_port;
        _pupBSR      = &(pullup_port->BSHR); // To set bits
        _pupBCR      = &(pullup_port->BCR); // To clear bits
        _pupPinIndex = pullup_pin;
        _pupCfgMask  = ~(0xF << (pullup_pin * 4));
        _pupPortCFGR = (pullup_pin < 8) ? &(pullup_port->CFGLR) : &(pullup_port->CFGHR);


        gpioRccClockEnable(pullup_port); // Enable pull-up port clock
        // Configure the pull-up pin as a push-pull output
        *_pupPortCFGR = (pullup_port->CFGLR & _pupCfgMask) | (GPIO_CFGLR_OUT_10Mhz_PP << (pullup_pin * 4));
        enableBus(); // Default to bus being powered off
    }

    if (reset() != SwiError_e::SUCCESS) {
        return SwiError_e::FAILED_RESET;
    }

    _slaveAddress = 0;

    if (scan_devices() != SwiError_e::SUCCESS)
        return SwiError_e::NO_DEVICE_PRESENT;

    Delay_Ms(100);


    return SwiError_e::SUCCESS;
}

void AT21CS01::enableBus(uint32_t charge_delay_us)
{
    if (_pupPortCFGR) {
        *_pupPortCFGR = (*_pupPortCFGR & _pupCfgMask) | (GPIO_CFGLR_OUT_2Mhz_PP << (_pupPinIndex << 2)); // Make it a slow P-P output.
        *_pupBSR      = 1U << _pupPinIndex; // Set high indefinitely
        if (charge_delay_us) {
            // Let the SWI slaves parasitic-power-supplies charge their capacitors for a time. 500µs isn't canon, but it will do.
            Delay_Us(charge_delay_us);
        }
    }
}

void AT21CS01::disableBus()
{
    *_pupBCR      = 1U << _pupPinIndex; // Set it low (drains the bus for a couple of clock cycles)
    *_pupPortCFGR = (*_pupPortCFGR & _pupCfgMask) | (GPIO_CFGLR_IN_FLOAT << (_pupPinIndex << 2)); // Make it an input
}

void AT21CS01::enableChangeDetection()
{
    *_sioCFGR = (*_sioCFGR & _sioCfgMask) | _sioCfgFlt; // make it a floating input.
    EXTI->EVENR |= _sioPinMask;
    EXTI->FTENR |= _sioPinMask;
    EXTI->RTENR |= _sioPinMask;
    AFIO->EXTICR |= indrToPortIndex(_sioINDR) * _sioPinMask; // links the appropriate port to EXTIn
}

void AT21CS01::disableChangeDetection()
{
    // Just undo what enableChangeDetection() did by clearing the `EXTI` configs.
    EXTI->EVENR &= ~_sioPinMask;
    EXTI->FTENR &= ~_sioPinMask;
    EXTI->RTENR &= ~_sioPinMask;
    // resetting `AFIO->EXTICR` wouldn't achieve anything.
}


SwiError_e AT21CS01::reset()
{
    // Pin initialization
    pin_init();

    // Reset
    pin_low();
    // tDSCHG delay: 150+us
    Delay_Us(160);
    pin_release();
    // tRRT delay: 8+us
    Delay_Us(10);

    // Discovery
    pin_low();
    // tDRR delay: 1-2us
    Delay_Us(1);
    pin_release();
    // tDACK delay: 8-24us
    Delay_Us(9);

    if (pin_get()) {
        return SwiError_e::FAILED_RESET;
    }
    return SwiError_e::SUCCESS;
}


/*
void AT21CS01::start_stop()
{
    pin_release();
    Delay_Us(160); // tHTSS delay: 150+us
}

void AT21CS01::restart()
{
    pin_low();
    Delay_Tiny(1);
    pin_release();
    Delay_Us(160);
}

void AT21CS01::logic_write_0()
{
    pin_low();
    // tLOW0 delay: 6-16us
    Delay_Us(10);
    pin_release();
    // tBIT - tLOW0 delay: 8-24us - 6-16us
    Delay_Us(6);
}

void AT21CS01::logic_write_1()
{
    pin_low();
    // tLOW1 delay: 1-2us
    Delay_Us(1);
    pin_release();
    // tBIT - tLOW1 delay: 8-24us - 1-2us
    Delay_Us(15);
}

inline uint8_t AT21CS01::logic_read()
{
    pin_low();
    // tRD delay: 1-2us
    Delay_Us(1);
    pin_release();
    // tMRS delay: 1-2us
    Delay_Us(1);
    uint8_t pin_state = pin_get();
    // tBIT - tRD - tMRS delay: 8-24us - 1-2us - 1-2us
    Delay_Us(14);
    return pin_state;
}
*/



SwiError_e AT21CS01::writeByteAt(uint8_t start_addr, const uint8_t data_in)
{
    if (start_addr >= memSize()) {
        return SwiError_e::OFFSET_OUT_OF_BOUNDS;
    }

    return (write_memory(_slaveAddress, start_addr, 1, const_cast<const uint8_t*>(&data_in)) == SWI_SUCCESS) ? SUCCESS : SINGLE_WRITE_FAILED;
}



SwiError_e AT21CS01::writePage(uint8_t start_addr, const uint8_t* data_in, uint8_t len)
{
    if ((NULL == data_in) || (len > 8) || ((start_addr + len) > memSize())) {
        return SwiError_e::FAILED;
    }
   

    return (write_memory(_slaveAddress, start_addr, len, data_in) == SWI_SUCCESS) ? SwiError_e::SUCCESS : SwiError_e::FAILED;
}


SwiError_e AT21CS01::writeBytes(uint8_t offset, const uint8_t* buff, uint8_t len, uint8_t* writtenCount)
{
    if (buff == nullptr)
        return SwiError_e::NULL_PARAMETER;
    if (offset >= memSize())
        return SwiError_e::OFFSET_OUT_OF_BOUNDS;
    if (len == 0) {
        if (writtenCount != nullptr)
            *writtenCount = 0;
        return SUCCESS;
    }

    if (scan_devices() != SUCCESS)
        return FAILED;

    if ((offset + len) >= memSize()) // Cap length of
        len = memSize() - offset;
    uint8_t written = 0;
    while (len) {
        if (len >= 8) { // write a full page of 8.
            if (writePage(offset, buff, 8))
                break;
            written += 8;
            len -= 8;
            offset += 8;
            buff = &buff[8];
        } else { // last run
            if (writePage(offset, buff, len))
                break;
            written += len;
            len = 0;
        }
    }
    if (writtenCount != nullptr) {
        *writtenCount = written;
    }
    if (len != 0) {
        return FAILED;
    }
    return SUCCESS;
}


SwiError_e AT21CS01::readBytes(uint8_t start_addr, uint8_t* data_out, uint8_t len)
{
    if ((data_out == nullptr) || ((start_addr + len) > memSize())) {
        return SwiError_e::FAILED;
    }

    return (read_memory(_slaveAddress, start_addr, len, data_out) == SWI_SUCCESS) ? SwiError_e::SUCCESS : SwiError_e::FAILED;
}


SwiError_e AT21CS01::readUID(uint8_t* uidOut)
{
    if (uidOut == nullptr)
        return FAILED;

    return (read_serial_number(_slaveAddress, uidOut) == SWI_SUCCESS) ? SwiError_e::SUCCESS : SwiError_e::FAILED;
}

SwiError_e AT21CS01::eraseAll(int* failurePageIndex)
{
    const uint8_t CLEAR_PAGE_ARRAY[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    if (failurePageIndex != nullptr)
        *failurePageIndex = -1;
    for (uint8_t cnt = 0; cnt < memSize(); cnt += 8) {
        if (writePage(cnt, CLEAR_PAGE_ARRAY, 8)) {
            if (failurePageIndex != nullptr)
                *failurePageIndex = cnt;
            return FAILED;
        }
    }
    return SUCCESS;
}


SwiError_e AT21CS01::scan_devices()
{
    _slaveAddress = scan_swi_device_addr();
    return _slaveAddress == NO_DEVICE_DETECTED ? FAILED : SUCCESS;
}

/*************************************************************************************************************************/
/***********************************************                        **************************************************/
/***********************************************       SWI_PHY          **************************************************/
/***********************************************                        **************************************************/
/*************************************************************************************************************************/


#define low  0x00 //defines "low"
#define high 0x01 //defines "high"
#define ACK  0x00 //defines "ACK" or acknowledge

#define initDeviceWriteHi()                                                                                                                          \
    {                                                                                                                                                \
        pin_low();                                                                                                                                   \
        Delay_Us(1);                                                                                                                                 \
        pin_release();                                                                                                                               \
    } //defines high-speed mode master read bit frame where delay is tRD
#define initDeviceWriteStd()                                                                                                                         \
    {                                                                                                                                                \
        pin_low();                                                                                                                                   \
        Delay_Us(6);                                                                                                                                 \
        pin_release();                                                                                                                               \
    } //defines standard speed mode master read bit frame where delay is tRD


//write operation
uint8_t AT21CS01::swi_write(const swi_package_t* packet)
{
    uint8_t index     = 0; //variable used to determine position within the writeBuffer array
    uint8_t writeSize = 1; //set initial value to writeSize variable

    cmdWriteBuffer[index++] = (((packet->opcode << 4) | packet->dev_addr) & ~0x01); //load device address w/ opcode and set R/W = 0
    if (packet->mem_addr_length) //get memory address
    {
        cmdWriteBuffer[index++] = packet->mem_addr; //memory address
        writeSize++; //increment the writeSize variable by 1
    }

    memcpy(&cmdWriteBuffer[index], packet->buffer, packet->wlen); //load data to be written
    writeSize += packet->wlen; //evaluate writeSize + wlen variable

    swi_start_stop_cond(); //start condition --> GO TO swi_start_stop_cond()
    swi_send_bytes(writeSize, &cmdWriteBuffer[0]); //transmit data bytes to the EEPROM --> GO TO swi_send_bytes()
    swi_start_stop_cond(); //stop condition --> GO TO swi_start_stop_cond()
    return SWI_SUCCESS; //return success code
}



//read operation
uint8_t AT21CS01::swi_read(const swi_package_t* packet)
{
    uint8_t index     = 0; //variable used to determine position within the writeBuffer array
    uint8_t writeSize = 1; //set initial value to writeSize variable

    cmdWriteBuffer[index++] = ((packet->opcode << 4) | packet->dev_addr); //load device address w/ opcode
    if (packet->mem_addr_length) //get memory word address
    {
        cmdWriteBuffer[index++] = packet->mem_addr; //set write buffer to memory word address
        writeSize++; //increment the writeSize variable by 1
    }

    swi_start_stop_cond(); //send start condition --> GO TO swi_start_stop_cond()

    if (packet->mem_addr_length) //perform dummy write
    {
        cmdWriteBuffer[0] &= 0xFE; //bitwiseAND cmdWriteBuffer array with 0xFEh
        swi_send_bytes(writeSize, &cmdWriteBuffer[0]); //send device address byte with R/W bit = 1 --> GO TO swi_send_bytes()
        swi_start_stop_cond(); //start condition --> GO TO swi_start_stop_cond()
    }

    cmdWriteBuffer[0] |= 0x01; //cmdWriteBuffer ORed with 0x01 or set the R/W bit = 1 in the device address byte
    swi_send_bytes((writeSize - packet->mem_addr_length), &cmdWriteBuffer[0]); //send device address byte --> GO TO swi_send_bytes()

    if (!packet->chk_ack_only_flag) //if the EEPROM ACKs the device address byte
    {
        swi_receive_bytes(packet->rlen, packet->buffer); //perform read operation --> GO TO swi_receive_bytes()
    }

    swi_start_stop_cond(); //stop condition --> GO TO swi_start_stop_cond()
    return SWI_SUCCESS; //return success code
}


//transmit data bytes
uint8_t AT21CS01::swi_send_bytes(uint8_t count, uint8_t* buffer)
{
    uint8_t bit_mask; //declares variable for bit mask for data to be transmitted
    uint8_t retCode; //declares variable for the returned code if a NACK is received

    for (uint8_t ii = 0; ii < count; ii++) //for loop for number of byte to be written
    {
        for (bit_mask = 0x80; bit_mask >= 1; bit_mask >>= 1) //for loop for bit mask
        {
            //if the next bit transmitted is a logic '1'
            if (bit_mask & buffer[ii]) //send a logic '1'
            {
                pin_low(); //drive SI/O pin low
                tLOW1_DLY; //SI/O low time, logic '1' delay
                pin_release(); //release SI/O pin and set as input
                tRCV1_DLY; //slave recovery time delay for logic '1'
            }
            //if the next bit transmitted is a logic '0'
            else //send a logic '0'
            {
                pin_low(); //drive SI/O pin low
                tLOW0_DLY; //SI/O low time, logic '0' delay
                pin_release(); //release SI/O pin and set as input
                tRCV0_DLY; //slave recovery time delay for logic'0'
            }
        }
        //this checks for an ACK or NACK from EEPROM
        pin_low(); //drive SI/O pin low
        tRD_DLY; //SI/O low time during read delay
        pin_release(); //release SI/O pin and set as input
        tLOW1_DLY; //master read strobe time (same as SI/O low time, logic '1') delay
        //check for ACK/NACK
        if (pin_get()) //if a NAK is detected
        {
            pin_release(); //release SI/O pin and set as input


            if (ii == 0) {
                retCode = SWI_ADDR_NAK_FAIL;
            } //EEPROM failed to send ACK during device address byte
            else {
                retCode = SWI_DATA_NAK_FAIL;
            } //EEPROM failed to send ACK during word address byte
            return retCode; //return which byte the EEPROM did not ACK
        }

        tRCV0_DLY; //slave recovery time delay (same for logic'0' and logic '1')
    }

    pin_release(); //release SI/O pin and set as input

    return SWI_FUNCTION_RETCODE_SUCCESS; //return success code
}


//receive bytes
uint8_t AT21CS01::swi_receive_bytes(uint8_t count, uint8_t* buffer)
{
    uint8_t bit_mask; //declares variable for bit mask for data to be transmitted
    memset(&buffer[0], 0, count); //clear buffer before reading
    pin_release();

    //data phase,... Receive bits and store in buffer.
    for (uint8_t ii = 0; ii < count; ii++) //for loop for number of byte to be received
    {
        for (bit_mask = 0x80; bit_mask >= 1; bit_mask >>= 1) //for loop for bit mask
        {
#ifdef STANDARD_SPEED //device is set for standard speed communication
            initDeviceWriteStd(); //standard speed mode master read bit frame where delay is tRD
#else //device is set for high-speed communication
            initDeviceWriteHi(); //high-speed mode master read bit frame where delay is tRD
#endif

            tSWIN_DLY; //delay to put master read inside the master sampling window

            //check for SI/O state
            if (pin_get()) {
                buffer[ii] |= bit_mask;
            } //if a logic '1' is detected; received "one" bit
            tBIT_DLY; //bit frame duration (tBIT) before reading the next bit
        }
        if (ii < (count - 1)) {
            sendAck();
        } //send ACK except for last byte of read --> GO TO sendAck()
    }
    sendNack(); //send NACK to EEPROM signaling read is complete
    return SWI_FUNCTION_RETCODE_SUCCESS; //return success code
}


//set device to operate in standard speed mode
uint8_t AT21CS01::swi_write_stdspeed_cmd(const swi_package_t* packet)
{

    uint8_t index     = 0; //variable used to determine position within the writeBuffer array
    uint8_t writeSize = 1; //set initial value to writeSize variable
    uint8_t bit_mask; //declares variable for bit mask for data to be transmitted
    uint8_t retCode; //declares variable for the returned code if a NACK is received


    cmdWriteBuffer[index++] = (((packet->opcode << 4) | packet->dev_addr) & ~0x01); //load device address w/ opcode and set R/W = 0
    if (packet->mem_addr_length) //get memory address
    {
        cmdWriteBuffer[index++] = packet->mem_addr; //memory address
        writeSize++; //increment the writeSize variable by 1
    }

    memcpy(&cmdWriteBuffer[index], packet->buffer, packet->wlen); //load data to be written
    writeSize += packet->wlen; //evaluate writeSize + wlen variable

    swi_start_stop_cond(); //start condition --> GO TO swi_start_stop_cond()


    for (uint8_t ii = 0; ii < writeSize; ii++) //for loop for number of byte to be written
    {
        for (bit_mask = 0x80; bit_mask >= 1; bit_mask >>= 1) //for loop for bit mask
        {
            //if the next bit transmitted is a logic '1'
            if (bit_mask & cmdWriteBuffer[ii]) //send a logic '1'
            {
                pin_low(); //drive SI/O pin low
                tLOW1_HDLY; //SI/O low time, logic '1' delay
                pin_release(); //release SI/O pin and set as input
                tRCV1_HDLY; //slave recovery time delay (same for logic'0' and logic '1')
            }
            //if the next bit transmitted is a logic '0'
            else //send a logic '0'
            {
                pin_low(); //drive SI/O pin low
                tLOW0_HDLY; //SI/O low time, logic '0' delay
                pin_release(); //release SI/O pin and set as input
                tRCV0_HDLY; //slave recovery time delay (same for logic'0' and logic '1')
            }
        }
        //this checks for an ACK or NACK from EEPROM
        pin_low(); //drive SI/O pin low
        tRD_HDLY; //SI/O low time during read delay
        pin_release(); //release SI/O pin and set as input
        tLOW1_HDLY; //master read strobe time (same as SI/O low time, logic '1') delay
        //check for ACK/NACK
        if (pin_get()) //if a NAK is detected
        {
            pin_release(); //release SI/O pin and set as input

            if (ii == 0) {
                retCode = SWI_ADDR_NAK_FAIL;
            } //EEPROM failed to send ACK during device address byte
            else {
                retCode = SWI_DATA_NAK_FAIL;
            } //EEPROM failed to send ACK during word address byte
            return retCode; //return which byte the EEPROM did not ACK
        }

        tRCV0_HDLY; //slave recovery time delay (same for logic'0' and logic '1')
    }

    pin_release(); //release SI/O pin and set as input
    return SWI_FUNCTION_RETCODE_SUCCESS; //return success code
}


//send ACK to EEPROM
void AT21CS01::sendAck()
{
    pin_low(); //drive SI/O pin low
    tLOW0_HDLY; //SI/O low time, logic '0' delay
    pin_release(); //release SI/O pin and set as input
    tRCV0_DLY; //slave recovery time delay for logic'0'
}


//send NACK to EEPROM
void AT21CS01::sendNack()
{
    pin_low(); //drive SI/O pin low
    tLOW1_DLY; //SI/O low time, logic '1' delay
    pin_release(); //release SI/O pin and set as input
    tRCV1_DLY; //slave recovery time delay for logic'1'
}


//perform device discovery response
uint8_t AT21CS01::swi_device_discovery()
{
    pin_low(); //drive SI/O pin low
    tDSCHG_DLY; //discharge low time delay
    pin_release(); //release SI/O pin and set as input;
    tRRT_DLY; //reset recovery time delay
    pin_low(); //drive SI/O pin low
    tDRR_DLY; //discovery response request time delay
    pin_release(); //release SI/O pin and set as input
    tDACK_DLY; //master strobe
    return (pin_get()); //return value of SI/O
}

//send start and stop condition
void AT21CS01::swi_start_stop_cond()
{
    pin_release(); //release SI/O pin and set as input;
    tHTSS_DLY; //SI/O high time before start/stop conditions
}



/*************************************************************************************************************************/
/***********************************************                        **************************************************/
/***********************************************        SWI_COMM        **************************************************/
/***********************************************                        **************************************************/
/*************************************************************************************************************************/


uint8_t AT21CS01::write_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t wlen, const uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = EEPROM_ADDRESS; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = mem_addr; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = wlen; //how many bytes to write
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = const_cast<uint8_t*>(buf); //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    //Perform write operation and check whether device ACKs data bytes
    if (swi_write(&packet) != SWI_SUCCESS) {
        return SWI_DATA_NAK_FAIL;
    } //return device didn't ACK data byte code
    Delay_Ms(5); //wait tWC for write cycle to complete
    return SWI_SUCCESS; //return device ACK'd  all data byte(s) code
}



//read the memory array

uint8_t AT21CS01::read_memory(uint8_t dev_addr, uint8_t mem_addr, uint8_t rlen, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = EEPROM_ADDRESS; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = mem_addr; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = rlen; //how many bytes to read
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    for (uint8_t ii = 0; ii < rlen; ii++) //for loop for the number of bytes to be read
    {
        if (swi_read(&packet) != SWI_SUCCESS) {
            return SWI_ADDR_NAK_FAIL;
        } //perform a read access & check result; return fail code if failure occurs
    }
    return SWI_SUCCESS; //return success code
}



//read the mfg id

uint8_t AT21CS01::read_mfg_id(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = MFGIDREAD; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x00; //word address for operation (not used for operation IE dummy value)


    packet.mem_addr_length = 0x00; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 3; //how many bytes to read
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    for (uint8_t ii = 0; ii < 3; ii++) //for loop for the number of bytes to be read
    {
        if (swi_read(&packet) != SWI_SUCCESS) {
            return SWI_ADDR_NAK_FAIL;
        } //perform a read access & check result; return fail code if failure occurs
    }
    return SWI_SUCCESS; //return success code
}



//Write to the security register

uint8_t AT21CS01::write_security_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t wlen, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = SECREGACCESS; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = mem_addr; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = wlen; //how many bytes to write
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    //Perform write operation and check whether device ACKs data bytes
    if (swi_write(&packet) != SWI_SUCCESS) {
        return SWI_DATA_NAK_FAIL;
    } //return device didn't ACK data byte code
    Delay_Ms(5); //wait tWC for write cycle to complete
    return SWI_SUCCESS; //return device ACK'd  all data byte(s) code
}



//read the security register

uint8_t AT21CS01::read_security_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t rlen, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = SECREGACCESS; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = mem_addr; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = rlen; //how many bytes to read
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only;

    for (uint8_t ii = 0; ii < rlen; ii++) //for loop for the number of bytes to be read
    {
        if (swi_read(&packet) != SWI_SUCCESS) {
            return SWI_ADDR_NAK_FAIL;
        } //perform a read access & check result; return fail code if failure occurs
    }
    return SWI_SUCCESS; //return success code
}



//read the serial number in the security register

uint8_t AT21CS01::read_serial_number(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = SECREGACCESS; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x00; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 8; //how many bytes to read
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only;

    for (uint8_t ii = 0; ii < 8; ii++) //for loop for the number of bytes to be read
    {
        if (swi_read(&packet) != SWI_SUCCESS) {
            return SWI_ADDR_NAK_FAIL;
        } //perform a read access & check result; return fail code if failure occurs
    }
    return SWI_SUCCESS; //return success code
}



//Lock the security register

uint8_t AT21CS01::lock_security_register(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = LOCKSECREG; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x60; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 1; //how many bytes to write
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    //Perform write operation and check whether device ACKs data bytes
    if (swi_write(&packet) != SWI_SUCCESS) {
        return SWI_DATA_NAK_FAIL;
    } //return device didn't ACK data byte code
    Delay_Ms(5); //wait tWC for write cycle to complete
    return SWI_SUCCESS; //return device ACK'd  all data byte(s) code
}



//check lock status of the security register

uint8_t AT21CS01::check_lock_command(uint8_t dev_addr, uint8_t* buf)
{
    uint8_t retCode = 0xFF;

    swi_package_t packet; //build packet for the operation
    packet.opcode   = LOCKSECREG; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x60; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    //perform a check lock command and return whether the device ACKs
    retCode = swi_write(&packet);
    return retCode;
}



//set write protection of a zone register

uint8_t AT21CS01::writing_rom_zone_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = ROMZONEREGACCESS; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = mem_addr; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 1; //how many bytes to write
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    //Perform write operation and check whether device ACKs data bytes
    if (swi_write(&packet) != SWI_SUCCESS) {
        return SWI_DATA_NAK_FAIL;
    } //return device didn't ACK data byte code
    Delay_Ms(5); //wait tWC for write cycle to complete
    return SWI_SUCCESS; //return device ACK'd  all data byte(s) code;
}



//determine the lock state of a zone register

uint8_t AT21CS01::reading_rom_zone_register(uint8_t dev_addr, uint8_t mem_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = ROMZONEREGACCESS; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = mem_addr; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 1; //how many bytes to read
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    for (uint8_t ii = 0; ii < 1; ii++) //for loop for the number of bytes to be read
    {
        if (swi_read(&packet) != SWI_SUCCESS) {
            return SWI_ADDR_NAK_FAIL;
        } //perform a read access & check result; return fail code if failure occurs
    }
    return SWI_SUCCESS; //return success code
}



//freeze the write protection of a zone register

uint8_t AT21CS01::freeze_rom_zone_state(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = FREEZEROMZONESTATE; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x55; //word address for operation


    packet.mem_addr_length = 0x01; //not sure this is necessary


    packet.wlen              = 1; //how many bytes to write
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 0; //this flag tells the physical level to check for ACK only

    //Perform write operation and check whether device ACKs data bytes
    if (swi_write(&packet) != SWI_SUCCESS) {
        return SWI_DATA_NAK_FAIL;
    } //return device didn't ACK data byte code
    Delay_Ms(5); //wait tWC for write cycle to complete
    return SWI_SUCCESS; //return device ACK'd  all data byte(s) code
}



//check whether the device is set for standard speed communication

uint8_t AT21CS01::chk_std_speed_mode(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = STDSPEEDMODE; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x00; //word address for operation (not used for operation IE dummy value)


    packet.mem_addr_length = 0x00; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 1; //this flag tells the physical level to check for ACK only

    if (swi_read(&packet) != SWI_SUCCESS) {
        return SWI_ADDR_NAK_FAIL;
    } //perform a read access & check result; return fail code if failure occurs
    return SWI_SUCCESS; //return success code
}



//check whether the device is set for high-speed communication

uint8_t AT21CS01::chk_high_speed_mode(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = HIGHSPEEDMODE; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x00; //word address for operation (not used for operation IE dummy value)


    packet.mem_addr_length = 0x00; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 1; //this flag tells the physical level to check for ACK only

    if (swi_read(&packet) != SWI_SUCCESS) {
        return SWI_ADDR_NAK_FAIL;
    } //perform a read access & check result; return fail code if failure occurs
    return SWI_SUCCESS; //return success code
}



//set the device for standard speed communication

uint8_t AT21CS01::set_std_speed_mode(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = STDSPEEDMODE; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x00; //word address for operation (not used for operation IE dummy value)


    packet.mem_addr_length = 0x00; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 1; //this flag tells the physical level to check for ACK only

    //Perform write operation and check whether device ACKs data bytes
    if (swi_write(&packet) != SWI_SUCCESS) {
        return SWI_DATA_NAK_FAIL;
    } //return device didn't ACK data byte code
    return SWI_SUCCESS; //return device ACK'd  all data byte(s) code
}



//set the device for high-speed communication

uint8_t AT21CS01::set_high_speed_mode(uint8_t dev_addr, uint8_t* buf)
{
    swi_package_t packet; //build packet for the operation
    packet.opcode   = HIGHSPEEDMODE; //opcode for operation
    packet.dev_addr = dev_addr; //slave address to communicate with
    packet.mem_addr = 0x00; //word address for operation (not used for operation IE dummy value)


    packet.mem_addr_length = 0x00; //not sure this is necessary


    packet.wlen              = 0; //how many bytes to write (0 for read)
    packet.rlen              = 0; //how many bytes to read (0 for write)
    packet.buffer            = buf; //where to put the read data
    packet.chk_ack_only_flag = 1; //this flag tells the physical level to check for ACK only

    //Perform write operation and check whether device ACKs data bytes
    if (swi_write(&packet) != SWI_SUCCESS) {
        return SWI_DATA_NAK_FAIL;
    } //return device didn't ACK data byte code
    return SWI_SUCCESS; //return device ACK'd  all data byte(s) code
}



//scan to determine slave device address

uint8_t AT21CS01::scan_swi_device_addr(void)
{
    uint8_t discoveredAddr = NO_DEVICE_DETECTED; //declares variable for the discovered slave address initially equal to error
    uint8_t errCode; //declares variable for the error code returned
    uint8_t addr; //declares variable for the slave address


    for (uint8_t ii = 0; ii < 8; ii++) //creates for loop for the eight slave address combos to be sent
    {
        if (swi_device_discovery() == 0) //perform device discovery and if the device returns an ACK to the discovery
        {
            swi_start_stop_cond(); //start condition --> GO TO swi_start_stop_cond()
            addr = ii; //placing device address on the bus
            addr <<= 1; //shift slave address to proper bit position (left one bit)
            addr |= 0xA0; //add EEPROM op code and set R/W = 0
            errCode = swi_send_bytes(0x01, &addr); //send one byte @ address of addr variable

            if (errCode == 0x00) //If the device ACKs
            {
                discoveredAddr = ii; //set discovered slave address equal to the loop count
                discoveredAddr <<= 1; //shift discovered slave address to proper bit position (left one bit)
                swi_start_stop_cond(); //stop condition --> GO TO swi_start_stop_cond()
                break; //break from loop
            }
        }
    }
    return discoveredAddr; //return discovered slave address to the main code loop
}



//set the VCC level on the SI/O pin

//Note: Vpup is controlled by a DAC

//2^12 = 4096 (12-bit resolution
//5.50V/4086 = //~1.12mV resolution
//Vcc = Vref = 5.50V
//vout = ((Vref*Dn)/2^12)*G -> = (5.50*Dn)/4096)*1

//Example:
//Dn = 744.73*Vout
//Desired vout = 3.33V,
//Dn = 2480

//uint8_t AT21CS01::set_vcc_level(uint16_t vcc)

//{

//	Spi_initialize();																//initialize SPI interface

//

//	gpio_set_pin_dir_output(ADCCS_DDR,ADCCS);										//configure ADCCS pin as output

//	gpio_set_pin_dir_output(LDAC_DDR,LDAC);											//configure LDAC pin as output

//	gpio_set_pin_dir_output(CSDAC_DDR,CSDAC);										//configure CSDAC pin as output

//

//	gpio_set_pin_high(LDAC_PORT,LDAC);												//initialize LDAC pin as output as HIGH

//	gpio_set_pin_high(CSDAC_PORT,CSDAC);											//initialize CSDAC pin as output as HIGH

//	gpio_set_pin_high(ADCCS_PORT,ADCCS);											//initialize ADCCS pin as output as HIGH

//

//	static uint8_t vcc_level[2];													//create an array of two characters

//	vcc_level[1] = vcc & 0xFF;														//break the 16-bit Vcc level into two bytes (byte1) and bitmask with 0xFFh

//	vcc_level[0] = (vcc>>8) & 0xFF;													//break the 16-bit Vcc level into two bytes (byte0), shift value and bitmask with 0xFFh

//

//	gpio_set_pin_low(CSDAC_PORT,CSDAC);												//assert the CS pin on the MCP4921

//	Spi_send_and_receive (vcc_level[0]);											//send the corresponding Vcc level byte0 to the MPC4921 input register

//	Spi_send_and_receive (vcc_level[1]);											//send the corresponding Vcc level byte1 to the MPC4921 input register

//	gpio_set_pin_high(CSDAC_PORT,CSDAC);											//de-assert the CS pin on the MCP4921

//

//	gpio_set_pin_low(LDAC_PORT,LDAC);												//set LDAC pin on MCP4921 LOW to trans DAC from input to output register

//	_delay_us(4);																	//delay for tLD (LDAC pulse width)

//	gpio_set_pin_high(LDAC_PORT,LDAC);												//set LDAC pin on MCP4921 HIGH

//

//	return 0x00;																	//return a dummy value to the main loop

//}
