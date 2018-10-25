#include "FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <math.h>
#include "MCP39F511N.h"
#include "driver/uart.h"

#include "espos_time.h"
#define millis() espos_ticks_to_ms(espos_get_tick_count())

#define highByte(x) ((x >> 8) & 0xFF)
#define lowByte(x) (x & 0xFF)

#define BUF_SIZE (1024) // UART buffer size


uint8_t _msg_buffer[MCP_BUFFER_LEN];
uint8_t _buf_count;
uint8_t _rx_status;

uint16_t ssr;     // system status register
uint16_t volts;   // volts  * 10^1

uint32_t amps1;   // amps   * 10^3?
uint32_t vars1;   // vars   * 10^3?
uint32_t watts1;  // watts  * 10^3?
uint32_t amps2;
uint32_t vars2;
uint32_t watts2;

uint16_t frequency; // frequency * 10^3
uint16_t pf1;
uint16_t pf2;

uint64_t importEnergy1;
uint64_t importEnergy2;

uint16_t gain_amps1;
uint16_t gain_amps2;
uint16_t gain_volts;
uint16_t gain_watts1;
uint16_t gain_watts2;
uint16_t gain_vars1;
uint16_t gain_vars2;
uint16_t gain_frequency;

mcp_range1_t range1;
mcp_range2_t range2;

uint8_t range_volts;
uint8_t range_amps1;
uint8_t range_power1;
uint8_t range_amps2;
uint8_t range_power2;

uint16_t divisor1;
uint16_t divisor2;

// output register precision (i.e. how many decimals...)
uint8_t _precisionVolts = 1;
uint8_t _precisionAmps1 = 3;
uint8_t _precisionPower1 = 3;
uint8_t _precisionAmps2 = 3;
uint8_t _precisionPower2 = 3;

uint16_t _caliVolts = 1100; // 110.0 V
uint16_t _caliFreq = 60000; // 60.000 Hz  Resolutin: 1mHz

uint32_t _caliAmps1 = 1000; // 1.000 A
uint32_t _caliWatt1 = 15000; // 15 W
uint32_t _caliVAR1 = 1000;

uint32_t _caliAmps2 = 1000; // 1.000A
uint32_t _caliWatt2 = 15000;
uint32_t _caliVAR2 = 1000;

// Add an extra byte to the end of the buffer
void _buf_append(uint8_t b);
// start a new outgoing message
void _start_tx_frame();
// complete outgoing message (byte count, checksum)
void _complete_tx_frame();

/**
 * @brief      Calculate the checksum of data in the array
 *
 * @param      data  Pointer to the array
 * @param[in]  len   The length of the array
 *
 * @return     checksum of array
 */
uint8_t _checksum(uint8_t const * data, uint8_t len);

// set address pointer to read or write
void _setAddressPointer(uint16_t addr);

/**
 * @brief      Write data to the specific register(s)
 * 
 *              Data Frame:
 *              | Header Byte |
 *              | Number of bytes in the frame |
 *              | Command (Set Address Pointer) |
 *              | Address High |
 *              | Address Low |
 *              | Command (Register Write) |
 *              | Number of bytes to write |
 *              | Data |
 *              | Checksum |
 *              
 * @param[in]  addr  The address of the pointed register
 * @param      data  The data of the message
 * @param[in]  len   The length of data
 *
 * @return     Data write success or not
 */
uint8_t _registerWrite(uint16_t addr, uint8_t const *data, uint8_t len);

/**
 * @brief      Read data from mcp39f511n register
 * 
 *              Data Frame:
 *              | Header Byte |
 *              | Number of bytes in the frame |
 *              | Command (Set Address Pointer) |
 *              | Address High |
 *              | Address Low |
 *              | Command (Register Read) |
 *              | Number of bytes to Read |
 *              | Checksum |
 *
 * @param[in]  numBytes  The number bytes of data to read
 */
void _registerRead(uint8_t numBytes);

/**
 * @brief      Makes a copy of all the calibration and configuration registers to flash
 *
 * @return     success or not
 */
uint8_t _saveToFlash();

// receive a byte into the buffer
uint8_t _rx_byte(uint8_t b);
uint8_t _set_rx_status(uint8_t status);

uint8_t _processBuffer(uint8_t cmd);
void _clearBuffer();
uint8_t _receiveResponse(uint8_t cmd);
uint8_t _readBytes(uint16_t addr, uint8_t len);

uint16_t read_int(uint8_t addr);
uint32_t read_long(uint8_t addr);
uint64_t read_long_long(uint8_t addr);

void mcp_init(void)
{
    // Configure parameters of an UART driver,
    // communication pins and install the driver
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(MCP_UART_NUM, &uart_config);
    uart_driver_install(MCP_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL);

    _rx_status = 0;
    _buf_count = 0;

    readConfig();
/*
    range1.volt = 17;
    _start_tx_frame();
    _setAddressPointer(MCP_REG_RANGE1);
    _buf_append(MCP_CMD_REGISTER_WRITE);
    _buf_append(0x04);
    _buf_append(range1.volt);
    _buf_append(range1.amp);
    _buf_append(range1.power);
    _buf_append(0x00);
    _complete_tx_frame();
    _clearBuffer();

    gain_volts = 30250;
    _start_tx_frame();
    _setAddressPointer(MCP_REG_GAIN_VOLTS);
    _buf_append(MCP_CMD_REGISTER_WRITE);
    _buf_append(0x02);
    _buf_append(lowByte(gain_volts));
    _buf_append(highByte(gain_volts));
    _complete_tx_frame();
    _clearBuffer();*/

    // if(_saveToFlash() == MCP_STATUS_RX_COMPLETE)
    //   DEBUG_PRINT(("\n_saveToFlash done!!\n"));
}

void autoCalibrateGain(enum MCPChannel ch)
{

}

void autoCalibrateFreq()
{

}


// Return the Nth byte from a long (0 indexed)
uint8_t extractByte(uint32_t data, uint8_t n)
{
    uint32_t msk = 0xFF << ((n)*8);
    uint32_t tmp = data & msk;
    tmp = tmp >> ((n)*8);
    return tmp;
}

uint16_t read_int(uint8_t addr)
{
    uint16_t offset = 2;
    return ((uint16_t)(_msg_buffer[addr + offset + 1])<<8 | (uint16_t)(_msg_buffer[addr + offset + 0]));
}

uint32_t read_long(uint8_t addr)
{
    uint32_t tmp = 0;
    uint32_t ret = 0;
    uint16_t offset = 2;
    tmp += _msg_buffer[addr+3 + offset];
    tmp = tmp << 24;
    ret += tmp;
    tmp = _msg_buffer[addr+2 + offset];
    tmp = tmp << 16;
    ret += tmp;
    tmp = _msg_buffer[addr+1 + offset];
    tmp = tmp << 8;
    ret += tmp;
    ret += _msg_buffer[addr+0 + offset];
    return ret;
}

uint64_t read_long_long(uint8_t addr)
{
    uint16_t offset = 2;
    uint64_t tmp = 0;

    for(int i=0;i<8;i++)
        tmp +=  (uint64_t)(_msg_buffer[addr+i + offset]) << (8*i);
    return tmp;
}

void _clearBuffer()
{
    DEBUG_PRINT(("_clearBuffer()\n"));
    _rx_status = 0;
    _buf_count = 0;
}

// Add an extra byte to the end of the buffer
void _buf_append(uint8_t b)
{
    // prevent overflow
    if(_buf_count >= MCP_BUFFER_LEN) return;
    // increase buffer length counter and add this byte to the end
    _buf_count++;
    _msg_buffer[_buf_count-1] = b;
}

void _start_tx_frame()
{
    DEBUG_PRINT(("_start_tx_frame()\n"));
    // clear the buffer
    for(uint8_t i=0; i<MCP_BUFFER_LEN; i++) _msg_buffer[i] = 0;
    // set the first character
    _buf_count = 0;
    _buf_append(MCP_HEADER_BYTE);
    // set the counter to *2* bytes, we need to reserve byte 2 for msg len
    _buf_count = 2;
}

// Append the checksum and send the frame
void _complete_tx_frame()
{
    DEBUG_PRINT(("_complete_tx_frame()\n"));
    // Second byte is the message len
    _msg_buffer[1] = _buf_count +1;
    // Calculate and include our checksum
    _buf_append(_checksum(_msg_buffer, _buf_count));

    DEBUG_PRINT(("data frame: \n"));
    for(int i=0;i<_buf_count;i++)
                DEBUG_PRINT(("0x%x, ", _msg_buffer[i]));
    DEBUG_PRINT(("\n"));

    // write data to the device
    int len = uart_write_bytes(MCP_UART_NUM, (const char * )&_msg_buffer, _buf_count);
    DEBUG_PRINT(("transfer data frame %d.\n", len));
}

uint8_t _processBuffer(uint8_t cmd)
{
    DEBUG_PRINT(("_processBuffer()\n"));
    switch (_msg_buffer[0])
    {
        case MCP_ACK:
            switch (cmd)
            {
                case MCP_CMD_REGISTER_READ:
                case MCP_CMD_READ_EEPROM:
                // check the message length
                if(_buf_count < 3)
                {
                    DEBUG_PRINT(("MCP_STATUS_ERR_MSG_TOO_SHORT\n"));
                    return _set_rx_status(MCP_STATUS_ERR_MSG_TOO_SHORT);
                }          
                // check the checksum (excluding the received checksum)
                uint8_t my_checksum = _checksum(_msg_buffer, _buf_count - 1);
                if(_msg_buffer[_buf_count-1] != my_checksum) 
                {
                    DEBUG_PRINT(("my_checksum: 0x%02x\n", my_checksum));
                    DEBUG_PRINT(("MCP_STATUS_ERR_CSFAIL_RX\n"));
                    return _set_rx_status(MCP_STATUS_ERR_CSFAIL_RX);
                }
            }
        // if we made it this far, msg received
        DEBUG_PRINT(("message received !\n"));
        return _set_rx_status(MCP_STATUS_RX_COMPLETE);
        case MCP_NACK:
            if(_msg_buffer[1] != MCP_ID_BYTE)
                DEBUG_PRINT(("The sensor is not MCP39F511N, but others\n"));
            DEBUG_PRINT(("MCP_NACK\n"));
            return _set_rx_status(MCP_NACK);
        case MCP_CSFAIL:
            DEBUG_PRINT(("MCP_STATUS_ERR_CSFAIL_TX\n"));
            return _set_rx_status(MCP_STATUS_ERR_CSFAIL_TX);
    }
    // if we made it this far, msg received
    DEBUG_PRINT(("message received !\n"));
    return MCP_STATUS_RX_COMPLETE;
}

// receive a byte into the buffer
uint8_t _rx_byte(uint8_t b)
{
    if(_buf_count >= MCP_BUFFER_LEN){return _processBuffer(MCP_CMD_REGISTER_READ);}
    _buf_append(b);
    return 0;
}

// Calculate the checksum of data in the array, up to len bytes of data
uint8_t _checksum(uint8_t const * data, uint8_t len)
{
    uint32_t cs = 0;
    uint32_t tmp = 0;
    DEBUG_PRINT(("_chechsum, len: %d\n",len));
    //DEBUG_PRINT("len is "+String(len));
    for(uint8_t i=0; i< len; i++)
    {
        tmp = data[i];
        cs = cs + tmp;
    }
    return (cs % 256) & 0xFF;
}

uint8_t _receiveResponse(uint8_t cmd)
{
    //DEBUG_PRINT("_receiveResponse...");
    uint32_t t_timeout = millis() + MCP_TIMEOUT_MS;
    uint8_t result = _processBuffer(cmd);
    bool accellerated = false;

    while(millis() < t_timeout && result != MCP_STATUS_RX_COMPLETE)
    {
        _buf_count = uart_read_bytes(MCP_UART_NUM, (uint8_t *)_msg_buffer, MCP_BUFFER_LEN, 20 / portTICK_RATE_MS);

        DEBUG_PRINT(("data received: \n"));
        for(int i=0;i<_buf_count;i++)
            DEBUG_PRINT(("0x%x, ", _msg_buffer[i]));
        DEBUG_PRINT("\n");

        DEBUG_PRINT(("_buf_count %d.\n", _buf_count));
        if(_buf_count > MCP_BUFFER_LEN)
        {
            DEBUG_PRINT(("buffer overflow"));
            return _set_rx_status(MCP_STATUS_ERR_MSG_BUFFER_OVERFLOW);
        }
        else
        {
            // message may have finished. give it 20ms to wrap up.
            if(!accellerated && _buf_count)
            {
                t_timeout = millis() + 20;
                accellerated = true;
            }
        }
        result = _processBuffer(cmd);
    }
    // check if we're here because of a timeout
    if(millis() >= t_timeout && !_buf_count)
    {
        DEBUG_PRINT(("timeout"));
        return _set_rx_status(MCP_STATUS_RX_TIMEOUT);
    }
    // All clear
    DEBUG_PRINT(("receive complete.\n"));
    //DEBUG_PRINT("receive complete");
    return _set_rx_status(0);
}

// Read 'len' bytes from the device, starting at address 'addr'
// These will be read in to the msg_buffer
uint8_t _readBytes(uint16_t addr, uint8_t len)
{
    _setAddressPointer(addr);
    _registerRead(len);
    _complete_tx_frame();
    _clearBuffer();
    // fill our RX buffer or timeout
    return _receiveResponse(MCP_CMD_REGISTER_READ);
}

uint8_t _saveToFlash()
{
    DEBUG_PRINT(("_saveToFlash()\n"));
    _start_tx_frame();
    _buf_append(MCP_CMD_SAVE_TO_FLASH);
    _complete_tx_frame();
    return _receiveResponse(MCP_CMD_SAVE_TO_FLASH);
}

void _setAddressPointer(uint16_t addr)
{
    _buf_append(MCP_CMD_SET_ADDRESS_POINTER);
    _buf_append(highByte(addr));
    _buf_append(lowByte(addr));
}

uint8_t _registerWrite(uint16_t addr, uint8_t const *data, uint8_t len)
{
    _start_tx_frame();
    _setAddressPointer(addr);
    _buf_append(MCP_CMD_REGISTER_WRITE);
    if(len < (MCP_BUFFER_LEN - 7))
    {
        _buf_append(len);
        for(uint8_t i=0;i<len;i++)
            _buf_append(data[i]);
    }
    _complete_tx_frame();
    return _receiveResponse(MCP_CMD_REGISTER_WRITE);
}

void _registerRead(uint8_t numBytes)
{
    _buf_append(MCP_CMD_REGISTER_READ);
    _buf_append(numBytes);
}

uint8_t _set_rx_status(uint8_t status)
{
    _rx_status = status;
    return _rx_status;
}

void readConfig()
{
    uint8_t result;

    _start_tx_frame();
    result = _readBytes(MCP_REG_RANGE1, 4);
    if(result) return;
    uint32_t buf;
    buf = read_long(0);

    range_volts = extractByte(buf,0);
    range_amps1 = extractByte(buf,1);
    range_power1 = extractByte(buf,2);

    range1.volt = range_volts;
    range1.amp = range_amps1;
    range1.power = range_power1;

    _start_tx_frame();
    result = _readBytes(MCP_REG_RANGE2, 4);
    if(result) return;
    buf = read_long(0);

    range_amps2 = extractByte(buf, 1);
    range_power2 = extractByte(buf, 2);

    _start_tx_frame();
    result = _readBytes(MCP_REG_DIVISOR_DIGITS1, 4);
    if(result) return;
    divisor1 = read_int(0);
    divisor2 = read_int(2);

    DEBUG_PRINT(("Range:\n"));
    DEBUG_PRINT(("V: %d\n", range1.volt));
    DEBUG_PRINT(("I1: %d\n", range1.amp));
    DEBUG_PRINT(("S1: %d\n", range1.power));
    DEBUG_PRINT(("I2: %d\n", range_amps2));
    DEBUG_PRINT(("S2: %d\n", range_power2));

    // Read gain registers
    _start_tx_frame();

    result = _readBytes(MCP_REG_GAIN_AMPS1, 16);
    // gain_amps1     (2b) 0-1
    // gain_amps2     (2b) 2-3
    // gain_volts     (2b) 4-5
    // gain_watts1    (2b) 6-7
    // gain_watts2    (2b) 8-9
    // gain_vars1     (2b) 10-11
    // gain_vars2     (2b) 12-13
    gain_amps1 = read_int(0);
    gain_amps2 = read_int(2);
    gain_volts  = read_int(4);
    gain_watts1 = read_int(6);
    gain_watts2 = read_int(8);
    gain_vars1 = read_int(10);
    gain_vars2  = read_int(12);
    gain_frequency = read_int(14);

    DEBUG_PRINT(("Gain:\n"));
    DEBUG_PRINT(("V: %d\n", gain_volts));
}

void readPower()
{
    // annoyingly, we need to read the sign of active/reactive power
    // from the system status Register
    _start_tx_frame();
    uint8_t result;
    result = _readBytes(MCP_REG_SYSTEM_STATUS, 4);
    if(result) return;

    ssr = read_int(0);

    _start_tx_frame();
    result = _readBytes(MCP_REG_VOLTS, 32);
    // volts  (2b) 0-1
    // freq   (2b) 2-3
    // pf1    (2b) 4-5
    // pf2    (2b) 6-7
    // amps1  (4b) 8-11
    // amps2  (4b) 12-15
    // watts1 (4b) 16-19
    // watts2 (4b) 20-23
    // vars1  (4b) 24-27
    // vars2  (4b) 28-31

    // if something went wrong, there will be an error code
    if(result) return;
    // no error code, process the received data
    volts = read_int(0);
    frequency = read_int(2);
    pf1 = read_int(4);
    pf2 = read_int(6);
    amps1 = read_long(8);
    amps2 = read_long(12);
    watts1 = read_long(16);
    watts2 = read_long(20);
    vars1 = read_long(24);
    vars2 = read_long(28);

    _start_tx_frame();
    result = _readBytes(MCP_REG_IMPORT_WH1, 32);
    // Import active energy1    (8b) 0-7
    // Import active energy2    (8b) 8-15
    // Export active energy1    (8b) 16-23
    // Export active energy2    (8b) 24-31

    // if something went wrong, there will be an error code
    if(result) return;

    importEnergy1 = read_long_long(0);
    importEnergy2 = read_long_long(8);
    
}

uint8_t setGain(uint16_t reg, uint16_t gain)
{
    uint8_t data[] = {lowByte(gain), highByte(gain)};
    switch (reg)
    {
        case MCP_REG_GAIN_AMPS1:
            gain_amps1 = gain;
            break;
        case MCP_REG_GAIN_AMPS2:
            gain_amps2 = gain;
            break;
        case MCP_REG_GAIN_VOLTS:
            gain_volts = gain;
            break;
        case MCP_REG_GAIN_WATTS1:
            gain_watts1 = gain;
            break;
        case MCP_REG_GAIN_WATTS2:
            gain_watts2 = gain;
            break;
        case MCP_REG_GAIN_VARS1:
            gain_vars1 = gain;
            break;
        case MCP_REG_GAIN_VARS2:
            gain_vars2 = gain;
            break;
        case MCP_REG_GAIN_FREQUENCY:
            gain_frequency = gain;
            break;        
        default:
            DEBUG_PRINT(("Error Gain register\n"));
            return 1;
            break;
    }
    return _registerWrite(reg, &data, 2);

}

void setPrecisionVolts(uint8_t decimals){ _precisionVolts = decimals; }
void setPrecisionAmps1(uint8_t decimals){ _precisionAmps1 = decimals; }
void setPrecisionPower1(uint8_t decimals){ _precisionPower1 = decimals; }
void setPrecisionAmps2(uint8_t decimals){ _precisionAmps2 = decimals; }
void setPrecisionPower2(uint8_t decimals){ _precisionPower2 = decimals; }

float getFrequency(){ return ((float)frequency) / MCP_FREQUENCY_DIVISOR; }
float getVolts()    { return ((float)volts)   / ((float)pow(10, _precisionVolts)) ; }
float getAmps1()    { return ((float)amps1)   / ((float)pow(10, _precisionAmps1)) * ((ssr && MCP_SSR_SIGN_PA_CH1) ? 1 : -1); }
float getWatts1()   { return ((float)watts1)  / ((float)pow(10, _precisionPower1)) * ((ssr && MCP_SSR_SIGN_PA_CH1) ? 1 : -1); }
float getVars1()    { return ((float)vars1)   / ((float)pow(10, _precisionPower1)) * ((ssr && MCP_SSR_SIGN_PR_CH1) ? 1 : -1); }
float getAmps2()    { return ((float)amps2)   / ((float)pow(10, _precisionAmps2)) * ((ssr && MCP_SSR_SIGN_PA_CH2) ? 1 : -1); }     // 4
float getWatts2()   { return ((float)watts2)  / ((float)pow(10, _precisionPower2)) * ((ssr && MCP_SSR_SIGN_PA_CH2) ? 1 : -1); }    // 2
float getVars2()    { return ((float)vars2)   / ((float)pow(10, _precisionPower2)) * ((ssr && MCP_SSR_SIGN_PR_CH2) ? 1 : -1); }    // 2
uint64_t getImportEnergy1() {return importEnergy1;}
uint64_t getImportEnergy2() {return importEnergy2;}
