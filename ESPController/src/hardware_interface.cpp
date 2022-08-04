#include "defines.h"
#include "hardware_interface.h"

// Some tasks require bytes to be written, then read. The original code 
// base had 10ms delays peppered in to accommodate. Currently just cargo
// culting this over to maintain same behaviour.
#define HWI_DELAY_TO_SETTLE vTaskDelay(pdMS_TO_TICKS(10));


/********************************************************************
 * Subcommands are additional commands that are accessed indirectly using
 * the 7-bit command address space and provide the capability for block data 
 * transfers. When a subcommand is initiated, a 16-bit subcommand address is 
 * first written to the 7-bit command addresses 0x3E (lower byte) and 0x3F 
 * (upper byte). The device initially assumes a read-back of data may be needed, 
 * and auto-populates existing data into the 32-byte transfer buffer (which 
 * uses 7-bit command addresses 0x40–0x5F), and writes the checksum for this 
 * data into address 0x60.
 * 
 *  Manual section 4.5 describes all subcommands available.
 *  Table 12-23. Subcommands Table lists all subcommand codes
 ********************************************************************/

// Note this works LIKE a direct command, but is described in section 4.5,
// and NOT listed in the direct commands list. 

// These are the relevant addresses for sub commands:
#define CMD_DIR_SUBCMD_LOW              0x3E
#define CMD_DIR_SUBCMD_HI               0x3F
#define CMD_DIR_SUBCMD_RESP_START       0x40
#define CMD_DIR_SUBCMD_RESP_CHKSUM      0x60
#define CMD_DIR_SUBCMD_RESP_LEN         0x61

// 32-byte buffer + 8 byte checksum. Should never be over 40 no matter what
// the hardware says.
#define SUBCMD_MAX_RESP_LEN 40

/********************************************************************
 * 
 * These functions let us grab the subcommand low & high byte easily
 ********************************************************************/
#define LOW_BYTE(data) (byte)(data & 0x00FF)
#define HIGH_BYTE(data) (byte)((data >> 8) & 0x00FF)

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/

static const char *TAG = "hwi";
QueueHandle_t queue_i2c = NULL;

/********************************************************************
*
* Initialization
*
********************************************************************/

hardware_interface::hardware_interface(uint8_t hwi_addr, HAL_ESP32 *hwi_hal, i2c_port_t hwi_port) {
    hal = hwi_hal;
    addr = hwi_addr;
    port = hwi_port;

    ESP_LOGD(TAG, "New HWI: HAL: %i - Address: %i - Port: %i", hal, addr, port);

    this->init();
}

// These 2 functions are needed for the I2C configuration
static void IRAM_ATTR TCA6408Interrupt() {
  if (queue_i2c == NULL) { return; }

  i2cQueueMessage m;
  m.command = 0x01;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}

// These 2 functions are needed for the I2C configuration
static void IRAM_ATTR TCA9534AInterrupt()
{
  if (queue_i2c == NULL) { return; }

  i2cQueueMessage m;
  m.command = 0x02;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}

// XXX TODO - refactor this; it can only be called once
void hardware_interface::init(void) {
    ESP_LOGD(TAG, "Initializing HWI");

    // Configure the I2C driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
    this->hal->ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt);
    this->hal->ConfigureVSPI();

    this->hal->SwapGPIO0ToOutput();
}

/********************************************************************
*
* Command methods
*
********************************************************************/

// Thin wrapper to hide the implementation of direct command -> 16 bit read
bool hardware_interface::direct_command(uint8_t command, unsigned int *value) {
    ESP_LOGD(TAG, "Direct Command: 0x%x", command);
    return this->read16(command, value);
}

// Thin wrapper to hide the implementation of sub command
void hardware_interface::sub_command(uint16_t command) {
    uint8_t cmd[2];
    cmd[0] = LOW_BYTE(command);
    cmd[1] = HIGH_BYTE(command);
    
    ESP_LOGD(TAG, "SubCommand: 0x%x", command);
    this->write(CMD_DIR_SUBCMD_LOW, cmd, 2);
}

bool hardware_interface::read_sub_command_response_block(unsigned int *data) {
    bool ret;
    unsigned int ret_len;

    // Read delayed to allow the device to repopulate & settle as needed
    // This is cargo culted over from the original code. If delay is not
    // needed, switch to ->read()
    ret = this->read_delayed( CMD_DIR_SUBCMD_RESP_LEN, &ret_len );
    
    // Safety check (cargo culted)
    if( ret_len > SUBCMD_MAX_RESP_LEN ) { ret_len = SUBCMD_MAX_RESP_LEN; }

    //ESP_LOGD(TAG, "Reading %d bytes", ret_len);

    ret = this->_read_N( CMD_DIR_SUBCMD_RESP_START, data, ret_len, false, true );

    //ESP_LOGD(TAG, "SubCommand RV: %i", ret);

    return ret;
}

/********************************************************************
*
* Low Level Interface methods
*
********************************************************************/

bool hardware_interface::is_connected(void) {
    return this->hal->isConnected(this->port, this->addr);
}

bool hardware_interface::read(uint8_t reg, unsigned int *value) {
    esp_err_t ret;
    uint8_t tmp_value;

    ret = ESP_ERROR_CHECK_WITHOUT_ABORT(this->hal->readByte(this->port, this->addr, reg, &tmp_value));
    if (ret != ESP_OK) {
        return false;
    }

    *value = tmp_value;

    return true;
}

// wrapper for read(16|32), subcommand reads, etc etc) - it's just a loop
bool hardware_interface::_read_N(uint8_t reg, unsigned int *value, int num, bool aggregate_result, bool delay) {
    bool ret;
    *value = 0;

    uint8_t tmp_value[num];
    if( delay ) { HWI_DELAY_TO_SETTLE; }

    // NOTE: if num == 1, you can not use readMultipleBytes(); it'll error out. We'll dispatch to read()
    // instead.
    if( num == 1 ) {
        ret = this->read(reg, value);
    } else {
        ret = ESP_ERROR_CHECK_WITHOUT_ABORT(this->hal->readMultipleBytes(this->port, this->addr, reg, &tmp_value[0], num));
        if( ret == ESP_OK ) {
            for( int i = 0; i < num; i++ ) {
                //ESP_LOGD(TAG, "SubCommand Response (byte %i): %i", i,tmp_value[i]);

                // Do you want to combine all the bytes in the result to one value (commond for single command -> single value)
                // or is it an array of results you want to keep as a block?                
                if( aggregate_result ) {
                    *value = *value | (tmp_value[i] << 8 * i);
                } else {
                    // we have uint8_t here
                    value[i] = (static_cast<unsigned int>(tmp_value[i]));
                }     
            }  
        }                                       
    }

    return ret == ESP_OK;    
}

bool hardware_interface::read_delayed(uint8_t reg, unsigned int *value) {
    return this->_read_N(reg, value, 1, true, true);
}

bool hardware_interface::read16(uint8_t reg, unsigned int *value) {
    return this->_read_N(reg, value, 2, true, false);
}

void hardware_interface::write(uint8_t reg, uint8_t *command, int num) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(this->hal->writeMultipleBytes(this->port, this->addr, reg, command, num));
    HWI_DELAY_TO_SETTLE;
}
