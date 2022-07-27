#include "defines.h"
#include "hardware_interface.h"

// Some tasks require bytes to be written, then read. The original code 
// base had 10ms delays peppered in to accommodate. Currently just cargo
// culting this over to maintain same behaviour.
#define HWI_DELAY_TO_SETTLE vTaskDelay(pdMS_TO_TICKS(10));


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

void hardware_interface::init(void) {
    ESP_LOGD(TAG, "Initializing HWI");

    // Configure the I2C driver: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
    this->hal->ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt);
}


/********************************************************************
*
* Task Runner
*
********************************************************************/

bool hardware_interface::isConnected(void) {
    return this->hal->isConnected(this->port, this->addr);
}

bool hardware_interface::read(uint8_t reg, unsigned int *value) {
    esp_err_t ret;
    uint8_t tmp_value;

    ret = this->hal->readByte(this->port, this->addr, reg, &tmp_value);
    if (ret != ESP_OK) {
        return false;
    }

    *value = tmp_value;

    return true;
}

// wrapper for read(16|32), subcommand reads, etc etc) - it's just a loop
bool hardware_interface::_read_N(uint8_t reg, unsigned int *value, int num, bool delay) {
    bool ret;
    *value = 0;

    unsigned int tmp_value[num];

    for( int i = 0; i < num; i++ ) {
        // delay before reads?
        if( delay ) { HWI_DELAY_TO_SETTLE; }

        ret = this->read( reg + i, &tmp_value[i] );     

        // Something went wrong doing the read...
        if( !ret ) { return false; }

        *value = *value | (tmp_value[i] << 8 * i);
    }

    return true;    
}

bool hardware_interface::read_delayed(uint8_t reg, unsigned int *value) {
    return this->_read_N(reg, value, 1, true);
}

bool hardware_interface::read16(uint8_t reg, unsigned int *value) {
    return this->_read_N(reg, value, 2, false);
}

void hardware_interface::write(uint8_t reg, uint8_t *command, int num) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(this->hal->writeMultipleBytes(this->port, this->addr, reg, command, num));
    HWI_DELAY_TO_SETTLE;
}


/*


// These are the relevant addresses for sub commands:
#define CMD_DIR_SUBCMD_LOW              0x3E
#define CMD_DIR_SUBCMD_HI               0x3F
#define CMD_DIR_SUBCMD_RESP_START       0x40
#define CMD_DIR_SUBCMD_RESP_CHKSUM      0x60
#define CMD_DIR_SUBCMD_RESP_LEN         0x61

// 32-byte buffer + 8 byte checksum. Should never be over 40 no matter what
// the hardware says.
#define SUBCMD_MAX_RESP_LEN 40



bool bq76952::subCommandResponseBlock(uint8_t *data, uint16_t len)
{
    bool ret;
    uint8_t ret_len;

    vTaskDelay(pdMS_TO_TICKS(10));
    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Check len");
    ret = BQhal->readByte(port, BQaddr, CMD_DIR_RESP_LEN, &ret_len);
    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Check len = %d, ret = %d", ret_len, ret);
    if (ret_len > 40) {
      ret_len = 40;
    }

    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Send");

    vTaskDelay(pdMS_TO_TICKS(10));
    for (int i = 0; i < ret_len; i++) {
      ret = BQhal->readByte(port, BQaddr, CMD_DIR_RESP_START + i, data + i);
      vTaskDelay(pdMS_TO_TICKS(10));
      //ESP_LOGD(TAG,"BQsubCommandResponseBlock data %x", data[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Reply %i", ret);
    return ret;
}
*/