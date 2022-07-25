#include "defines.h"
#include "hardware_interface.h"

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

bool hardware_interface::read16(uint8_t reg, unsigned int *value) {
    unsigned int ret;
    uint8_t tmp_value[2];

    ret = this->hal->readByte(this->port, this->addr, reg, &tmp_value[0]);
    if (ret != ESP_OK) {
        return false;
    }
    ret = this->hal->readByte(this->port, this->addr, reg + 1, &tmp_value[1]);
    if (ret != ESP_OK) {
        return false;
    }
    //ESP_LOGD(TAG,"read16 Reply %i", ret);
    *value = tmp_value[0] | (tmp_value[1] << 8);
    return true;
}
