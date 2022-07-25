#pragma once

/* Legacy Includes - Largely unaltered */
#include "HAL_ESP32.h"
#include "driver/i2c.h"

class hardware_interface {
public:
    hardware_interface(uint8_t hwi_addr, HAL_ESP32 *hwi_hal, i2c_port_t hwi_port);
    bool isConnected(void);
    bool read16(uint8_t reg, unsigned int *value);

private:
    void init(void);

    // Private variables
    HAL_ESP32 *hal;
	uint8_t addr;
	i2c_port_t port;
};