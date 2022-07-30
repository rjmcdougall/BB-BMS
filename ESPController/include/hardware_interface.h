#pragma once

/* Legacy Includes - Largely unaltered */
#include "HAL_ESP32.h"
#include "driver/i2c.h"

class hardware_interface {
public:
    hardware_interface(uint8_t hwi_addr, HAL_ESP32 *hwi_hal, i2c_port_t hwi_port);
    bool is_connected(void);

    /***************************************************
     * high level methods
     ***************************************************/    
    bool direct_command(uint8_t command, unsigned int *value);
    void sub_command(uint16_t command);
    bool read_sub_command_response_block(unsigned int *data);
    
    /***************************************************
     * Low level methods
     ***************************************************/
    bool read(uint8_t reg, unsigned int *value);
    bool read_delayed(uint8_t reg, unsigned int *value);
    bool read16(uint8_t reg, unsigned int *value);
    void write(uint8_t reg, uint8_t *command, int num);
    
private:
    void init(void);
    bool _read_N(uint8_t reg, unsigned int *value, int num, bool aggregate_result, bool delay);

    // Private variables
    HAL_ESP32 *hal;
	uint8_t addr;
	i2c_port_t port;
};