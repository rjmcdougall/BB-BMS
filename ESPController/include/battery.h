#pragma once

#include "hardware_interface.h"
#include <mutex>

class battery
{
public:
    // battery(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p);
    bool isConnected(void);
    void run(void);

    /**
     * Singletons should not be cloneable.
     */
    battery(battery &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const battery &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    static battery *GetInstance(hardware_interface *hwi);

    /********************************************************************
     * Battery information methods
     ********************************************************************/
    unsigned int get_cell_voltage(byte cellNumber);

private:
    void init(void);
    static void battery_task(void *param);
    
    // Private variables
    static battery * battery_;
    static std::mutex mutex_;
    static TaskHandle_t battery_task_handle;
    static hardware_interface *bq_hwi;
	
protected:
    battery(hardware_interface *hwi);
    ~battery();
    std::string value_;
};