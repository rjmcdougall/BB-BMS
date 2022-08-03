#pragma once
#include <defines.h>
#include "hardware_interface.h"
#include <mutex>

class battery
{
public:
    // battery(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p);
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
    static battery *GetInstance(hardware_interface *bq_hwi);
    
    /********************************************************************
     * Battery information methods
     ********************************************************************/
    bool is_connected(void);

    unsigned int get_cell_voltage(byte cellNumber);
    float get_internal_temp(void);
    bool is_charging(void);
    bool is_discharging(void);
    float min_cell_temp();
    float max_cell_temp();
    unsigned int max_cell_voltage(void);
    unsigned int min_cell_voltage(void);

    unsigned int get_stack_voltage(void);
    unsigned int get_pack_voltage(void);
    
private:
    void init(void);
    static void battery_task(void *param);
    
    // Private variables
    static battery * battery_;
    static std::mutex mutex_;
    static TaskHandle_t battery_task_handle;
    static hardware_interface *hwi;

    // Cache for multi-byte subcommands
    // '64' is cargo culted over from the original code.
    unsigned int _dastatus5_cache[64];    

    /********************************************************************
     * Battery information methods
     ********************************************************************/
    bool _fet_status(int reg);
    bool _update_dastatus5_cache(void);
    unsigned int _read_dastatus5_cache(unsigned int cmd);
    float milli_kelvin_to_c(float mk);

protected:
    battery(hardware_interface *hwi);
    ~battery();
};