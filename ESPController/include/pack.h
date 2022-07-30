#pragma once

#include "hardware_interface.h"
#include <mutex>

class pack
{
public:
    // pack(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p);
    void run(void);

    /**
     * Singletons should not be cloneable.
     */
    pack(pack &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const pack &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    static pack *GetInstance(hardware_interface *bq_hwi);
    
    /********************************************************************
     * pack information methods
     ********************************************************************/
    
    
private:
    void init(void);
    static void pack_task(void *param);
    
    // Private variables
    static pack * pack_;
    static std::mutex mutex_;
    static TaskHandle_t pack_task_handle;
    static hardware_interface *hwi;

    
    /********************************************************************
     * pack information methods
     ********************************************************************/    

protected:
    pack(hardware_interface *hwi);
    ~pack();
    std::string value_;
};