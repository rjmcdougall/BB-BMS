#pragma once
#include <defines.h>
#include <mutex>

class status
{
public:
    // status(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p);
    void run(void);

    /**
     * Singletons should not be cloneable.
     */
    status(status &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const status &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    static status *GetInstance();
    
    /********************************************************************
     * status information methods
     ********************************************************************/
    
private:
    void init(void);
    static void status_task(void *param);
    
    // Private variables
    static status * status_;
    static std::mutex mutex_;
    static TaskHandle_t status_task_handle;

    
    /********************************************************************
     * status information methods
     ********************************************************************/    

protected:
    status(void);
    ~status();
};