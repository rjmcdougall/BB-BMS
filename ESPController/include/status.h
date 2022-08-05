#pragma once
#include "rule_engine.h"
#include "pack.h"
#include "battery.h"
#include "display.h"
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
    static status *GetInstance(pack *p, battery *b, rule_engine *r);
    //static battery *GetInstance(hardware_interface *bq_hwi);

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

    static pack *pack_;
    static battery *battery_;
    static rule_engine *re_; 
    static display *display_;

    /********************************************************************
     * status information methods
     ********************************************************************/    

protected:
    status(pack *p, battery *b, rule_engine *r);
    ~status();
};