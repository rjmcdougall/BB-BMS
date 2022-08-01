#pragma once
#include <defines.h>
#include <mutex>

// This conflicts with (I THINK) mutex, where chrono.h also defines a min()
// macro, as well as something in M5Core2. So including it here explicitly
// rather than in defines.h, which goes everywhere. And making sure it goes
// LAST, as it has a conditional define, whereas chrono.h does not:
// https://github.com/m5stack/M5StickC/pull/139
#include <M5Core2.h>

class display
{
public:
    // display(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p);
    void run(void);

    /**
     * Singletons should not be cloneable.
     */
    display(display &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const display &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    static display *GetInstance();

    /********************************************************************
     * display information methods
     ********************************************************************/

    void clear(void);    
    void display_battery(int charge);    
    void display_diagnostics(void);

    
private:
    void init(void);
    static void display_task(void *param);
    
    // Private variables
    static display * display_;
    static std::mutex mutex_;
    static TaskHandle_t display_task_handle;

    
    /********************************************************************
     * display information methods
     ********************************************************************/    

protected:
    display(void);
    ~display();
};