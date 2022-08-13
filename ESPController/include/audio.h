#pragma once
#include <defines.h>
#include <mutex>

class audio
{
public:
    // audio(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p);
    void run(void);

    /**
     * Singletons should not be cloneable.
     */
    audio(audio &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const audio &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    static audio *GetInstance();

    void play_alert(int cnt);
    void enable_alerts();
    void disable_alerts();
    bool are_alerts_enabled();
    
    /********************************************************************
     * audio information methods
     ********************************************************************/
    
private:
    void init(void);
    static void audio_task(void *param);
    
    // Private variables
    static audio * audio_;
    static std::mutex mutex_;
    static TaskHandle_t audio_task_handle;
    static bool _alerts_enabled;
    
    /********************************************************************
     * audio information methods
     ********************************************************************/    

protected:
    audio(void);
    ~audio();
};