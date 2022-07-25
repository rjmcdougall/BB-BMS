/********************************************************************
*
* Includes
*
********************************************************************/
#include "defines.h"
#include "sample_task.h"
#include "hardware_interface.h"
#include "battery.h"

/********************************************************************
*
* Definitions
*
********************************************************************/

#define RUN_SAMPLE_TASK false
#define RUN_BATTERY_TASK true

static const char *TAG = "diybms";

/********************************************************************
*
* Initialization
*
********************************************************************/

void setup()
{   
    
    /* Interact with the hardware - use this as a (thin) wrapper around 
    * HAL, and a way to avoid repeating common code (address, port, etc)
    * This will also allow replacing HAL at some point, should we so choose
    */
    // XXX THESE MUST BE DECLARED STATIC as we are passing them around for various tasks
    // that interact with the hardware. It's member variables may get GC'd otherwise,
    // leading to unpredictable outcomes: https://www.freertos.org/a00125.html
    static HAL_ESP32 hal;
    static hardware_interface hwi = hardware_interface(BQ_ADDR, &hal, I2C_NUM_1);

    // XXX TODO: This does not seem to reset the log level - DEBUG remains in effect
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html#_CPPv415esp_log_level_t
    //esp_log_level_set("*", ESP_LOG_ERROR);
    // Run a sample task to exercise the system
    if( RUN_SAMPLE_TASK ) {        
        sample_task sample;
        sample.run();
    }

    if( RUN_BATTERY_TASK ) {        
        battery* bat = battery::GetInstance(&hwi);
        
        
        //ESP_LOGD(TAG, "Battery connected 2: %i", bat->bq_hwi->isConnected());
        //ESP_LOGD(TAG, "HWI 2: %s", battery_->bq_hwi);

        // How tasks work: https://www.freertos.org/a00125.html
        //xTaskCreate(battery::battery_task, TAG, 2048, nullptr, TASK_DEFAULT_PRIORITY, &battery::battery_task_handle);       

        
        bat->run();
    }    
}

/********************************************************************
*
* Runtime
*
********************************************************************/

void loop()
{
    //ESP_LOGD(TAG, "loop");   
}
