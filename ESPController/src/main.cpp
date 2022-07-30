/********************************************************************
*
* Includes
*
********************************************************************/
#include "defines.h"

#include "sample_task.h"
#include "hardware_interface.h"
#include "battery.h"
#include "pack.h"

// This conflicts with (I THINK) mutex, where chrono.h also defines a min()
// macro, as well as something in M5Core2. So including it here explicitly
// rather than in defines.h, which goes everywhere. And making sure it goes
// LAST, as it has a conditional define, whereas chrono.h does not:
// https://github.com/m5stack/M5StickC/pull/139
#include <M5Core2.h>

/********************************************************************
*
* Definitions
*
********************************************************************/

#define RUN_SAMPLE_TASK false
#define RUN_BATTERY_TASK false
#define RUN_PACK_TASK true

static const char *TAG = "diybms";

/********************************************************************
*
* Initialization
*
********************************************************************/

void setup()
{   
    // Initialize all items in the M5 stack as needed
    /*************************************************
     * You will get an error: E (1460) ledc: ledc_channel_config(369): gpio_num argument is invalid
     * This is a bug that's left over from a code clean - it's the -1 pin being used for backlight.
     * This is set via Axp apparently, so it's a non-issue. Massive red herring :(
     * https://github.com/m5stack/M5Core2/issues/92
     * ************************************************/
    M5.begin();
        
    M5.Lcd.setTextSize(2);  // Set the text size to 2.  设置文字大小为2
    M5.Lcd.println("M5Stack Speaker test");  // Screen printingformatted string.    
    M5.Spk.DingDong();


    /* Interact with the hardware - use this as a (thin) wrapper around 
    * HAL, and a way to avoid repeating common code (address, port, etc)
    * This will also allow replacing HAL at some point, should we so choose
    */
    // XXX THESE MUST BE DECLARED STATIC as we are passing them around for various tasks
    // that interact with the hardware. It's member variables may get GC'd otherwise,
    // leading to unpredictable outcomes: https://www.freertos.org/a00125.html
    static HAL_ESP32 hal;
    static hardware_interface bq_hwi = hardware_interface(BQ_ADDR, &hal, I2C_NUM_1);
    static hardware_interface bqz_hwi = hardware_interface(BQZ_ADDR, &hal, I2C_NUM_1);

    // XXX TODO: This does not seem to reset the log level - DEBUG remains in effect
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html#_CPPv415esp_log_level_t
    //esp_log_level_set("*", ESP_LOG_ERROR);
    // Run a sample task to exercise the system
    if( RUN_SAMPLE_TASK ) {        
        sample_task sample;
        sample.run();
    }

    if( RUN_BATTERY_TASK ) {        
        battery* bat = battery::GetInstance(&bq_hwi);
        bat->run();
    }    

    if( RUN_PACK_TASK ) {        
        pack* pack = pack::GetInstance(&bqz_hwi);
        //pack->run();
    }    
}

/********************************************************************
*
* Runtime
*
********************************************************************/

void loop()
{

}
