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
#include "rule_engine.h"
#include "status.h"
#include "audio.h"
#include "display.h"

#include "bq34z100.h"

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
#define RUN_BATTERY_TASK true
#define RUN_PACK_TASK true
#define RUN_RULES_TASK true
#define RUN_DEBUG_ORIG_BQZ true

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

    M5.Spk.DingDong();


    display *d = display::GetInstance();
    M5.Lcd.clear();
    d->display_battery(72);

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
    bq34z100 bqz = bq34z100(BQZ_ADDR, &hal, I2C_NUM_1);

    // XXX TODO: This does not seem to reset the log level - DEBUG remains in effect
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html#_CPPv415esp_log_level_t
    //esp_log_level_set("*", ESP_LOG_ERROR);
    // Run a sample task to exercise the system
    if( RUN_SAMPLE_TASK ) {        
        sample_task sample;
        sample.run();
    }

    // used in rules
    battery* bat;
    if( RUN_BATTERY_TASK ) {        
        bat = battery::GetInstance(&bq_hwi);
        bat->run();
    }    

    // used in rules
    pack* pack;
    if( RUN_PACK_TASK ) {        
        pack = pack::GetInstance(&bqz_hwi);
        //XXX not actually connecting, and not used in rules; hold off
        //pack->run();
    }    

    if( RUN_RULES_TASK ) {        
        if(bat && pack) {
            rule_engine* rules = rule_engine::GetInstance(pack, bat);
            rules->run();
        } else {
            ESP_LOGE(TAG, "Can not run rules without a pack & battery");
        }
    }    

    // XXX debug run for original bqz
    if( RUN_DEBUG_ORIG_BQZ ) {
        uint8_t tmp_value;
        unsigned int ret;

        ret = ESP_ERROR_CHECK_WITHOUT_ABORT(hal.readByte(I2C_NUM_1, BQZ_ADDR, 0x02, &tmp_value));
        if (ret != ESP_OK) {    
            ESP_LOGD(TAG, "PACK read failed: %i", ret);
        } else {
            ESP_LOGD(TAG, "PACK rv: %i", tmp_value);
        }

        uint8_t soc = bqz.state_of_charge();
        ESP_LOGD(TAG, "BQZ rv: %i", soc);
    }


}

/********************************************************************
*
* Runtime
*
********************************************************************/

void loop()
{


    display *d = display::GetInstance();
    int charge = 99;

    d->clear();
    d->display_battery(charge);
    d->display_diagnostics("Hello: %s", "world");
    delay(10000);

    // d->lcd.setCursor(10,10);    
    // d->lcd.fillScreen(d->lcd.alphaBlend(128, BLACK, DARKGREEN));
    // d->lcd.printf("OK: hello world");

    // delay(1000);

    // d->lcd.setCursor(10,10);
    // d->lcd.fillScreen(d->lcd.alphaBlend(128, BLACK, YELLOW));
    // d->lcd.printf("WARNING: hello world");

    // delay(1000);

    // d->lcd.setCursor(10,10);
    // d->lcd.fillScreen(d->lcd.alphaBlend(128, BLACK, RED));
    // d->lcd.printf("ERROR: hello world");

    // delay(1000);
}
