/********************************************************************
*
* Includes
*
********************************************************************/
// #include "defines.h"

// #include "sample_task.h"
// #include "hardware_interface.h"
// #include "battery.h"

// // This conflicts with (I THINK) mutex, where chrono.h also defines a min()
// // macro, as well as something in M5Core2. So including it here explicitly
// // rather than in defines.h, which goes everywhere. And making sure it goes
// // LAST, as it has a conditional define, whereas chrono.h does not:
// // https://github.com/m5stack/M5StickC/pull/139
// #include <M5Stack.h>

// /********************************************************************
// *
// * Definitions
// *
// ********************************************************************/

// #define RUN_SAMPLE_TASK false
// #define RUN_BATTERY_TASK true

// static const char *TAG = "diybms";

// /********************************************************************
// *
// * Initialization
// *
// ********************************************************************/

// void setup()
// {   
//     // Initialize all items in the M5 stack as needed
//     M5.begin();
    
//     M5.Speaker.begin();
//     unsigned int volume = 100;
//     M5.Speaker.setVolume(volume);
//     M5.Power.begin();       // Init Power module.  初始化电源
//     M5.Lcd.begin();

//     M5.Lcd.setTextSize(2);  // Set the text size to 2.  设置文字大小为2
//     M5.Lcd.println("M5Stack Speaker test");  // Screen printingformatted string.    

//     // /* Interact with the hardware - use this as a (thin) wrapper around 
//     // * HAL, and a way to avoid repeating common code (address, port, etc)
//     // * This will also allow replacing HAL at some point, should we so choose
//     // */
//     // // XXX THESE MUST BE DECLARED STATIC as we are passing them around for various tasks
//     // // that interact with the hardware. It's member variables may get GC'd otherwise,
//     // // leading to unpredictable outcomes: https://www.freertos.org/a00125.html
//     // static HAL_ESP32 hal;
//     // static hardware_interface hwi = hardware_interface(BQ_ADDR, &hal, I2C_NUM_1);

//     // // XXX TODO: This does not seem to reset the log level - DEBUG remains in effect
//     // // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html#_CPPv415esp_log_level_t
//     // //esp_log_level_set("*", ESP_LOG_ERROR);
//     // // Run a sample task to exercise the system
//     // if( RUN_SAMPLE_TASK ) {        
//     //     sample_task sample;
//     //     sample.run();
//     // }

//     // if( RUN_BATTERY_TASK ) {        
//     //     battery* bat = battery::GetInstance(&hwi);
        
        
//     //     //ESP_LOGD(TAG, "Battery connected 2: %i", bat->bq_hwi->isConnected());
//     //     //ESP_LOGD(TAG, "HWI 2: %s", battery_->bq_hwi);

//     //     // How tasks work: https://www.freertos.org/a00125.html
//     //     //xTaskCreate(battery::battery_task, TAG, 2048, nullptr, TASK_DEFAULT_PRIORITY, &battery::battery_task_handle);       

        
//     //     bat->run();
//     // }    
// }

// /********************************************************************
// *
// * Runtime
// *
// ********************************************************************/

// void loop()
// {
//     M5.Speaker.beep();
//     M5.Speaker.tone(600, 1000);

//     //ESP_LOGD(TAG, "loop");   
// }
#include <M5Core2.h>

void setup() {
    M5.begin();

    M5.Axp.begin();

    M5.Lcd.wakeup();
    M5.Lcd.setBrightness(255);

    M5.Lcd.fillScreen(RED);

    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print("Hello World");

    M5.Spk.DingDong();
}

void loop() {
}