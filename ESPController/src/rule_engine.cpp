#include "defines.h"
#include "battery.h"
#include "pack.h"
#include "rule_engine.h"
#include <mutex>

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/


static const char *TAG = "rule_engine";

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 1000; // ms


TaskHandle_t rule_engine::rule_engine_task_handle = NULL;
pack *rule_engine::pack_;
battery *rule_engine::battery_;

rule_engine* rule_engine::re_{nullptr};
std::mutex rule_engine::mutex_;

bool rule_engine::rule_outcome[RELAY_RULES] = {false};

/********************************************************************
*
* Initialization
*
********************************************************************/

/**
 * The first time we call GetInstance we will lock the storage location
 *      and then we make sure again that the variable is null and then we
 *      set the value. RU:
 */
 rule_engine *rule_engine::GetInstance(pack *pack_obj, battery *battery_obj) {
    std::lock_guard<std::mutex> lock(rule_engine::mutex_);
    
    // No instance yet - create one
    if( re_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        re_ = new rule_engine(pack_obj, battery_obj);
        re_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return re_;
}

// Private
rule_engine::rule_engine(pack *pack_obj, battery *battery_obj) {
    pack_ = pack_obj;
    battery_ = battery_obj;
}

void rule_engine::init(void) {
    ESP_LOGD(TAG, "Initializing Rule Engine");
}

/********************************************************************
*
* Invidividual rule checks
*
********************************************************************/

bool rule_engine::is_hardware_connected() {
    return this->battery_->is_connected() && this->pack_->is_connected();
}

/********************************************************************
*
* Task Runner
*
********************************************************************/


// Tasks Must be infinite loops:
void rule_engine::rule_engine_task(void *param) {
    UBaseType_t uxHighWaterMark;
    // XXX Can we just wrap this???
    if( DEBUG_TASKS ) {
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGD(TAG, "Starting HWM/Stack = %i/%i", uxHighWaterMark, TASK_SIZE);
    }        

    for(;;) {

        // // XXX Can we just wrap this???
        if( DEBUG_TASKS ) {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGD(TAG, "Current HWM/Stack = %i/%i", uxHighWaterMark, TASK_SIZE);
        }

        if( !re_->is_hardware_connected() ) {
            ESP_LOGD(TAG, "Hardware is disconnected");
            rule_outcome[Rule::BMSError] = true;
        }

        vTaskDelay( TASK_INTERVAL );
    }
}

void rule_engine::run(void) {
    // How tasks work: https://www.freertos.org/a00125.html
    xTaskCreate(rule_engine::rule_engine_task, TAG, TASK_SIZE, nullptr, TASK_DEFAULT_PRIORITY, &rule_engine_task_handle);       
}