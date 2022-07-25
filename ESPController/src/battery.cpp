#include "defines.h"
#include "battery.h"
#include <mutex>

// Inline functions
#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/

static const char *TAG = "battery";

TaskHandle_t battery::battery_task_handle = NULL;
hardware_interface *battery::bq_hwi;

static const int TASK_SIZE = TASK_STACK_SIZE_MEDIUM;
static const int TASK_INTERVAL = 1000; // ms

static const int CELL_COUNT = 13; // XXX get this from config or elsewhere

battery* battery::battery_{nullptr};
std::mutex battery::mutex_;


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
 battery *battery::GetInstance(hardware_interface *hwi) {
    std::lock_guard<std::mutex> lock(battery::mutex_);
    
    // No instance yet - create one
    if( battery_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        battery_ = new battery(hwi);
        battery_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return battery_;
}

// Private
battery::battery(hardware_interface *hwi) {
    bq_hwi = hwi;
}

void battery::init(void) {
    ESP_LOGD(TAG, "Battery connected: %s", this->bq_hwi->isConnected() ? "true" : "false");
}

/********************************************************************
*
* Battery information methods
*
********************************************************************/

// Read single cell voltage
unsigned int battery::get_cell_voltage(byte cellNumber) {
    unsigned int value;
    if (battery_->bq_hwi->read16(CELL_NO_TO_ADDR(cellNumber), &value)) {
        return value;
    } else {
        return 0;
    }
}


/********************************************************************
*
* Task Runner
*
********************************************************************/


// Tasks Must be infinite loops:
void battery::battery_task(void *param) {
    UBaseType_t uxHighWaterMark;
    // XXX Can we just wrap this???
    if( DEBUG_TASKS ) {
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGD(TAG, "Starting HWM/Stack = %i/%i", uxHighWaterMark, TASK_SIZE);
    }        

    for(;;) {
        // XXX Can we just wrap this???
        if( DEBUG_TASKS ) {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGD(TAG, "Current HWM/Stack = %i/%i - Connected: %s", uxHighWaterMark, TASK_SIZE, battery_->bq_hwi->isConnected() ? "true" : "false");
        }

        for( int i = 0; i < CELL_COUNT; ++i ) {
            unsigned int v = battery_->get_cell_voltage(i);
            ESP_LOGD(TAG, "Voltage for cell %i: %i", i, v);
        }


        vTaskDelay( TASK_INTERVAL );
    }
}

void battery::run(void) {
    // How tasks work: https://www.freertos.org/a00125.html
    xTaskCreate(battery::battery_task, TAG, TASK_SIZE, nullptr, TASK_DEFAULT_PRIORITY, &battery_task_handle);       
}



/********************************************************************
*
* Utility methods
*
********************************************************************/

