#include "defines.h"
#include "status.h"
#include <mutex>

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/


static const char *TAG = "status";

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 1000; // ms


TaskHandle_t status::status_task_handle = NULL;

status* status::status_{nullptr};
std::mutex status::mutex_;


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
 status *status::GetInstance() {
    std::lock_guard<std::mutex> lock(status::mutex_);
    
    // No instance yet - create one
    if( status_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        status_ = new status();
        status_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return status_;
}

// Private
status::status() {
    ESP_LOGD(TAG, "Initializing Status");
    
}

void status::init(void) {
}

/********************************************************************
*
* status information methods
*
********************************************************************/
