#include "defines.h"
#include "sample_task.h"

static const char *TAG = "sample_task";

TaskHandle_t sample_task_handle = NULL;
static const int TASK_SIZE = TASK_STACK_SIZE_SMALL;
static const int TASK_INTERVAL = 1000;

// Tasks Must be infinite loops:
 void sample_task::task_one(void *param) {

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
            ESP_LOGD(TAG, "Current HWM/Stack = %i/%i", uxHighWaterMark, TASK_SIZE);
        }

        vTaskDelay( TASK_INTERVAL );
    }
}

void sample_task::run(void) {
    // How tasks work: https://www.freertos.org/a00125.html
    xTaskCreate(sample_task::task_one, TAG, TASK_SIZE, nullptr, TASK_DEFAULT_PRIORITY, &sample_task_handle);       
}

