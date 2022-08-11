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
static const int TASK_INTERVAL = 10000; // ms
// Check more often if we're still booting; data will be coming in
static const int TASK_BOOT_INTERVAL = 1000; // ms

static const char *TASK_STATUS_OK = "Status: OK";
static const char *TASK_STATUS_BOOT = "Status: Booting";

TaskHandle_t status::status_task_handle = NULL;

status* status::status_{nullptr};
std::mutex status::mutex_;

pack *status::pack_;
battery *status::battery_;
rule_engine *status::re_; 
display *status::display_;

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
 status *status::GetInstance(pack *p, battery *b, rule_engine *r) {
    std::lock_guard<std::mutex> lock(status::mutex_);
    
    // No instance yet - create one
    if( status_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        status_ = new status(p, b, r);

        status_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return status_;
}

// Private
status::status(pack *p, battery *b, rule_engine *r) {
    pack_ = p;
    battery_ = b;
    re_ = r;

    display_ = display::GetInstance();
}

void status::init(void) {
    ESP_LOGD(TAG, "Initializing Status");
}

/********************************************************************
*
* status information methods
*
********************************************************************/

/********************************************************************
*
* status task runner
*
********************************************************************/

// Tasks Must be infinite loops:
void status::status_task(void *param) {
    UBaseType_t uxHighWaterMark;
    // XXX Can we just wrap this???
    if( DEBUG_TASKS ) {
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGD(TAG, "Starting HWM/Stack = %i/%i", uxHighWaterMark, TASK_SIZE);
    }        

    display *d = status_->display_;
    battery *bat = status_->battery_;
    pack *pack = status_->pack_;
    rule_engine *re = status_->re_;

    for(;;) {
        // Depending on whether we have all data or not, we'll return to this task earlier or later.
        int interval = TASK_INTERVAL;
        char *status = (char *)TASK_STATUS_OK;

        // // XXX Can we just wrap this???
        if( DEBUG_TASKS ) {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGD(TAG, "Current HWM/Stack = %i/%i", uxHighWaterMark, TASK_SIZE);
        }

        // Visual displays    
        d->clear();

        if( pack->has_data() ) {
            d->display_battery( pack->state_of_charge() );
        }

        if( bat->has_data() ) {
            d->display_cell_temp( bat->min_cell_temp(), bat->max_cell_temp() );
            d->display_cell_voltage_delta( bat->max_cell_voltage() - bat->min_cell_voltage() );
            // This is returned in mV, and display is in Volts.
            d->display_stack_voltage( bat->get_stack_voltage()/1000 );
        }    

        // Use temp value so we can draw the border last
        bool display_error_border = false;
        if( re->has_data() ) {
            bool rule_error = false;

            // Max amount of rules we can have
            bool rules[re->max_rule_count()]; 
            int error_count = re->get_rule_outcomes( &rules[0] );
            
            if(error_count > 0) {
                display_error_border = true;
                status = (char * )"ERROR COUNT: XXX TODO";
            }
        }    

        if( !bat->has_data() || !pack->has_data() || !re->has_data() ) {
            interval = TASK_BOOT_INTERVAL;
            status = (char *)TASK_STATUS_BOOT;
        }

        // everything oK?
        d->display_diagnostics(status);
        if( display_error_border) {
            d->display_border(d->DISPLAY_COLOR_ERROR);
        }

        ESP_LOGD("TAG", "Task sleeping for: %i ms", interval);
        vTaskDelay( interval );
    }
}

void status::run(void) {
    // How tasks work: https://www.freertos.org/a00125.html
    xTaskCreate(status::status_task, TAG, TASK_SIZE, nullptr, TASK_DEFAULT_PRIORITY, &status_task_handle);       
}
