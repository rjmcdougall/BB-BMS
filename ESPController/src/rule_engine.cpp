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

// XXX TODO: Check these numbers
#define RULE_MAX_CELL_TEMP 60.0     // in Celsius
#define RULE_MIN_CELL_TEMP 0.0      // in Celsius
#define RULE_MAX_CELL_VOLTAGE 4000  // milliVolts
#define RULE_MIN_CELL_VOLTAGE 2800  // milliVolts
#define RULE_MAX_MODULE_TEMP 60.0   // in Celsius
#define RULE_MIN_MODULE_TEMP 0.0    // in Celsius
#define RULE_MAX_PACK_VOLTAGE 50000 // milliVolts
#define RULE_MIN_PACK_VOLTAGE 36000 // milliVolts

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

// Error code: BMSError = 1,
bool rule_engine::is_hardware_connected() {
    bool rv = this->battery_->is_connected() && this->pack_->is_connected();
    if( !rv ) {
        ESP_LOGD(TAG, "Hardware is disconnected");
    }
    return rv;
}

// Error code: Individualcellovervoltage = 2,
bool rule_engine::is_battery_cell_over_max_voltage() {
    bool rv = false;

    unsigned int v = this->battery_->max_cell_voltage();
    
    if( v > RULE_MAX_CELL_VOLTAGE ) {
        rv = true;
        ESP_LOGD(TAG, "Cell over max voltage: %i > %i", v, RULE_MAX_CELL_VOLTAGE);
    }
    return rv;

}

// Error code: Individualcellundervoltage = 3,
bool rule_engine::is_battery_cell_under_min_voltage() {
    bool rv = false;

   unsigned int v = this->battery_->min_cell_voltage();
    if( v < RULE_MIN_CELL_VOLTAGE ) {
        rv = true;
        ESP_LOGD(TAG, "Cell under min voltage: %i < %i", v, RULE_MIN_CELL_VOLTAGE);
    }
    return rv;    
}

// Error code: ModuleOverTemperatureInternal = 4,
bool rule_engine::is_battery_module_over_max_temp() {
    bool rv = false;

    float t = this->battery_->get_internal_temp();
    
    if( t > RULE_MAX_MODULE_TEMP ) {
        rv = true;
        ESP_LOGD(TAG, "Module over max temp: %f > %f", t, RULE_MAX_MODULE_TEMP);
    }
    return rv;
}

// Error code: ModuleUnderTemperatureInternal = 5,
bool rule_engine::is_battery_module_under_min_temp() {
    bool rv = false;

    float t = this->battery_->get_internal_temp();
    if( t < RULE_MIN_MODULE_TEMP ) {
        rv = true;
        ESP_LOGD(TAG, "Module under min temp: %f < %f", t, RULE_MIN_MODULE_TEMP);
    }
    return rv;
}


// Error code: IndividualcellovertemperatureExternal = 6,
bool rule_engine::is_battery_cell_over_max_temp() {
    bool rv = false;

    float t = this->battery_->max_cell_temp();
    if( t > RULE_MAX_CELL_TEMP ) {
        rv = true;
        ESP_LOGD(TAG, "Cell over max temp: %f > %f", t, RULE_MAX_CELL_TEMP);
    }
    return rv;
}


// Error code: IndividualcellundertemperatureExternal = 7,
bool rule_engine::is_battery_cell_under_min_temp() {
    bool rv = false;

    float t = this->battery_->min_cell_temp();
    if( t < RULE_MIN_CELL_TEMP ) {
        rv = true;
        ESP_LOGD(TAG, "Cell under min temp: %f < %f", t, RULE_MIN_CELL_TEMP);
    }
    return rv;
}


// Error code: PackOverVoltage = 8,
bool rule_engine::is_battery_pack_over_max_voltage() {
    bool rv = false;

    // XXX this returns an int - is taht right?
    unsigned int v = this->battery_->get_pack_voltage();
    
    if( v > RULE_MAX_PACK_VOLTAGE ) {
        rv = true;
        ESP_LOGD(TAG, "Pack over max voltage: %i > %i", v, RULE_MAX_PACK_VOLTAGE);
    }
    return rv;

}

// Error code: PackUnderVoltage = 9,
bool rule_engine::is_battery_pack_under_min_voltage() {
    bool rv = false;

    // XXX this returns an int - is taht right?
    unsigned int v = this->battery_->get_pack_voltage();
    
    if( v < RULE_MIN_PACK_VOLTAGE ) {
        rv = true;
        ESP_LOGD(TAG, "Pack under min voltage: %i < %i", v, RULE_MIN_PACK_VOLTAGE);
    }
    return rv;    
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

        /* Error codes:
            EmergencyStop = 0,
            BMSError = 1,
            Individualcellovervoltage = 2,
            Individualcellundervoltage = 3,
            ModuleOverTemperatureInternal = 4,
            ModuleUnderTemperatureInternal = 5,
            IndividualcellovertemperatureExternal = 6,
            IndividualcellundertemperatureExternal = 7,
            PackOverVoltage = 8,
            PackUnderVoltage = 9,
            Timer2 = 10,
            Timer1 = 11
        */

        // XXX No error condition for this yet.
        rule_outcome[Rule::EmergencyStop] = false;

        // Step 1 - can we even measure?
        rule_outcome[Rule::BMSError] = re_->is_hardware_connected();

        // Individual cell voltage
        // Individualcellovervoltage = 2,
        // Individualcellundervoltage = 3,            
        rule_outcome[Rule::Individualcellovervoltage] = re_->is_battery_cell_over_max_voltage();
        rule_outcome[Rule::Individualcellundervoltage] = re_->is_battery_cell_under_min_voltage();
            
        // Module temperature
        // ModuleOverTemperatureInternal = 4,
        // ModuleUnderTemperatureInternal = 5,          
        rule_outcome[Rule::ModuleOverTemperatureInternal] = re_->is_battery_module_over_max_temp();
        rule_outcome[Rule::ModuleUnderTemperatureInternal] = re_->is_battery_module_under_min_temp();

        // Single cell over or under temp?
        // IndividualcellovertemperatureExternal = 6,
        // IndividualcellundertemperatureExternal = 7,                    
        rule_outcome[Rule::IndividualcellovertemperatureExternal] = re_->is_battery_cell_over_max_temp();
        rule_outcome[Rule::IndividualcellundertemperatureExternal] = re_->is_battery_cell_under_min_temp();

        // Total pack voltage ok?
        // PackOverVoltage = 8,
        // PackUnderVoltage = 9,
        rule_outcome[Rule::PackOverVoltage] = re_->is_battery_pack_over_max_voltage();
        rule_outcome[Rule::PackUnderVoltage] = re_->is_battery_pack_under_min_voltage();
        
        vTaskDelay( TASK_INTERVAL );
    }
}

void rule_engine::run(void) {
    // How tasks work: https://www.freertos.org/a00125.html
    xTaskCreate(rule_engine::rule_engine_task, TAG, TASK_SIZE, nullptr, TASK_DEFAULT_PRIORITY, &rule_engine_task_handle);       
}