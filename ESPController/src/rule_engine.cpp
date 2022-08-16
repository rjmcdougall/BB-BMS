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
#define RULE_LOW_STATE_OF_CHARGE 30         // in %
#define RULE_CRITICAL_STATE_OF_CHARGE 15    // in %
#define RULE_MAX_CELL_TEMP 60.0     // in Celsius
#define RULE_MIN_CELL_TEMP -0.1     // in Celsius - set below 0, as it's initialized with 0
#define RULE_MAX_CELL_VOLTAGE 4000  // milliVolts
#define RULE_MIN_CELL_VOLTAGE 2800  // milliVolts
#define RULE_MAX_MODULE_TEMP 60.0   // in Celsius
#define RULE_MIN_MODULE_TEMP -0.1   // in Celsius - set below 0, as it's initialized with 0
#define RULE_MAX_PACK_VOLTAGE 50000 // milliVolts
#define RULE_MIN_PACK_VOLTAGE 36000 // milliVolts

static const char *TAG = "rule_engine";

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 10000; // ms


TaskHandle_t rule_engine::rule_engine_task_handle = NULL;
pack *rule_engine::pack_;
battery *rule_engine::battery_;

rule_engine* rule_engine::re_{nullptr};
std::mutex rule_engine::mutex_;
std::mutex rule_engine::run_all_rules_mutex_;


const int rule_engine::_rule_count = RELAY_RULES;
bool rule_engine::_rule_outcome[RELAY_RULES] = {false};
bool rule_engine::_has_data = false;
int rule_engine::_error_count = 0;                   // Amount of rules that failed
int rule_engine::_error_rules[RELAY_RULES] = {-1};   // Rule numbers that failed

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
* Accessors
*
********************************************************************/


bool::rule_engine::has_data(void) {
    return this->_has_data;
}

// Create a copy - don't want the caller to mess up the contents of our
// checks. Returns the amount of rules we have checked so they can 
// iterate
int rule_engine::get_all_rule_outcomes( bool *outcome ) {

    if( !this->has_data() ) {
        ESP_LOGD(TAG, "No data yet - returning 0");
        return 0;
    }

    int rv = this->_rule_count;

    for( int i = 0; i < this->_rule_count; i++ ) {
        outcome[i] = this->_rule_outcome[i];
    }

    return rv;
}

// Create a copy - don't want the caller to mess up the contents of our
// checks. Returns the amount of rules we FAILED so they can iterate
int rule_engine::get_error_rule_outcomes( int *outcome ) {
    if( !this->has_data() ) {
        ESP_LOGD(TAG, "No data yet - returning 0");
        return 0;
    }

    int rv = this->_error_count;

    for( int i = 0; i < this->_error_count; i++ ) {
        ESP_LOGD(TAG, "Returning error code: %i at index %i", this->_error_rules[i], i);
        outcome[i] = this->_error_rules[i];
    }

    return rv;
}    

// Simply return how many errros we currently know about
int rule_engine::get_active_error_count() {
    return this->_error_count;
}

/********************************************************************
*
* Invidividual rule checks
*
********************************************************************/

int rule_engine::max_rule_count() {
    return this->_rule_count;
}

// Error code: BMSError = 1,
bool rule_engine::is_hardware_connected() {
    bool rv = this->battery_->is_connected() && this->pack_->is_connected();
    if( !rv ) {
        ESP_LOGD(TAG, "Hardware is disconnected");
    }
    return rv;
}

bool rule_engine::is_state_of_charge_low() {
    bool rv = this->pack_->state_of_charge() < RULE_LOW_STATE_OF_CHARGE;
    if(rv) {
        ESP_LOGD(TAG, "State of charge LOW: < %i", RULE_LOW_STATE_OF_CHARGE);
    }
    return rv;
}

bool rule_engine::is_state_of_charge_critical() {
    bool rv = this->pack_->state_of_charge() < RULE_CRITICAL_STATE_OF_CHARGE;
    if(rv) {
        ESP_LOGD(TAG, "State of charge CRITICAL: < %i", RULE_CRITICAL_STATE_OF_CHARGE);
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

void rule_engine::_run_all_rules() {
    std::lock_guard<std::mutex> lock(rule_engine::run_all_rules_mutex_);

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
    _rule_outcome[Rule::EmergencyStop] = false;

    // Step 1 - can we even measure?
    _rule_outcome[Rule::BMSError] = !this->is_hardware_connected();

    // NOTE: These methods only make sense if we have battery data - so check that FIRST, and 
    // then go on evaluating the rules.
    if( this->battery_->has_data() ) {

        // Individual cell voltage
        // Individualcellovervoltage = 2,
        // Individualcellundervoltage = 3,            
        _rule_outcome[Rule::Individualcellovervoltage] = this->is_battery_cell_over_max_voltage();
        _rule_outcome[Rule::Individualcellundervoltage] = this->is_battery_cell_under_min_voltage();
            
        // Module temperature
        // ModuleOverTemperatureInternal = 4,
        // ModuleUnderTemperatureInternal = 5,          
        _rule_outcome[Rule::ModuleOverTemperatureInternal] = this->is_battery_module_over_max_temp();
        _rule_outcome[Rule::ModuleUnderTemperatureInternal] = this->is_battery_module_under_min_temp();

        // Single cell over or under temp?
        // IndividualcellovertemperatureExternal = 6,
        // IndividualcellundertemperatureExternal = 7,                    
        _rule_outcome[Rule::IndividualcellovertemperatureExternal] = this->is_battery_cell_over_max_temp();
        _rule_outcome[Rule::IndividualcellundertemperatureExternal] = this->is_battery_cell_under_min_temp();

        // Total pack voltage ok?
        // PackOverVoltage = 8,
        // PackUnderVoltage = 9,
        _rule_outcome[Rule::PackOverVoltage] = this->is_battery_pack_over_max_voltage();
        _rule_outcome[Rule::PackUnderVoltage] = this->is_battery_pack_under_min_voltage();


        // Now do the inventory; what failed and how many?
        int _tmp_error_count = 0;
        for( int i = 0; i < this->_rule_count; i++ ) {
            // Reset the value
            this->_error_rules[i] = -1;

            if( this->_rule_outcome[i] == true ) {
                ESP_LOGE(TAG, "ERROR: Rule %i triggered", i);
                this->_error_rules[_tmp_error_count++] = i;
            }
        }

        this->_error_count = _tmp_error_count;

        // We have data to work with, so errors count now.
        this->_has_data = true;

    } else {
        ESP_LOGD(TAG, "No battery data available yet - skipping individual battery rules");
    }
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

        // Every time we call this, we want to lock the thread, so call a separate function.
        re_->_run_all_rules();

        vTaskDelay( TASK_INTERVAL );
    }
}

void rule_engine::run(void) {
    // How tasks work: https://www.freertos.org/a00125.html
    xTaskCreate(rule_engine::rule_engine_task, TAG, TASK_SIZE, nullptr, TASK_DEFAULT_PRIORITY, &rule_engine_task_handle);       
}