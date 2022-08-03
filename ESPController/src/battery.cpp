#include "defines.h"
#include "battery.h"
#include <mutex>

/********************************************************************
 * Technical reference / battery spec:
 * 
 * https://www.ti.com/lit/ug/sluuby2b/sluuby2b.pdf
 * 
 ********************************************************************/

/********************************************************************
 * The BQ76952 device includes support for direct commands and subcommands. 
 * The direct commands are accessed using a 7-bit command address that is 
 * sent from a host through the device serial communications interface and 
 * either triggers an action, or provides a data value to be written to the 
 * device, or instructs the device to report data back to the host.
 ********************************************************************/

// Table 12-1. Direct Commands Table (incomplete)
#define CMD_DIR_SPROTEC           0x02
#define CMD_DIR_FPROTEC           0x03
#define CMD_DIR_STEMP             0x04
#define CMD_DIR_FTEMP             0x05
#define CMD_DIR_SFET              0x06
#define CMD_DIR_FFET              0x07
#define CMD_DIR_VCELL_1           0x14
#define CMD_DIR_INT_TEMP          0x68
#define CMD_DIR_CC2_CUR           0x3A
#define CMD_DIR_FET_STAT          0x7F

// Voltage direct commands
#define CMD_READ_VOLTAGE_STACK    0x34
#define CMD_READ_VOLTAGE_PACK     0x36


/********************************************************************
 * Direct Command CMD_DIR_FET_STAT returns a register with states
 * turned on or off. Below is the table:

Table 12-21. FET Status Register Field Descriptions
Bit Field Description
6 ALRT_PIN Indicates the status of the ALERT pin.
0 = The ALERT pin is not asserted.
1 = The ALERT pin is asserted.
5 DDSG_PIN Indicates the status of the DDSG pin.
0 = The DDSG pin is not asserted.
1 = The DDSG pin is asserted.
4 DCHG_PIN Indicates the status of the DCHG pin.
0 = The DCHG pin is not asserted.
1 = The DCHG pin is asserted.
3 PDSG_FET Indicates the status of the PDSG FET.
0 = The PDSG FET is off.
1 = The PDSG FET is on.
2 DSG_FET Indicates the status of the DSG FET.
0 = The DSG FET is off.
1 = The DSG FET is on.
1 PCHG_FET Indicates the status of the PCHG FET.
0 = The PCHG FET is off.
1 = The PCHG FET is on.
0 CHG_FET Indicates the status of the CHG FET.
0 = The CHG FET is off.
1 = The CHG FET is on.
********************************************************************/

#define BATTERY_CHARGING_BIT 0x01
#define BATTERY_DISCHARGING_BIT 0x04

/********************************************************************
 * We currently use one SubCommand, DASTATUS5, which gives details on 
 * current and temperature in our battery cells. 
 * 
 * Table 4-5. 0x0075 DASTATUS5() Subcommand Detail
********************************************************************/
#define CMD_DASTATUS5 0x0075
#define DASTATUS_VREG18 0
#define DASTATUS_VSS 2
#define DASTATUS_MAXCELL 4
#define DASTATUS_MINCELL 6
#define DASTATUS_BATSUM 8
#define DASTATUS_TEMPCELLAVG 10
#define DASTATUS_TEMPFET 12
#define DASTATUS_TEMPCELLMAX 14
#define DASTATUS_TEMPCELLMIN 16
#define DASTATUS_TEMCELLLAVG 18
#define DASTATUS_CURRENTCC3 20
#define DASTATUS_CURRENTCC1 22
#define DASTATUS_RAWCC2_COUNTS 24
#define DASTATUS_RAWCC3_COUNTS 28

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/

// Inline functions
// Table 4-1. Commands to Read 16-Bit Voltage Measurements
// The offset for cell addresses is 0x14, as per the table above ^
#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))

static const char *TAG = "battery";

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 10000; // ms
static const float KELVIN_TO_CELSIUS = 273.15;


/*
diybms_eeprom_settings mysettings;
uint16_t ConfigHasChanged = 0;
uint16_t TotalNumberOfCells() { return mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules; }
*/
static const int CELL_COUNT = 4; // XXX get this from config or elsewhere
static unsigned int cell_voltage[CELL_COUNT];


TaskHandle_t battery::battery_task_handle = NULL;
hardware_interface *battery::hwi;

battery* battery::battery_{nullptr};
std::mutex battery::mutex_;

// Caches for multi-byte subcommands
// '64' is cargo culted over from the original code.
unsigned int _dastatus5_cache[64] = {0};

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
 battery *battery::GetInstance(hardware_interface *bq_hwi) {
    std::lock_guard<std::mutex> lock(battery::mutex_);
    
    // No instance yet - create one
    if( battery_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        battery_ = new battery(bq_hwi);
        battery_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return battery_;
}

// Private
battery::battery(hardware_interface *bq_hwi) {
    hwi = bq_hwi;
}

void battery::init(void) {
    ESP_LOGD(TAG, "Battery connected: %s", this->hwi->is_connected() ? "true" : "false");
}

/********************************************************************
*
* Battery information methods
*
********************************************************************/

bool battery::is_connected(void) {
    return this->hwi->is_connected();
}

// Read single cell voltage - in mV
unsigned int battery::get_cell_voltage(byte cellNumber) {
    unsigned int value;
    if (battery_->hwi->direct_command(CELL_NO_TO_ADDR(cellNumber), &value)) {
        return value;
    } else {
        return 0;
    }
}

// Measure chip temperature in °C
float battery::get_internal_temp(void) {
    unsigned int value;
    // This is returned in mKelvin: 
    if (battery_->hwi->direct_command(CMD_DIR_INT_TEMP, &value)) {
        return milli_kelvin_to_c(value);
    } else {
        return 0;
    }
}

/* FET Status Methods */
bool battery::_fet_status(int reg){
    unsigned int value;
    if (battery_->hwi->direct_command(CMD_DIR_FET_STAT, &value)) {
        ESP_LOGD(TAG, "FET Status: %i - Checking for bit set: %i", value, reg);
        return (value & reg);
    } else {
        ESP_LOGE(TAG, "FAILED to get FET Status");
        return 0;
    }
}

// is Charging FET ON?
bool battery::is_charging(void) {
    if( battery_->_fet_status(BATTERY_CHARGING_BIT) ) {
        ESP_LOGD(TAG, "Battery Charging: True");
        return true;
    }                        
    return false;
}    

// is Discharging FET ON?
bool battery::is_discharging(void) {
    if( battery_->_fet_status(BATTERY_DISCHARGING_BIT) ) {
        ESP_LOGD(TAG, "Battery Discharging: True");
        return true;
    }                                               
    return false;
}    

// Individual cell data is returned by dastatus5

bool battery::_update_dastatus5_cache(void){
    this->hwi->sub_command(CMD_DASTATUS5);
    return this->hwi->read_sub_command_response_block(&_dastatus5_cache[0]);
}

unsigned int battery::_read_dastatus5_cache(unsigned int cmd) {
    return (_dastatus5_cache[cmd] | (_dastatus5_cache[cmd + 1] << 8));
}

float battery::max_cell_temp(void) {
    return this->milli_kelvin_to_c(this->_read_dastatus5_cache(DASTATUS_TEMPCELLMAX));
}
float battery::min_cell_temp(void) {
    return this->milli_kelvin_to_c(this->_read_dastatus5_cache(DASTATUS_TEMPCELLMIN));
}

// millivolts
unsigned int battery::max_cell_voltage(void) {
    return this->_read_dastatus5_cache(DASTATUS_MAXCELL);    
}
// millivolts
unsigned int battery::min_cell_voltage(void) {
    return this->_read_dastatus5_cache(DASTATUS_MINCELL);    
}

// Stack & Pack Voltage
// XXX TODO: these are in 'userV' units, which aren't described :( what does this value mean?
// Sample: [161378][D][battery.cpp:310] battery_task(): [TAG] Stack Voltage: 1175 - Pack Voltage: 49
// XXX everything ELSE is in milivolts - so assume it's x1000?
unsigned int battery::get_stack_voltage(void) {
    unsigned int voltage;
    if( battery_->hwi->direct_command(CMD_READ_VOLTAGE_STACK, &voltage) ) {
        return voltage * 1000;
    } else {
        return 0;
    }        
}

unsigned int battery::get_pack_voltage(void) {
    unsigned int voltage;
    if( battery_->hwi->direct_command(CMD_READ_VOLTAGE_PACK, &voltage) ) {
        return voltage * 1000;
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


        // // XXX Can we just wrap this???
        if( DEBUG_TASKS ) {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGD(TAG, "Current HWM/Stack = %i/%i - Connected: %s", uxHighWaterMark, TASK_SIZE, battery_->hwi->is_connected() ? "true" : "false");
        }

        for( int i = 0; i < CELL_COUNT; ++i ) {
            unsigned int v = battery_->get_cell_voltage(i);
            cell_voltage[i] = v;

            ESP_LOGD(TAG, "Voltage for cell %i: %i", i, v);
        }

        // XXX TODO: these are in 'userV' units, which aren't described :( what does this value mean?
        // Sample: [161378][D][battery.cpp:310] battery_task(): [TAG] Stack Voltage: 1175 - Pack Voltage: 1073432620
        ESP_LOGD(TAG, "Stack Voltage: %i - Pack Voltage: %i", battery_->get_stack_voltage(), battery_->get_pack_voltage());
        
        bool charging = battery_->is_charging();
        bool discharging = battery_->is_discharging();
        ESP_LOGD(TAG, "Charging: %i - Discharging: %i", charging, discharging);

        float temp_in_c = battery_->get_internal_temp();
        ESP_LOGD(TAG, "Internal board temp: %g", temp_in_c);

        battery_->_update_dastatus5_cache();
        ESP_LOGD(TAG, "Battery - Min Temp: %g - Max Temp: %g", battery_->min_cell_temp(), battery_->max_cell_temp());
        ESP_LOGD(TAG, "Battery - Min mVolt: %i - Max mVolt: %i", battery_->min_cell_voltage(), battery_->max_cell_voltage());

        ESP_LOGD("TAG", "Task sleeping for: %i ms", TASK_INTERVAL);

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

float battery::milli_kelvin_to_c(float mk) {
    float kelvin = mk / 10.0;
    return (kelvin - KELVIN_TO_CELSIUS);
}

