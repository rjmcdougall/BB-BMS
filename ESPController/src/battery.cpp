#include "defines.h"
#include "battery.h"
#include <mutex>

// We can get this from dastatus, but if the cell config on the hardware
// is wrong, one of them will report 0, and therefor all the numbers will
// be off. This actively reads all voltages of known activey cells, and 
// keeps track of the min/max cell voltages that way.
#define FEATURE_USE_MANUAL_MIN_MAX_CELL_VOLTAGE_COUNT true

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
unsigned int battery::cell_voltage[BATTERY_MAX_CELL_COUNT] = {0};   // individual voltages
unsigned int battery::active_cells[BATTERY_MAX_CELL_COUNT] = {0};   // the slots of the active cells
unsigned int battery::active_cell_count = 0;                        // the amount of active cells

unsigned int battery::_min_cell_voltage = 0; // in milivolt
unsigned int battery::_max_cell_voltage = 0; // in milivolt    


TaskHandle_t battery::battery_task_handle = NULL;
hardware_interface *battery::hwi;

battery* battery::battery_{nullptr};
std::mutex battery::mutex_;

// Caches for multi-byte subcommands
// '64' is cargo culted over from the original code.
unsigned int _dastatus5_cache[64] = {0};

// Do we have anythign to report? Don't show garbage in status/calls
bool battery::_has_data = false;

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

    // Report on features
    ESP_LOGD(TAG, "Feature Manual Min/Max Cell Voltage Enabled: %s", FEATURE_USE_MANUAL_MIN_MAX_CELL_VOLTAGE_COUNT ? "true" : "false");
}

/********************************************************************
*
* Battery information methods
*
********************************************************************/

/* XXX NOTE: 
 * So, getting the active cells is a PITA via a direct command. See below this
 * comment the original code. Here's the issues:
 * 1) You have to do a write, then a read (so far so good)
 * 2) The subsequent read returns the response length as 36 bytes (this is wrong);
 *    it's 2 bytes, as per the spec:
 *    // From Table 13-37. Data Memory Table
 *    0x9304 Vcell Mode H2 0x0000 0xFFFF
 *    So if you run the 'standard' code to read all the bytes in the return value
 *    you end up with total gibberish. 
 * 3) The original code compensated by 'just' reading 2 bytes, because that is
 *    what the spec says, but would require another custom method to override 
 *    the response length based on the 'user knowing better' :(
 * 4) Add to that that the test hardware doesn't report its cells accurate, so it
 *    would require a manual override when using that.
 * 
 * So instead, we just probe all the cells up to MAX count, and see if they resport
 * any meaningful data once we have connected. If yes, they're active cells. If not,
 * they are not. This simplifies the code greatly.
 * 
 * If we feel like implementing the dynamic look up later, we can by just reimplementing
 * the function below. For now, I am happy with this method.

// this is for the test hardware. Set to false if we fix the hardware
// or run against actual batteries
#define BATTERY_HARDCODED_VCELL 0x1000000000000011  
#define CMD_VCELL_COUNT 0x9304

bool bq76952::readDataMemory(uint16_t addr, uint8_t *data)
{
    ESP_LOGD(TAG,"BQreadDataMemory Send Addr %i", addr);

    data[0] = LOW_BYTE(addr);
    data[1] = HIGH_BYTE(addr);

    esp_err_t ret = BQhal->writeMultipleBytes(port, BQaddr, CMD_DIR_SUBCMD_LOW, data, 2);
    if (ret != ESP_OK) {
      //ESP_LOGD(TAG, "writecmd failed");
      return false;
    }

    ESP_LOGD(TAG,"BQreadDataMemory Send Read Req %i", addr);
    ret = BQhal->readByte(port, BQaddr, CMD_DIR_RESP_START, data);
    ESP_LOGD(TAG,"BQreadDataMemory Reply %x %i", *data, ret);
    return (ret == ESP_OK);
}

// Read vcell mode
unsigned int bq76952::getVcellMode(void) {
  uint8_t vCellMode[2];
  if (!readDataMemory(0x9304, &vCellMode[0])) {
    ESP_LOGD(TAG,"Error reading vcellMode0");
    return 0;
  }
  if (!readDataMemory(0x9305, &vCellMode[1])) {
    ESP_LOGD(TAG,"Error reading vcellMode1");
    return 0;
  }
  return vCellMode[1] << 8 | vCellMode[0];
}
*/

unsigned int battery::get_active_cells(unsigned int *cells, unsigned int max) {
    int cell_index = 0;    
    // Cells start counting at index 1, not index 0, so offset by 1!
    for( int i = 1; i < max + 1; i++ ) {
        int voltage = this->get_cell_voltage(i);
        if(voltage >= BATTERY_MIN_CELL_VOLTAGE) {
            ESP_LOGD(TAG, "Found active cell: %i - %i mV", i, voltage);
            // Store that cell N is active in the next available slot;
            cells[cell_index++] = i;
        } else {
            ESP_LOGD(TAG, "Cell %i is not active: %i mV (< %i mV reported)", i, voltage, BATTERY_MIN_CELL_VOLTAGE);
        }
    }

    // The amount of active cells
    return cell_index;
}

unsigned int battery::get_active_cell_count(void) {
    return this->active_cell_count;
}


bool battery::is_connected(void) {
    return this->hwi->is_connected();
}

bool::battery::has_data(void) {
    return this->_has_data;
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

// XXX TODO: the 'minimum is being reported as 0 on the test hardware. This could be because
// it's measuring phantom cells. At which case, no problem, it'll just work on the real batteries.
// If it's not, we should switch this to keep track of the min and max reported from each cell,
// and update the status that way.
// millivolts
unsigned int battery::max_cell_voltage(void) {
    return FEATURE_USE_MANUAL_MIN_MAX_CELL_VOLTAGE_COUNT ? this->_max_cell_voltage : this->_read_dastatus5_cache(DASTATUS_MAXCELL);    
}
// millivolts
unsigned int battery::min_cell_voltage(void) {
    return FEATURE_USE_MANUAL_MIN_MAX_CELL_VOLTAGE_COUNT ? this->_min_cell_voltage : this->_read_dastatus5_cache(DASTATUS_MINCELL);    
}

// Stack & Pack Voltage
// XXX TODO: these are in 'userV' units, which aren't described :( what does this value mean?
// Sample: [161378][D][battery.cpp:310] battery_task(): [TAG] Stack Voltage: 1175 - Pack Voltage: 49
// XXX everything ELSE is in milivolts - so assume it's x1000?
// No, it's userV == milliV/10 (it's a 16bit issue). So multiple by
unsigned int battery::get_stack_voltage(void) {
    unsigned int voltage;
    if( battery_->hwi->direct_command(CMD_READ_VOLTAGE_STACK, &voltage) ) {
        return voltage * 10;
    } else {
        return 0;
    }        
}

unsigned int battery::get_pack_voltage(void) {
    unsigned int voltage;
    if( battery_->hwi->direct_command(CMD_READ_VOLTAGE_PACK, &voltage) ) {
        return voltage * 10;
    } else {
        return 0;
    }        
}

bool battery::_update_min_max_cell_voltage(int cell, unsigned int voltage) {
    ESP_LOGD(TAG, "Updating Min/Max Cell Voltage with Cell %i - %i mV", cell, voltage);

    bool change_detected = false;

    // Update min
    if( this->_min_cell_voltage == 0 || this->_min_cell_voltage > voltage ) {
        bool change_detected = true;
        this->_min_cell_voltage = voltage;
        ESP_LOGD(TAG, "Cell %i has new minimum voltage: %i mV", cell, voltage);
    }

    // Update max
    if( this->_max_cell_voltage == 0 || this->_max_cell_voltage < voltage ) {
        bool change_detected = true;
        this->_max_cell_voltage = voltage;
        ESP_LOGD(TAG, "Cell %i has new maximum voltage: %i mV", cell, voltage);
    }

    return change_detected;
};

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

        if( battery_->is_connected() ) {

            // First things first - let's get the active cell count on our battery:
            if( !battery_->active_cell_count ) {
                ESP_LOGD(TAG, "Getting active cell count");
                battery_->active_cell_count = battery_->get_active_cells(&battery_->active_cells[0], BATTERY_MAX_CELL_COUNT);
                ESP_LOGD(TAG, "Active cell count found: %i", battery_->active_cell_count);
            }

            // // XXX Can we just wrap this???
            if( DEBUG_TASKS ) {
                uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
                ESP_LOGD(TAG, "Current HWM/Stack = %i/%i - Connected: %s", uxHighWaterMark, TASK_SIZE, battery_->hwi->is_connected() ? "true" : "false");
            }

            for( int i = 0; i < battery_->active_cell_count; ++i ) {
                int cell_number = battery_->active_cells[i];
                // Get the voltage for this cell index
                unsigned int v = battery_->get_cell_voltage( cell_number );
                // And store it
                battery_->cell_voltage[i] = v;

                ESP_LOGD(TAG, "Voltage for cell %i: %i", cell_number, v);

                if( FEATURE_USE_MANUAL_MIN_MAX_CELL_VOLTAGE_COUNT ) {
                    battery_->_update_min_max_cell_voltage(cell_number, v);
                }
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

            
            battery_->_has_data = true;

            ESP_LOGD("TAG", "Task sleeping for: %i ms", TASK_INTERVAL);
            vTaskDelay( TASK_INTERVAL );
        } else {
            ESP_LOGD(TAG, "Battery is not connected!");
        }
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

