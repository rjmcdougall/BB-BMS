#include "defines.h"
#include "pack.h"
#include <mutex>

/********************************************************************
 * Technical reference / pack spec:
 * 
 * https://www.ti.com/lit/ug/sluubw5a/sluubw5a.pdf
 * 
 ********************************************************************/

/********************************************************************
 * The BQ34Z100-G1 uses a series of 2-byte standard commands to enable 
 * host reading and writing of battery information. Each standard command 
 * has an associated command-code pair, as indicated in Table 2-1. 
 * Because each command consists of two bytes of data, two consecutive 
 * HDQ/I2C transmissions must be executed to initiate the command 
 * function and to read or write the corresponding two bytes of data. 
 * Standard commands are accessible in NORMAL operation. Also, two block 
 * commands are available to read Manufacturer Name and Device Chemistry. 
 * Read/Write permissions depend on the active access mode.
 ********************************************************************/

/* XXX NOTE: StateOfCharge and MaxError are single byte commands, despite  
 * what the above says. They DO go hand in hand though, so you'll want both

Table 2-1. Commands
0x00/0x01 Control() CNTL
0x02 StateOfCharge() SOC
0x03 MaxError() ME 
0x04/0x05 RemainingCapacity() RM
0x06/0x07 FullChargeCapacity() FCC
0x08/0x09 Voltage() VOLT
0x0A/0x0B AverageCurrent() AI
0x0C/0x0D Temperature() TEMP
0x0E/0x0F Flags() FLAGS
0x10/0x11 Current() I
0x12/0x13 FlagsB() FLAGSB
 */
      
 #define CMD_CONTROl                0x00
 #define CMD_STATE_OF_CHARGE        0x02 // 1 byte!
 #define CMD_MAX_SOC_ERROR          0x03 // 1 byte!
 #define CMD_CAPACITY_REMAINING     0x04
 #define CMD_CAPACITY_FULL_CHARGE   0x06
 #define CMD_VOLTAGE                0x08
 #define CMD_AVERAGE_CURRENT        0x0A
 #define CMD_TEMPERATURE            0x0C
 #define CMD_FLAGS                  0x0E
 #define CMD_CURRENT                0x10
 #define CMD_FLAGSB                 0x12

 /* Note there are extended commands as well:
   Table 2-8. Extended Commands

StateOfHealth()    0x2E/0x2F    %
*/
#define CMD_STATE_OF_HEALTH     0x2E




/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/

// XXX FIXME - currenlty don't have actual return values from the hardware.
// use a bogus value so we can keep on going.
#define PACK_DEBUG_RETURN_BOGUS_VALUE true

static const char *TAG = "pack";

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 10000; // ms


TaskHandle_t pack::pack_task_handle = NULL;
hardware_interface *pack::hwi;
bool pack::_has_data = false;

pack* pack::pack_{nullptr};
std::mutex pack::mutex_;


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
 pack *pack::GetInstance(hardware_interface *bqz_hwi) {
    std::lock_guard<std::mutex> lock(pack::mutex_);
    
    // No instance yet - create one
    if( pack_ == nullptr ) {
        ESP_LOGD(TAG, "Creating new instance");
        
        pack_ = new pack(bqz_hwi);
        pack_->init();

    } else {
        ESP_LOGD(TAG, "Returning existing instance");
    }

    return pack_;
}

// Private
pack::pack(hardware_interface *bqz_hwi) {
    hwi = bqz_hwi;
}

void pack::init(void) {
    ESP_LOGD(TAG, "pack connected: %s", this->hwi->is_connected() ? "true" : "false");

    unsigned int value;
    if( this->hwi->direct_command(CMD_VOLTAGE, &value) ) {
        ESP_LOGD(TAG, "pack voltage: %i", value);

    }
}

/********************************************************************
*
* Pack information methods
*
********************************************************************/

/* From the original:
[ 21125][D][main.cpp:681] i2c_task(): [diybms] bqz soc = 23.522499
[ 21133][D][main.cpp:682] i2c_task(): [diybms] bqz soh = 0.000000
[ 21139][D][main.cpp:683] i2c_task(): [diybms] bqz voltage = 0.000000
[ 21146][D][main.cpp:685] i2c_task(): [diybms] bqz remainingCapacityAh = 0.000000
*/

bool pack::is_connected(void) {
    return this->hwi->is_connected();
}

bool::pack::has_data(void) {
    return this->_has_data;
}

// percentage
unsigned int pack::state_of_charge(void) {
    unsigned int value;
    if( this->hwi->read(CMD_STATE_OF_CHARGE, &value) ) {
        return value;
    } else {
        return PACK_DEBUG_RETURN_BOGUS_VALUE ? 21 : 0;
    }        
}

// percentage
unsigned int pack::state_of_health(void) {
    unsigned int value;
    if( this->hwi->read16(CMD_STATE_OF_HEALTH, &value) ) {
        return value;
    } else {
        return PACK_DEBUG_RETURN_BOGUS_VALUE ? 42 : 0;
    }        
}

// amp hours - 40,000 is a 'real number'. The code will return
// 4,000 becuase of 16 bit limtis - so multiply by 10
unsigned int pack::capacity_remaining(void) {
    unsigned int value;
    if( this->hwi->read16(CMD_CAPACITY_REMAINING, &value) ) {
        return value * 10;
    } else {
        return PACK_DEBUG_RETURN_BOGUS_VALUE ? 40000 : 0;
    }        
}

/********************************************************************
*
* Task Runner
*
********************************************************************/

void pack::pack_task(void *param) {
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
            ESP_LOGD(TAG, "Current HWM/Stack = %i/%i - Connected: %s", uxHighWaterMark, TASK_SIZE, pack_->is_connected() ? "true" : "false");
        }
    
        ESP_LOGD(TAG, "State of Charge: %i%%", pack_->state_of_charge());
        ESP_LOGD(TAG, "State of Health: %i%%", pack_->state_of_health());
        ESP_LOGD(TAG, "State of Capacity Remaining: %i aH", pack_->capacity_remaining());

        pack_->_has_data = true;

        ESP_LOGD("TAG", "Task sleeping for: %i ms", TASK_INTERVAL);
        vTaskDelay( TASK_INTERVAL );
    }        
}

void pack::run(void) {
    // How tasks work: https://www.freertos.org/a00125.html
    xTaskCreate(pack::pack_task, TAG, TASK_SIZE, nullptr, TASK_DEFAULT_PRIORITY, &pack_task_handle);       
}