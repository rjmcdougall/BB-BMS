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
 #define CMD_SOC                    0x02 // 1 byte!
 #define CMD_MAX_SOC_ERROR          0x03 // 1 byte!
 #define CMD_CAPACITY_REMAINING     0x04
 #define CMD_CAPACITY_FULL_CHARGE   0x06
 #define CMD_VOLTAGE                0x08
 #define CMD_AVERAGE_CURRENT        0x0A
 #define CMD_TEMPERATURE            0x0C
 #define CMD_FLAGS                  0x0E
 #define CMD_CURRENT                0x10
 #define CMD_FLAGSB                 0x12

/********************************************************************
 * Singleton class! Implemented using:
 * 
 * https://refactoring.guru/design-patterns/singleton/cpp/example#example-1
 * 
 ********************************************************************/


static const char *TAG = "pack";

static const int TASK_SIZE = TASK_STACK_SIZE_LARGE;
static const int TASK_INTERVAL = 1000; // ms


TaskHandle_t pack::pack_task_handle = NULL;
hardware_interface *pack::hwi;

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
