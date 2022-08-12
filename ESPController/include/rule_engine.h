#pragma once

#include "battery.h"
#include "pack.h"
#include <mutex>

/******************************************
 * Cargo culted from original rules file, so we can
 * reuse the HTML from the original code base
 ******************************************/

// XXX Cargo Culted. Note, to use legacy code:
// Needs to match the ordering on the HTML screen
#define RELAY_RULES 12
enum Rule : uint8_t
{
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

};

// //Define a max constant for the highest value (change if you add more warnings)
// #define MAXIMUM_InternalWarningCode 6
// enum InternalWarningCode : uint8_t
// {
//     NoWarning = 0,
//     ModuleInconsistantBypassVoltage = 1,
//     ModuleInconsistantBypassTemperature = 2,
//     ModuleInconsistantCodeVersion = 3,
//     ModuleInconsistantBoardRevision = 4,
//     LoggingEnabledNoSDCard = 5,
//     AVRProgrammingMode =6
// };

// //Define a max constant for the highest value (change if you add more errors)
// #define MAXIMUM_InternalErrorCode 7
// enum InternalErrorCode : uint8_t
// {
//     NoError = 0,
//     CommunicationsError = 1,
//     ModuleCountMismatch = 2,
//     TooManyModules = 3,
//     WaitingForModulesToReply = 4,
//     ZeroVoltModule = 5,
//     ControllerMemoryError = 6,
//     ErrorEmergencyStop = 7
// };

class rule_engine
{
public:
    void run(void);

    /**
     * Singletons should not be cloneable.
     */
    rule_engine(rule_engine &other) = delete;
    /**
     * Singletons should not be assignable.
     */
    void operator=(const rule_engine &) = delete;
    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */
    static rule_engine *GetInstance(pack *pack_obj, battery *battery_obj);
    
    /********************************************************************
     * rule_engine 
     ********************************************************************/
    bool is_hardware_connected(); 
    bool has_data(); 
    int max_rule_count();
    int get_rule_outcomes( bool *outcome );
    int get_all_rule_outcomes( bool *outcome );
    int get_active_error_count();
    bool is_battery_module_under_min_temp();
    bool is_battery_module_over_max_temp();
    bool is_battery_cell_under_min_temp();
    bool is_battery_cell_over_max_temp();
    bool is_battery_cell_under_min_voltage();
    bool is_battery_cell_over_max_voltage();
    bool is_battery_pack_under_min_voltage();
    bool is_battery_pack_over_max_voltage();

private:
    void init(void);
    static void rule_engine_task(void *param);
    void _run_all_rules();
    
    // Private variables
    static rule_engine *re_;
    static std::mutex mutex_;
    static std::mutex run_all_rules_mutex_;
    static TaskHandle_t rule_engine_task_handle;
    static pack *pack_;
    static battery *battery_;
    
    static const int _rule_count;
    static int _error_count;
    static int _error_rules[RELAY_RULES];
    static bool _rule_outcome[RELAY_RULES];
    static bool _has_data;

    /********************************************************************
     * rule_engine 
     ********************************************************************/    

protected:
    rule_engine(pack *pack_obj, battery *battery_obj);
    ~rule_engine();
};