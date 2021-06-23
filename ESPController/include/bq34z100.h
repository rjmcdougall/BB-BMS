
#include "defines.h"
#include "HAL_ESP32.h"
#include "crc16.h"

class bq34z100 {
public:
	bq34z100(uint8_t addr, HAL_ESP32 *hal);
	void begin(void);
	bool isConnected(void);



    uint16_t control_status();
    uint16_t device_type();
    uint16_t fw_version();
    uint16_t hw_version();
    uint16_t reset_data();
    uint16_t prev_macwrite();
    uint16_t chem_id();
    uint16_t board_offset();
    uint16_t cc_offset();
    uint16_t cc_offset_save();
    uint16_t df_version();
    uint16_t set_fullsleep();
    uint16_t static_chem_chksum();
    uint16_t sealed();
    uint16_t it_enable();
    uint16_t cal_enable();
    uint16_t reset();
    uint16_t exit_cal();
    uint16_t enter_cal();
    uint16_t offset_cal();
    
    uint8_t state_of_charge(); // 0 to 100%
    uint8_t state_of_charge_max_error(); // 1 to 100%
    uint16_t remaining_capacity(); // mAh
    uint16_t full_charge_capacity(); // mAh
    uint16_t voltage(); // mV
    int16_t average_current(); // mA
    uint16_t temperature(); // Unit of x10 K
    uint16_t flags();
    uint16_t flags_b();
    int16_t current(); // mA
    
    uint16_t average_time_to_empty(); // Minutes
    uint16_t average_time_to_full(); // Minutes
    int16_t passed_charge(); // mAh
    uint16_t do_d0_time(); // Minutes
    uint16_t available_energy(); // 10 mWh
    uint16_t average_power(); // 10 mW
    uint16_t serial_number();
    uint16_t internal_temperature(); // Unit of x10 K
    uint16_t cycle_count(); // Counts
    uint16_t state_of_health(); // 0 to 100%
    uint16_t charge_voltage(); // mV
    uint16_t charge_current(); // mA
    uint16_t pack_configuration();
    uint16_t design_capacity(); // mAh
    uint8_t grid_number();
    uint8_t learned_status();
    uint16_t dod_at_eoc();
    uint16_t q_start(); // mAh
    uint16_t true_fcc(); // mAh
    uint16_t state_time(); // s
    uint16_t q_max_passed_q(); // mAh
    uint16_t dod_0();
    uint16_t q_max_dod_0();
    uint16_t q_max_time();



private:
	void initBQ(void);
    uint16_t read_register(uint8_t reg, uint8_t len);
    uint16_t read_control(uint8_t low, uint8_t high);
    byte computeChecksum(byte oldChecksum, uint8_t data);


    HAL_ESP32 *BQhal;
    uint8_t BQaddr;
};