

#include "bq34z100.h"
static const char *TAG = "bq34z100";

// Library config
//#define DBG_BAUD            115200
//#define BQ_I2C_ADDR_WRITE   0x10
//#define BQ_I2C_ADDR_READ    0x11

// Inline functions
#define LOW_BYTE(data) (byte)(data & 0x00FF)
#define HIGH_BYTE(data) (byte)((data >> 8) & 0x00FF)

//// LOW LEVEL FUNCTIONS ////

void bq34z100::initBQ()
{
}

uint16_t bq34z100::read_register(uint8_t reg, uint8_t len)
{
    unsigned int ret;
    uint8_t tmp_value[2] = {0, 0};

    //ESP_LOGD(TAG,"read16 Send cmd %i", reg);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->directCommand(port, BQaddr, command, &value[0], 2));
    ret = BQhal->readByte(port, BQaddr, reg, &tmp_value[0]);
    if (ret != ESP_OK)
    {
        return false;
    }
    if (len == 2)
    {
        ret = BQhal->readByte(port, BQaddr, reg + 1, &tmp_value[1]);
        if (ret != ESP_OK)
        {
            return false;
        }
    }
    //ESP_LOGD(TAG,"read16 Reply %i", ret);
    return (tmp_value[0] | (tmp_value[1] << 8));
}

uint16_t bq34z100::read_control(uint8_t low, uint8_t high)
{
    ESP_LOGD(TAG, "readControl Send %x %x", low, high);
    uint8_t cmd[2];
    cmd[0] = low;
    cmd[1] = high;
    ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->writeMultipleBytes(port, BQaddr, 0, cmd, 2));

    uint8_t value[2];
    int16_t ret;

    ESP_LOGD(TAG, "readControl Send");
    ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->readMultipleBytes(port, BQaddr, 0, &value[0], 2));
    ret = value[0] | value[1] << 8;
    ESP_LOGD(TAG, "readControl Reply %i", ret);
    return ret;
}

bool bq34z100::isConnected(void)
{
    return BQhal->isConnected(port, BQaddr);
}

// Compute checksome = ~(sum of all bytes)
byte bq34z100::computeChecksum(byte oldChecksum, byte data)
{
    if (!oldChecksum)
        oldChecksum = data;
    else
        oldChecksum = ~(oldChecksum) + data;
    return ~(oldChecksum);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

/////// API FUNCTIONS ///////

bq34z100::bq34z100(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p)
{
    // Constructor
    BQhal = hal;
    BQaddr = addr;
    port = p;

    //pinMode(alertPin, INPUT);
    // TODO - Attach IRQ here
}

void bq34z100::begin(void)
{
    initBQ();
    ESP_LOGD(TAG, "Initializing BQ34z100...");
}

uint16_t bq34z100::control_status()
{
    return read_control(0x00, 0x00);
}

uint16_t bq34z100::device_type()
{
    return read_control(0x01, 0x00);
}

uint16_t bq34z100::fw_version()
{
    return read_control(0x02, 0x00);
}

uint16_t bq34z100::hw_version()
{
    return read_control(0x03, 0x00);
}

uint16_t bq34z100::reset_data()
{
    return read_control(0x05, 0x00);
}

uint16_t bq34z100::prev_macwrite()
{
    return read_control(0x07, 0x00);
}

uint16_t bq34z100::chem_id()
{
    return read_control(0x08, 0x00);
}

uint16_t bq34z100::board_offset()
{
    return read_control(0x09, 0x00);
}

uint16_t bq34z100::cc_offset()
{
    return read_control(0x0a, 0x00);
}

uint16_t bq34z100::cc_offset_save()
{
    return read_control(0x0b, 0x00);
}

uint16_t bq34z100::df_version()
{
    return read_control(0x0c, 0x00);
}

uint16_t bq34z100::set_fullsleep()
{
    return read_control(0x10, 0x00);
}

uint16_t bq34z100::static_chem_chksum()
{
    return read_control(0x17, 0x00);
}

uint16_t bq34z100::sealed()
{
    return read_control(0x20, 0x00);
}

uint16_t bq34z100::it_enable()
{
    return read_control(0x21, 0x00);
}

uint16_t bq34z100::cal_enable()
{
    return read_control(0x2d, 0x00);
}

uint16_t bq34z100::reset()
{
    return read_control(0x41, 0x00);
}

uint16_t bq34z100::exit_cal()
{
    return read_control(0x80, 0x00);
}

uint16_t bq34z100::enter_cal()
{
    return read_control(0x81, 0x00);
}

uint16_t bq34z100::offset_cal()
{
    return read_control(0x82, 0x00);
}

uint8_t bq34z100::state_of_charge()
{
    return (uint8_t)read_register(0x02, 1);
}

uint8_t bq34z100::state_of_charge_max_error()
{
    return (uint8_t)read_register(0x03, 1);
}

uint16_t bq34z100::remaining_capacity()
{
    return read_register(0x04, 2);
}

uint16_t bq34z100::full_charge_capacity()
{
    return read_register(0x06, 2);
}

uint16_t bq34z100::voltage()
{
    return read_register(0x08, 2);
}

int16_t bq34z100::average_current()
{
    return (int16_t)read_register(0x0a, 2);
}

// TODO: Change this to float and return temp in C
// raw / 10 - 273.15
uint16_t bq34z100::temperature()
{
    uint16_t raw = read_register(0x0c, 2) / 10;
    return (raw - 273);
}

uint16_t bq34z100::flags()
{
    return read_register(0x0e, 2);
}

uint16_t bq34z100::flags_b()
{
    return read_register(0x12, 2);
}

int16_t bq34z100::current()
{
    return (int16_t)read_register(0x10, 2);
}

uint16_t bq34z100::average_time_to_empty()
{
    return read_register(0x18, 2);
}

uint16_t bq34z100::average_time_to_full()
{
    return read_register(0x1a, 2);
}

int16_t bq34z100::passed_charge()
{
    return read_register(0x1c, 2);
}

uint16_t bq34z100::do_d0_time()
{
    return read_register(0x1e, 2);
}

uint16_t bq34z100::available_energy()
{
    return read_register(0x24, 2);
}

uint16_t bq34z100::average_power()
{
    return read_register(0x26, 2);
}

uint16_t bq34z100::serial_number()
{
    return read_register(0x28, 2);
}

uint16_t bq34z100::internal_temperature()
{
    //return read_register(0x2a, 2);
    uint16_t raw = read_register(0x2a, 2) / 10;
    return (raw - 273);
}

uint16_t bq34z100::cycle_count()
{
    return read_register(0x2c, 2);
}

uint16_t bq34z100::state_of_health()
{
    return read_register(0x2e, 2);
}

uint16_t bq34z100::charge_voltage()
{
    return read_register(0x30, 2);
}

uint16_t bq34z100::charge_current()
{
    return read_register(0x32, 2);
}

uint16_t bq34z100::pack_configuration()
{
    return read_register(0x3a, 2);
}

uint16_t bq34z100::design_capacity()
{
    return read_register(0x3c, 2);
}

uint8_t bq34z100::grid_number()
{
    return (uint8_t)read_register(0x62, 1);
}

uint8_t bq34z100::learned_status()
{
    return (uint8_t)read_register(0x63, 1);
}

uint16_t bq34z100::dod_at_eoc()
{
    return read_register(0x64, 2);
}

uint16_t bq34z100::q_start()
{
    return read_register(0x66, 2);
}

uint16_t bq34z100::true_fcc()
{
    return read_register(0x6a, 2);
}

uint16_t bq34z100::state_time()
{
    return read_register(0x6c, 2);
}

uint16_t bq34z100::q_max_passed_q()
{
    return read_register(0x6e, 2);
}

uint16_t bq34z100::dod_0()
{
    return read_register(0x70, 2);
}

uint16_t bq34z100::q_max_dod_0()
{
    return read_register(0x72, 2);
}

uint16_t bq34z100::q_max_time()
{
    return read_register(0x74, 2);
}

bool bq34z100::is_soc_full() {
    return BQ34Z100_FLAG_SOCF(flagsa_cache);
}

bool bq34z100::is_discharge() {
    return BQ34Z100_FLAG_DSC(flagsa_cache);
}

bool bq34z100::is_soc_1() {
    return BQ34Z100_FLAG_SOC1(flagsa_cache);
}

bool bq34z100::is_config_update() {
    return BQ34Z100_FLAG_CFGUP(flagsa_cache);
}

bool bq34z100::is_full_charge() {
    return BQ34Z100_FLAG_FC(flagsa_cache);
}

bool bq34z100::is_overtemp_discharge() {
    return BQ34Z100_FLAG_DSC(flagsa_cache);
}

bool bq34z100::is_overtemp_charge() {
    return BQ34Z100_FLAG_OTC(flagsa_cache);
}

bool bq34z100::is_undertemp() {
    return BQ34Z100_FLAG_UT(flagsa_cache);
}

bool bq34z100::is_overtemp() {
    return BQ34Z100_FLAG_OT(flagsa_cache);
}
