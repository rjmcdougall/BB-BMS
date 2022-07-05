
#include "defines.h"
#include "HAL_ESP32.h"

#define USE_ESP_IDF_LOG 1
static constexpr const char *  TAG = "hal32";


bool HAL_ESP32::isConnected(i2c_port_t i2c_num, uint8_t dev)
{
    //We use the native i2c commands for ESP32 as the Arduino library
    //seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        //Select the correct register on the i2c device
        i2c_master_start(cmd);
        esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true));
        i2c_cmd_link_delete(cmd);
        Releasei2cMutex();
        if (ret != ESP_OK) {
            return false;
        } else {
            return true;
        }
    }
    return false;
}

esp_err_t HAL_ESP32::readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg, uint8_t *data)
{
    //We use the native i2c commands for ESP32 as the Arduino library
    //seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        //Select the correct register on the i2c device
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        // Send repeated start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
        //Read single byte and expect NACK in reply
        i2c_master_read_byte(cmd, data, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));

        //ESP_LOGD(TAG,"I2C reply %i",ret);

        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();
        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t HAL_ESP32::readMultipleBytes(i2c_port_t i2c_num, uint8_t dev, uint8_t reg, uint8_t data[], int num)
{
    //We use the native i2c commands for ESP32 as the Arduino library
    //seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        //Select the correct register on the i2c device
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        // Send repeated start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
        //Read two bytes and expect NACK in reply
        // i2c_master_read(i2c_cmd_handle_tcmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_tack)
        i2c_master_read(cmd, data, num - 1, i2c_ack_type_t::I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, data + num - 1, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

        ESP_LOGD(TAG,"I2C reply %i",ret);

        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();

        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t HAL_ESP32::directCommand(i2c_port_t i2c_num, uint8_t dev, uint8_t command, uint8_t *data)
{
    //We use the native i2c commands for ESP32 as the Arduino library
    //seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        //Select the correct register on the i2c device
        ESP_LOGD(TAG,"I2C directCommand %i", command);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, command, true);
        //i2c_master_stop(cmd);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
        //Read two bytes and expect NACK in reply
        //i2c_master_read(i2c_cmd_handle_tcmd_handle, uint8_t *data, size_t data_len, i2c_ack_type_tack)
        i2c_master_read_byte(cmd, &data[0], i2c_ack_type_t::I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[1], i2c_ack_type_t::I2C_MASTER_NACK);

        //i2c_master_read(cmd, data, num, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

        ESP_LOGD(TAG,"I2C directCommand ret code = %i",ret);

        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();

        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

//i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeByte(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint8_t data)
{
    if (Geti2cMutex())
    {
        //We use the native i2c commands for ESP32 as the Arduino library
        //seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, i2cregister, true);
        i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);

        esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));
        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();
        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

//i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeMultipleBytes(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint8_t data[], int num)
{
    if (Geti2cMutex())
    {
        //We use the native i2c commands for ESP32 as the Arduino library
        //seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, i2cregister, true);
        i2c_master_write(cmd, data,  num, true);
        i2c_master_stop(cmd);

        esp_err_t ret = ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));
        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();
        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

uint8_t HAL_ESP32::ReadTCA6408InputRegisters()
{
#ifdef RMC
    TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    return TCA6408_Value & TCA6408_INPUTMASK;
#endif
}

uint8_t HAL_ESP32::ReadTCA9534InputRegisters()
{
#ifdef RMC
    //TCA9534APWR_Value = readByte(i2c_port_t::I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_INPUT);
    //return TCA9534APWR_Value & TCA9534APWR_INPUTMASK;
#endif
}

void HAL_ESP32::SetOutputState(uint8_t outputId, RelayState state)
{
    ESP_LOGD(TAG, "SetOutputState %u=%u", outputId, state);
#ifdef RMC
    //Relays connected to TCA6408A
    //P4 = RELAY1 (outputId=0)
    //P5 = RELAY2 (outputId=1)
    //P6 = RELAY3_SSR (outputId=2)
    //P7 = EXT_IO_E (outputId=3)

    if (outputId <= 3)
    {
        //TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
        uint8_t bit = outputId + 4;
        TCA6408_Value = (state == RelayState::RELAY_ON) ? (TCA6408_Value | (1 << bit)) : (TCA6408_Value & ~(1 << bit));
        ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT, TCA6408_Value);
        ESP_LOGD(TAG, "TCA6408 reply %i", ret);
        //TODO: Check return value
        //TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    }
#endif 
}

void HAL_ESP32::Led(uint8_t bits)
{
    //Clear LED pins
    TCA9534APWR_Value = TCA9534APWR_Value & B11111000;
    //Set on
    TCA9534APWR_Value = TCA9534APWR_Value | (bits & B00000111);
    //esp_err_t ret =
    //ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value));

    //ESP_LOGD(TAG,"TCA9534 LED reply %i",ret);
    //TODO: Check return value
}

// Control Silent mode control input on TJA1051T/3
// True = enable CANBUS
void HAL_ESP32::CANBUSEnable(bool value)
{
    //Pin P5
    //Low = Normal mode
    //High = Silent
    TCA9534APWR_Value = TCA9534APWR_Value & B11011111;

    if (value == false)
    {
        //Set on
        TCA9534APWR_Value = TCA9534APWR_Value | B00100000;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, TCA9534APWR_Value));
}

// Control TFT backlight LED
void HAL_ESP32::TFTScreenBacklight(bool value)
{
  // M5
  //ledcWrite(M5_BLK_PWM_CHANNEL, value ? 100 : 0);
}

void HAL_ESP32::ConfigurePins(void (*WiFiPasswordResetInterrupt)(void))
{
    // M5: Init the back-light LED PWM
    //ledcSetup(M5_BLK_PWM_CHANNEL, 44100, 8);
    //ledcAttachPin(TFT_BL, M5_BLK_PWM_CHANNEL);

    //GPIO39 is interrupt pin from TCA6408 (doesnt have pull up/down resistors)
    //pinMode(TCA6408_INTERRUPT_PIN, INPUT);

    //GPIO34 is interrupt pin from TCA9534A (doesnt have pull up/down resistors)
    //pinMode(TCA9534A_INTERRUPT_PIN, INPUT);

    //dac_output_disable(DAC_CHANNEL_1);
    //pinMode(GPIO_NUM_14, OUTPUT);
    //gpio_reset_pin(GPIO_NUM_14);
    //gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLUP_ONLY);

    //BOOT Button on ESP32 module is used for resetting wifi details
    pinMode(GPIO_NUM_0, INPUT_PULLUP);
    attachInterrupt(GPIO_NUM_0, WiFiPasswordResetInterrupt, CHANGE);

    //For touch screen
    //pinMode(GPIO_NUM_36, INPUT_PULLUP);
    //attachInterrupt(GPIO_NUM_36, TFTScreenTouch, FALLING);

    // Baja Pins
    //pinMode(3, OUTPUT);
    //digitalWrite(BAJA_HEADLIGHT_PIN, HIGH);
    //pinMode(BAJA_TAILLIGHT_PIN, OUTPUT);
    //digitalWrite(BAJA_TAILLIGHT_PIN, HIGH);

    //Configure the CHIP SELECT pins as OUTPUT and set HIGH
    //pinMode(TOUCH_CHIPSELECT, OUTPUT);
    //digitalWrite(TOUCH_CHIPSELECT, HIGH);
    //pinMode(SDCARD_CHIPSELECT, OUTPUT);
    //digitalWrite(SDCARD_CHIPSELECT, HIGH);

    //pinMode(RS485_ENABLE, OUTPUT);
    //Enable receive
    //digitalWrite(RS485_ENABLE, LOW);
}

void HAL_ESP32::SwapGPIO0ToOutput()
{
    //BOOT Button on ESP32 module is used for resetting wifi details
    detachInterrupt(GPIO_NUM_0);
    pinMode(GPIO_NUM_0, OUTPUT);
    digitalWrite(GPIO_NUM_0, HIGH);
}


void HAL_ESP32::ConfigureI2C(void (*TCA6408Interrupt)(void), void (*TCA9534AInterrupt)(void))
{
    ESP_LOGI(TAG, "Configure I2C0");

    //SDA / SCL
    //ESP32 = I2C0-SDA / I2C0-SCL
    //I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    //I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);

    // Initialize
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    // RMC conf.sda_io_num = gpio_num_t::GPIO_NUM_27;
    // RMC conf.scl_io_num = gpio_num_t::GPIO_NUM_26;
    conf.sda_io_num = gpio_num_t::GPIO_NUM_21;
    conf.scl_io_num = gpio_num_t::GPIO_NUM_22;
    //conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    //conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);

    i2c_set_pin(I2C_NUM_0, gpio_num_t::GPIO_NUM_21, gpio_num_t::GPIO_NUM_22,  GPIO_PULLUP_ENABLE,GPIO_PULLUP_ENABLE, I2C_MODE_MASTER);

    ESP_LOGD(TAG,"I2C0 i2c_param_config %i",ret);
    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    ESP_LOGD(TAG,"I2C0 i2c_driver_install %i",ret);



    ESP_LOGI(TAG, "Configure I2C1");

    //SDA / SCL
    //ESP32 = I2C0-SDA / I2C0-SCL
    //I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    //I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);

    // Initialize
    //i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    // RMC conf.sda_io_num = gpio_num_t::GPIO_NUM_27;
    // RMC conf.scl_io_num = gpio_num_t::GPIO_NUM_26;
    conf.sda_io_num = gpio_num_t::GPIO_NUM_32;
    conf.scl_io_num = gpio_num_t::GPIO_NUM_33;
    //conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    //conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 20000;
    conf.clk_flags = 0;
    ret = i2c_param_config(I2C_NUM_1, &conf);

    i2c_set_pin(I2C_NUM_1, gpio_num_t::GPIO_NUM_32, gpio_num_t::GPIO_NUM_33,  GPIO_PULLUP_ENABLE,GPIO_PULLUP_ENABLE, I2C_MODE_MASTER);

    ESP_LOGD(TAG,"I2C1 i2c_param_config %i",ret);
    ret = i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);
    ESP_LOGD(TAG,"I2C1 i2c_driver_install %i",ret);


    

#ifdef RMC
    // https://datasheet.lcsc.com/szlcsc/1809041633_Texas-Instruments-TCA9534APWR_C206010.pdf
    // TCA9534APWR Remote 8-Bit I2C and Low-Power I/O Expander With Interrupt Output and Configuration Registers
    // https://lcsc.com/product-detail/Interface-I-O-Expanders_Texas-Instruments-TCA9534APWR_C206010.html
    // A0/A1/A2 are LOW, so i2c address is 0x38

    //PINS
    //P0= BLUE
    //P1= RED
    //P2= GREEN
    //P3= DISPLAY BACKLIGHT LED
    //P4= SPARE on J13
    //P5= Canbus RS
    //P6= SPARE on J13
    //P7= ESTOP (pull to ground to trigger)
    //INTERRUPT PIN = ESP32 IO34

    //BIT  76543210
    //PORT 76543210
    //MASK=10000000

    //All off
    esp_err_t ret = writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_OUTPUT, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TCA9534APWR Error");
        Halt(RGBLED::Purple);
    }

    //0×03 Configuration, P7 as input, others outputs (0=OUTPUT)
    ret = writeByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_CONFIGURATION, TCA9534APWR_INPUTMASK);

    //0×02 Polarity Inversion, zero = off
    //writeByte(TCA9534APWR_ADDRESS, TCA9534APWR_POLARITY_INVERSION, 0);
    //TCA9534APWR_Value = readByte(I2C_NUM_0, TCA9534APWR_ADDRESS, TCA9534APWR_INPUT);
    //SERIAL_DEBUG.println("Found TCA9534APWR");

    attachInterrupt(TCA9534A_INTERRUPT_PIN, TCA9534AInterrupt, FALLING);

    ESP_LOGI(TAG, "Found TCA9534A");

    /*
Now for the TCA6408
*/

    //P0=EXT_IO_A
    //P1=EXT_IO_B
    //P2=EXT_IO_C
    //P3=EXT_IO_D
    //P4=RELAY 1
    //P5=RELAY 2
    //P6=RELAY 3 (SSR)
    //P7=EXT_IO_E

    //Set ports to off before we set configuration
    ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_OUTPUT, 0);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "TCA6408 Error");
        Halt(RGBLED::Green);
    }

    //Ports A/B/C/D/E inputs, RELAY1/2/3 outputs
    ret = writeByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_CONFIGURATION, TCA6408_INPUTMASK);
    //ret =writeByte(i2c_port_t::I2C_NUM_0,TCA6408_ADDRESS, TCA6408_POLARITY_INVERSION, B00000000);
    //TCA6408_Value = readByte(I2C_NUM_0, TCA6408_ADDRESS, TCA6408_INPUT);
    //TODO: Validate if there was a read error or not.

    ESP_LOGI(TAG, "Found TCA6408");

    attachInterrupt(TCA6408_INTERRUPT_PIN, TCA6408Interrupt, FALLING);
#endif
}
