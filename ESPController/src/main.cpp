/*

 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017 to 2021 Stuart Pittaway

  This is the code for the controller - it talks to the V4.X cell modules over isolated serial bus

  This code runs on ESP32 DEVKIT-C and compiles with VS CODE and PLATFORM IO environment
*/

#if defined(ESP8266)
#error ESP8266 is not supported by this code
#endif

#undef CONFIG_DISABLE_HAL_LOCKS

static const char *TAG = "diybms";

#include "esp_log.h"
#include <Arduino.h>

//#define PACKET_LOGGING_RECEIVE
//#define PACKET_LOGGING_SEND
//#define RULES_LOGGING
//#define MQTT_LOGGING



#include "FS.h"
#include <LITTLEFS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include "time.h"
#include <esp_wifi.h>
#include <Preferences.h>
#include "tft_splash_image.h"

// Libraries for SD card
#include "SD.h"
#include "driver/gpio.h"
//#include "driver/twai.h"

#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <ArduinoOTA.h>
#include <SerialEncoder.h>
#include <cppQueue.h>
#include <XPT2046_Touchscreen.h>
#include <TelnetStream.h>

#include "defines.h"
#include "HAL_ESP32.h"
#include "bq76952.h"
#include "bq34z100.h"
#include "bms_can.h"

//SDM sdm(SERIAL_RS485, 9600, RS485_ENABLE, SERIAL_8N1, RS485_RX, RS485_TX); // pins for DIYBMS => RX pin 21, TX pin 22

#include "Rules.h"

#include "avrisp_programmer.h"

#include "tft.h"


#define BQ_ADDR 0x8
#define BQZ_ADDR 0x55


HAL_ESP32 hal;

bq76952 bq = bq76952(BQ_ADDR, &hal);
bq34z100 bqz = bq34z100(BQZ_ADDR, &hal);

bms_can *can = new bms_can(&mysettings, &cmi[0], &pi);

#include "cellular_modem.h"

cellular_modem cell_modem = cellular_modem();

XPT2046_Touchscreen touchscreen(TOUCH_CHIPSELECT, TOUCH_IRQ); // Param 2 - Touch IRQ Pin - interrupt enabled polling

volatile bool emergencyStop = false;

extern bool _tft_screen_available;

Rules rules;

bool _sd_card_installed = false;

diybms_eeprom_settings mysettings;
uint16_t ConfigHasChanged = 0;

uint16_t TotalNumberOfCells() { return mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules; }

bool server_running = false;
RelayState previousRelayState[RELAY_TOTAL];
bool previousRelayPulse[RELAY_TOTAL];

volatile enumInputState InputState[INPUTS_TOTAL];

AsyncWebServer server(80);


int LOG_TO_TELNET(const char *fmt, va_list args)
{
  char buffer[256];
  int res = vsprintf(buffer, fmt, args);
  TelnetStream.println("log");
  TelnetStream.println(buffer);
}

void LED(uint8_t bits)
{
  hal.Led(bits);
}

QueueHandle_t queue_i2c = NULL;

TaskHandle_t i2c_task_handle = NULL;
TaskHandle_t ledoff_task_handle = NULL;
TaskHandle_t wifiresetdisable_task_handle = NULL;
TaskHandle_t sdcardlog_task_handle = NULL;
TaskHandle_t sdcardlog_outputs_task_handle = NULL;
TaskHandle_t avrprog_task_handle = NULL;
TaskHandle_t mqtt1_task_handle = NULL;
TaskHandle_t mqtt2_task_handle = NULL;
TaskHandle_t enqueue_task_handle = NULL;
TaskHandle_t transmit_task_handle = NULL;
TaskHandle_t replyqueue_task_handle = NULL;
TaskHandle_t lazy_task_handle = NULL;
TaskHandle_t rule_task_handle = NULL;
TaskHandle_t influxdb_task_handle = NULL;
TaskHandle_t pulse_relay_off_task_handle = NULL;
TaskHandle_t cell_modem_task_handle = NULL;
TaskHandle_t updatetftdisplay_task_handle = NULL;
TaskHandle_t tftsleep_task_handle = NULL;
TaskHandle_t tftwakeup_task_handle = NULL;

//This large array holds all the information about the modules
//up to 4x16
CellModuleInfo cmi[maximum_controller_cell_modules];
PackInfo pi;

avrprogramsettings _avrsettings;

#include "crc16.h"

#include "settings.h"
#include "SoftAP.h"
#include "DIYBMSServer.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

// Instantiate queue to hold packets ready for transmission
cppQueue requestQueue(sizeof(PacketStruct), 24, FIFO);

cppQueue replyQueue(sizeof(PacketStruct), 8, FIFO);

PacketRequestGenerator prg = PacketRequestGenerator(&requestQueue);

PacketReceiveProcessor receiveProc = PacketReceiveProcessor();

// Memory to hold in and out serial buffer
uint8_t SerialPacketReceiveBuffer[2 * sizeof(PacketStruct)];

SerialEncoder myPacketSerial;

uint16_t sequence = 0;

ControllerState _controller_state = ControllerState::Unknown;

AsyncMqttClient mqttClient;

void QueueLED(uint8_t bits)
{
  i2cQueueMessage m;
  //3 = LED
  m.command = 0x03;
  //Lowest 3 bits are RGB led GREEN/RED/BLUE
  m.data = bits & B00000111;
  xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
}

void QueueReadCells()
{
  i2cQueueMessage m;
  m.command = 0x80;
  xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);
}

//Very wasteful of 8K of precious memory, AVR Programmer won't be used much
//so move to malloc ?
//uint8_t program[8192];

void avrprog_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //TODO: This needs to be passed into this as a parameter
    avrprogramsettings *s;
    s = (avrprogramsettings *)param;

    //memset(program, 0, sizeof(program));

    ESP_LOGI(TAG, "AVR setting e=%02X h=%02X l=%02X mcu=%08X file=%s", s->efuse, s->hfuse, s->lfuse, s->mcu, s->filename);

    //Now we load the file into program array, from LITTLEFS (SPIFF)
    if (LITTLEFS.exists(s->filename))
    {
      File binaryfile = LITTLEFS.open(s->filename);

      //void *blob = pvPortMalloc(binaryfile.size());
      //vPortFree(blob);

      s->programsize = binaryfile.size();

      //s->programsize = binaryfile.readBytes((char *)program, sizeof(program));

      //ESP_LOGD(TAG, "Read %i bytes", s->programsize);

      //Reserve the SPI bus for programming purposes
      if (hal.GetVSPIMutex())
      {
        hal.SwapGPIO0ToOutput();

        //This will block for the 6 seconds it takes to program ATTINY841...
        //although AVRISP_PROGRAMMER will call the watchdog to prevent reboots

        uint32_t starttime = millis();
        AVRISP_PROGRAMMER isp = AVRISP_PROGRAMMER(&(hal.vspi), GPIO_NUM_0, false, VSPI_SCK);

        ESP_LOGI(TAG, "Programming AVR");
        //This would be much better using a stream instead of a in ram buffer
        s->progresult = isp.ProgramAVRDevice(s->mcu, s->programsize, binaryfile, s->lfuse, s->hfuse, s->efuse);

        s->duration = millis() - starttime;

        hal.ConfigureVSPI();
        hal.ReleaseVSPIMutex();

        if (s->progresult == AVRISP_PROGRAMMER_RESULT::SUCCESS)
        {
          //sprintf(message, "Programming complete, duration %ums, %i bytes", s->duration, programsize);
          ESP_LOGI(TAG, "Success");
        }
        else
        {
          //sprintf(message, "Programming failed, reason %i", (int)progresult);
          ESP_LOGE(TAG, "Failed %i", s->progresult);
        }

        binaryfile.close();
      }
      else
      {
        ESP_LOGE(TAG, "Unable to obtain Mutex");
      }
    }
    else
    {
      ESP_LOGE(TAG, "AVR file not found %s", s->filename);
    }

    s->inProgress = false;

  } //end for
}

//Output a status log to the SD Card in CSV format
void sdcardlog_task(void *param)
{
  for (;;)
  {
    //Wait X seconds
    for (size_t i = 0; i < mysettings.loggingFrequencySeconds; i++)
    {
      //Delay 1 second
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (_sd_card_installed && !_avrsettings.programmingModeEnabled && mysettings.loggingEnabled && _controller_state == ControllerState::Running && hal.IsVSPIMutexAvailable())
    {
      //ESP_LOGD(TAG, "sdcardlog_task");

      struct tm timeinfo;
      //getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        //Month is 0 to 11 based!
        timeinfo.tm_mon++;

        //ESP_LOGD(TAG, "%04u-%02u-%02u %02u:%02u:%02u", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        char filename[32];
        sprintf(filename, "/data_%04u%02u%02u.csv", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday);

        File file;

        //Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {
          if (SD.exists(filename))
          {
            //Open existing file (assumes there is enough SD card space to log)
            file = SD.open(filename, FILE_APPEND);
            //ESP_LOGD(TAG, "Open log %s", filename);
          }
          else
          {
            //Create a new file
            uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

            //Ensure there is more than 25MB of free space on SD card before creating a file
            if (freeSpace > (uint64_t)(25 * 1024 * 1024))
            {
              //Create the file
              File file = SD.open(filename, FILE_WRITE);
              if (file)
              {
                ESP_LOGI(TAG, "Create log %s", filename);

                file.print("DateTime,");

                for (uint8_t i = 0; i < TotalNumberOfCells(); i++)
                {
                  file.print("VoltagemV_");
                  file.print(i);
                  file.print(",InternalTemp_");
                  file.print(i);
                  file.print(",ExternalTemp_");
                  file.print(i);
                  file.print(",Bypass_");
                  file.print(i);
                  file.print(",PWM_");
                  file.print(i);
                  file.print(",BypassOverTemp_");
                  file.print(i);
                  file.print(",BadPackets_");
                  file.print(i);
                  file.print(",BalancemAh_");
                  file.print(i);

                  if (i < TotalNumberOfCells() - 1)
                  {
                    file.print(',');
                  }
                }
                file.println();
              }
            }
            else
            {
              ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
              //We had an error, so switch off logging (this is only in memory so not written perm.)
              mysettings.loggingEnabled = false;
            }
          }

          if (file && mysettings.loggingEnabled)
          {
            char dataMessage[255];

            sprintf(dataMessage, "%04u-%02u-%02u %02u:%02u:%02u,", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            file.print(dataMessage);

            for (uint8_t i = 0; i < TotalNumberOfCells(); i++)
            {
              //This may output invalid data when controller is first powered up
              sprintf(dataMessage, "%u,%i,%i,%c,%u,%c,%u,%u",
                      cmi[i].voltagemV, cmi[i].internalTemp,
                      cmi[i].externalTemp, cmi[i].inBypass ? 'Y' : 'N',
                      (int)((float)cmi[i].PWMValue / (float)255.0 * 100), cmi[i].bypassOverTemp ? 'Y' : 'N',
                      cmi[i].badPacketCount, cmi[i].BalanceCurrentCount);
              file.print(dataMessage);
              if (i < TotalNumberOfCells() - 1)
              {
                file.print(',');
              }
            }
            file.println();
            file.close();

            ESP_LOGD(TAG, "Wrote to SD log");
          }
          else
          {
            ESP_LOGE(TAG, "Failed to create/append SD logging file");
            //We had an error opening the file, so switch off logging
            //mysettings.loggingEnabled = false;
          }
        }
        else
        {
          ESP_LOGE(TAG, "Invalid datetime");
        }

        //Must be the last thing...
        hal.ReleaseVSPIMutex();
      }
    }
  } //end for loop

  //vTaskDelete( NULL );
}

//Writes a status log of the OUTPUT STATUES to the SD Card in CSV format
void sdcardlog_outputs_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (_sd_card_installed && !_avrsettings.programmingModeEnabled && mysettings.loggingEnabled)
    {
      ESP_LOGD(TAG, "sdcardlog_outputs_task");

      struct tm timeinfo;
      //getLocalTime has delay() functions in it :-(
      if (getLocalTime(&timeinfo, 1))
      {
        timeinfo.tm_year += 1900;
        //Month is 0 to 11 based!
        timeinfo.tm_mon++;

        //ESP_LOGD(TAG, "%04u-%02u-%02u %02u:%02u:%02u", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        char filename[32];
        sprintf(filename, "/output_status_%04u%02u%02u.csv", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday);

        File file;

        //Prevent other devices using the VSPI bus
        if (hal.GetVSPIMutex())
        {

          if (SD.exists(filename))
          {
            //Open existing file (assumes there is enough SD card space to log)
            file = SD.open(filename, FILE_APPEND);

            //ESP_LOGD(TAG, "Open log %s", filename);
          }
          else
          {
            //Create a new file
            uint64_t freeSpace = SD.totalBytes() - SD.usedBytes();

            //Ensure there is more than 25MB of free space on SD card before creating a file
            if (freeSpace > (uint64_t)(25 * 1024 * 1024))
            {
              //Create the file
              File file = SD.open(filename, FILE_WRITE);
              if (file)
              {
                //ESP_LOGD(TAG, "Create log %s", filename);

                file.print("DateTime,Bits,");

                for (uint8_t i = 0; i < RELAY_TOTAL; i++)
                {
                  file.print("Output_");
                  file.print(i);
                  if (i < RELAY_TOTAL - 1)
                  {
                    file.print(',');
                  }
                }
                file.println();
              }
            }
            else
            {
              ESP_LOGE(TAG, "SD card has less than 25MiB remaining, logging stopped");
              //We had an error, so switch off logging
              //mysettings.loggingEnabled = false;
            }
          }

          if (file && mysettings.loggingEnabled)
          {
            char dataMessage[255];

            sprintf(dataMessage, "%04u-%02u-%02u %02u:%02u:%02u,", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            file.print(dataMessage);
            file.print(hal.LastTCA6408Value(), BIN);
            file.print(',');

            for (uint8_t i = 0; i < RELAY_TOTAL; i++)
            {
              //This may output invalid data when controller is first powered up
              sprintf(dataMessage, "%c", previousRelayState[i] == RelayState::RELAY_ON ? 'Y' : 'N');
              file.print(dataMessage);
              if (i < RELAY_TOTAL - 1)
              {
                file.print(',');
              }
            }
            file.println();
            file.close();

            ESP_LOGD(TAG, "Wrote to SD log");
          }
          else
          {
            ESP_LOGE(TAG, "Failed to create/append SD logging file");
            //We had an error opening the file, so switch off logging
            //mysettings.loggingEnabled = false;
          }
        }
        else
        {
          ESP_LOGE(TAG, "Invalid datetime");
        }

        //Must be the last thing...
        hal.ReleaseVSPIMutex();
      } //end if
    }   //end if
  }     //end for loop

  //vTaskDelete( NULL );
}

//Disable the BOOT button from acting as a WIFI RESET
//button which clears the EEPROM settings for WIFI connection
void wifiresetdisable_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //Wait for 20 seconds before disabling button/pin
    for (size_t i = 0; i < 20; i++)
    {
      //Wait 1 second
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    hal.SwapGPIO0ToOutput();
  }

  //vTaskDelete( NULL );
}

void ledoff_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered https://www.freertos.org/ulTaskNotifyTake.html
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //Wait 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //LED OFF
    QueueLED(RGBLED::OFF);
  }
}

// Handle all calls to i2c devices in this single task
// Provides thread safe mechanism to talk to i2c
void i2c_task(void *param)
{
  for (;;)
  {
    i2cQueueMessage m;

    if (xQueueReceive(queue_i2c, &m, portMAX_DELAY) == pdPASS)
    {
#ifdef RMC
      // do some i2c task
      if (m.command == 0x01)
      {
        // Read ports A/B/C/D inputs (on TCA6408)
        uint8_t v = hal.ReadTCA6408InputRegisters();
        //P0=A
        InputState[0] = (v & B00000001) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P1=B
        InputState[1] = (v & B00000010) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P2=C
        InputState[2] = (v & B00000100) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P3=D
        InputState[3] = (v & B00001000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P7=E
        InputState[4] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
      }

      if (m.command == 0x02)
      {
        //Read ports
        //The 9534 deals with internal LED outputs and spare IO on J10
        uint8_t v = hal.ReadTCA9534InputRegisters();
        //P6 = spare I/O (on PCB pin)
        InputState[5] = (v & B01000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;
        //P7 = Emergency Stop
        InputState[6] = (v & B10000000) == 0 ? enumInputState::INPUT_LOW : enumInputState::INPUT_HIGH;

        //Emergency Stop (J1) has triggered
        if (InputState[6] == enumInputState::INPUT_LOW)
        {
          emergencyStop = true;
        }
      }

      if (m.command == 0x03)
      {
        hal.Led(m.data);
      }

      if (m.command == 0x04)
      {
        hal.TFTScreenBacklight(m.data);
      }

      if (m.command >= 0xE0 && m.command <= 0xE0 + RELAY_TOTAL)
      {
        //Set state of relays
        hal.SetOutputState(m.command - 0xe0, (RelayState)m.data);
      }
#endif
      // 	void initBQ(void);
	    // unsigned int directCommand(byte);
	    // void subCommand(unsigned int);
	    // unsigned int subCommandResponseInt(void);
	    // void enterConfigUpdate(void);
	    // void exitConfigUpdate(void);
	    // byte computeChecksum(byte, byte);
	    // void writeDataMemory(unsigned int , unsigned int, byte);
	    // byte readDataMemory(unsigned int);
      if (m.command == 0x80)
      {
        pi.soc = bqz.state_of_charge();
        pi.soh = bqz.state_of_health();
        pi.voltage = bqz.voltage() / 1000.0;
        ESP_LOGD(TAG, "bqz soc = %f", pi.soc);
        ESP_LOGD(TAG, "bqz soh = %f", pi.soc);
        ESP_LOGD(TAG, "bqz voltage = %f", pi.voltage);
        pi.remainingCapacityAh = bqz.remaining_capacity() * 10.0 / 1000.0;
        bqz.flagsa_cache = bqz.flags();
        pi.averageCurrent = bqz.average_current() * 10.0;
        pi.current = bqz.current() * 10.0;
        pi.fullChargeCapacityAh = bqz.full_charge_capacity() * 10.0 / 1000.0;
        pi.temperature_ic1 = bqz.internal_temperature();
        pi.temperature1 = bqz.temperature();

        CellModuleInfo *cellptr;
        // BQ
        uint16_t mode = bq.getVcellMode();
        ESP_LOGD(TAG, "bq mode = %x", mode);

        // Min/Max temps
        bq.getDASTATUS5();
        pi.maxCellTemperature = bq.MaxCellTemp();
        pi.minCellTemperature = bq.MinCellTemp();
        pi.minCellVolt = bq.MinCellVolt();
        pi.maxCellVolt = bq.MaxCellVolt();
        ESP_LOGD(TAG, "min cell temp %f", pi.minCellTemperature);
        ESP_LOGD(TAG, "max cell temp %f", pi.maxCellTemperature);
        ESP_LOGD(TAG, "min cell volt %f", pi.minCellVolt);
        ESP_LOGD(TAG, "max cell volt %f", pi.maxCellVolt);
        //TelnetStream.print("voltage = ");
        //TelnetStream.println(bqz.voltage() / 1000.0);
        //TelnetStream.print("current = ");
        //TelnetStream.println(bqz.current());
        //TelnetStream.print("remaining_capacity = ");
        //TelnetStream.println(bqz.remaining_capacity());
        
        // Additional 3 to read top of stack voltages
        unsigned int cells[19];
        bq.getAllCellVoltages(&cells[0]);
        int configuredCell = 0;
        for (int i = 0; i < 16; i++) {
          if ((mode & (1 << i)) == false) {
            continue;
          }
          ESP_LOGD(TAG, "cell %i (bq cell %i) %d", configuredCell, i, cells[i]);
          cellptr = &cmi[configuredCell];
          if (cells[i] > 0) {
            cellptr->voltagemV = cells[i];
            if (cellptr->voltagemV > 0)
            { 
             cellptr->valid = true;
            }
            if (i == 0) {
              cellptr->internalTemp = pi.minCellTemperature;
            } else {
              cellptr->internalTemp = pi.maxCellTemperature;
            }
            if (cellptr->voltagemV < cellptr->voltagemVMin)
            { 
              cellptr->voltagemVMin = cellptr->voltagemV;
            }
            if (cellptr->voltagemV > cellptr->voltagemVMax)
            { 
              cellptr->voltagemVMax = cellptr->voltagemV;
            }
          }
          configuredCell++;
        }
        if (_tft_screen_available)
        {
          //Refresh the TFT display
          xTaskNotify(updatetftdisplay_task_handle, 0x00, eNotifyAction::eNoAction);
        }
      }
    }
  }
}

volatile uint32_t WifiPasswordClearTime;
volatile bool ResetWifi = false;

// Check if BOOT button is pressed, if held down for more than 4 seconds
// trigger a wifi password reset/clear from EEPROM.
void IRAM_ATTR WifiPasswordClear()
{
  // RMC: check boot button
  if (true || (digitalRead(GPIO_NUM_0) == LOW))
  {
    //Button pressed, store time
    WifiPasswordClearTime = millis() + 4000;
    ResetWifi = false;
  }
  else
  {
    //Button released
    //Did user press button for longer than 4 seconds?
    if (millis() > WifiPasswordClearTime)
    {
      ResetWifi = true;
    }
  }
}

void IRAM_ATTR TCA6408Interrupt()
{
  if (queue_i2c == NULL)
    return;
  i2cQueueMessage m;
  m.command = 0x01;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}

void IRAM_ATTR TCA9534AInterrupt()
{
  if (queue_i2c == NULL)
    return;
  i2cQueueMessage m;
  m.command = 0x02;
  m.data = 0;
  xQueueSendToBackFromISR(queue_i2c, &m, NULL);
}

/*
void dumpByte(uint8_t data)
{
  if (data <= 0x0F)
  {
    SERIAL_DEBUG.print('0');
  }
  SERIAL_DEBUG.print(data, HEX);
}
*/

const char *packetType(uint8_t cmd)
{
  switch (cmd)
  {
  case COMMAND::ResetBadPacketCounter:
    return "ResetC";
    break;
  case COMMAND::ReadVoltageAndStatus:
    return "RdVolt";
    break;
  case COMMAND::Identify:
    return "Ident";
    break;
  case COMMAND::ReadTemperature:
    return "RdTemp";
    break;
  case COMMAND::ReadBadPacketCounter:
    return "RdBadPkC";
    break;
  case COMMAND::ReadSettings:
    return "RdSettin";
    break;
  case COMMAND::WriteSettings:
    return "WriteSet";
    break;
  case COMMAND::ReadBalancePowerPWM:
    return "RdBalanc";
    break;
  case COMMAND::Timing:
    return "Timing";
    break;
  case COMMAND::ReadBalanceCurrentCounter:
    return "Current";
    break;
  case COMMAND::ReadPacketReceivedCounter:
    return "PktRvd";
    break;
  }

  return " ??????   ";
}

void dumpPacketToDebug(char indicator, PacketStruct *buffer)
{
  //Filter on some commands
  //if ((buffer->command & 0x0F) != COMMAND::Timing)    return;

  ESP_LOGD(TAG, "%c %02X-%02X H:%02X C:%02X SEQ:%04X CRC:%04X %s",
           indicator,
           buffer->start_address,
           buffer->end_address,
           buffer->hops,
           buffer->command,
           buffer->sequence,
           buffer->crc,
           packetType(buffer->command & 0x0F));

  //ESP_LOG_BUFFER_HEX("packet", &(buffer->moduledata[0]), sizeof(buffer->moduledata), ESP_LOG_DEBUG);
}

const char *ControllerStateString(ControllerState value)
{
  switch (value)
  {
  case ControllerState::PowerUp:
    return "PowerUp";
  case ControllerState::ConfigurationSoftAP:
    return "ConfigurationSoftAP";
  case ControllerState::Stabilizing:
    return "Stabilizing";
  case ControllerState::Running:
    return "Running";
  case ControllerState::Unknown:
    return "Unknown";
  }

  return "?";
}


void SetControllerState(ControllerState newState)
{
  if (_controller_state != newState)
  {
    ESP_LOGI(TAG, "** Controller changed state from %s to %s **", ControllerStateString(_controller_state), ControllerStateString(newState));

    _controller_state = newState;

    switch (_controller_state)
    {
    case ControllerState::PowerUp:
      //Purple during start up, don't use the LED as thats not setup at this state
      hal.Led(RGBLED::Purple);
      break;
    case ControllerState::ConfigurationSoftAP:
      //Don't use the LED as thats not setup at this state
      hal.Led(RGBLED::White);
      break;
    case ControllerState::Stabilizing:
      LED(RGBLED::Yellow);
      break;
    case ControllerState::Running:
      LED(RGBLED::Green);
      //Fire task to switch off BOOT button after 30 seconds
      xTaskNotify(wifiresetdisable_task_handle, 0x00, eNotifyAction::eNoAction);
      break;
    case ControllerState::Unknown:
      //Do nothing
      break;
    }
  }
}


uint16_t minutesSinceMidnight()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return 0;
  }
  else
  {
    return (timeinfo.tm_hour * 60) + timeinfo.tm_min;
  }
}

void replyqueue_task(void *param)
{
  for (;;)
  {
    //Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (!replyQueue.isEmpty())
    {
      PacketStruct ps;
      replyQueue.pop(&ps);

#if defined(PACKET_LOGGING_RECEIVE)
// Process decoded incoming packet
//dumpPacketToDebug('R', &ps);
#endif

      if (!receiveProc.ProcessReply(&ps))
      {
        //Error blue
        QueueLED(RGBLED::Blue);

        ESP_LOGE(TAG, "Packet Failed");

        //SERIAL_DEBUG.print(F("*FAIL*"));
        //dumpPacketToDebug('F', &ps);
      }

      //Small delay to allow watchdog to be fed
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

void onPacketReceived()
{
  PacketStruct ps;
  memcpy(&ps, SerialPacketReceiveBuffer, sizeof(PacketStruct));

  if ((ps.command & 0x0F) == COMMAND::Timing)
  {
    //Timestamp at the earliest possible moment
    uint32_t t = millis();
    ps.moduledata[2] = (t & 0xFFFF0000) >> 16;
    ps.moduledata[3] = t & 0x0000FFFF;
    //Ensure CRC is correct
    ps.crc = CRC16::CalculateArray((uint8_t *)&ps, sizeof(PacketStruct) - 2);
  }

  if (!replyQueue.push(&ps))
  {
    ESP_LOGE(TAG, "*Failed to queue reply*");
  }
}

void transmit_task(void *param)
{
  for (;;)
  {
    //Delay 1 second
    vTaskDelay(pdMS_TO_TICKS(1000));

    //TODO: Move to proper RTOS QUEUE...
    if (requestQueue.isEmpty() == false)
    {
      // Called to transmit the next packet in the queue need to ensure this procedure
      // is called more frequently than items are added into the queue

      PacketStruct transmitBuffer;

      requestQueue.pop(&transmitBuffer);
      sequence++;
      transmitBuffer.sequence = sequence;

      if (transmitBuffer.command == COMMAND::Timing)
      {
        //Timestamp at the last possible moment
        uint32_t t = millis();
        transmitBuffer.moduledata[0] = (t & 0xFFFF0000) >> 16;
        transmitBuffer.moduledata[1] = t & 0x0000FFFF;
      }

      transmitBuffer.crc = CRC16::CalculateArray((uint8_t *)&transmitBuffer, sizeof(PacketStruct) - 2);
      myPacketSerial.sendBuffer((byte *)&transmitBuffer);

      // Output the packet we just transmitted to debug console
      //#if defined(PACKET_LOGGING_SEND)
      //      dumpPacketToDebug('S', &transmitBuffer);
      //#endif
    }
  }
}

//Runs the rules and populates rule_outcome array with true/false for each rule
//Rules based on module parameters/readings like voltage and temperature
//are only processed once every module has returned at least 1 reading/communication
void ProcessRules()
{
  rules.ClearValues();
  rules.ClearWarnings();
  rules.ClearErrors();

  rules.rule_outcome[Rule::BMSError] = false;

  QueueReadCells();

  uint16_t totalConfiguredModules = TotalNumberOfCells();
  if (totalConfiguredModules > maximum_controller_cell_modules)
  {
    //System is configured with more than maximum modules - abort!
    rules.SetError(InternalErrorCode::TooManyModules);
  }

  if (receiveProc.totalModulesFound > 0 && receiveProc.totalModulesFound != totalConfiguredModules)
  {
    //Found more or less modules than configured for
    rules.SetError(InternalErrorCode::ModuleCountMismatch);
  }

  //Communications error...
  //if (receiveProc.HasCommsTimedOut())
  //{
  //  rules.SetError(InternalErrorCode::CommunicationsError);
  //  rules.rule_outcome[Rule::BMSError] = true;
  //}

  if (rules.rule_outcome[Rule::EmergencyStop])
  {
    //Lowest 3 bits are RGB led GREEN/RED/BLUE
    rules.SetError(InternalErrorCode::ErrorEmergencyStop);
  }

  rules.numberOfBalancingModules = 0;
  uint8_t cellid = 0;
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    for (int8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
    {
      rules.ProcessCell(bank, &cmi[cellid]);

      if (cmi[cellid].valid && cmi[cellid].settingsCached)
      {

        if (cmi[cellid].BypassThresholdmV != mysettings.BypassThresholdmV)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassVoltage);
        }

        if (cmi[cellid].BypassOverTempShutdown != mysettings.BypassOverTempShutdown)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassTemperature);
        }

        if (cmi[cellid].inBypass)
        {
          rules.numberOfBalancingModules++;
        }

        if (cmi[0].settingsCached && cmi[cellid].CodeVersionNumber != cmi[0].CodeVersionNumber)
        {
          //Do all the modules have the same version of code as module zero?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantCodeVersion);
        }

        if (cmi[0].settingsCached && cmi[cellid].BoardVersionNumber != cmi[0].BoardVersionNumber)
        {
          //Do all the modules have the same hardware revision?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBoardRevision);
        }
      }

      cellid++;
    }
    rules.ProcessBank(bank, &pi);
  }

  if (mysettings.loggingEnabled && !_sd_card_installed && !_avrsettings.programmingModeEnabled)
  {
    rules.SetWarning(InternalWarningCode::LoggingEnabledNoSDCard);
  }

  if (_avrsettings.programmingModeEnabled)
  {
    rules.SetWarning(InternalWarningCode::AVRProgrammingMode);
  }

  if (rules.invalidModuleCount > 0)
  {
    //Some modules are not yet valid
    rules.SetError(InternalErrorCode::WaitingForModulesToReply);
  }

  if (_controller_state == ControllerState::Running && rules.zeroVoltageModuleCount > 0)
  {
    rules.SetError(InternalErrorCode::ZeroVoltModule);
    rules.rule_outcome[Rule::BMSError] = true;
  }

  rules.RunRules(
      mysettings.rulevalue,
      mysettings.rulehysteresis,
      emergencyStop,
      minutesSinceMidnight());

  if (_controller_state == ControllerState::Stabilizing)
  {
    //Check for zero volt modules - not a problem whilst we are in stabilizing start up mode
    if (rules.zeroVoltageModuleCount == 0 && rules.invalidModuleCount == 0)
    {
      //Every module has been read and they all returned a voltage move to running state
      SetControllerState(ControllerState::Running);
    }
  }

  if (rules.rule_outcome[Rule::EmergencyStop])
  {
    //Lowest 3 bits are RGB led GREEN/RED/BLUE
    LED(RGBLED::Red);
  }

  if (rules.numberOfActiveErrors > 0 && _tft_screen_available)
  {
    //We have active errors

    //Wake up the screen, this will also trigger it to update the display
    xTaskNotify(tftwakeup_task_handle, 0x00, eNotifyAction::eNoAction);
  }
}

void pulse_relay_off_task(void *param)
{
  for (;;)
  {
    //Wait until this task is triggered
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //Now wait 200ms before switching off the relays
    vTaskDelay(pdMS_TO_TICKS(200));

    for (int8_t y = 0; y < RELAY_TOTAL; y++)
    {
      if (previousRelayPulse[y])
      {
        //We now need to rapidly turn off the relay after a fixed period of time (pulse mode)
        //However we leave the relay and previousRelayState looking like the relay has triggered (it has!)
        //to prevent multiple pulses being sent on each rule refresh

        i2cQueueMessage m;
        //Different command for each relay
        m.command = 0xE0 + y;
        m.data = previousRelayState[y] == RelayState::RELAY_ON ? RelayState::RELAY_OFF : RelayState::RELAY_ON;
        xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);

        previousRelayPulse[y] = false;
      }
    }

    //Fire task to record state of outputs to SD Card
    xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
  }
}

void rules_task(void *param)
{
  for (;;)
  {
    //3 seconds
    vTaskDelay(pdMS_TO_TICKS(3000));

    //Run the rules
    ProcessRules();

#if defined(RULES_LOGGING)
    for (int8_t r = 0; r < RELAY_RULES; r++)
    {
      if (rules.rule_outcome[r])
      {
        ESP_LOGD(TAG, "Rule outcome %i=TRUE", r);
      }
    }
#endif

#ifdef RMC

    RelayState relay[RELAY_TOTAL];

    //Set defaults based on configuration
    for (int8_t y = 0; y < RELAY_TOTAL; y++)
    {
      relay[y] = mysettings.rulerelaydefault[y] == RELAY_ON ? RELAY_ON : RELAY_OFF;
    }

    //Test the rules (in reverse order)
    for (int8_t n = RELAY_RULES - 1; n >= 0; n--)
    {
      if (rules.rule_outcome[n] == true)
      {
        for (int8_t y = 0; y < RELAY_TOTAL; y++)
        {
          //Dont change relay if its set to ignore/X
          if (mysettings.rulerelaystate[n][y] != RELAY_X)
          {
            if (mysettings.rulerelaystate[n][y] == RELAY_ON)
            {
              relay[y] = RELAY_ON;
            }
            else
            {
              relay[y] = RELAY_OFF;
            }
          }
        }
      }
    }

    uint8_t changes = 0;
    bool firePulse = false;
    for (int8_t n = 0; n < RELAY_TOTAL; n++)
    {
      if (previousRelayState[n] != relay[n])
      {
        changes++;
        //Would be better here to use the WRITE8 to lower i2c traffic

        ESP_LOGI(TAG, "Set Relay %i=%i", n, relay[n] == RelayState::RELAY_ON ? 1 : 0);

        //This would be better if we worked out the bit pattern first and then just submitted that as a single i2c read/write transaction

        i2cQueueMessage m;
        //Different command for each relay
        m.command = 0xE0 + n;
        m.data = relay[n];
        xQueueSendToBack(queue_i2c, &m, 10 / portTICK_PERIOD_MS);

        previousRelayState[n] = relay[n];

        if (mysettings.relaytype[n] == RELAY_PULSE)
        {
          previousRelayPulse[n] = true;
          firePulse = true;

          ESP_LOGI(TAG, "Relay %i PULSED", n);
        }
      }
    }

    if (firePulse)
    {
      xTaskNotify(pulse_relay_off_task_handle, 0x00, eNotifyAction::eNoAction);
    }

    if (changes)
    {
      //Fire task to record state of outputs to SD Card
      xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);
    }
#endif
  }
}

void enqueue_task(void *param)
{
  for (;;)
  {
    //Ensure we service the cell modules every 5 or 10 seconds, depending on number of cells being serviced
    //slower stops the queues from overflowing when a lot of cells are being monitored
    vTaskDelay(pdMS_TO_TICKS((TotalNumberOfCells() <= maximum_cell_modules_per_packet) ? 5000 : 10000));

    QueueLED(RGBLED::Green);
    //Fire task to switch off LED in a few ms
    // RMC xTaskNotify(ledoff_task_handle, 0x00, eNotifyAction::eNoAction);

    uint16_t i = 0;
    uint16_t max = TotalNumberOfCells();

    uint8_t startmodule = 0;

    while (i < max)
    {
      uint16_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

      //Limit to number of modules we have configured
      if (endmodule > max)
      {
        endmodule = max - 1;
      }

      //Need to watch overflow of the uint8 here...
      prg.sendCellVoltageRequest(startmodule, endmodule);
      prg.sendCellTemperatureRequest(startmodule, endmodule);

      //If any module is in bypass then request PWM reading for whole bank
      for (uint8_t m = startmodule; m <= endmodule; m++)
      {
        if (cmi[m].inBypass)
        {
          prg.sendReadBalancePowerRequest(startmodule, endmodule);
          //We only need 1 reading for whole bank
          break;
        }
      }

      //Move to the next bank
      startmodule = endmodule + 1;
      i += maximum_cell_modules_per_packet;
    }
  }
}

void connectToWifi()
{
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED)
  {
    return;
  }

  /*
WiFi.status() only returns:

    switch(status) {
        case STATION_GOT_IP:
            return WL_CONNECTED;
        case STATION_NO_AP_FOUND:
            return WL_NO_SSID_AVAIL;
        case STATION_CONNECT_FAIL:
        case STATION_WRONG_PASSWORD:
            return WL_CONNECT_FAILED;
        case STATION_IDLE:
            return WL_IDLE_STATUS;
        default:
            return WL_DISCONNECTED;
    }
*/

  WiFi.mode(WIFI_STA);

  char hostname[40];

  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8)
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  sprintf(hostname, "DIYBMS-%08X", chipId);
  WiFi.setHostname(hostname);

  ESP_LOGI(TAG, "Hostname: %s, current state %i", hostname, status);

  //ESP_LOGD(TAG, "WiFi begin");
  WiFi.begin(DIYBMSSoftAP::WifiSSID(), DIYBMSSoftAP::WifiPassword());
}

void connectToMqtt()
{
  if (mysettings.mqtt_enabled && WiFi.isConnected())
  {
    if (mqttClient.connected() == false)
    {
      //ESP_LOGD(TAG, "MQTT Enabled");
      mqttClient.setServer(mysettings.mqtt_server, mysettings.mqtt_port);
      mqttClient.setCredentials(mysettings.mqtt_username, mysettings.mqtt_password);

      ESP_LOGI(TAG, "Connect MQTT");
      mqttClient.connect();
    }
  }

  if (mysettings.mqtt_enabled == false && mqttClient.connected())
  {
    //We are connected but shouldn't be!
    ESP_LOGI(TAG, "Disconnecting MQTT");
    mqttClient.disconnect(true);
  }
}

static AsyncClient *aClient = NULL;

void setupInfluxClient()
{

  if (aClient) //client already exists
    return;

  aClient = new AsyncClient();
  if (!aClient) //could not allocate client
    return;

  aClient->onError([](void *arg, AsyncClient *client, err_t error) {
    ESP_LOGE(TAG, "Influx connect error");

    aClient = NULL;
    delete client;
  },
                   NULL);

  aClient->onConnect([](void *arg, AsyncClient *client) {
    ESP_LOGI("Influx connected");

    //Send the packet here

    aClient->onError(NULL, NULL);

    client->onDisconnect([](void *arg, AsyncClient *c) {
      ESP_LOGI("Influx disconnected");
      aClient = NULL;
      delete c;
    },
                         NULL);

    client->onData([](void *arg, AsyncClient *c, void *data, size_t len) {
      //Data received
      ESP_LOGD("Influx data received");
      //SERIAL_DEBUG.print(F("\r\nData: "));
      //SERIAL_DEBUG.println(len);
      //uint8_t* d = (uint8_t*)data;
      //for (size_t i = 0; i < len; i++) {SERIAL_DEBUG.write(d[i]);}
    },
                   NULL);

    //send the request

    //Construct URL for the influxdb
    //See API at https://docs.influxdata.com/influxdb/v1.7/tools/api/#write-http-endpoint

    String poststring;

    for (uint8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
    {
      //TODO: We should send a request per bank not just a single POST as we are likely to exceed capabilities of ESP
      for (uint8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
      {
        //Data in LINE PROTOCOL format https://docs.influxdata.com/influxdb/v1.7/write_protocols/line_protocol_tutorial/
        poststring = poststring + "cells," + "cell=" + String(bank) + "_" + String(i) + " v=" + String((float)cmi[i].voltagemV / 1000.0, 3) + ",i=" + String(cmi[i].internalTemp) + "i" + ",e=" + String(cmi[i].externalTemp) + "i" + ",b=" + (cmi[i].inBypass ? String("true") : String("false")) + "\n";
      }
    }

    //TODO: Need to URLEncode these values
    String url = "/write?db=" + String(mysettings.influxdb_database) + "&u=" + String(mysettings.influxdb_user) + "&p=" + String(mysettings.influxdb_password);
    String header = "POST " + url + " HTTP/1.1\r\n" + "Host: " + String(mysettings.influxdb_host) + "\r\n" + "Connection: close\r\n" + "Content-Length: " + poststring.length() + "\r\n" + "Content-Type: text/plain\r\n" + "\r\n";

    //SERIAL_DEBUG.println(header.c_str());
    //SERIAL_DEBUG.println(poststring.c_str());

    client->write(header.c_str());
    client->write(poststring.c_str());

    ESP_LOGD("Influx data sent");
  },
                     NULL);
}

void influxdb_task(void *param)
{
  for (;;)
  {
    //Delay 30 seconds
    vTaskDelay(pdMS_TO_TICKS(30000));

    if (mysettings.influxdb_enabled && WiFi.isConnected())
    {
      ESP_LOGI("Send Influxdb data");

      setupInfluxClient();

      if (!aClient->connect(mysettings.influxdb_host, mysettings.influxdb_httpPort))
      {
        ESP_LOGE(TAG, "Influxdb connect fail");
        AsyncClient *client = aClient;
        aClient = NULL;
        delete client;
      }
    }
  }
}

void SetupOTA()
{

  ArduinoOTA.setPort(3232);

  ArduinoOTA.setPassword("1jiOOx12AQgEco4e");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        ESP_LOGI(TAG, "Start updating %s", type);
      });
  ArduinoOTA.onEnd([]() {
    ESP_LOGD(TAG, "\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    ESP_LOGD(TAG, "Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    ESP_LOGD(TAG, "Error [%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      ESP_LOGE(TAG, "Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      ESP_LOGE(TAG, "Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      ESP_LOGE(TAG, "Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      ESP_LOGE(TAG, "Receive Failed");
    else if (error == OTA_END_ERROR)
      ESP_LOGE(TAG, "End Failed");
  });

  ArduinoOTA.setHostname(WiFi.getHostname());
  ArduinoOTA.setMdnsEnabled(true);
  ArduinoOTA.begin();
}

void mountSDCard()
{
  return;
  /*
SD CARD TEST
*/
  ESP_LOGI(TAG, "Mounting SD card");
  return;

  hal.GetVSPIMutex();
  // Initialize SD card
  //SD.begin(SDCARD_CHIPSELECT, hal.vspi);
/*
  if (SD.begin(SDCARD_CHIPSELECT, hal.vspi))
  {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
      ESP_LOGI(TAG, "No SD card attached");
    }
    else
    {
      ESP_LOGI(TAG, "SD card available");
      _sd_card_installed = true;
    }
  }
  else
  {
    ESP_LOGE(TAG, "Card Mount Failed");
  }
  hal.ReleaseVSPIMutex();
  */
}

void sdcardaction_callback(uint8_t action)
{
  switch (action)
  {
  case 0:
    //Unmount
    ESP_LOGI(TAG, "Unmounting SD card");
    hal.GetVSPIMutex();
    SD.end();
    hal.ReleaseVSPIMutex();
    _sd_card_installed = false;
    break;
  case 1:
    mountSDCard();
    break;
  }
}

void onWifiConnect(WiFiEvent_t event, WiFiEventInfo_t info)
{

  ESP_LOGI(TAG, "Wi-Fi status=%i", (uint16_t)WiFi.status());

  ESP_LOGI(TAG, "Request NTP from %s", mysettings.ntpServer);

  //Use native ESP32 code
  configTime(mysettings.minutesTimeZone * 60, mysettings.daylight * 60, mysettings.ntpServer);

  /*
  TODO: CHECK ERROR CODES BETTER!
  0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
  1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
  3 : WL_CONNECTED after successful connection is established
  4 : WL_CONNECT_FAILED if password is incorrect
  6 : WL_DISCONNECTED if module is not configured in station mode
  */
  if (!server_running)
  {
    DIYBMSServer::StartServer(&server, &mysettings, &SD, &prg, &receiveProc, &_controller_state, &rules, &sdcardaction_callback, &hal);
    server_running = true;
  }

  connectToMqtt();

  SetupOTA();

  TelnetStream.begin();
  //vprintf_like_t ret = 
  //esp_log_set_vprintf(&LOG_TO_TELNET);
  //TelnetStream.println("start logging = ");
  //TelnetStream.println(ret);


  // Set up mDNS responder:
  // - first argument is the domain name, in this example
  //   the fully-qualified domain name is "esp8266.local"
  // - second argument is the IP address to advertise
  //   we send our IP address on the WiFi network
  if (!MDNS.begin(WiFi.getHostname()))
  {
    ESP_LOGE(TAG, "Error setting up MDNS responder!");
  }
  else
  {
    ESP_LOGI(TAG, "mDNS responder started");
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
  }

  ESP_LOGI(TAG, "You can access DIYBMS interface at http://%s.local or http://%s", WiFi.getHostname(), WiFi.localIP().toString().c_str());

  //Wake up the screen, this will show the IP address etc.
  xTaskNotify(tftwakeup_task_handle, 0x01, eNotifyAction::eSetValueWithOverwrite);
}

void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
  ESP_LOGE(TAG, "Disconnected from Wi-Fi.");

  //Indicate to loop() to reconnect, seems to be
  //ESP issues using Wifi from timers - https://github.com/espressif/arduino-esp32/issues/2686

  //Wake up the screen, this will also trigger it to update the display
  xTaskNotify(tftwakeup_task_handle, 0x01, eNotifyAction::eSetValueWithOverwrite);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  ESP_LOGE(TAG, "Disconnected from MQTT.");
}

void mqtt2(void *param)
{
  for (;;)
  {
    //Delay 25 seconds
    vTaskDelay(pdMS_TO_TICKS(25000));

    if (mysettings.mqtt_enabled && mqttClient.connected())
    {
      ESP_LOGI(TAG, "Send MQTT Status");

      char topic[80];
      char jsonbuffer[220];
      DynamicJsonDocument doc(220);
      JsonObject root = doc.to<JsonObject>();

      root["banks"] = mysettings.totalNumberOfBanks;
      root["cells"] = mysettings.totalNumberOfSeriesModules;
      root["uptime"] = millis() / 1000; // I want to know the uptime of the device.

      // Set error flag if we have attempted to send 2*number of banks without a reply
      root["commserr"] = receiveProc.HasCommsTimedOut() ? 1 : 0;
      root["sent"] = prg.packetsGenerated;
      root["received"] = receiveProc.packetsReceived;
      root["badcrc"] = receiveProc.totalCRCErrors;
      root["ignored"] = receiveProc.totalNotProcessedErrors;
      root["oos"] = receiveProc.totalOutofSequenceErrors;
      root["roundtrip"] = receiveProc.packetTimerMillisecond;

      serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
      sprintf(topic, "%s/status", mysettings.mqtt_topic);
      mqttClient.publish(topic, 0, false, jsonbuffer);
#if defined(MQTT_LOGGING)
      ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
//SERIAL_DEBUG.print("MQTT - ");SERIAL_DEBUG.print(topic);  SERIAL_DEBUG.print('=');  SERIAL_DEBUG.println(jsonbuffer);
#endif

      //Output bank level information (just voltage for now)
      for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
      {
        doc.clear();
        doc["voltage"] = (float)rules.packvoltage[bank] / (float)1000.0;

        serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
        sprintf(topic, "%s/bank/%d", mysettings.mqtt_topic, bank);
        mqttClient.publish(topic, 0, false, jsonbuffer);
#if defined(MQTT_LOGGING)
        ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
//SERIAL_DEBUG.print("MQTT - ");SERIAL_DEBUG.print(topic);  SERIAL_DEBUG.print('=');  SERIAL_DEBUG.println(jsonbuffer);
#endif
      }

      //Using Json for below reduced MQTT messages from 14 to 2. Could be combined into same json object too. But even better is status + event driven.
      doc.clear(); // Need to clear the json object for next message
      sprintf(topic, "%s/rule", mysettings.mqtt_topic);
      for (uint8_t i = 0; i < RELAY_RULES; i++)
      {
        doc[(String)i] = rules.rule_outcome[i] ? 1 : 0; // String conversion should be removed but just quick to get json format nice
      }
      serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
#if defined(MQTT_LOGGING)
      ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
      mqttClient.publish(topic, 0, false, jsonbuffer);

      doc.clear(); // Need to clear the json object for next message
      sprintf(topic, "%s/output", mysettings.mqtt_topic);
      for (uint8_t i = 0; i < RELAY_TOTAL; i++)
      {
        doc[(String)i] = (previousRelayState[i] == RelayState::RELAY_ON) ? 1 : 0;
      }

      serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
#if defined(MQTT_LOGGING)
      ESP_LOGD(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
      mqttClient.publish(topic, 0, false, jsonbuffer);
    }
  }
}

void mqtt1(void *param)
{
  //Send a few MQTT packets and keep track so we send the next batch on following calls
  static uint8_t mqttStartModule = 0;

  for (;;)
  {
    //Delay 5 seconds
    vTaskDelay(pdMS_TO_TICKS(5000));

    if (mysettings.mqtt_enabled && mqttClient.connected())
    {
      //ESP_LOGI(TAG, "Send MQTT Packet");

      char topic[80];
      char jsonbuffer[200];
      StaticJsonDocument<200> doc;

      //If the BMS is in error, stop sending MQTT packets for the data
      if (!rules.rule_outcome[Rule::BMSError])
      {

        if (mqttStartModule > TotalNumberOfCells())
        {
          mqttStartModule = 0;
        }

        uint8_t counter = 0;
        uint8_t i = mqttStartModule;

        while (i < TotalNumberOfCells() && counter < 8)
        {
          //Only send valid module data
          if (cmi[i].valid)
          {
            uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
            uint8_t module = i - (bank * mysettings.totalNumberOfSeriesModules);

            doc.clear();
            doc["voltage"] = (float)cmi[i].voltagemV / (float)1000.0;
            doc["vMax"] = (float)cmi[i].voltagemVMax / (float)1000.0;
            doc["vMin"] = (float)cmi[i].voltagemVMin / (float)1000.0;
            doc["inttemp"] = cmi[i].internalTemp;
            doc["exttemp"] = cmi[i].externalTemp;
            doc["bypass"] = cmi[i].inBypass ? 1 : 0;
            doc["PWM"] = (int)((float)cmi[i].PWMValue / (float)255.0 * 100);
            doc["bypassT"] = cmi[i].bypassOverTemp ? 1 : 0;
            doc["bpc"] = cmi[i].badPacketCount;
            doc["mAh"] = cmi[i].BalanceCurrentCount;
            serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));

            sprintf(topic, "%s/%d/%d", mysettings.mqtt_topic, bank, module);

            mqttClient.publish(topic, 0, false, jsonbuffer);

#if defined(MQTT_LOGGING)
            ESP_LOGI(TAG, "MQTT %s %s", topic, jsonbuffer);
#endif
          }

          counter++;

          i++;
        }

        //After transmitting this many packets over MQTT, store our current state and exit the function.
        //this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
        mqttStartModule = i + 1;
      }
    }
  }
}

void onMqttConnect(bool sessionPresent)
{
  ESP_LOGI("Connected to MQTT");
  //myTimerSendMqttPacket.detach();
  //myTimerSendMqttStatus.detach();
  //myTimerSendMqttPacket.attach(5, sendMqttPacket);
  //myTimerSendMqttStatus.attach(25, sendMqttStatus);
}

void LoadConfiguration()
{
  if (Settings::ReadConfig("diybms", (char *)&mysettings, sizeof(mysettings)))
    return;

  ESP_LOGI("Apply default config");

  //Zero all the bytes
  memset(&mysettings, 0, sizeof(mysettings));

  mysettings.controller_id = 10;

  //Default to a single module
  mysettings.totalNumberOfBanks = 1;
  mysettings.totalNumberOfSeriesModules = 1;
  mysettings.BypassOverTempShutdown = 65;
  //4.10V bypass
  mysettings.BypassThresholdmV = 4100;
  mysettings.graph_voltagehigh = 4.5;
  mysettings.graph_voltagelow = 2.75;

  //EEPROM settings are invalid so default configuration
  mysettings.mqtt_enabled = false;
  mysettings.mqtt_port = 1883;

  mysettings.loggingEnabled = false;
  mysettings.loggingFrequencySeconds = 15;

  //Default to EMONPI default MQTT settings
  strcpy(mysettings.mqtt_topic, "emon/diybms");
  strcpy(mysettings.mqtt_server, "192.168.0.26");
  strcpy(mysettings.mqtt_username, "emonpi");
  strcpy(mysettings.mqtt_password, "emonpimqtt2016");

  mysettings.influxdb_enabled = false;
  strcpy(mysettings.influxdb_host, "myinfluxserver");
  strcpy(mysettings.influxdb_database, "database");
  strcpy(mysettings.influxdb_user, "user");
  strcpy(mysettings.influxdb_password, "");

  mysettings.timeZone = 0;
  mysettings.minutesTimeZone = 0;
  mysettings.daylight = false;
  strcpy(mysettings.ntpServer, "time.google.com");

  for (size_t x = 0; x < RELAY_TOTAL; x++)
  {
    mysettings.rulerelaydefault[x] = RELAY_OFF;
  }

  //Emergency stop
  mysettings.rulevalue[Rule::EmergencyStop] = 0;
  //Internal BMS error (communication issues, fault readings from modules etc)
  mysettings.rulevalue[Rule::BMSError] = 0;
  //Individual cell over voltage
  mysettings.rulevalue[Rule::Individualcellovervoltage] = 4150;
  //Individual cell under voltage
  mysettings.rulevalue[Rule::Individualcellundervoltage] = 3000;
  //Individual cell over temperature (external probe)
  mysettings.rulevalue[Rule::IndividualcellovertemperatureExternal] = 55;
  //Pack over voltage (mV)
  mysettings.rulevalue[Rule::IndividualcellundertemperatureExternal] = 5;
  //Pack under voltage (mV)
  mysettings.rulevalue[Rule::PackOverVoltage] = 4200 * 8;
  //RULE_PackUnderVoltage
  mysettings.rulevalue[Rule::PackUnderVoltage] = 3000 * 8;
  mysettings.rulevalue[Rule::Timer1] = 60 * 8;  //8am
  mysettings.rulevalue[Rule::Timer2] = 60 * 17; //5pm

  mysettings.rulevalue[Rule::ModuleOverTemperatureInternal] = 60;
  mysettings.rulevalue[Rule::ModuleUnderTemperatureInternal] = 50;

  for (size_t i = 0; i < RELAY_RULES; i++)
  {
    mysettings.rulehysteresis[i] = mysettings.rulevalue[i];

    //Set all relays to don't care
    for (size_t x = 0; x < RELAY_TOTAL; x++)
    {
      mysettings.rulerelaystate[i][x] = RELAY_X;
    }
  }

  for (size_t x = 0; x < RELAY_TOTAL; x++)
  {
    mysettings.relaytype[x] = RELAY_STANDARD;
  }
}

uint8_t lazyTimerMode = 0;
//Do activities which are not critical to the system like background loading of config, or updating timing results etc.
void lazy_tasks(void *param)
{
  for (;;)
  {
    //Delay 8 seconds
    vTaskDelay(pdMS_TO_TICKS(7000));

    if (requestQueue.getRemainingCount() > 6)
    {
      bool done_for_this_loop = false;

      lazyTimerMode++;

      if (lazyTimerMode == 1 && !done_for_this_loop)
      {
        //Send a "ping" message through the cells to get a round trip time
        prg.sendTimingRequest();
        done_for_this_loop = true;
      }

      if (lazyTimerMode == 2 && !done_for_this_loop)
      {
        done_for_this_loop = true;
        uint8_t counter = 0;
        //Find modules that don't have settings cached and request them
        for (uint8_t module = 0; module < TotalNumberOfCells(); module++)
        {
          if (cmi[module].valid && !cmi[module].settingsCached)
          {
            if (requestQueue.getRemainingCount() < 6)
            {
              //Exit here to avoid flooding the queue
              break;
            }

            prg.sendGetSettingsRequest(module);
            counter++;
          }
        }
      }

      if (!done_for_this_loop)
      {

        //Send these requests to all banks of modules
        uint16_t i = 0;
        uint16_t max = TotalNumberOfCells();

        uint8_t startmodule = 0;

        while (i < max)
        {
          uint16_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

          //Limit to number of modules we have configured
          if (endmodule > max)
          {
            endmodule = max - 1;
          }

          if (lazyTimerMode == 3)
          {
            prg.sendReadBalanceCurrentCountRequest(startmodule, endmodule);
          }

          if (lazyTimerMode == 4)
          {
            prg.sendReadPacketsReceivedRequest(startmodule, endmodule);
          }

          //Ask for bad packet count (saves battery power if we dont ask for this all the time)
          if (lazyTimerMode == 5)
          {
            prg.sendReadBadPacketCounter(startmodule, endmodule);
          }

          //Move to the next bank
          startmodule = endmodule + 1;
          i += maximum_cell_modules_per_packet;
        } //end while
      }

      //Reset at end of cycle
      if (lazyTimerMode >= 5)
      {
        lazyTimerMode = 0;
      }
    }
    else
    {
      //Exit here to avoid overflowing the queue
      ESP_LOGE(TAG, "ERR: Lazy overflow Q=%i", requestQueue.getRemainingCount());
      return;
    }
  } //end for
}

void resetAllRules()
{
  //Clear all rules
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    rules.rule_outcome[r] = false;
  }
}

bool CaptureSerialInput(HardwareSerial stream, char *buffer, int buffersize, bool OnlyDigits, bool ShowPasswordChar)
{
  int length = 0;
  unsigned long timer = millis() + 30000;

  while (true)
  {

    //Abort after 30 seconds of inactivity
    if (millis() > timer)
      return false;

    //We should add a timeout in here, and return FALSE when we abort....
    while (stream.available())
    {
      //Reset timer on serial input
      timer = millis() + 30000;

      int data = stream.read();
      if (data == '\b' || data == '\177')
      { // BS and DEL
        if (length)
        {
          length--;
          stream.write("\b \b");
        }
      }
      else if (data == '\n')
      {
        //Ignore
      }
      else if (data == '\r')
      {
        if (length > 0)
        {
          stream.write("\r\n"); // output CRLF
          buffer[length] = '\0';

          //Soak up any other characters on the buffer and throw away
          while (stream.available())
          {
            stream.read();
          }

          //Return to caller
          return true;
        }

        length = 0;
      }
      else if (length < buffersize - 1)
      {
        if (OnlyDigits && (data < '0' || data > '9'))
        {
          //We need to filter out non-digit characters
        }
        else
        {
          buffer[length++] = data;
          if (ShowPasswordChar)
          {
            //Hide real character
            stream.write('*');
          }
          else
          {
            stream.write(data);
          }
        }
      }
    }
  }
}

void TerminalBasedWifiSetup(HardwareSerial stream)
{
  stream.println(F("\r\n\r\nDIYBMS CONTROLLER - Scanning Wifi"));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  int n = WiFi.scanNetworks();

  if (n == 0)
    stream.println(F("no networks found"));
  else
  {
    for (int i = 0; i < n; ++i)
    {
      if (i < 10)
      {
        stream.print(' ');
      }
      stream.print(i);
      stream.print(':');
      stream.print(WiFi.SSID(i));

      //Pad out the wifi names into 2 columns
      for (size_t spaces = WiFi.SSID(i).length(); spaces < 36; spaces++)
      {
        stream.print(' ');
      }

      if ((i + 1) % 2 == 0)
      {
        stream.println();
      }
      delay(5);
    }
    stream.println();
  }

  WiFi.mode(WIFI_OFF);

  stream.print(F("Enter the NUMBER of the Wifi network to connect to:"));

  bool result;
  char buffer[10];
  result = CaptureSerialInput(stream, buffer, 10, true, false);
  if (result)
  {
    int index = String(buffer).toInt();
    stream.print(F("Enter the password to use when connecting to '"));
    stream.print(WiFi.SSID(index));
    stream.print("':");

    char passwordbuffer[80];
    result = CaptureSerialInput(stream, passwordbuffer, 80, false, true);

    if (result)
    {
      wifi_eeprom_settings config;
      memset(&config, 0, sizeof(config));
      WiFi.SSID(index).toCharArray(config.wifi_ssid, sizeof(config.wifi_ssid));
      strcpy(config.wifi_passphrase, passwordbuffer);
      Settings::WriteConfig("diybmswifi", (char *)&config, sizeof(config));
    }
  }

  stream.println(F("REBOOTING IN 5..."));
  delay(5000);
  ESP.restart();
}



void createFile(fs::FS &fs, const char *path, const char *message)
{
  //ESP_LOGD(TAG,"Writing file: %s", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    ESP_LOGD(TAG, "File written");
  }
  else
  {
    ESP_LOGE(TAG, "Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
  //ESP_LOGD(TAG,("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    ESP_LOGE(TAG, "Failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    ESP_LOGD(TAG, "Message appended");
  }
  else
  {
    ESP_LOGE(TAG, "Append failed");
  }
  file.close();
}





void dumpByte(uint8_t data)
{
  if (data <= 0x0F)
  {
    SERIAL_DEBUG.print('0');
  }
  SERIAL_DEBUG.print(data, HEX);
}



void cell_modem_task(void *param)
{
    cell_modem.init();
    while (1) {
        cell_modem.loop();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}




void setup()
{
  WiFi.mode(WIFI_OFF);

  btStop();
  esp_log_level_set("*", ESP_LOG_DEBUG);    // set all components to WARN level
  esp_log_level_set("wifi", ESP_LOG_WARN);  // enable WARN logs from WiFi stack
  esp_log_level_set("dhcpc", ESP_LOG_WARN); // enable INFO logs from DHCP client

  const char *diybms_logo = "\r\n\r\n\r\n                _          __ \r\n    _|  o      |_)  |\\/|  (_  \r\n   (_|  |  \\/  |_)  |  |  __) \r\n           /                  ";

  //ESP32 we use the USB serial interface for console/debug messages
  SERIAL_DEBUG.begin(115200, SERIAL_8N1);
  SERIAL_DEBUG.setDebugOutput(true);

  SERIAL_DEBUG.println(diybms_logo);

  ESP_LOGI(TAG, "CONTROLLER - ver:%s compiled %s", GIT_VERSION, COMPILE_DATE_TIME);

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  ESP_LOGI(TAG, "ESP32 Chip model = %u, Rev %u, Cores=%u, Features=%u", chip_info.model, chip_info.revision, chip_info.cores, chip_info.features);

  //We generate a unique number which is used in all following JSON requests
  //we use this as a simple method to avoid cross site scripting attacks
  DIYBMSServer::generateUUID();

  hal.ConfigurePins(WifiPasswordClear);
  hal.ConfigureI2C(TCA6408Interrupt, TCA9534AInterrupt);
  hal.ConfigureVSPI();

  // Start CAN
  can->begin();

  //See if we can get a sensible reading from the TFT touch chip XPT2046
  //if we can, then a screen is fitted, so enable it
  _tft_screen_available = hal.IsScreenAttached();

  if (_tft_screen_available)
  {
    ESP_LOGI(TAG, "TFT screen is INSTALLED");
    //Only attach, if device is fitted otherwise false triggers may occur
    //Touch screen IRQ (GPIO_NUM_36) is active LOW (XPT2046 chip)
    attachInterrupt(TOUCH_IRQ, TFTScreenTouchInterrupt, FALLING);
  }
  else
  {
    ESP_LOGI(TAG, "TFT screen is NOT installed");
  }
  


  SetControllerState(ControllerState::PowerUp);

  //hal.Led(0);

  if (!LITTLEFS.begin(false))
  {
    ESP_LOGE(TAG, "LITTLEFS mount failed, did you upload file system image? - SYSTEM HALTED");

    hal.Halt(RGBLED::White);
  }
  else
  {
    ESP_LOGI(TAG, "LITTLEFS mounted, totalBytes=%u, usedBytes=%u", LITTLEFS.totalBytes(), LITTLEFS.usedBytes());
    //listDir(LITTLEFS, "/", 0);
  }

  mountSDCard();


  //All comms to i2c needs to go through this single task
  //to prevent issues with thread safety on the i2c hardware/libraries
  queue_i2c = xQueueCreate(10, sizeof(i2cQueueMessage));

  //Create i2c task on CPU 0 (normal code runs on CPU 1)
  xTaskCreatePinnedToCore(i2c_task, "i2c", 2048, nullptr, configMAX_PRIORITIES - 1, &i2c_task_handle, 0);
  //xTaskCreatePinnedToCore(ledoff_task, "ledoff", 1048, nullptr, 1, &ledoff_task_handle, 0);

  //xTaskCreate(avrprog_task, "avrprog", 3000, &_avrsettings, configMAX_PRIORITIES - 5, &avrprog_task_handle);

  xTaskCreate(updatetftdisplay_task, "tftupd", 2048, nullptr, 1, &updatetftdisplay_task_handle);
  xTaskCreate(tftsleep_task, "tftslp", 900, nullptr, 1, &tftsleep_task_handle);
  xTaskCreate(tftwakeup_task, "tftwake", 1900, nullptr, 1, &tftwakeup_task_handle);

  xTaskCreate(wifiresetdisable_task, "wifidbl", 1048, nullptr, 1, &wifiresetdisable_task_handle);

  xTaskCreate(sdcardlog_task, "sdlog", 4096, nullptr, 1, &sdcardlog_task_handle);
  xTaskCreate(sdcardlog_outputs_task, "sdout", 4096, nullptr, 1, &sdcardlog_outputs_task_handle);



  //xTaskCreate(mqtt1, "mqtt1", 3000, nullptr, 1, &mqtt1_task_handle);
  //xTaskCreate(mqtt2, "mqtt2", 3000, nullptr, 1, &mqtt2_task_handle);

  //We process the transmit queue every 1 second (this needs to be lower delay than the queue fills)
  //and slower than it takes a single module to process a command (about 200ms @ 2400baud)

  //xTaskCreate(transmit_task, "tx", 1024, nullptr, configMAX_PRIORITIES - 3, &transmit_task_handle);
  //xTaskCreate(replyqueue_task, "rxq", 1024, nullptr, configMAX_PRIORITIES - 2, &replyqueue_task_handle);
  //xTaskCreate(lazy_tasks, "lazyt", 1024, nullptr, 1, &lazy_task_handle);

  //xTaskCreate(pulse_relay_off_task, "pulse", 1024, nullptr, configMAX_PRIORITIES - 1, &pulse_relay_off_task_handle);

  //xTaskCreate(cell_modem_task, "cell_modem", 4096, nullptr, configMAX_PRIORITIES - 1, &cell_modem_task_handle);

// RMC Just added back
  hal.ConfigureVSPI();
  init_tft_display();

  //Pre configure the array
  memset(&cmi, 0, sizeof(cmi));
  for (size_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    DIYBMSServer::clearModuleValues(i);
  }

  resetAllRules();

  //Receive is IO2 which means the RX1 plug must be disconnected for programming to work!
  SERIAL_DATA.begin(COMMS_BAUD_RATE, SERIAL_8N1, 2, 32); // Serial for comms to modules

  myPacketSerial.begin(&SERIAL_DATA, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

  LoadConfiguration();

#ifdef RMC
  //Set relay defaults
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    previousRelayState[y] = mysettings.rulerelaydefault[y];
    //Set relay defaults
    hal.SetOutputState(y, mysettings.rulerelaydefault[y]);
  }
  #endif
  //Fire task to record state of outputs to SD Card
  xTaskNotify(sdcardlog_outputs_task_handle, 0x00, eNotifyAction::eNoAction);

  //Allow user to press SPACE BAR key on serial terminal
  //to enter text based WIFI setup
  SERIAL_DEBUG.print(F("Press SPACE BAR to enter terminal based configuration...."));
  for (size_t i = 0; i < (3000 / 250); i++)
  {
    SERIAL_DEBUG.print('.');
    while (SERIAL_DEBUG.available())
    {
      int x = SERIAL_DEBUG.read();
      //SPACE BAR
      if (x == 32)
      {
        TerminalBasedWifiSetup(SERIAL_DEBUG);
      }
    }
    delay(250);
  }
  SERIAL_DEBUG.println(F("skipped"));
  SERIAL_DEBUG.flush();

  //Temporarly force WIFI settings
  //wifi_eeprom_settings xxxx;
  //strcpy(xxxx.wifi_ssid,"XXXXXX");
  //strcpy(xxxx.wifi_passphrase,"XXXXXX");
  //Settings::WriteConfig("diybmswifi",(char *)&config, sizeof(config));
  //clearAPSettings = 0;

  if (!DIYBMSSoftAP::LoadConfigFromEEPROM())
  {
    //We have just started up and the EEPROM is empty of configuration
    SetControllerState(ControllerState::ConfigurationSoftAP);

    ESP_LOGI(TAG, "Setup Access Point");
    //We are in initial power on mode (factory reset)
    DIYBMSSoftAP::SetupAccessPoint(&server);
  }
  else
  {

    /* Explicitly set the ESP to be a WiFi-client, otherwise by default,
      would try to act as both a client and an access-point */

    WiFi.onEvent(onWifiConnect, system_event_id_t::SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(onWifiDisconnect, system_event_id_t::SYSTEM_EVENT_STA_DISCONNECTED);
    //Newer IDF version will need this...
    //WiFi.onEvent(onWifiConnect, arduino_event_id_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
    //WiFi.onEvent(onWifiDisconnect, arduino_event_id_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);

    connectToMqtt();

    xTaskCreate(enqueue_task, "enqueue", 1024, nullptr, configMAX_PRIORITIES / 2, &enqueue_task_handle);

    xTaskCreate(rules_task, "rules", 2048, nullptr, configMAX_PRIORITIES - 5, &rule_task_handle);

    xTaskCreate(influxdb_task, "influxdb", 1500, nullptr, configMAX_PRIORITIES - 5, &influxdb_task_handle);

    //We have just started...
    SetControllerState(ControllerState::Stabilizing);

    //Attempt connection in setup(), loop() will also try every 30 seconds
    connectToWifi();

    //Wake screen on power up
    xTaskNotify(tftwakeup_task_handle, 0x00, eNotifyAction::eNoAction);

  }

  //SDM sdm(SERIAL_RS485, 9600, RS485_ENABLE, SERIAL_8N1, RS485_RX, RS485_TX); // pins for DIYBMS => RX pin 21, TX pin 22
  SERIAL_RS485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
}

unsigned long wifitimer = 0;

void loop()
{

  unsigned long currentMillis = millis();

  if (_controller_state != ControllerState::ConfigurationSoftAP)
  {
    //on first pass wifitimer is zero
    if (currentMillis - wifitimer > 30000)
    {
      //Attempt to connect to WiFi every 30 seconds, this caters for when WiFi drops
      //such as AP reboot, its written to return without action if we are already connected
      connectToWifi();
      wifitimer = currentMillis;

      connectToMqtt();
    }
  }

  /*
  if (touchscreen.tirqTouched())
  {
    if (hal.IsVSPIMutexAvailable())
    {
      if (hal.GetVSPIMutex())
      {
        if (touchscreen.touched())
        {
        
      TS_Point p = touchscreen.getPoint();
      SERIAL_DEBUG.print("Pressure = ");
      SERIAL_DEBUG.print(p.z);
      SERIAL_DEBUG.print(", x = ");
      SERIAL_DEBUG.print(p.x);
      SERIAL_DEBUG.print(", y = ");
      SERIAL_DEBUG.print(p.y);
      SERIAL_DEBUG.println();
      
        }

        hal.ReleaseVSPIMutex();
      }
    }
  }
*/
  if (ResetWifi)
  {
    //Password reset, turn LED CYAN
    QueueLED(RGBLED::Cyan);

    //Wipe EEPROM WIFI setting
    DIYBMSSoftAP::FactoryReset();
  }

  ArduinoOTA.handle();

  // Call update to receive, decode and process incoming packets
  myPacketSerial.checkInputStream();
}

