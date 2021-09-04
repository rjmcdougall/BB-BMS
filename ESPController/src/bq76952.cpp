/*
* Description :   Source file of BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   Pranjal Joshi
* Date        :   17/10/2020 
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#include "bq76952.h"
static const char *TAG = "bq76952";

// Library config
//#define DBG_BAUD            115200
//#define BQ_I2C_ADDR_WRITE   0x10
//#define BQ_I2C_ADDR_READ    0x11
bool BQ_DEBUG = false;

// BQ76952 - Address Map
#define CMD_DIR_SUBCMD_LOW            0x3E
#define CMD_DIR_SUBCMD_HI             0x3F
#define CMD_DIR_RESP_LEN              0x61
#define CMD_DIR_RESP_START            0x40
#define CMD_DIR_RESP_CHKSUM           0x60

// BQ76952 - Voltage measurement commands
#define CMD_READ_VOLTAGE_CELL_1   0x14
#define CMD_READ_VOLTAGE_CELL_2   0x16
#define CMD_READ_VOLTAGE_CELL_3   0x18
#define CMD_READ_VOLTAGE_CELL_4   0x1A
#define CMD_READ_VOLTAGE_CELL_5   0x1C
#define CMD_READ_VOLTAGE_CELL_6   0x1E
#define CMD_READ_VOLTAGE_CELL_7   0x20
#define CMD_READ_VOLTAGE_CELL_8   0x22
#define CMD_READ_VOLTAGE_CELL_9   0x24
#define CMD_READ_VOLTAGE_CELL_10  0x26
#define CMD_READ_VOLTAGE_CELL_11  0x28
#define CMD_READ_VOLTAGE_CELL_12  0x2A
#define CMD_READ_VOLTAGE_CELL_13  0x2C
#define CMD_READ_VOLTAGE_CELL_14  0x2E
#define CMD_READ_VOLTAGE_CELL_15  0x30
#define CMD_READ_VOLTAGE_CELL_16  0x32
#define CMD_READ_VOLTAGE_STACK    0x34
#define CMD_READ_VOLTAGE_PACK     0x36

// BQ76952 - Direct Commands
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

/*
0-1 VREG18 16-bit ADC counts
2–3 VSS 16-bit ADC counts
4–5 Max Cell Voltage mV
6–7 Min Cell Voltage mV
8–9 Battery Voltage Sum cV
10–11 Avg Cell Temperature 0.1 K
12–13 FET Temperature 0.1 K
14–15 Max Cell Temperature 0.1 K
16–17 Min Cell Temperature 0.1 K
18–19 Avg Cell Temperature 0.1 K
20–21 CC3 Current userA
22–23 CC1 Current userA
24–27 Raw CC2 Counts 32-bit ADC counts
28–31 Raw CC3 Counts 32-bit ADC counts
*/


// 0x0075 DASTATUS5() Subcommand
#define CMD_DASTATUS5 0x75
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


/* 0x68 Int Temperature Internal die temperature
0x6A CFETOFF Temperature CFETOFF pin thermistor
0x6C DFETOFF Temperature DFETOFF pin thermistor
0x6E ALERT Temperature ALERT pin thermistor
0x70 TS1 Temperature TS1 pin thermistor
0x72 TS2 Temperature TS2 pin thermistor
0x74 TS3 Temperature TS3 pin thermistor
0x76 HDQ Temperature HDQ pin thermistor
0x78 DCHG Temperature DCHG pin thermistor
0x7A DDSG Temperature DDSG pin thermistor
*/

// Alert Bits in BQ76952 registers
#define BIT_SA_SC_DCHG            7
#define BIT_SA_OC2_DCHG           6
#define BIT_SA_OC1_DCHG           5
#define BIT_SA_OC_CHG             4
#define BIT_SA_CELL_OV            3
#define BIT_SA_CELL_UV            2

#define BIT_SB_OTF                7
#define BIT_SB_OTINT              6
#define BIT_SB_OTD                5
#define BIT_SB_OTC                4
#define BIT_SB_UTINT              2
#define BIT_SB_UTD                1
#define BIT_SB_UTC                0

// Inline functions
#define CELL_NO_TO_ADDR(cellNo) (0x14 + ((cellNo-1)*2))
#define LOW_BYTE(data) (byte)(data & 0x00FF)
#define HIGH_BYTE(data) (byte)((data >> 8) & 0x00FF)

//// LOW LEVEL FUNCTIONS ////


/*
    initBQ(void);
	unsigned int directCommand(byte);
	void subCommand(unsigned int);
	unsigned int subCommandResponseInt(void);
	void enterConfigUpdate(void);
	void exitConfigUpdate(void);
	byte computeChecksum(byte, byte);
	void writeDataMemory(unsigned int , unsigned int, byte);
	byte readDataMemory(unsigned int);
    */
void bq76952::initBQ()
{
}

unsigned int bq76952::directCommand(uint8_t command)
{
    esp_err_t ret;
    uint8_t value;

    ESP_LOGD(TAG,"directCommand Send cmd %i", command);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->directCommand(I2C_NUM_0, BQaddr, command, &value[0], 2));
    ret = BQhal->readByte(I2C_NUM_0, BQaddr, command, &value);
    //ret = value[0] | (value[1] << 8);
    //ESP_LOGD(TAG,"directCommand Reply %i", ret);
    vTaskDelay(pdMS_TO_TICKS(10));
    return value;
}

bool bq76952::read16(uint8_t reg, unsigned int *value)
{
    unsigned int ret;
    uint8_t tmp_value[2];

    //ESP_LOGD(TAG,"read16 Send cmd %i", reg);
    //ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->directCommand(I2C_NUM_0, BQaddr, command, &value[0], 2));
    ret = BQhal->readByte(I2C_NUM_0, BQaddr, reg, &tmp_value[0]);
    if (ret != ESP_OK) {
        return false;
    }
    ret = BQhal->readByte(I2C_NUM_0, BQaddr, reg + 1, &tmp_value[1]);
    if (ret != ESP_OK) {
        return false;
    }
    //ESP_LOGD(TAG,"read16 Reply %i", ret);
    *value = tmp_value[0] | (tmp_value[1] << 8);
    return true;
}


void bq76952::subCommand(uint16_t value)
{
    //ESP_LOGD(TAG,"BQsubCommand Send %i", value);
    uint8_t cmd[2];
    cmd[0] = LOW_BYTE(value);
    cmd[1] = HIGH_BYTE(value);
    ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->writeMultipleBytes(I2C_NUM_0, BQaddr, CMD_DIR_SUBCMD_LOW, cmd, 2));
    vTaskDelay(pdMS_TO_TICKS(10));
}

uint16_t bq76952::subCommandResponseInt(void)
{
    uint8_t value[2];
    int16_t ret;

    //ESP_LOGD(TAG,"BQsubCommandResponseInt Send");
    ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->readMultipleBytes(I2C_NUM_0, BQaddr, CMD_DIR_RESP_START, &value[0], 2));
    ret = value[0] | value[1] << 8;
    //ESP_LOGD(TAG,"BQsubCommandResponseInt Reply %i", ret);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

bool bq76952::subCommandResponseBlock(uint8_t *data, uint16_t len)
{
    bool ret;
    uint8_t ret_len;

    vTaskDelay(pdMS_TO_TICKS(10));
    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Check len");
    ret = BQhal->readByte(I2C_NUM_0, BQaddr, CMD_DIR_RESP_LEN, &ret_len);
    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Check len = %d, ret = %d", ret_len, ret);
    if (ret_len > 40) {
      ret_len = 40;
    }

    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Send");

    vTaskDelay(pdMS_TO_TICKS(10));
    for (int i = 0; i < ret_len; i++) {
      ret = BQhal->readByte(I2C_NUM_0, BQaddr, CMD_DIR_RESP_START + i, data + i);
      vTaskDelay(pdMS_TO_TICKS(10));
      //ESP_LOGD(TAG,"BQsubCommandResponseBlock data %x", data[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    //ESP_LOGD(TAG,"BQsubCommandResponseBlock Reply %i", ret);
    return ret;
}

void bq76952::enterConfigUpdate(void)
{
    subCommand(0x0090);
}
void bq76952::exitConfigUpdate(void)
{
    subCommand(0x0092);
}

void bq76952::writeDataMemory(int16_t addr, uint16_t value, uint8_t len)
{
    uint8_t data[4];

    if (len > 4) {
        return;
    }

    byte chksum = 0;
    chksum = computeChecksum(chksum, BQaddr);
    chksum = computeChecksum(chksum, CMD_DIR_SUBCMD_LOW);
    chksum = computeChecksum(chksum, LOW_BYTE(addr));
    chksum = computeChecksum(chksum, HIGH_BYTE(addr));
    chksum = computeChecksum(chksum, value);

    enterConfigUpdate();

    data[0] = LOW_BYTE(addr);
    data[1] = HIGH_BYTE(addr);
    data[2] = LOW_BYTE(value);
    data[3] = HIGH_BYTE(value);
    ESP_LOGD(TAG,"BQwriteDataMemory Send data %x", value);
    ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->writeMultipleBytes(I2C_NUM_0, BQaddr, CMD_DIR_SUBCMD_LOW, data, len + 2));

    data[0] = chksum;
    data[1] = 0x05;
    ESP_LOGD(TAG,"BQwriteDataMemory Send checksum %x", value);
    ESP_ERROR_CHECK_WITHOUT_ABORT(BQhal->writeMultipleBytes(I2C_NUM_0, BQaddr, CMD_DIR_RESP_CHKSUM, data, 2));

    exitConfigUpdate();
}

bool bq76952::readDataMemory(uint16_t addr, uint8_t *data)
{
    ESP_LOGD(TAG,"BQreadDataMemory Send Addr %i", addr);

    data[0] = LOW_BYTE(addr);
    data[1] = HIGH_BYTE(addr);

    esp_err_t ret = BQhal->writeMultipleBytes(I2C_NUM_0, BQaddr, CMD_DIR_SUBCMD_LOW, data, 2);
    if (ret != ESP_OK) {
      ESP_LOGD(TAG, "writecmd failed");
      return false;
    }

    ESP_LOGD(TAG,"BQreadDataMemory Send Read Req %i", addr);
    ret = BQhal->readByte(I2C_NUM_0, BQaddr, CMD_DIR_RESP_START, data);
    ESP_LOGD(TAG,"BQreadDataMemory Reply %i", ret);
    return (ret == ESP_OK);
}

bool bq76952::isConnected(void) {
  return BQhal->isConnected(I2C_NUM_0, BQaddr);
}

#ifdef OLD_BQ

void bq76952::initBQ(void) {
  Wire.begin();
}

// Send Direct command
unsigned int bq76952::directCommand(byte command) {
  Wire.beginTransmission(BQ_I2C_ADDR_WRITE);
  Wire.write(command);
  Wire.endTransmission();

  Wire.requestFrom(BQ_I2C_ADDR_READ, 2);
  while(!Wire.available());
  byte lsb = Wire.read();
  byte msb = Wire.read();

  debugPrint(F("[+] Direct Cmd SENT -> "));
  debugPrintlnCmd((uint16_t)command);
  debugPrint(F("[+] Direct Cmd RESP <- "));
  debugPrintlnCmd((uint16_t)(msb << 8 | lsb));

  return (unsigned int)(msb << 8 | lsb);
}

// Send Sub-command
void bq76952::subCommand(unsigned int data) {
  Wire.beginTransmission(BQ_I2C_ADDR_WRITE);
  Wire.write(CMD_DIR_SUBCMD_LOW);
  Wire.write((byte)data & 0x00FF);
  Wire.write((byte)(data >> 8) & 0x00FF);
  Wire.endTransmission();

  debugPrint(F("[+] Sub Cmd SENT to 0x3E -> "));
  debugPrintlnCmd((uint16_t)data);
}

// Read subcommand response
unsigned int bq76952::subCommandResponseInt(void) {
  Wire.beginTransmission(BQ_I2C_ADDR_WRITE);
  Wire.write(CMD_DIR_RESP_START);
  Wire.endTransmission();

  Wire.requestFrom(BQ_I2C_ADDR_READ, 2);
  while(!Wire.available());
  byte lsb = Wire.read();
  byte msb = Wire.read();

  debugPrint(F("[+] Sub Cmd uint16_t RESP at 0x40 -> "));
  debugPrintlnCmd((uint16_t)(msb << 8 | lsb));

  return (unsigned int)(msb << 8 | lsb);
}

// Enter config update mode
void bq76952::enterConfigUpdate(void) {
  subCommand(0x0090);
  delayMicroseconds(2000);
}

// Exit config update mode
void bq76952::exitConfigUpdate(void) {
  subCommand(0x0092);
  delayMicroseconds(1000);
}

// Write Byte to Data memory of BQ76952
void bq76952::writeDataMemory(unsigned int addr, unsigned int data, byte noOfBytes) {
  byte chksum = 0;
  chksum = computeChecksum(chksum, BQ_I2C_ADDR_WRITE);
  chksum = computeChecksum(chksum, CMD_DIR_SUBCMD_LOW);
  chksum = computeChecksum(chksum, LOW_BYTE(addr));
  chksum = computeChecksum(chksum, HIGH_BYTE(addr));
  chksum = computeChecksum(chksum, data);

  enterConfigUpdate();
  Wire.beginTransmission(BQ_I2C_ADDR_WRITE);
  Wire.write(CMD_DIR_SUBCMD_LOW);
  Wire.write(LOW_BYTE(addr));
  Wire.write(HIGH_BYTE(addr));
  Wire.write(LOW_BYTE(data));
  if(noOfBytes == 2)
    Wire.write(HIGH_BYTE(data));
  Wire.endTransmission();

  Wire.beginTransmission(BQ_I2C_ADDR_WRITE);
  Wire.write(CMD_DIR_RESP_CHKSUM);
  Wire.write(chksum);
  Wire.write(0x05);
  Wire.endTransmission();
  exitConfigUpdate();
}

// Read Byte from Data memory of BQ76952
byte bq76952::readDataMemory(unsigned int addr) {
  Wire.beginTransmission(BQ_I2C_ADDR_WRITE);
  Wire.write(CMD_DIR_SUBCMD_LOW);
  Wire.write(LOW_BYTE(addr));
  Wire.write(HIGH_BYTE(addr));
  Wire.endTransmission();

  Wire.beginTransmission(BQ_I2C_ADDR_WRITE);
  Wire.write(CMD_DIR_RESP_START);
  Wire.endTransmission();

  Wire.requestFrom(BQ_I2C_ADDR_READ, 1);
  while(!Wire.available());
  return (byte)Wire.read();
}

#endif //OLD_BQ

// Compute checksome = ~(sum of all bytes)
byte bq76952::computeChecksum(byte oldChecksum, byte data) {
  if(!oldChecksum)
    oldChecksum = data;
  else
    oldChecksum = ~(oldChecksum) + data;
  return ~(oldChecksum);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////


/////// API FUNCTIONS ///////

bq76952::bq76952(uint8_t addr, HAL_ESP32 *hal) {
	// Constructor
    BQhal = hal;
    BQaddr = addr;
  //pinMode(alertPin, INPUT);
  // TODO - Attach IRQ here
}

void bq76952::begin(void) {
  initBQ();
  if(BQ_DEBUG) {
    ESP_LOGD(TAG,"Initializing BQ76952...");
  }
}



// Reset the BQ chip
void bq76952::reset(void) {
    subCommand(0x0012);
    ESP_LOGD(TAG,"Resetting BQ76952...");
}

// Read single cell voltage
unsigned int bq76952::getCellVoltage(byte cellNumber) {
  unsigned int value;
  if (read16(CELL_NO_TO_ADDR(cellNumber), &value)) {
      return value;
  } else {
      return 0;
  }
}

// Read All cell voltages in given array - Call like readAllCellVoltages(&myArray)
void bq76952::getAllCellVoltages(unsigned int* cellArray) {
  // Additional 3 to read top of stack voltages
    for(byte x=0;x<19;x++)
//  for(byte x=1;x<2;x++)
    cellArray[x] = getCellVoltage(x + 1);
}

// Measure CC2 current
unsigned int bq76952::getCurrent(void) {
  return directCommand(CMD_DIR_CC2_CUR);
}

// Measure chip temperature in °C
float bq76952::getInternalTemp(void) {
  float raw = directCommand(CMD_DIR_INT_TEMP)/10.0;
  return (raw - 273.15);
}

// Get DASTATUS5
bool bq76952::getDASTATUS5() {
  unsigned int value;
  subCommand(CMD_DASTATUS5);
 /* 
  for (int i = 0; i < 10; i++) {
    read16(CMD_DASTATUS5, &value);
    ESP_LOGD(TAG," CMD_DASTATUS5 status %x", value);
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  */

  return subCommandResponseBlock(&subcmdCache[0], 32);
}

float bq76952::MaxCellTemp() {
   float raw  = (subcmdCache[DASTATUS_TEMPCELLMAX] | (subcmdCache[DASTATUS_TEMPCELLMAX + 1] << 8)) / 10.0;
  return (raw - 273.15);
}

float bq76952::MinCellTemp() {
   float raw  = (subcmdCache[DASTATUS_TEMPCELLMIN] | (subcmdCache[DASTATUS_TEMPCELLMIN + 1] << 8)) / 10.0;
  return (raw - 273.15);
}

float bq76952::MaxCellVolt() {
  float raw  = (subcmdCache[DASTATUS_MAXCELL] | (subcmdCache[DASTATUS_MAXCELL + 1] << 8)) / 1000.0;
  return (raw);
}

float bq76952::MinCellVolt() {
  float raw  = (subcmdCache[DASTATUS_MINCELL] | (subcmdCache[DASTATUS_MINCELL + 1] << 8)) / 1000.0;
  return (raw);
}



// Measure thermistor temperature in °C
float bq76952::getThermistorTemp(bq76952_thermistor thermistor) {
  byte cmd = 0;
  switch(thermistor) {
    case TS1:
      cmd = 0x70;
      break;
    case TS2:
      cmd = 0x72;
      break;
    case TS3:
      cmd = 0x74;
      break;
    case HDQ:
      cmd = 0x76;
      break;
    case DCHG:
      cmd = 0x78;
      break;
    case DDSG:
      cmd = 0x7A;
      break;
  }
  if (cmd) {
      float raw = directCommand(cmd)/10.0;
      return (raw - 273.15);
  }
  return 0.0;
}

// Check Primary Protection status
bq76952_protection_t bq76952::getProtectionStatus(void) {
  bq76952_protection_t prot;
  byte regData = (byte)directCommand(CMD_DIR_FPROTEC);
  prot.bits.SC_DCHG = bitRead(regData, BIT_SA_SC_DCHG);
  prot.bits.OC2_DCHG = bitRead(regData, BIT_SA_OC2_DCHG);
  prot.bits.OC1_DCHG = bitRead(regData, BIT_SA_OC1_DCHG);
  prot.bits.OC_CHG = bitRead(regData, BIT_SA_OC_CHG);
  prot.bits.CELL_OV = bitRead(regData, BIT_SA_CELL_OV);
  prot.bits.CELL_UV = bitRead(regData, BIT_SA_CELL_UV);
  return prot;
}

// Check Temperature Protection status
bq76952_temperature_t bq76952::getTemperatureStatus(void) {
  bq76952_temperature_t prot;
  byte regData = (byte)directCommand(CMD_DIR_FTEMP);
  prot.bits.OVERTEMP_FET = bitRead(regData, BIT_SB_OTC);
  prot.bits.OVERTEMP_INTERNAL = bitRead(regData, BIT_SB_OTINT);
  prot.bits.OVERTEMP_DCHG = bitRead(regData, BIT_SB_OTD);
  prot.bits.OVERTEMP_CHG = bitRead(regData, BIT_SB_OTC);
  prot.bits.UNDERTEMP_INTERNAL = bitRead(regData, BIT_SB_UTINT);
  prot.bits.UNDERTEMP_DCHG = bitRead(regData, BIT_SB_UTD);
  prot.bits.UNDERTEMP_CHG = bitRead(regData, BIT_SB_UTC);
  return prot;
}

void bq76952::setFET(bq76952_fet fet, bq76952_fet_state state) {
  unsigned int subcmd;
  switch(state) {
    case FET_OFF:
      switch(fet) {
        case DCH:
          subcmd = 0x0093;
          break;
        case CHG:
          subcmd = 0x0094;
          break;
        default:
          subcmd = 0x0095;
          break;
      }
      break;
    case FET_ON:
      subcmd = 0x0096;
      break;
  }
  subCommand(subcmd);
}

// is Charging FET ON?
bool bq76952::isCharging(void) {
  byte regData = (byte)directCommand(CMD_DIR_FET_STAT);
  if(regData & 0x01) {
    ESP_LOGD(TAG,"Charging FET -> ON");
    return true;
  }
  ESP_LOGD(TAG,"Charging FET -> OFF");
  return false;
}

// is Discharging FET ON?
bool bq76952::isDischarging(void) {
  byte regData = (byte)directCommand(CMD_DIR_FET_STAT);
  if(regData & 0x04) {
    ESP_LOGD(TAG,"Discharging FET -> ON");
    return true;
  }
  ESP_LOGD(TAG,"Discharging FET -> OFF");
  return false;
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

// Set user-defined overvoltage protection
void bq76952::setCellOvervoltageProtection(unsigned int mv, unsigned int ms) {
  byte thresh = (byte)mv/50.6;
  uint16_t dly = (uint16_t)(ms/3.3)-2;
  if(thresh < 20 || thresh > 110)
    thresh = 86;
  else {
    ESP_LOGD(TAG,"COV Threshold => ", thresh);
    writeDataMemory(0x9278, thresh, 1);
  }
  if(dly < 1 || dly > 2047)
    dly = 74;
  else {
    ESP_LOGD(TAG,"COV Delay => %i", dly);
    writeDataMemory(0x9279, dly, 2);
  }
}

// Set user-defined undervoltage protection
void bq76952::setCellUndervoltageProtection(unsigned int mv, unsigned int ms) {
  byte thresh = (byte)mv/50.6;
  uint16_t dly = (uint16_t)(ms/3.3)-2;
  if(thresh < 20 || thresh > 90)
    thresh = 50;
  else {
    ESP_LOGD(TAG,"CUV Threshold => %i", thresh);
    writeDataMemory(0x9275, thresh, 1);
  }
  if(dly < 1 || dly > 2047)
    dly = 74;
  else {
    ESP_LOGD(TAG,"CUV Delay => %i", dly);
    writeDataMemory(0x9276, dly, 2);
  }
}

// Set user-defined charging current protection
void bq76952::setChargingOvercurrentProtection(byte mv, byte ms) {
  byte thresh = (byte)mv/2;
  byte dly = (byte)(ms/3.3)-2;
  if(thresh < 2 || thresh > 62)
    thresh = 2;
  else {
    ESP_LOGD(TAG,"OCC Threshold => %i", thresh);
    writeDataMemory(0x9280, thresh, 1);
  }
  if(dly < 1 || dly > 127)
    dly = 4;
  else {
    ESP_LOGD(TAG,"OCC Delay => %i", dly);
    writeDataMemory(0x9281, dly, 1);
  }
}

// Set user-defined discharging current protection
void bq76952::setDischargingOvercurrentProtection(byte mv, byte ms) {
  byte thresh = (byte)mv/2;
  byte dly = (byte)(ms/3.3)-2;
  if(thresh < 2 || thresh > 100)
    thresh = 2;
  else {
    ESP_LOGD(TAG,"OCD Threshold => %i", thresh);
    writeDataMemory(0x9282, thresh, 1);
  }
  if(dly < 1 || dly > 127)
    dly = 1;
  else {
    ESP_LOGD(TAG,"OCD Delay => %i", dly);
    writeDataMemory(0x9283, dly, 1);
  }
}

// Set user-defined discharging current protection
void bq76952::setDischargingShortcircuitProtection(bq76952_scd_thresh thresh, unsigned int us) {
  byte dly = (byte)(us/15)+1;
  ESP_LOGD(TAG,"SCD Threshold => %i", thresh);
  writeDataMemory(0x9286, thresh, 1);
  if(dly < 1 || dly > 31)
    dly = 2;
  else {
    ESP_LOGD(TAG,"SCD Delay (uS) => %i", dly);
    writeDataMemory(0x9287, dly, 1);
  }
}

// Set user-defined charging over temperature protection
void bq76952::setChargingTemperatureMaxLimit(signed int temp, byte sec) {
  if(temp < -40 || temp > 120)
    temp = 55;
  else {
    ESP_LOGD(TAG,"OTC Threshold => %i", temp);
    writeDataMemory(0x929A, temp, 1);
  }
  if(sec< 0 || sec > 255)
    sec = 2;
  else {
    ESP_LOGD(TAG,"OTC Delay => %i", sec);
    writeDataMemory(0x929B, sec, 1);
  }
}

// Set user-defined discharging over temperature protection
void bq76952::setDischargingTemperatureMaxLimit(signed int temp, byte sec) {
  if(temp < -40 || temp > 120)
    temp = 60;
  else {
    ESP_LOGD(TAG,"OTD Threshold => %i", temp);
    writeDataMemory(0x929D, temp, 1);
  }
  if(sec< 0 || sec > 255)
    sec = 2;
  else {
    ESP_LOGD(TAG,"OTD Delay => %i", sec);
    writeDataMemory(0x929E, sec, 1);
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

