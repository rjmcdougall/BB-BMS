/*
* Description :   Simulator file for battery modules
* Author      :   Jos Boumans
* Date        :   05/07/2022
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

// Use the same signatures as the real module
#include "bq_simulator.h"
static const char *TAG = "bq_simulator";

bool BQ_SIMULATOR_DEBUG = true;

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
void bq_simulator::initBQ()
{
}

unsigned int bq_simulator::directCommand(uint8_t command)
{
  ESP_LOGI(TAG,"Simluated directCommand");
  return 42;
}


bool bq_simulator::read16(uint8_t reg, unsigned int *value)
{
    ESP_LOGI(TAG,"Simluated read16");
    return true;
}


void bq_simulator::subCommand(uint16_t value)
{
    ESP_LOGI(TAG,"Simluated subCommand");
    return;
}

uint16_t bq_simulator::subCommandResponseInt(void)
{
  ESP_LOGI(TAG,"Simluated subCommandResponseInt");
  return 42;
}

bool bq_simulator::subCommandResponseBlock(uint8_t *data, uint16_t len)
{
    ESP_LOGI(TAG,"Simluated subCommandResponseBlock");
   return true;
}

void bq_simulator::enterConfigUpdate(void)
{
  ESP_LOGI(TAG,"Simluated enterConfigUpdate");
}
void bq_simulator::exitConfigUpdate(void)
{
  ESP_LOGI(TAG,"Simluated exitConfigUpdate");
}

void bq_simulator::writeDataMemory(int16_t addr, uint16_t value, uint8_t len)
{
  ESP_LOGI(TAG,"Simluated writeDataMemory");
  exitConfigUpdate();
}

bool bq_simulator::readDataMemory(uint16_t addr, uint8_t *data)
{
  ESP_LOGI(TAG,"Simluated readDataMemory");
  return true;
}

bool bq_simulator::isConnected(void) {
  return BQhal->isConnected(port, BQaddr);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////


/////// API FUNCTIONS ///////

bq_simulator::bq_simulator(uint8_t addr, HAL_ESP32 *hal, i2c_port_t p) {
	// Constructor
  BQhal = hal;
  BQaddr = addr;
  port = p;
  //pinMode(alertPin, INPUT);
  // TODO - Attach IRQ here
}

void bq_simulator::begin(void) {
  initBQ();
  if(BQ_SIMULATOR_DEBUG) {
    ESP_LOGI(TAG,"Initializing BQ Simulator");
  }
}



// Reset the BQ chip
void bq_simulator::reset(void) {
  ESP_LOGI(TAG,"Simulated reset");
  return;
}

// Read single cell voltage
unsigned int bq_simulator::getCellVoltage(byte cellNumber) {
  ESP_LOGI(TAG,"Simulated getCellVoltage");
  return 21;
}

// Read All cell voltages in given array - Call like readAllCellVoltages(&myArray)
void bq_simulator::getAllCellVoltages(unsigned int* cellArray) {
  ESP_LOGI(TAG,"Simulated getAllCellVoltages");
  return;
}

// Get Cell Balancing Status per cell
void bq_simulator::getCellBalanceStatus(bool *cellArray){
  ESP_LOGI(TAG,"Simulated getCellBalanceTimes");
  return;
}

// Get balance times per cell
void bq_simulator::getCellBalanceTimes(uint32_t *cellArray) {
    ESP_LOGI(TAG,"Simulated getCellBalanceTimes");
    return;
}

// Measure CC2 current
unsigned int bq_simulator::getCurrent(void) {
ESP_LOGI(TAG,"Simulated getCurrent");
return 42;
}

// Measure chip temperature in °C
float bq_simulator::getInternalTemp(void) {
  ESP_LOGI(TAG,"Simulated getInternalTemp");
  return 42.0;
}

// Get DASTATUS5
bool bq_simulator::getDASTATUS5() {
  ESP_LOGI(TAG,"Simulated getDASTATUS5");
  return true;
}

float bq_simulator::MaxCellTemp() {
  ESP_LOGI(TAG,"Simulated MaxCellTemp");
  return 84.0;
}

float bq_simulator::MinCellTemp() {
  ESP_LOGI(TAG,"Simulated MinCellTemp");
  return 42.0;
}

float bq_simulator::MaxCellVolt() {
  ESP_LOGI(TAG,"Simulated MaxCellVolt");
  return 4.2;
}

float bq_simulator::MinCellVolt() {
  ESP_LOGI(TAG,"Simulated MaxCellVolt");
  return 2.1;
}

// Measure thermistor temperature in °C
float bq_simulator::getThermistorTemp(bq_simulator_thermistor thermistor) {
  ESP_LOGI(TAG,"Simulated getThermistorTemp");
  return 42.0;
}

// Check Primary Protection status
bq_simulator_protection_t bq_simulator::getProtectionStatus(void) {
  ESP_LOGI(TAG,"Simulated getProtectionStatus");

  bq_simulator_protection_t prot;
  return prot;
}

// Check Temperature Protection status
bq_simulator_temperature_t bq_simulator::getTemperatureStatus(void) {
  ESP_LOGI(TAG,"Simulated getTemperatureStatus");

  bq_simulator_temperature_t prot;
  return prot;
}

void bq_simulator::setFET(bq_simulator_fet fet, bq_simulator_fet_state state) {
}

// is Charging FET ON?
bool bq_simulator::isCharging(void) {
  ESP_LOGI(TAG,"Simulated isCharging");
  return true;
}

// is Discharging FET ON?
bool bq_simulator::isDischarging(void) {    
  ESP_LOGI(TAG,"Simulated isDischarging");
  return true;
}

// Read vcell mode
unsigned int bq_simulator::getVcellMode(void) {
    ESP_LOGI(TAG,"Simulated vcellMode");
    return 0;
}

// Set user-defined overvoltage protection
void bq_simulator::setCellOvervoltageProtection(unsigned int mv, unsigned int ms) {
}

// Set user-defined undervoltage protection
void bq_simulator::setCellUndervoltageProtection(unsigned int mv, unsigned int ms) {
}

// Set user-defined charging current protection
void bq_simulator::setChargingOvercurrentProtection(byte mv, byte ms) {
}

// Set user-defined discharging current protection
void bq_simulator::setDischargingOvercurrentProtection(byte mv, byte ms) {
}

// Set user-defined discharging current protection
void bq_simulator::setDischargingShortcircuitProtection(bq_simulator_scd_thresh thresh, unsigned int us) {
}

// Set user-defined charging over temperature protection
void bq_simulator::setChargingTemperatureMaxLimit(signed int temp, byte sec) {
}

// Set user-defined discharging over temperature protection
void bq_simulator::setDischargingTemperatureMaxLimit(signed int temp, byte sec) {
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

