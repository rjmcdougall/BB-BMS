/*
* Description :   Header file of BQ76952 BMS IC (by Texas Instruments) for Arduino platform.
* Author      :   Pranjal Joshi
* Date        :   17/10/2020 
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#include "defines.h"
#include "HAL_ESP32.h"
#include "crc16.h"

enum bq76952_thermistor
{
	TS1,
	TS2,
	TS3,
	HDQ,
	DCHG,
	DDSG
};

enum bq76952_fet
{
	CHG,
	DCH,
	ALL
};

enum bq76952_fet_state
{
	FET_OFF,
	FET_ON
};

enum bq76952_scd_thresh
{
	SCD_10,
	SCD_20,
	SCD_40,
	SCD_60,
	SCD_80,
	SCD_100,
	SCD_125,
	SCD_150,
	SCD_175,
	SCD_200,
	SCD_250,
	SCD_300,
	SCD_350,
	SCD_400,
	SCD_450,
	SCD_500
};

typedef union protection
{
	struct
	{
		uint8_t SC_DCHG : 1;
		uint8_t OC2_DCHG : 1;
		uint8_t OC1_DCHG : 1;
		uint8_t OC_CHG : 1;
		uint8_t CELL_OV : 1;
		uint8_t CELL_UV : 1;
	} bits;
} bq76952_protection_t;

typedef union temperatureProtection
{
	struct
	{
		uint8_t OVERTEMP_FET : 1;
		uint8_t OVERTEMP_INTERNAL : 1;
		uint8_t OVERTEMP_DCHG : 1;
		uint8_t OVERTEMP_CHG : 1;
		uint8_t UNDERTEMP_INTERNAL : 1;
		uint8_t UNDERTEMP_DCHG : 1;
		uint8_t UNDERTEMP_CHG : 1;
	} bits;
} bq76952_temperature_t;

class bq76952
{
public:
	bq76952(uint8_t addr, HAL_ESP32 *hal);
	void begin(void);
	void reset(void);
	bool isConnected(void);
	unsigned int getCellVoltage(byte cellNumber);
	void getAllCellVoltages(unsigned int *cellArray);
	unsigned int getCurrent(void);
	unsigned int getVcellMode(void);
	float getInternalTemp(void);
	float getThermistorTemp(bq76952_thermistor);
	bq76952_protection_t getProtectionStatus(void);
	bq76952_temperature_t getTemperatureStatus(void);
	void setFET(bq76952_fet, bq76952_fet_state);
	bool isDischarging(void);
	bool isCharging(void);
	void setDebug(bool);
	void setCellOvervoltageProtection(unsigned int, unsigned int);
	void setCellUndervoltageProtection(unsigned int, unsigned int);
	void setChargingOvercurrentProtection(byte, byte);
	void setChargingTemperatureMaxLimit(signed int, byte);
	void setDischargingOvercurrentProtection(byte, byte);
	void setDischargingShortcircuitProtection(bq76952_scd_thresh, unsigned int);
	void setDischargingTemperatureMaxLimit(signed int, byte);
	bool getDASTATUS5();
	float MaxCellTemp();
	float MinCellTemp();
	float MaxCellVolt();
	float MinCellVolt();

	// Cache for multi-byte subcommands
	uint8_t subcmdCache[64];

private:
	void initBQ(void);
	unsigned int directCommand(byte value);
	void subCommand(uint16_t value);
	uint16_t subCommandResponseInt(void);
	bool subCommandResponseBlock(uint8_t *data, uint16_t len);
	void enterConfigUpdate(void);
	void exitConfigUpdate(void);
	void writeDataMemory(int16_t addr, uint16_t value, uint8_t len);
	bool readDataMemory(uint16_t addr, uint8_t *data);
	byte computeChecksum(byte oldChecksum, uint8_t data);
	bool read16(uint8_t reg, unsigned int *value);

	HAL_ESP32 *BQhal;
	uint8_t BQaddr;
};