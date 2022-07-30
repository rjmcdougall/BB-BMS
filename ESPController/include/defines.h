/********************************************************************
*
* Omni-present includes
*
********************************************************************/
#pragma once

#include "esp_log.h"
#include <Arduino.h>

/********************************************************************
*
* Diagnostics
*
********************************************************************/

#define DEBUG_TASKS true

/********************************************************************
*
* Tasks
*
********************************************************************/

#define TASK_DEFAULT_PRIORITY 1

// Almost all tasks need >1k of memory for their stack, so just start with 2k
#define TASK_STACK_SIZE_SMALL 2048
#define TASK_STACK_SIZE_MEDIUM 4096
#define TASK_STACK_SIZE_LARGE 8192


/********************************************************************
*
* HAL (Hardware Abstraction Layer)
*
********************************************************************/

#define BQ_ADDR 0x8
#define BQZ_ADDR 0x55

enum RelayState : uint8_t
{
  RELAY_ON = 0xFF,
  RELAY_OFF = 0x99,
  RELAY_X = 0x00
};

enum RGBLED : uint8_t
{
  OFF = 0,
  Blue = B00000001,
  Red = B00000010,
  Purple = B00000011,
  Green = B00000100,
  Cyan = B00000101,
  Yellow = B00000110,
  White = B00000111
};

struct i2cQueueMessage
{
  uint8_t command;
  uint8_t data;
};
