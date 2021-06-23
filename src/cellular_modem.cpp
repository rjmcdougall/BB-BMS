#include "defines.h"
#include "cellular_modem.h"

#define SerialMon Serial
#define SerialAT Serial1

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 650
// Range to attempt to autobaud
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200
#define TINY_GSM_USE_GPRS true

// set GSM PIN, if any
#define GSM_PIN ""


const char apn[]  = "h2g2";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char gsm_server[] = "vsh.pp.ua";
const char gsm_resource[] = "/TinyGSM/logo.txt";
#include <TinyGsmClient.h>

#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);

static const char *TAG = "Cell Modem";

cellular_modem::cellular_modem() {
}

bool cellular_modem::init() {

ESP_LOGD(TAG,"Setting up cell modem...");
// Set GSM module baud rate
  //TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  SerialAT.begin(9600);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
    ESP_LOGD(TAG,"Initializing cell modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  ESP_LOGD(TAG, "Modem Info: %s", modemInfo);

#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }
#endif

}

bool cellular_modem::loop() {

}