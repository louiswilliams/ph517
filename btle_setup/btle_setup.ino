#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BLEGatt.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#define BT_RX 8
#define BT_TX 9
#define BT_MOD 10
#define BT_CTS 11
#define BT_RTS 12

#define VERBOSE_MODE false
#define FACTORY_RESET false
#define SETUP true

SoftwareSerial bluefruitSS = SoftwareSerial(BT_RX,BT_TX); // RX, TX
Adafruit_BluefruitLE_UART ble(bluefruitSS, BT_MOD, BT_CTS, BT_RTS); // MOD, CTS, RTS
//Adafruit_BluefruitLE_UART ble(Serial2, 9); // MOD, CTS, RTS
Adafruit_BLEGatt gatt(ble);

uint8_t service_uuid[] = {0xB3, 0x4A, 0x10, 0x00, 0x23, 0x03, 0x47, 0xC5, 0x83, 0xD5, 0x86, 0x83, 0x62, 0xDE, 0xEB, 0xA6};

// A small helper
void error(const __FlashStringHelper*err) {
//  Serial.println(err);
  while (1);
}

void setup(void) {
  delay(500);

  Serial.begin(115200);

  /* Check if it's using the faster rate. Then use slower rate to change back to the faster rate */
  Serial.println(F("Setting up Bluefruit. Trying 57600 baud"));
  if ( !ble.begin(VERBOSE_MODE, 57600) ) {
    Serial.println(F("Trying 9600 baud"));
    if (!ble.begin(VERBOSE_MODE, 9600)) {
      error(F("Couldn't find Bluefruit")); 
    } else {
      Serial.println(F("Increasing baud rate to 57600"));
      ble.println(F("AT+BAUDRATE=57600"));
      ble.waitForOK();
      ble.println(F("ATZ"));
      Serial.println(F("Resetting..."));
      if (!ble.begin(VERBOSE_MODE, 57600)) {
        error(F("Couldn't find Bluefruit"));
      }
    }
  }

  Serial.println( F("OK!") );

  if ( FACTORY_RESET )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  if (SETUP){ 
    ble.echo(true);

  // Setup GATT service
    gatt.clear();
    gatt.addService(service_uuid);

    gatt.addCharacteristic(0x1001, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "batt_voltage", NULL);
    gatt.addCharacteristic(0x1002, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "batt_current", NULL);
    gatt.addCharacteristic(0x1003, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "batt_amphrs", NULL);
    gatt.addCharacteristic(0x1004, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "batt_soc", NULL);
    gatt.addCharacteristic(0x1005, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "batt_time", NULL);
    gatt.addCharacteristic(0x1006, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "batt_temp", NULL);
    gatt.addCharacteristic(0x1007, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "motor_rpm", NULL);
    gatt.addCharacteristic(0x1008, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "motor_temp", NULL);
    gatt.addCharacteristic(0x1009, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "motor_current", NULL);
    gatt.addCharacteristic(0x100A, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "motor_voltage", NULL);
    gatt.addCharacteristic(0x100B, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "motor_stator", NULL);
    gatt.addCharacteristic(0x100C, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "engine_rpm", NULL);
    gatt.addCharacteristic(0x100D, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "engine_fuel", NULL);
    
    ble.println("ATZ");
    ble.waitForOK();
    Serial.println("Setup done");
    ble.echo(false);
  }

//  /* Disable command echo from Bluefruit */
//  ble.echo(false);
}

short value = 0;

char charBuf[32];

void loop() {

  String msg = "value: ";
  String v = String(value);
  v.toCharArray(charBuf, 32);
  
  Serial.println(msg + value);
  for (int i = 1; i <= 13; i++) {
    bool isOK = gatt.setChar(i, charBuf);
    if (!isOK) {
      Serial.print("Error setting characteristic value for "); 
      Serial.println(i);
    }

  }

  value++;
}

