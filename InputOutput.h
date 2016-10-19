/**
  @author Louis Williams
  @date 2016

  PH517 Controls Runner
**/

#ifndef _InputOutput_H
#define _InputOutput_H

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include "mcp_can.h"

// Digital pins
#define ENGINE_PIN 4
#define SWITCH_IN_START 32
#define SWITCH_IN_END 35
#define SWITCH_LED_START 36
#define SWITCH_LED_END 39
#define RELAY_PIN_START 22
#define RELAY_PIN_END 27
// Analog pins
#define THROTTLE_IN A1
#define THROTTLE_COMP A2
#define BRAKE_IN A3
#define BATT_TEMP_IN A4
// I2C Addresses
#define MOTOR_ACCEL_ADDR 0x62
#define MOTOR_REGEN_ADDR 0x63
// Digital Analog Converter command
#define CMD_WRITE_DAC 0x40
// CAN bus over SPI
#define CAN_CS 9
// Serial ports
#define MICRO_HWSERIAL Serial1
#define BT_HWSERIAL Serial2
#define BATT_HWSERIAL Serial3

// Battery data from SOC meter
typedef struct {
  uint16_t voltage; // x0.01V
  uint16_t current; // x0.01A
  uint16_t ampHours; // x0.1Ah
  uint16_t stateOfCharge; // x0.1%
  uint16_t timeToGo; // x1 min
  uint16_t temp; // x1/256C
  uint8_t currentSign; // 1 for positive, 0 for negative
  uint8_t ampHourSign; // 1 for positive, 0 for negative
} BattData;

// CAN data from motor
typedef struct {
  uint16_t rpm; // x1
  uint8_t temp; // [0,240] - 40
  uint8_t controllerTemp; // [0,240] - 40
  uint16_t rmsCurrent; // x10
  uint16_t capVoltage; // x10
  uint16_t statorFreq; // x1
} MotorData;

// Engine data from Arduino Micro  
typedef struct {
  uint16_t rpm;
  uint16_t fuelRate;
} EngineData;

// Inputs
typedef struct {
  uint16_t throttle; // [0,1023]
  uint16_t brake; // [0,1023]
  uint16_t battTemp; // [0,1023]
  uint8_t modeSwitches;
  BattData batt;
  EngineData engine;
  MotorData motor;
} DataInputs;

// Outputs
typedef struct {
  uint16_t motorAccel; // [0,4095]
  uint16_t motorRegen; // [0,4095]
  uint8_t engineServo; // [0-255]
  uint8_t modeLEDs;
  uint8_t thermalRelays;

} DataOutputs;


class InputOutput {
public:
  InputOutput(bool debug = false) :
    _debug(debug),
    _btSerial(BT_HWSERIAL),
    _can(CAN_CS) {};

  // Initialize hardware
  bool setup();

  //** INPUTS ** //
  // Returns a value between 0 and 1023
  uint16_t readBattTemp();
  
  // Returns a value between 0 and 1023
  uint16_t readThrottle();
  
  // Returns a value between 0 and 1023
  uint16_t readBrake();

  // Gets engine data from the Arduino Micro and stores in engineData
  void getEngineData(EngineData& engineData);

  // Gets motor data from CAN and stores in motorData
  void getMotorData(MotorData& motorData);

  // Gets battery data from the SOC meter (UART) and stores in battData
  void getBattData(BattData& battData);

  // Return switch mask 
  uint8_t readModeSwitches();

  // ** OUTPUTS ** // 
  // Send engine acceleration value between 0 and 255
  void sendEngineAccel(uint8_t value);
  
  // Send 12-bit motor command over I2C bus. Values between 0 and 4096
  void sendMotorAccel(uint16_t output);

  // Send 12-bit brake command over I2C bus. Values between 0 and 4096
  void sendMotorRegen(uint16_t output);
private:
  bool _debug;
  Adafruit_BluefruitLE_UART _btSerial; // Bluetooth LE module
  Servo _engineServo;
  MCP_CAN _can;
};

#endif