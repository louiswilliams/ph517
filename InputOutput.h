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

class InputOutput {
public:
  InputOutput(bool debug = false) :
    _debug(debug),
    _btSerial(BT_HWSERIAL),
    _can(CAN_CS) {};

  // Initialize hardware
  bool setup();

  // Send engine acceleration value between 0 and 255
  void sendEngineAccel(uint8_t value);

  // Returns a value between 0 and 1023
  uint16_t readBattTemp();
  
  // Returns a value between 0 and 1023
  uint16_t readThrottle();
  
  // Returns a value between 0 and 1023
  uint16_t readBrake();
  
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

typedef struct {
  // Inputs
  uint16_t throttle; // [0,1023]
  uint16_t brake; // [0,1023]
  uint16_t battTemp; 
  uint16_t battCharge;
  uint16_t engineRpm;
  uint16_t engineFuelRate;
  uint8_t modeSwitches;
  // From CAN
  uint16_t motorRpm; // x1
  uint8_t motorTemp; // [0,240] - 40
  uint8_t motorControllerTemp; // [0,240] - 40
  uint16_t motorRmsCurrent; // x10
  uint16_t motorCapVoltgae; // x10
  uint16_t motorStatorFreq; // x1

  // Outputs
  uint16_t motorAccel; // [0,4095]
  uint16_t motorRegen; // [0,4095]
  uint8_t engineServo; // [0-180]
  uint8_t modeLEDs;
  uint8_t thermalRelays;
} DataSample;

#endif