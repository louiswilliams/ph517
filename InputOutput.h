/**
  @author Louis Williams
  @date 2016

  PH517 Controls Runner
**/

#ifndef _InputOutput_H
#define _InputOutput_H

#include <Arduino.h>
#include <assert.h>
#include <Servo.h>
#include <Wire.h>

#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BluefruitLE_UART.h>
#include "mcp_can.h"
#include "Arduhdlc.h"

// Digital pins
#define ENGINE_PIN 4
#define BUTTON_1 33 // White
#define BUTTON_2 35 // Green
#define BUTTON_3 37 // Blue
#define BUTTON_4 39 // Red
#define BUTTON(i) (1<<(i-1))
#define BUTTON_LED_1 32 // White
#define BUTTON_LED_2 34 // Green
#define BUTTON_LED_3 36 // Blue
#define BUTTON_LED_4 38 // Red
#define RELAY_PIN_START 22
#define RELAY_PIN_END 29
#define RELAY_DCDC 24
#define RELAY_REVERSE 27
#define RELAY_MOTORDIR 28
#define RELAY_MOTOR 29
#define RELAY_CRANK 22
#define RELAY_POWER 23
#define RELAY_COOLANT 25
#define BT_MOD 5
// Analog pins
#define THROTTLE_COMP A1
#define THROTTLE_IN A2
#define BRAKE_IN A3
#define BATT_TEMP_IN A4
// I2C Addresses
#define MOTOR_ACCEL_ADDR 0x62
#define MOTOR_REGEN_ADDR 0x63
// Digital Analog Converter command
#define CMD_WRITE_DAC 0x40
// CAN interrupt and chip-select pins
#define CAN_INT digitalPinToInterrupt(2)
#define CAN_CS 9
#define CAN_ADDR1 0x601
#define CAN_ADDR2 0x602
// Serial ports
#define ENGINE_HWSERIAL Serial1
#define BT_HWSERIAL Serial2
#define BATT_HWSERIAL Serial3
// HLDC Constants
#define HLDC_MAX_FRAME_LEN 32
// Extrema
#define ENGINE_SERVO_MIN 129
#define ENGINE_SERVO_MAX 36
#define THROTTLE_MIN 875
#define THROTTLE_MAX 560

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
  bool reverseActive;
  bool crankActive;
  bool enginePoweroffActive;
} DataOutputs;

// CAN Interrupt Handler
extern bool canAvailable;
void canISR(void);

// Engine data HDLC object and handlers
extern Arduhdlc hdlc;
void hdlcSendChar(uint8_t data);
void hdldRecvFrame(const uint8_t* data, uint16_t length);

class InputOutput {
public:
  InputOutput(bool debug = false) :
    _debug(debug),
    _btSerial(BT_HWSERIAL, BT_MOD),
    _gatt(new Adafruit_BLEGatt(_btSerial)),
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

  // Used to copy data directly from the HDLC frame buffer. Parses the raw data
  void setEngineData(const uint16_t* data, uint16_t length);

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

  // Parse engine data from raw data and store in engineData. We don't do this
  // in the runner because that's not the job of the runner.
  static void getEngineDataFromBuffer(const uint8_t* data, uint16_t length,
                                      EngineData& engineData);

private:

  bool _debug;
  Adafruit_BluefruitLE_UART _btSerial; // Bluetooth LE module
  Adafruit_BLEGatt* _gatt;
  Servo _engineServo;
  MCP_CAN _can;
};

#endif
