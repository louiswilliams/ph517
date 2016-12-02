/**
  @author Louis Williams
  @date 2016

  PH571 Data storage classes
**/

#ifndef _Data_h
#define _Data_h

#include <Arduino.h>

// Battery data from SOC meter
typedef struct {
  uint16_t voltage = 0; // x0.01V
  uint16_t current = 0; // x0.01A
  uint16_t ampHours = 0; // x0.1Ah
  uint16_t stateOfCharge = 0; // x0.1%
  uint16_t timeToGo = 0; // x1 min
  uint16_t temp = 0; // x1/256C
  uint8_t currentSign = 0; // 1 for positive, 0 for negative
  uint8_t ampHourSign = 0; // 1 for positive, 0 for negative
} BattData;
// sizeof(BattData) = 14

// CAN data from motor
typedef struct {
  uint16_t rpm = 0; // x1
  uint16_t rmsCurrent = 0; // x10
  uint16_t capVoltage = 0; // x10
  uint16_t statorFreq = 0; // x1
  uint8_t temp = 0; // [0,240] - 40
  uint8_t controllerTemp = 0; // [0,240] - 40
} MotorData;
// sizeof(BattData) = 10

// Engine data from Arduino Micro  
typedef struct {
  uint32_t rpm = 0;
  uint32_t pulses = 0;
  uint32_t timeOn = 0;
} EngineData;

// Inputs
class DataInput {
public:
  uint16_t throttle = 0; // [0,1023]
  uint16_t brake = 0; // [0,1023]
  uint16_t battTemp = 0; // [0,1023]
  uint16_t modeSwitches = 0;
  BattData batt;
  MotorData motor;
  EngineData engine;

  // Get switch/LED n, n > 0
  bool isSwitchPressed(uint8_t i) const;

  // Parse engine data from raw data and store in engineData. We don't do this
  // in the runner because that's not the job of the runner.
  void setEngineData(const uint8_t* data, uint16_t length);
};

// Outputs
class DataOutput {
public:
  int16_t errors = 0;
  uint16_t motorAccel = 0; // [0,4095]
  uint16_t motorRegen = 0; // [0,4095]
  uint8_t engineServo = 0; // [0-255]
  uint8_t modeLEDs = 0;
  uint8_t thermalRelays = 0;

  // Enable/disable LED
  bool isLedOn(uint8_t i) const;
  bool setLedOn(uint8_t i, bool on);
  void setError(uint16_t error);
  void clearError(uint16_t error);
};

#endif // _Data_h