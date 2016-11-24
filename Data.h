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
  uint32_t rpm;
  uint32_t pulses;
  uint32_t timeOn;
} EngineData;

// Inputs
class DataInput {
public:
  uint16_t throttle; // [0,1023]
  uint16_t brake; // [0,1023]
  uint16_t battTemp; // [0,1023]
  uint8_t modeSwitches;
  BattData batt;
  EngineData engine;
  MotorData motor;

  // Get switch/LED n, n > 0
  bool isSwitchPressed(uint8_t i);

  // Parse engine data from raw data and store in engineData. We don't do this
  // in the runner because that's not the job of the runner.
  void setEngineData(const uint8_t* data, uint16_t length);
};

// Outputs
class DataOutput {
public:
  int16_t errors;
  uint16_t motorAccel; // [0,4095]
  uint16_t motorRegen; // [0,4095]
  uint8_t engineServo; // [0-255]
  uint8_t modeLEDs;
  uint8_t thermalRelays;
  bool reverseActive;
  bool crankActive;
  bool enginePoweroffActive;

  // Enable/disable LED
  bool isLedOn(uint8_t i);
  bool setLedOn(uint8_t i, bool on);
  void setError(uint16_t error);
  void clearError(uint16_t error);
};

#endif // _Data_h