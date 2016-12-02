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
#include "Data.h"

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
#define BATT_HWSERIAL Serial3
// Baud rates
#define ENGINE_BAUD 9600
#define BT_BAUD 9600
#define BT_BAUD_FAST 57600
#define BATT_BAUD 9600

// HDLC Constants
#define HDLC_MAX_FRAME_LEN 32
// Batt meter constants
#define IDHT 128
// Extrema
#define ENGINE_SERVO_MIN 129
#define ENGINE_SERVO_MAX 36
#define THROTTLE_MIN 863
#define THROTTLE_MAX 560

// BT GATT Characteristic indexes
#define BATT_VOLTAGE 1
#define BATT_CURRENT 2
#define BATT_AMPHRS 3
#define BATT_SOC 4
#define BATT_TIME 5
#define BATT_TEMP 6
#define MOTOR_RPM 7
#define MOTOR_TEMP 8
#define MOTOR_CURRENT 9
#define MOTOR_VOLTAGE 10
#define MOTOR_STATOR 11
#define ENGINE_RPM 12
#define ENGINE_PULSES 13
#define ENGINE_TIMEON 14

// Error constants
#define ERR_BT_CONN 1

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

  // Read engine data from the external Arduino
  void readEngineData();

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
  bool _btOkay;
  Servo _engineServo;
  MCP_CAN _can;
};

#endif
