/**
  @author Louis Williams, 2016

  Class wrapping all input and output layer functionality
**/
#include "InputOutput.h"

bool InputOutput::setup() {

  // Set up SPI port for CAN bus
  Wire.begin();

  // Setup BT LE module
  if (!_btSerial.begin(_debug)) {
    Serial.println("Failed to init BT module");
    return false;
  }
  // if (!_btSerial.factoryReset()) {
  //   Serial.println("Could not factory reset"); 
  //   return false;
  // }
  _btSerial.echo(false);
  _btSerial.info();

  // Attach engine servo to the PWM pin
  _engineServo.attach(ENGINE_PIN);
  _engineServo.write(0);

  return true;
}

// Send engine acceleration value between 0 and 255
void InputOutput::sendEngineAccel(uint8_t value) {
  int mapped = map(value, 0, 255, 129, 36);
  _engineServo.write(mapped);
}

// Returns a value between 0 and 1023
uint16_t InputOutput::readThrottle() {
  return analogRead(THROTTLE_IN);
}

// Returns a value between 0 and 1023
uint16_t InputOutput::readBrake() {
  return analogRead(BRAKE_IN);
}

// Returns a value between 0 and 1023
uint16_t InputOutput::readBattTemp() {
  return analogRead(BATT_TEMP_IN);
}

// Send 12-bit motor command over I2C bus. Values between 0 and 4096
void InputOutput::sendMotorAccel(uint16_t output) {
  uint8_t twbrback = TWBR;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  Wire.beginTransmission(MOTOR_ACCEL_ADDR);
  Wire.write(CMD_WRITE_DAC);
  Wire.write(output / 16);                   // Upper data bits          (D11.D10.D9.D8.D7.D6.D5.D4)
  Wire.write((output % 16) << 4);            // Lower data bits          (D3.D2.D1.D0.x.x.x.x)
  Wire.endTransmission();
  TWBR = twbrback;
}

// Send 12-bit brake command over I2C bus. Values between 0 and 4096
void InputOutput::sendMotorRegen(uint16_t output) {
  uint8_t twbrback = TWBR;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  Wire.beginTransmission(MOTOR_REGEN_ADDR);
  Wire.write(CMD_WRITE_DAC);
  Wire.write(output / 16);                   // Upper data bits          (D11.D10.D9.D8.D7.D6.D5.D4)
  Wire.write((output % 16) << 4);            // Lower data bits          (D3.D2.D1.D0.x.x.x.x)
  Wire.endTransmission();
  TWBR = twbrback;
}