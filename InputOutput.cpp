/**
  @author Louis Williams, 2016

  Class wrapping all input and output layer functionality
**/
#include "InputOutput.h"

bool InputOutput::setup() {

  bool setupOkay = true;
  // Setup digital pins
  // Switches
  for (int i=SWITCH_IN_START; i <= SWITCH_IN_END; i++) {
    pinMode(i, INPUT);
  }
  // Switch LEDs
  for (int i=SWITCH_LED_START; i <= SWITCH_LED_END; i++) {
    pinMode(i, OUTPUT);
  }
  // Relays
  for (int i=RELAY_PIN_START; i <= RELAY_PIN_END; i++) {
    pinMode(i, OUTPUT);
  }

  // Set up SPI port for CAN bus
  Wire.begin();

  // Attach engine servo to the PWM pin
  _engineServo.attach(ENGINE_PIN);
  _engineServo.write(0);

  // TODO: Choose a different baud rate
  MICRO_HWSERIAL.begin(19200);
  // TODO: Determine actual baud rate
  BATT_HWSERIAL.begin(9600);

  // Setup BT LE module
  if (!_btSerial.begin(_debug)) {
    Serial.println("Failed to init BT module");
    setupOkay = false;
  } else {
    _btSerial.echo(false);
    _btSerial.info();
  }

  if (CAN_OK != _can.begin(CAN_500KBPS)) {
    Serial.println("Failed to init CAN bus shield");
    setupOkay = false;
  }

  return setupOkay;
}

// Send engine acceleration value between 0 and 255
void InputOutput::sendEngineAccel(uint8_t value) {
  int mapped = map(value, 0, 255, 36, 129);
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

// Gets engine data from the Arduino Micro and stores in engineData
void InputOutput::getEngineData(EngineData& engineData) {
  if (MICRO_HWSERIAL.available()) {
    // Process input
  }
}

// Gets motor data from CAN and stores in motorData
void InputOutput::getMotorData(MotorData& motorData) {
  // If CAN flag set from ISR routine, then read data
}

// Gets battery data from the SOC meter (UART) and stores in battData
void InputOutput::getBattData(BattData& battData) {
  if (BATT_HWSERIAL.available()) {
    // Process input
  }
}

// Return switch mask 
uint8_t InputOutput::readModeSwitches() {
  uint8_t mask = 0;
  for (int i=0; i <= (SWITCH_IN_END-SWITCH_IN_START); i++) {
    mask = mask + (digitalRead(i)<<i);
  }
  return mask;
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