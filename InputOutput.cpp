/**
  @author Louis Williams, 2016

  Class wrapping all input and output layer functionality
**/
#include "InputOutput.h"

// CAN receive buffer
#define CAN_BUF_LEN 8
uint8_t canBufLen = CAN_BUF_LEN;
uint8_t canBuf[CAN_BUF_LEN];

#define CHAR_BUF_LEN 8
char charBuf[CHAR_BUF_LEN];

bool InputOutput::setup() {

  bool setupOkay = true;
  // Setup digital pins
  // Switches
  pinMode(BUTTON_1, INPUT);
  digitalWrite(BUTTON_1, HIGH);
  pinMode(BUTTON_2, INPUT);
  digitalWrite(BUTTON_2, HIGH);
  pinMode(BUTTON_3, INPUT);
  digitalWrite(BUTTON_3, HIGH);
  pinMode(BUTTON_4, INPUT);
  digitalWrite(BUTTON_4, HIGH);

  // Switch LEDs
  pinMode(BUTTON_LED_1, OUTPUT);
  digitalWrite(BUTTON_LED_1, HIGH);
  pinMode(BUTTON_LED_2, OUTPUT);
  digitalWrite(BUTTON_LED_2, HIGH);
  pinMode(BUTTON_LED_3, OUTPUT);
  digitalWrite(BUTTON_LED_3, HIGH);
  pinMode(BUTTON_LED_4, OUTPUT);
  digitalWrite(BUTTON_LED_4, HIGH);

  // Relays
  for (int i=RELAY_PIN_START; i <= RELAY_PIN_END; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }
  digitalWrite(RELAY_DCDC, LOW);

  // Set up SPI port for CAN bus
  Wire.begin();

  // Attach engine servo to the PWM pin
  _engineServo.attach(ENGINE_PIN);
  _engineServo.write(ENGINE_SERVO_MIN);

  // Start, and set timeout
  ENGINE_HWSERIAL.begin(ENGINE_BAUD);
  ENGINE_HWSERIAL.setTimeout(3000);
  char buffer;
  // If no data received, consider this a failure
  if (ENGINE_HWSERIAL.readBytes(&buffer, 1) == 0) {
    setupOkay = false;
    Serial.println(F("Received no data from engine serial"));
  }

  // TODO: Determine actual baud rate
  BATT_HWSERIAL.begin(BATT_BAUD);

  if (CAN_OK != _can.begin(CAN_125KBPS)) {
    Serial.println("Failed to init CAN bus shield");
    setupOkay = false;
  }


  // Wait to turn on motor controller
  delay(1000);
  digitalWrite(RELAY_MOTOR, LOW);

  return setupOkay;
}

// Send engine acceleration value between 0 and 255
void InputOutput::sendEngineAccel(uint8_t value) {
  int mapped = map(value, 0, 255, ENGINE_SERVO_MIN, ENGINE_SERVO_MAX);
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

// Gets motor data from CAN and stores in motorData
void InputOutput::getMotorData(MotorData& motorData) {
  // If CAN messages available, read buffer

  while (CAN_MSGAVAIL == _can.checkReceive()) {
    _can.readMsgBuf(&canBufLen, canBuf);
    uint32_t id = _can.getCanId();

    // Parse data based on which address it came from
    if (id == CAN_ADDR1) {
      motorData.rpm = canBuf[1] + (canBuf[0] << 8);

      // Convert to non-negative, normalized values
      uint8_t temp = ((int8_t) canBuf[2]) + 40;
      uint8_t controllerTemp = ((int8_t)canBuf[3]) + 40;

      // Assumptions made from the motor manual values
      assert(temp >= 0);
      assert(temp <= 240);
      assert(controllerTemp >= 0);
      assert(controllerTemp <= 240);

      motorData.temp = temp; 
      
      motorData.controllerTemp = controllerTemp;
      motorData.rmsCurrent = canBuf[5] + (canBuf[4] << 8);
      
      motorData.capVoltage = canBuf[7] + (canBuf[6] << 8);

    } else if (id == CAN_ADDR2) {
      motorData.statorFreq = canBuf[1] + (canBuf[0] << 8);
    } 
    else {
      // Serial.print("Received message from unknown ID: ");
      // Serial.println(id, HEX);
    }
  }
}

// Gets battery data from the SOC meter (UART) and stores in battData
void InputOutput::getBattData(BattData& battData) {
  // If data available, consume all of it
  if (BATT_HWSERIAL.available()) {
    uint8_t current;
    
    // Wait for IDHT message start bit
    do {
      current = BATT_HWSERIAL.read();
    }
    while (BATT_HWSERIAL.available() && !(current & IDHT));
    // Process input
  }
}

// Reads all data from the serial device, which is passed to the HDLC decoder
void InputOutput::readEngineData() {
  while (ENGINE_HWSERIAL.available()) {
    hdlc.charReceiver((char) ENGINE_HWSERIAL.read());
  }
}

// Return switch mask 
uint8_t InputOutput::readModeSwitches() {
  uint8_t mask = 0;
  mask += ((digitalRead(BUTTON_1) == LOW));
  mask += ((digitalRead(BUTTON_2) == LOW)<<1);
  mask += ((digitalRead(BUTTON_3) == LOW)<<2);
  mask += ((digitalRead(BUTTON_4) == LOW)<<3);
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

