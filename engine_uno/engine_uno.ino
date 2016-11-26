#include "Arduhdlc.h"

#include <Adafruit_BLE.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <SoftwareSerial.h>

#define HDLC_MAX_FRAME_LEN 48
#define BT_BAUD 9600
#define BT_BAUD_FAST 57600
#define CHAR_BUF_LEN 16

#define SEND_DELAY 1000

// GATT service indexes
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
#define CAR_MODE 15

// Storage struct for receiving engine data
typedef struct {
  volatile uint32_t rpm;
  volatile uint32_t pulses;
  volatile uint32_t timeOn; 
} EngineData;

// Storage struct for receiving car data for GATT service
typedef struct {
  uint16_t throttle; // [0,1023]
  uint16_t brake; // [0,1023]
  uint16_t battTherm; // [0,1023] 
  uint16_t modeSwitches;
                                  // 8
  // BattData
  uint16_t battVoltage; // x0.01V
  uint16_t battCurrent; // x0.01A
  uint16_t battAmpHours; // x0.1Ah
  uint16_t battStateOfCharge; // x0.1%
                                  // 16
  uint16_t battTimeToGo; // x1 min
  uint16_t battTemp; // x1/256C
  uint8_t battCurrentSign; // 1 for positive
  uint8_t battAmpHourSign; // 1 for positive

  // MotorData
  uint16_t motorRpm; // x1
                                  // 24
  uint16_t motorRmsCurrent; // x10
  uint16_t motorCapVoltage; // x10
  uint16_t motorStatorFreq; // x1
  uint8_t motorTemp; // [0,240] - 40
  uint8_t motorControllerTemp; // [0,240] - 40
                                  // 32
} CarData;

// Pins
int PullHigh = 4;
int FuelPin = 2;
int SparkPin = 3;

// RPM/Fuel calculation variables
volatile uint32_t timeold = 0;
volatile uint32_t timeOff = 0;
volatile uint32_t t = 0;
volatile uint32_t timeoldRPM = 0;
volatile uint32_t delta = 0;

// Storage structures
EngineData engineData = {0, 0, 0};
CarData carData;

// Car GATT service UUID
uint8_t service_uuid[] = {0xB3, 0x4A, 0x10, 0x00, 0x23, 0x03, 0x47, 0xC5, 0x83, 0xD5, 0x86, 0x83, 0x62, 0xDE, 0xEB, 0xA6};
char charBuf[CHAR_BUF_LEN];

// Configure software serial devices
SoftwareSerial ssMega = SoftwareSerial(8, 9); // (RX, TX) Pin 8 supports interrupts on the Uno
SoftwareSerial ssBt = SoftwareSerial(10, 11); // Adafruit BTLE module

bool btSetupOkay;
// Configure BTLE module and GATT object
Adafruit_BluefruitLE_UART btle(ssBt, 12, 6, 7); // (softwareSerial, MODE, CTS, RTS) Bluetooth LE module
Adafruit_BLEGatt gatt(btle);

Arduhdlc hdlc = Arduhdlc(&hdlcSendChar, &hdlcRecvFrame, HDLC_MAX_FRAME_LEN);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ssMega.begin(9600);

  btSetupOkay = setUpBluefruit();
  if (!btSetupOkay) {
    Serial.println(F("Failed to setup BT module"));
  }

  // Add indicator LED
  pinMode(13, OUTPUT);

  // Set pullup resistor for when phototransistor is not closed
  pinMode(PullHigh, INPUT);
  digitalWrite(PullHigh, HIGH);

  // Attach interrupt and 
  attachInterrupt(digitalPinToInterrupt(FuelPin), Fuel, CHANGE);
  pinMode(FuelPin, INPUT);
  digitalWrite(FuelPin, HIGH);

  Serial.println(F("Setup done"));
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t now = millis();

  detachInterrupt(digitalPinToInterrupt(FuelPin));
  
  // Send engine data in little-endian (native) order over serial port
  Serial.print("rpm: ");
  Serial.println(engineData.rpm, DEC);
  Serial.print("pulses: ");
  Serial.println(engineData.pulses, DEC);
  Serial.print("timeOn: ");
  Serial.println(engineData.timeOn, DEC);
  hdlc.frameDecode((const char*)  &engineData, sizeof(EngineData));

  attachInterrupt(digitalPinToInterrupt(FuelPin), Fuel, CHANGE);

  // Receive data
  while (ssMega.available()) {
    hdlc.charReceiver((char) ssMega.read());
  }

  uint32_t end = millis();
  uint32_t delayTime = SEND_DELAY - (end-now);
  delay(delayTime);
}

// Send data
void hdlcSendChar(uint8_t data) {
  ssMega.write((char)data);
}

void hdlcRecvFrame(const uint8_t* data, uint16_t len) {
  if (len == sizeof(CarData)) {
    Serial.println(F("Received data frame from car"));
    // Copy data to struct
    memcpy(&carData, data, len);

    // Send to GATT service
    setGattChars();
  } else {
    Serial.print(F("Received data of unknown length: "));
    Serial.println(len, DEC);
  }
}

// Fuel change interrupt
void Fuel() {
  digitalWrite(13, HIGH);
  unsigned long t;
  t = millis();
  if (digitalRead(FuelPin) == HIGH)  {               // if fuel injector Turns on
    timeOff = t - timeold;
    delta = t - timeoldRPM;
  } else if (digitalRead(FuelPin) == LOW) {          // If fuel injector turns off
    engineData.timeOn += (t - timeold);      // Calculate new delta
    engineData.pulses++;                           // increase # of pulses
    engineData.rpm = 120000 / (t - timeoldRPM) ;  // Calc RPM
    timeoldRPM = t;                     // Time between injection cycles. (Use for RPM)
  }
  timeold = t;
  digitalWrite(13, LOW);
}

// Setup BT LE module
bool setUpBluefruit() {
  Serial.println(F("Setting up Bluefruit. Trying fast baud rate"));

  // Try fast baud rate
  bool btOkay = true;
  if (!btle.begin(false, BT_BAUD_FAST)) {

    Serial.println(F("Trying slow baud"));

    // Then try slow baud rate
    if (!btle.begin(false, BT_BAUD)) {
      Serial.println("Failed to init BT module");
      btOkay = false;
    } else {
      // Increase and reset
      Serial.println(F("Increasing baud rate..."));
      btle.print(F("AT+BAUDRATE="));
      btle.print(BT_BAUD_FAST + "\n");
      btle.waitForOK();
      btle.println(F("ATZ"));
      Serial.println(F("Resetting..."));
      if (!btle.begin(false, BT_BAUD_FAST)) {
        Serial.println(F("Couldn't find Bluefruit"));
        btOkay = false;
      }
    }
  }

  // Set up GATT service 
  if (btOkay) {
    btle.echo(true);
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
    gatt.addCharacteristic(0x100D, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "engine_pulses", NULL);
    gatt.addCharacteristic(0x100E, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "engine_timeon", NULL);
    gatt.addCharacteristic(0x100F, 0x10, 4, 4, BLE_DATATYPE_INTEGER, "car_mode", NULL);

    btle.println("ATZ");
    btle.waitForOK();
    Serial.println("GATT Setup done");

    btle.info();
  }
  btle.echo(false);

  return btOkay;

}

void setGattChars() {
  setChar(0x1, carData.battVoltage);
  setChar(0x2, carData.battCurrent);
  setChar(0x3, carData.battAmpHours);
  setChar(0x4, carData.battStateOfCharge);
  setChar(0x5, carData.battTimeToGo);
  setChar(0x6, carData.battTherm);
  setChar(0x7, carData.motorRpm);
  setChar(0x8, carData.motorTemp);
  setChar(0x9, carData.motorRmsCurrent);
  setChar(0xA, carData.motorCapVoltage);
  setChar(0xB, carData.motorStatorFreq);
  setChar(0xC, engineData.rpm);
  setChar(0xD, engineData.pulses);
  setChar(0xE, engineData.timeOn);
  setChar(0xF, carData.modeSwitches);
}

// Set characteristic value by index
void setChar(uint8_t index, uint16_t value) {
  if (btSetupOkay) {
    String v = String(value);
    v.toCharArray(charBuf, CHAR_BUF_LEN);
    bool isOK = gatt.setChar(index, charBuf);
  }
}

