#include "Arduhdlc.h"

#include <Adafruit_BLE.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <SoftwareSerial.h>

#define HDLC_MAX_FRAME_LEN 48
#define BT_BAUD 9600
#define BT_BAUD_FAST 57600
#define CHAR_BUF_LEN 16

#define SEND_DELAY 250

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
int SparkPullup = 4;
int FuelPullup = 10;
int FuelPin = 2;
int SparkPin = 3;

// RPM/Fuel calculation variables
volatile uint32_t timeold = 0;
volatile uint32_t timeOff = 0;
volatile uint32_t t = 0;
volatile uint32_t timeoldRPM = 0;
volatile uint32_t delta = 0;

volatile bool settingGatt = false;

// Storage structures
EngineData engineData = {0, 0, 0};
CarData carData;

// Car GATT service UUID
uint8_t service_uuid[] = {0xB3, 0x4A, 0x10, 0x00, 0x23, 0x03, 0x47, 0xC5, 0x83, 0xD5, 0x86, 0x83, 0x62, 0xDE, 0xEB, 0xA6};
char charBuf[CHAR_BUF_LEN];

#define MEGA_TX 12
#define MEGA_RX 11

#define BT_TX 9
#define BT_RX 8

// Do we want to use the HW Serial port to communicate with the Mega?
// Set to false to debug the Uno
#define USE_HWSERAL true

// Configure software serial devices
// Printing will be a noop if using HW serial
#ifdef USE_HWSERAL
  #define ssMega Serial
  #define PRINT(msg,args...)
  #define PRINTLN(msg,args...)
#else
  SoftwareSerial ssMega = SoftwareSerial(MEGA_RX, MEGA_TX); // (RX, TX) Arduino Mega serial
  #define PRINT(msg,args...) Serial.print(msg,args)
  #define PRINTLN(msg, args...) Serial.println(msg,args)
#endif

SoftwareSerial ssBt = SoftwareSerial(BT_RX, BT_TX); // Adafruit BTLE module

bool btSetupOkay = false;
// Configure BTLE module and GATT object
Adafruit_BluefruitLE_UART btle(ssBt, 5, 6, 7); // (softwareSerial, MODE, CTS, RTS) Bluetooth LE module
Adafruit_BLEGatt gatt(btle);

Arduhdlc hdlc = Arduhdlc(&hdlcSendChar, &hdlcRecvFrame, HDLC_MAX_FRAME_LEN);

void setup() {

#ifndef USE_HWSERAL
  Serial.begin(115200);
#endif

  ssMega.begin(9600);

  btSetupOkay = setUpBluefruit();
  if (!btSetupOkay) {
    PRINTLN(F("Failed to setup BT module"));
  }

  // Add indicator LED
  pinMode(13, OUTPUT);

  // Set pullup resistors for when phototransistor is not closed
  pinMode(SparkPullup, INPUT);
  digitalWrite(SparkPullup, HIGH);

  pinMode(FuelPullup, INPUT);
  digitalWrite(FuelPullup, HIGH);

  // Attach interrupt and 
  pinMode(FuelPin, INPUT);
  digitalWrite(FuelPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(FuelPin), Fuel, CHANGE);

  PRINTLN(F("Setup done"));
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t start = millis();

  detachInterrupt(digitalPinToInterrupt(FuelPin));
  
  // Send engine data in little-endian (native) order over serial port
  PRINT("rpm: ");
  PRINTLN(engineData.rpm, DEC);
  PRINT("pulses: ");
  PRINTLN(engineData.pulses, DEC);
  PRINT("timeOn: ");
  PRINTLN(engineData.timeOn, DEC);
  hdlc.frameDecode((const char*)  &engineData, sizeof(EngineData));

  attachInterrupt(digitalPinToInterrupt(FuelPin), Fuel, CHANGE);  

  // Receive data
  bool data = false;
  while (ssMega.available()) {
    hdlc.charReceiver((char) ssMega.read());
    data = true;
  }

  if (data) {
    // Send to GATT service
    setGattChars();    
  }

  uint32_t end = millis();
  uint32_t delayTime = end - start;
  if (delayTime < SEND_DELAY) {
    delay(SEND_DELAY - delayTime);    
  }
}

// Send data
void hdlcSendChar(uint8_t data) {
  ssMega.write((char)data);
}

void hdlcRecvFrame(const uint8_t* data, uint16_t len) {
  if (len == sizeof(CarData)) {
    PRINTLN(F("Received data frame from car"));
    // Copy data to struct
    memcpy(&carData, data, len);
  } else {
    PRINT(F("Received data of unknown length: "));
    PRINTLN(len, DEC);
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
  PRINTLN(F("Setting up Bluefruit. Trying fast baud rate"));

  bool debug = false;
  // Try fast baud rate
  bool btOkay = true;
  if (!btle.begin(debug, BT_BAUD_FAST)) {

    PRINTLN(F("Trying slow baud"));

    // Then try slow baud rate
    if (!btle.begin(debug, BT_BAUD)) {
      PRINTLN("Failed to init BT module");
      btOkay = false;
    } else {
      // Increase and reset
      PRINTLN(F("Increasing baud rate..."));
      btle.print(F("AT+BAUDRATE="));
      btle.println(BT_BAUD_FAST, DEC);
      btle.waitForOK();
      btle.println(F("ATZ"));
      PRINTLN(F("Resetting..."));
      if (!btle.begin(debug, BT_BAUD_FAST)) {
        PRINTLN(F("Couldn't find Bluefruit"));
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
    PRINTLN("GATT Setup done");

    btle.info();
  }
  btle.echo(false);

  return btOkay;

}

void setGattChars() {
  // setChar(0x1, carData.battVoltage);
  // setChar(0x2, carData.battCurrent);
  // setChar(0x3, carData.battAmpHours);
  // setChar(0x4, carData.battStateOfCharge);
  // setChar(0x5, carData.battTimeToGo);
  setChar(0x6, carData.battTherm);
  setChar(0x7, carData.motorRpm);
  setChar(0x8, carData.motorTemp);
  setChar(0x9, carData.motorRmsCurrent);
  setChar(0xA, carData.motorCapVoltage);
  setChar(0xB, carData.motorStatorFreq);
  // setChar(0xF, carData.modeSwitches);
  setChar(0xC, engineData.rpm);
  setChar(0xD, engineData.pulses);
  setChar(0xE, engineData.timeOn);
}

// Set characteristic value by index
void setChar(uint8_t index, uint16_t value) {
  if (btSetupOkay) {
    String v = String(value);
    v.toCharArray(charBuf, CHAR_BUF_LEN);
    bool isOK = gatt.setChar(index, charBuf);
  }
}

