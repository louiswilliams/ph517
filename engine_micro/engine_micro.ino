#include "Arduhdlc.h"
#include <SoftwareSerial.h>

#define HDLC_MAX_FRAME_LEN 16

int PullHigh = 4;
int FuelPin = 2;
int SparkPin = 3;

typedef struct {
  volatile uint32_t rpm;
  volatile uint32_t pulses;
  volatile uint32_t timeOn; 
} EngineFrame;

EngineFrame engineFrame = {0, 0, 0};

volatile uint32_t timeold = 0;
volatile uint32_t timeOff = 0;
volatile uint32_t t = 0;
volatile uint32_t timeoldRPM = 0;
volatile uint32_t delta = 0;

SoftwareSerial ss = SoftwareSerial(8,9); // Pin 8 supports interrupts on the Uno
Arduhdlc hdlc = Arduhdlc(&hdlcSendChar, &hdldRecvFrame, HDLC_MAX_FRAME_LEN);

// Send data
void hdlcSendChar(uint8_t data) {
  ss.write((char)data);
}

void hdldRecvFrame(const uint8_t* data, uint16_t len) {
  Serial.print("Received bytes: ");
  Serial.println(len, DEC);
  Serial.println((char*) data);
}

// Capture data available on the engine serial line
//void serialEvent() {
//  while (ss.available()) {
//    hdlc.charReceiver((char) ss.read());
//  }
//}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ss.begin(9600);
  attachInterrupt(digitalPinToInterrupt(FuelPin), Fuel, CHANGE);
  pinMode(FuelPin, INPUT);
  pinMode(PullHigh, INPUT);
  digitalWrite(FuelPin, HIGH);
  digitalWrite(PullHigh, HIGH);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);
  detachInterrupt(digitalPinToInterrupt(FuelPin));
  // Send in little-endian (native) order over serial port
  Serial.print("rpm: ");
  Serial.println(engineFrame.rpm, DEC);
  Serial.print("pulses: ");
  Serial.println(engineFrame.pulses, DEC);
  Serial.print("timeOn: ");
  Serial.println(engineFrame.timeOn, DEC);
  hdlc.frameDecode((const char*)  &engineFrame, sizeof(EngineFrame));
  attachInterrupt(digitalPinToInterrupt(FuelPin), Fuel, CHANGE);

  while (ss.available()) {
    hdlc.charReceiver((char) ss.read());
  }
}


void Fuel() {
  digitalWrite(13, HIGH);
  unsigned long t;
  t = millis();
  if (digitalRead(FuelPin) == HIGH)  {               // if fuel injector Turns on
    timeOff = t - timeold;
    delta = t - timeoldRPM;
  } else if (digitalRead(FuelPin) == LOW) {          // If fuel injector turns off
    engineFrame.timeOn += (t - timeold);      // Calculate new delta
    engineFrame.pulses++;                           // increase # of pulses
    engineFrame.rpm = 120000 / (t - timeoldRPM) ;  // Calc RPM
    timeoldRPM = t;                     // Time between injection cycles. (Use for RPM)
  }
  timeold = t;
  digitalWrite(13, LOW);
}


