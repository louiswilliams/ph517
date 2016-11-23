#include "Arduhdlc.h"

#define HDLC_MAX_FRAME_LEN 16

int FuelPin = 8;
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

Arduhdlc hdlc = Arduhdlc(&hdlcSendChar, &hdldRecvFrame, HDLC_MAX_FRAME_LEN);

// Send data
void hdlcSendChar(uint8_t data) {
  Serial.write((char)data);
}

void hdldRecvFrame(const uint8_t* data, uint16_t len) {
  // Nothing defined for receiving frames
}

// Capture data available on the engine serial line
void serialEvent() {
  while (Serial.available()) {
    hdlc.charReceiver((char) Serial.read());
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3), Fuel, CHANGE);
  pinMode(FuelPin, INPUT);
  digitalWrite(FuelPin, HIGH);
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);
  detachInterrupt(digitalPinToInterrupt(3));
  digitalWrite(13, HIGH);
  // Send in little-endian (native) order over serial port
  Serial.write("test");
//  hdlc.frameDecode((const char*)  &engineFrame, sizeof(EngineFrame));
  digitalWrite(13, LOW);
  attachInterrupt(digitalPinToInterrupt(3), Fuel, CHANGE);

}


void Fuel() {
  unsigned long t;
  t = millis();

  if (digitalRead(FuelPin) == HIGH)  {               // if fuel injector Turns on
    timeOff = t - timeold;
    delta = t - timeoldRPM;
    timeoldRPM = t;                     // Time between injection cycles. (Use for RPM)
  } else if (digitalRead(FuelPin) == LOW) {          // If fuel injector turns off
    engineFrame.timeOn = t - timeold;      // Calculate new delta
    engineFrame.pulses++;                           // increase # of pulses
    engineFrame.rpm = 120 * 1000 / (t - timeold) ;  // Calc RPM
  }
  timeold = t;
}


