#include "Arduhdlc.h"

typedef struct {
  uint16_t rpm;
  uint16_t fuelRate;
} EngineData;

#define HDLC_MAX_FRAME_LEN 16
volatile float lastChange, x;
volatile uint16_t numBangs;
EngineData engineData;
Arduhdlc hdlc = Arduhdlc(&hdlcSendChar, &hdldRecvFrame, HDLC_MAX_FRAME_LEN);


void pointsOpening(){
    unsigned long t;
    t = millis();
    
    x = t - lastChange;
    lastChange = t;
    ++numBangs;
}

// Send data
void hdlcSendChar(uint8_t data) {
  Serial.print((char)data);
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
  Serial.begin(9600);
  pinMode(3, INPUT);
  digitalWrite(3, HIGH);
}

void loop() {
   // Number of coil pulses
  numBangs = 0;
  // Sample engine RPM for 25ms
  attachInterrupt(digitalPinToInterrupt(2), pointsOpening, FALLING);
  delay(25);
  detachInterrupt(2);
  //numBangs = ((60000/75)*numBangs)/2;
  numBangs = (1000/x)*120;

  // Send in big-endian order over serial port
  char data[4] = {0};
  data[0] = numBangs >> 8;
  data[1] = numBangs & 0xFF;
  hdlc.frameDecode((const char*)  data, 4);
}
